#include "websocket_interface.hpp"

WebsocketInterface::WebsocketInterface(uint16_t port,
                                       const std::string& docroot)
    : m_port(port), m_docroot(docroot)
{
    m_server.clear_access_channels(websocketpp::log::alevel::all);
    m_server.init_asio();
    m_server.set_reuse_addr(true);

    // Registra os handlers, incluindo o HTTP
    m_server.set_open_handler(bind(&WebsocketInterface::on_open, this, ::_1));
    m_server.set_close_handler(bind(&WebsocketInterface::on_close, this, ::_1));
    m_server.set_message_handler(
        bind(&WebsocketInterface::on_message, this, ::_1, ::_2));
    m_server.set_http_handler(
        bind(&WebsocketInterface::on_http, this, ::_1)); // <-- NOVO
}

WebsocketInterface::~WebsocketInterface()
{
    stop();
}

void WebsocketInterface::run()
{
    try
    {
        m_server.listen(m_port);
        m_server.start_accept();

        std::cout << "[WebsocketInterface] Servidor iniciado na porta "
                  << m_port << std::endl;

        // Inicia o loop de processamento do ASIO em um thread separado
        m_server_thread = std::thread([this]() {
            try
            {
                m_server.run();
            }
            catch (const std::exception& e)
            {
                std::cerr
                    << "[WebsocketInterface] Exceção no thread do servidor: "
                    << e.what() << std::endl;
            }
        });
    }
    catch (websocketpp::exception const& e)
    {
        std::cerr << "[WebsocketInterface] Falha ao iniciar o servidor: "
                  << e.what() << std::endl;
    }
}

void WebsocketInterface::stop()
{
    if (m_server_thread.joinable())
    {
        m_server.stop_listening();
        m_server.stop();
        m_server_thread.join();
        std::cout << "[WebsocketInterface] Servidor parado." << std::endl;
    }
}

void WebsocketInterface::on_open(connection_hdl hdl)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_connections.insert(hdl);
    std::cout << "[WebsocketInterface] Nova conexão." << std::endl;

    // Envia uma mensagem de boas-vindas ao novo cliente
    json welcome_msg;
    welcome_msg["type"] = "log";
    welcome_msg["message"] = "Conectado ao servidor ROS 2 do robô.";
    m_server.send(hdl, welcome_msg.dump(), websocketpp::frame::opcode::text);
}

void WebsocketInterface::on_close(connection_hdl hdl)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_connections.erase(hdl);
    std::cout << "[WebsocketInterface] Conexão fechada." << std::endl;
}

void WebsocketInterface::on_message(connection_hdl hdl,
                                    server_t::message_ptr msg)
{
    std::string payload = msg->get_payload();
    // std::cout << "[WebsocketInterface] Mensagem recebida: " << payload <<
    // std::endl;

    try
    {
        json data = json::parse(payload);
        std::string type = data.at("type");

        // --- Roteamento de Mensagens da GUI ---

        if (type == "set_pid" && m_pid_callback)
        {
            double p = data.at("p");
            double i = data.at("i");
            double d = data.at("d");
            // Chama o callback registrado pelo nó ROS
            m_pid_callback(p, i, d);
        }
        else if (type == "set_speed_config" && m_speed_config_callback)
        {
            double max_vel = data.at("max_vel");
            double max_acc = data.at("max_acc");
            m_speed_config_callback(max_vel, max_acc);
        }
        else if (type == "command" && m_command_callback)
        {
            std::string cmd = data.at("command");
            json cmd_payload = data.value("payload", json::object());
            m_command_callback(cmd, cmd_payload);
        }
        else
        {
            std::cerr << "[WebsocketInterface] Tipo de mensagem desconhecido "
                         "ou callback não registrado: "
                      << type << std::endl;
        }
    }
    catch (json::parse_error& e)
    {
        std::cerr << "[WebsocketInterface] Erro de parse JSON: " << e.what()
                  << std::endl;
    }
    catch (json::type_error& e)
    {
        std::cerr << "[WebsocketInterface] JSON faltando campo obrigatório: "
                  << e.what() << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << "[WebsocketInterface] Erro ao processar mensagem: "
                  << e.what() << std::endl;
    }
}

void WebsocketInterface::broadcast(const std::string& message)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& hdl : m_connections)
    {
        try
        {
            m_server.send(hdl, message, websocketpp::frame::opcode::text);
        }
        catch (websocketpp::exception const& e)
        {
            std::cerr << "[WebsocketInterface] Falha ao enviar broadcast: "
                      << e.what() << std::endl;
        }
    }
}

void WebsocketInterface::send_robot_status(const json& robot_data)
{
    // O tipo já deve estar no objeto json, mas podemos garantir
    // (Melhor prática é quem chama 'send_robot_status' já montar o JSON
    // completo) Ex: jsonData["type"] = "robot_status";
    broadcast(robot_data.dump());
}

void WebsocketInterface::send_log(const std::string& message)
{
    json log_msg;
    log_msg["type"] = "log";
    log_msg["message"] = message;
    broadcast(log_msg.dump());
}

void WebsocketInterface::on_http(connection_hdl hdl)
{
    server_t::connection_ptr con = m_server.get_con_from_hdl(hdl);
    std::string resource = con->get_resource();

    // Se a requisição for para a raiz, sirva o index.html
    if (resource == "/")
    {
        resource = "/index.html";
    }

    std::string full_path = m_docroot + resource;

    std::ifstream file(full_path, std::ios::in | std::ios::binary);
    if (!file)
    {
        // Arquivo não encontrado, retorna 404
        std::string body = "404 Not Found";
        con->set_body(body);
        con->set_status(websocketpp::http::status_code::not_found);
        return;
    }

    // Lê o conteúdo do arquivo para a resposta
    std::string body((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());

    con->set_body(body);
    con->set_status(websocketpp::http::status_code::ok);
    // Define o tipo de conteúdo para que o navegador renderize corretamente
    con->append_header("Content-Type", get_mime_type(full_path));
}

std::string WebsocketInterface::get_mime_type(const std::string& path)
{
    size_t dot_pos = path.rfind('.');
    if (dot_pos == std::string::npos)
    {
        return "application/octet-stream";
    }
    std::string ext = path.substr(dot_pos);

    if (ext == ".html")
        return "text/html";
    if (ext == ".css")
        return "text/css";
    if (ext == ".js")
        return "application/javascript";
    if (ext == ".json")
        return "application/json";
    if (ext == ".png")
        return "image/png";
    if (ext == ".jpg")
        return "image/jpeg";
    if (ext == ".svg")
        return "image/svg+xml";

    return "application/octet-stream";
}
