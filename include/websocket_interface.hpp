#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <set>
#include <functional>
#include <fstream> // Para servir arquivos
#include <sstream> // Para servir arquivos
#include <nlohmann/json.hpp>

// Aliases
typedef websocketpp::server<websocketpp::config::asio> server_t;
typedef websocketpp::connection_hdl connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
using json = nlohmann::json;

class WebsocketInterface {
public:
    using pid_callback_t = std::function<void(double p, double i, double d)>;
    using speed_config_callback_t = std::function<void(double max_vel, double max_acc)>;
    using command_callback_t = std::function<void(const std::string& command, const json& payload)>;

    /**
     * @brief Construtor.
     * @param port Porta para o servidor WebSocket e HTTP.
     * @param docroot Caminho para a pasta raiz dos arquivos da web (ex: "install/seu_pacote/share/seu_pacote/web").
     */
    WebsocketInterface(uint16_t port, const std::string& docroot);

    ~WebsocketInterface();
    void run();
    void stop();
    void send_robot_status(const json& robot_data);
    void send_log(const std::string& message);

    // Métodos de registro de callback (sem alterações)
    void register_pid_callback(pid_callback_t cb) { m_pid_callback = std::move(cb); }
    void register_speed_config_callback(speed_config_callback_t cb) { m_speed_config_callback = std::move(cb); }
    void register_command_callback(command_callback_t cb) { m_command_callback = std::move(cb); }

private:
    void on_open(connection_hdl hdl);
    void on_close(connection_hdl hdl);
    void on_message(connection_hdl hdl, server_t::message_ptr msg);
    /**
     * @brief Handler para requisições HTTP. Serve os arquivos da GUI.
     */
    void on_http(connection_hdl hdl);

    void broadcast(const std::string& message);
    std::string get_mime_type(const std::string& path);

    uint16_t m_port;
    std::string m_docroot; // Caminho para os arquivos web
    server_t m_server;
    std::thread m_server_thread;
    std::mutex m_mutex;
    std::set<connection_hdl, std::owner_less<connection_hdl>> m_connections;

    pid_callback_t m_pid_callback;
    speed_config_callback_t m_speed_config_callback;
    command_callback_t m_command_callback;
};
