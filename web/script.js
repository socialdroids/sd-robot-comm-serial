document.addEventListener("DOMContentLoaded", () => {
    const statusDiv = document.getElementById("connection-status");
    const logContainer = document.getElementById("log-messages");

    // Elementos de dados
    const valIds = [
        "linear-vel", "angular-vel", "setpoint-linear", "setpoint-angular",
        "current-a", "current-b", "bumper"
    ];
    const valElements = valIds.reduce((acc, id) => {
        acc[id] = document.getElementById(`val-${id}`);
        return acc;
    }, {});

    // Formulários
    const pidForm = document.getElementById("pid-form");
    const speedForm = document.getElementById("speed-form");

    let ws;

    function connect() {
        // Usa o host da página para se conectar ao WebSocket na mesma origem
        ws = new WebSocket("ws://" + window.location.host);

        ws.onopen = () => {
            statusDiv.textContent = "CONECTADO";
            statusDiv.className = "status-connected";
            addLog("Conexão WebSocket estabelecida.");
        };

        ws.onclose = () => {
            statusDiv.textContent = "DESCONECTADO";
            statusDiv.className = "status-disconnected";
            addLog("Conexão perdida. Tentando reconectar em 3 segundos...");
            setTimeout(connect, 3000);
        };

        ws.onerror = (error) => {
            addLog(`Erro no WebSocket: ${error.message || 'Erro desconhecido'}`);
            ws.close();
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                handleMessage(data);
            } catch (e) {
                console.error("Erro ao analisar JSON:", e);
                addLog(`Dado inválido recebido: ${event.data}`);
            }
        };
    }

    function addLog(message) {
        const entry = document.createElement("div");
        entry.className = "log-entry";
        entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
        logContainer.appendChild(entry);
        // Rola para o final
        logContainer.scrollTop = logContainer.scrollHeight;
    }

    function handleMessage(data) {
        switch (data.type) {
            case "log":
                addLog(`[Servidor] ${data.message}`);
                break;
            case "robot_status":
                updateStatusDisplay(data);
                break;
            case "sensor_update": // Para atualizações individuais
                if (data.sensor === 'bumper' && valElements.bumper) {
                    valElements.bumper.textContent = data.value ? "Ativado" : "Inativo";
                }
                break;
            default:
                addLog(`Tipo de mensagem desconhecido: ${data.type}`);
        }
    }

    function updateStatusDisplay(status) {
        if (status.velocity && valElements["linear-vel"]) {
            valElements["linear-vel"].textContent = status.velocity.linear.toFixed(2);
            valElements["angular-vel"].textContent = status.velocity.angular.toFixed(2);
        }
        if (status.setpoint_linear !== undefined && valElements["setpoint-linear"]) {
            valElements["setpoint-linear"].textContent = status.setpoint_linear.toFixed(2);
        }
        if (status.setpoint_angular !== undefined && valElements["setpoint-angular"]) {
            valElements["setpoint-angular"].textContent = status.setpoint_angular.toFixed(2);
        }
        if (status.current_sensor_A !== undefined && valElements["current-a"]) {
            valElements["current-a"].textContent = status.current_sensor_A.toFixed(2);
        }
        if (status.current_sensor_B !== undefined && valElements["current-b"]) {
            valElements["current-b"].textContent = status.current_sensor_B.toFixed(2);
        }
    }

    // --- Envio de Comandos ---
    pidForm.addEventListener("submit", (e) => {
        e.preventDefault();
        const payload = {
            type: "set_pid",
            p: parseFloat(document.getElementById("pid-p").value),
            i: parseFloat(document.getElementById("pid-i").value),
            d: parseFloat(document.getElementById("pid-d").value)
        };
        ws.send(JSON.stringify(payload));
        addLog(`Enviando novos parâmetros PID: P=${payload.p}, I=${payload.i}, D=${payload.d}`);
    });

    speedForm.addEventListener("submit", (e) => {
        e.preventDefault();
        const payload = {
            type: "set_speed_config",
            max_vel: parseFloat(document.getElementById("max-vel").value),
            max_acc: parseFloat(document.getElementById("max-acc").value)
        };
        ws.send(JSON.stringify(payload));
        addLog(`Enviando novos limites: Vel=${payload.max_vel}, Acel=${payload.max_acc}`);
    });

    // Inicia a conexão
    connect();
});
