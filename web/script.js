document.addEventListener("DOMContentLoaded", () => {
    // --- Constantes ---
    const MAX_CHART_POINTS = 300; // ~30s a 10Hz, ~60s a 5Hz
    const MAX_POSE_POINTS = 150;  // Metade dos pontos do gráfico
    const POSE_CANVAS_SIZE = 300; // Deve bater com o <canvas>
    const MAX_LOG_ENTRIES = 200;

    // --- Elementos da UI ---
    const statusDiv = document.getElementById("connection-status");
    const logContainer = document.getElementById("log-messages");
    const ecuInfoDiv = document.getElementById("ecu-info");

    // Indicadores
    const indicators = {
        estop: document.getElementById("status-estop"),
        charging: document.getElementById("status-charging"),
        bumper_fl: document.getElementById("bumper-fl"),
        bumper_fr: document.getElementById("bumper-fr"),
        bumper_bl: document.getElementById("bumper-bl"),
        bumper_br: document.getElementById("bumper-br")
    };

    // --- Variáveis de Estado ---
    let ws;
    let charts = {};
    let gauges = {};
    let poseHistory = [];
    let poseCtx = document.getElementById("poseCanvas").getContext("2d");

    // --- Inicialização ---
    function init() {
        initTabs();
        initCharts();
        initGauges();
        initPoseCanvas();
        initForms();
        initGraphToggles();
        connect();
    }

    // --- Abas ---
    function initTabs() {
        const tabButtons = document.querySelectorAll(".tab-button");
        const tabContents = document.querySelectorAll(".tab-content");

        tabButtons.forEach(button => {
            button.addEventListener("click", () => {
                const targetTab = button.getAttribute("data-tab");
                
                tabButtons.forEach(btn => btn.classList.remove("active"));
                button.classList.add("active");
                
                tabContents.forEach(content => {
                    content.id === targetTab ? content.classList.add("active") : content.classList.remove("active");
                });
            });
        });
    }

    // --- Conexão WebSocket ---
    function connect() {
        ws = new WebSocket("ws://" + window.location.host);

        ws.onopen = () => {
            statusDiv.textContent = "CONECTADO";
            statusDiv.className = "status-connected";
            addLog("Conexão WebSocket estabelecida.");
            // Solicita as informações da ECU ao conectar
            ws.send(JSON.stringify({ type: "get_ecu_info" }));
            // Solicita os valores de configuração atuais
            ws.send(JSON.stringify({ type: "get_all_configs" }));
        };

        ws.onclose = () => {
            statusDiv.textContent = "DESCONECTADO";
            statusDiv.className = "status-disconnected";
            addLog("Conexão perdida. Tentando reconectar em 3s...");
            setTimeout(connect, 3000);
        };

        ws.onerror = (error) => { addLog(`Erro no WebSocket: ${error.message || 'Erro'}`); ws.close(); };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                handleMessage(data);
            } catch (e) {
                console.error("Erro ao analisar JSON:", e, event.data);
            }
        };
    }

    function addLog(message) {
        const entry = document.createElement("div");
        entry.className = "log-entry";
        entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
        
        // Adiciona a nova entrada no final
        logContainer.appendChild(entry);

        // NOVO: Remove entradas antigas se o log estiver muito grande
        while (logContainer.children.length > MAX_LOG_ENTRIES) {
            logContainer.removeChild(logContainer.firstChild);
        }

        // Rola para o final (apenas se o usuário já não estiver rolando para cima)
        // (Uma pequena melhoria de usabilidade)
        const isScrolledToBottom = logContainer.scrollHeight - logContainer.clientHeight <= logContainer.scrollTop + 10;
        if (isScrolledToBottom) {
            logContainer.scrollTop = logContainer.scrollHeight;
        }
    }

    // --- Roteador de Mensagens ---
    function handleMessage(data) {
        switch (data.type) {
            case "log":
                addLog(`[Servidor] ${data.message}`);
                break;
            case "full_status":
                updateCharts(data);
                updateGauges(data);
                updatePose(data.pose);
                updateIndicators(data.status_flags);
                break;
            case "ecu_info":
                updateEcuInfo(data.info);
                break;
            case "current_configs":
                updateConfigForms(data.configs);
                break;
            default:
                addLog(`Tipo de mensagem desconhecido: ${data.type}`);
        }
    }

    // --- Inicialização de Gráficos e Gauges ---
    function initCharts() {
        // Gráfico de Velocidade
        charts.velocity = new Chart(document.getElementById('velChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Fusão Linear', data: [], borderColor: '#3498db', tension: 0.1, hidden: false },
                    { label: 'Fusão Angular', data: [], borderColor: '#e74c3c', tension: 0.1, hidden: false },
                    { label: 'Encoder Linear', data: [], borderColor: '#2ecc71', tension: 0.1, hidden: true },
                    { label: 'Encoder Angular', data: [], borderColor: '#f1c40f', tension: 0.1, hidden: true },
                    { label: 'IMU Angular', data: [], borderColor: '#9b59b6', tension: 0.1, hidden: true },
                    { label: 'Setpoint Linear', data: [], borderColor: '#3498db', borderDash: [5, 5], tension: 0.1, hidden: false },
                    { label: 'Setpoint Angular', data: [], borderColor: '#e74c3c', borderDash: [5, 5], tension: 0.1, hidden: false }
                ]
            },
            options: chartOptions('Velocidade (m/s, rad/s)', true)
        });

        // Gráfico de Aceleração
        charts.acceleration = new Chart(document.getElementById('accelChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Accel X', data: [], borderColor: '#3498db', tension: 0.1 },
                    { label: 'Accel Y', data: [], borderColor: '#e74c3c', tension: 0.1 },
                    { label: 'Accel Z', data: [], borderColor: '#2ecc71', tension: 0.1 }
                ]
            },
            options: chartOptions('Aceleração (m/s²)', true)
        });

        // Gráfico de Encoders
        charts.encoders = new Chart(document.getElementById('encoderChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Pulsos Esq.', data: [], borderColor: '#3498db', tension: 0.1 },
                    { label: 'Pulsos Dir.', data: [], borderColor: '#e74c3c', tension: 0.1 }
                ]
            },
            options: chartOptions('Pulsos', true)
        });
    }

    function chartOptions(title, showLegend = false) {
        return {
            animation: false,
            scales: {
                y: { 
                    title: { display: true, text: title, color: '#e0e0e0' }, 
                    ticks: { color: '#e0e0e0' },
                    grid: { color: '#444' } // Linhas horizontais OK
                },
                x: { 
                    ticks: { display: false }, 
                    grid: { 
                        display: false // NOVO: Remove linhas de grade verticais
                    } 
                }
            },
            plugins: { 
                legend: { 
                    // MODIFICADO: Usa o parâmetro
                    display: showLegend,
                    labels: {
                        color: '#e0e0e0' // Cor do texto da legenda
                    }
                } 
            },
            maintainAspectRatio: false,
            elements: { point: { radius: 0 } }
        };
    }

    function initGauges() {
        let gaugeOpts = {
            angle: -0.25,
            lineWidth: 0.2,
            radiusScale: 0.9,
            pointer: { length: 0.6, strokeWidth: 0.035, color: '#e0e0e0' },
            // staticLabels: { font: "12px sans-serif", labels: [0, 50, 100], color: "#e0e0e0" },
            staticZones: [
                {strokeStyle: "#f44336", min: 0, max: 20},
                {strokeStyle: "#f1c40f", min: 20, max: 40},
                {strokeStyle: "#4caf50", min: 40, max: 100}
            ],
            limitMax: true, limitMin: true,
            colorStart: '#f44336', colorStop: '#4caf50',
            strokeColor: '#444', generateGradient: true
        };

        gauges.battery = new Gauge(document.getElementById('gauge-battery')).setOptions(gaugeOpts);
        gauges.battery.maxValue = 100; gauges.battery.set(0);

        gauges.motorLeft = new Gauge(document.getElementById('gauge-motor-left')).setOptions(gaugeOpts);
        gauges.motorLeft.maxValue = 10; gauges.motorLeft.set(0);

        gauges.motorRight = new Gauge(document.getElementById('gauge-motor-right')).setOptions(gaugeOpts);
        gauges.motorRight.maxValue = 10; gauges.motorRight.set(0);

        gauges.chargeCurrent = new Gauge(document.getElementById('gauge-charge-current')).setOptions(gaugeOpts);
        gauges.chargeCurrent.maxValue = 5; gauges.chargeCurrent.set(0);

        gaugeOpts.staticZones[0].min = 80; // Red
        gaugeOpts.staticZones[0].max = 100;
        gaugeOpts.staticZones[1].min = 50; // Yellow
        gaugeOpts.staticZones[1].max = 80;
        gaugeOpts.staticZones[2].min = 0; // Green
        gaugeOpts.staticZones[2].max = 50;

        gauges.tempECU = new Gauge(document.getElementById('gauge-temp-ecu')).setOptions(gaugeOpts);
        gauges.tempECU.maxValue = 100; gauges.tempECU.set(0);

        gauges.tempMCU = new Gauge(document.getElementById('gauge-temp-mcu')).setOptions(gaugeOpts);
        gauges.tempMCU.maxValue = 100; gauges.tempMCU.set(0);
    }
    
    // --- Atualização de Dados ---
    function updateCharts(data) {
        const now = data.timestamp ? new Date(data.timestamp * 1000).toLocaleTimeString() : new Date().toLocaleTimeString();

        // Velocidade
        if (charts.velocity) {
            charts.velocity.data.labels.push(now);
            charts.velocity.data.datasets[0].data.push(data.velocity.fusion.linear);
            charts.velocity.data.datasets[1].data.push(data.velocity.fusion.angular);
            charts.velocity.data.datasets[2].data.push(data.velocity.encoder.linear);
            charts.velocity.data.datasets[3].data.push(data.velocity.encoder.angular);
            charts.velocity.data.datasets[4].data.push(data.velocity.imu.angular);
            charts.velocity.data.datasets[5].data.push(data.setpoints.linear);
            charts.velocity.data.datasets[6].data.push(data.setpoints.angular);
            trimChartHistory(charts.velocity);
            charts.velocity.update('none');
        }

        // Aceleração
        if (charts.acceleration) {
            charts.acceleration.data.labels.push(now);
            charts.acceleration.data.datasets[0].data.push(data.acceleration.x);
            charts.acceleration.data.datasets[1].data.push(data.acceleration.y);
            charts.acceleration.data.datasets[2].data.push(data.acceleration.z);
            trimChartHistory(charts.acceleration);
            charts.acceleration.update('none');
        }
        
        // Encoders
        if (charts.encoders) {
            charts.encoders.data.labels.push(now);
            charts.encoders.data.datasets[0].data.push(data.encoders.left_pulses);
            charts.encoders.data.datasets[1].data.push(data.encoders.right_pulses);
            trimChartHistory(charts.encoders);
            charts.encoders.update('none');
        }
    }

    function trimChartHistory(chart) {
        while (chart.data.labels.length > MAX_CHART_POINTS) {
            chart.data.labels.shift();
            chart.data.datasets.forEach(dataset => {
                dataset.data.shift();
            });
        }
    }

    function updateGauges(data) {
        if (!data.gauges) return;
        
        gauges.battery.set(data.gauges.battery_level);
        document.getElementById('gauge-val-battery').textContent = data.gauges.battery_level.toFixed(1);

        gauges.motorLeft.set(data.gauges.motor_current_left);
        document.getElementById('gauge-val-motor-left').textContent = data.gauges.motor_current_left.toFixed(1);

        gauges.motorRight.set(data.gauges.motor_current_right);
        document.getElementById('gauge-val-motor-right').textContent = data.gauges.motor_current_right.toFixed(1);

        gauges.chargeCurrent.set(data.gauges.charging_current);
        document.getElementById('gauge-val-charge-current').textContent = data.gauges.charging_current.toFixed(1);

        gauges.tempECU.set(data.gauges.temp_ecu);
        document.getElementById('gauge-val-temp-ecu').textContent = data.gauges.temp_ecu.toFixed(1);

        gauges.tempMCU.set(data.gauges.temp_mcu);
        document.getElementById('gauge-val-temp-mcu').textContent = data.gauges.temp_mcu.toFixed(1);
    }
    
    function updateIndicators(flags) {
        // E-Stop
        updateIndicator(indicators.estop, flags.estop, "red", "E-STOP");
        // Bumpers
        updateIndicator(indicators.bumper_fl, flags.bumpers.fl, "orange");
        updateIndicator(indicators.bumper_fr, flags.bumpers.fr, "orange");
        updateIndicator(indicators.bumper_bl, flags.bumpers.bl, "orange");
        updateIndicator(indicators.bumper_br, flags.bumpers.br, "orange");
        // Carregamento
        let chargeText = "DESCARREGANDO";
        let chargeColor = "yellow";
        if (flags.charging_status === "charging") {
            chargeText = "CARREGANDO";
            chargeColor = "green";
        } else if (flags.charging_status === "idle") {
            chargeText = "OCIOSO";
            chargeColor = "grey";
        }
        updateIndicator(indicators.charging, true, chargeColor, chargeText);
    }

    function updateIndicator(el, isActive, activeClass, text = null) {
        if (!el) return;
        if (text) el.textContent = text;
        isActive ? el.classList.add("active", activeClass) : el.classList.remove("active", activeClass);
        if (activeClass === 'grey') {
            el.style.backgroundColor = '#888';
            el.style.color = 'white';
        } else {
            el.style.backgroundColor = '';
            el.style.color = '';
        }
    }

    function updateEcuInfo(info) {
        info.wheel_distance = info.wheel_distance.toFixed(3);
        info.wheel_diameter = info.wheel_diameter.toFixed(3);
        
        ecuInfoDiv.textContent = `Nome do Robô:   ${info.robot_name || 'N/A'}
Versão ECU:       ${info.ecu_version || 'N/A'}
Versão Driver:    ${info.driver_version || 'N/A'}
Versão Motor:     ${info.motor_version || 'N/A'}
Dist. Rodas:    ${info.wheel_distance || 'N/A'} m
Diâmetro Rodas: ${info.wheel_diameter || 'N/A'} m
Git Commit:       ${info.git_hash || 'N/A'}
Git Branch:       ${info.git_branch || 'N/A'}
Git Tag:          ${info.git_tag || 'N/A'}
Data do Build:    ${info.build_date || 'N/A'}`;
    }

    // --- Desenho da Pose ---
    function initPoseCanvas() {
        poseCtx.canvas.width = POSE_CANVAS_SIZE;
        poseCtx.canvas.height = POSE_CANVAS_SIZE;
        drawPose(); // Desenha o grid inicial
    }

    function updatePose(pose) {
        poseHistory.push(pose);
        while (poseHistory.length > MAX_POSE_POINTS) {
            poseHistory.shift();
        }

        document.getElementById('pose-val-x').textContent = pose.x.toFixed(3);
        document.getElementById('pose-val-y').textContent = pose.y.toFixed(3);
        document.getElementById('pose-val-theta').textContent = pose.theta.toFixed(3);
        
        drawPose();

        drawPose();
    }

    function drawPose() {
        const ctx = poseCtx;
        const w = POSE_CANVAS_SIZE;
        const h = POSE_CANVAS_SIZE;

        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = "#111";
        ctx.fillRect(0, 0, w, h);

        if (poseHistory.length === 0) return;

        // Acha o centro da trajetória
        let avgX = 0, avgY = 0;
        poseHistory.forEach(p => { avgX += p.x; avgY += p.y; });
        avgX /= poseHistory.length;
        avgY /= poseHistory.length;
        
        // Fator de zoom (provisório, idealmente seria dinâmico)
        const zoom = 20; // 20 pixels por metro

        ctx.save();
        // Centraliza o canvas na trajetória média
        ctx.translate(w / 2 - avgX * zoom, h / 2 - avgY * zoom);

        // Desenha grid (relativo ao centro)
        // ... (lógica de grid omitida por brevidade)

        // Desenha trajetória
        ctx.beginPath();
        ctx.strokeStyle = "#444";
        const firstPose = poseHistory[0];
        ctx.moveTo(firstPose.x * zoom, firstPose.y * zoom);
        poseHistory.forEach(p => {
            ctx.lineTo(p.x * zoom, p.y * zoom);
        });
        ctx.stroke();

        // Desenha o robô (posição atual)
        const currentPose = poseHistory[poseHistory.length - 1];
        ctx.translate(currentPose.x * zoom, currentPose.y * zoom);
        ctx.rotate(currentPose.theta);

        // Desenha um triângulo para o robô
        ctx.beginPath();
        ctx.moveTo(10, 0); // Frente
        ctx.lineTo(-5, -5); // Canto esquerdo
        ctx.lineTo(-5, 5); // Canto direito
        ctx.closePath();
        ctx.fillStyle = "#e74c3c";
        ctx.fill();
        
        ctx.restore();
    }
    
    // --- Lógica da Aba de Configuração ---
    function initForms() {
        // PIDs
        document.getElementById("pid-form").addEventListener("submit", (e) => {
            e.preventDefault();
            const target = e.submitter.dataset.target;
            if (!target) return;
            
            const payload = {
                type: "set_pid",
                target: target,
                p: parseFloat(document.getElementById(`pid-${target}-p`).value),
                i: parseFloat(document.getElementById(`pid-${target}-i`).value),
                d: parseFloat(document.getElementById(`pid-${target}-d`).value)
            };
            ws.send(JSON.stringify(payload));
            addLog(`Enviando PID para: ${target}`);
        });

        // Limites
        document.getElementById("limits-form").addEventListener("submit", (e) => {
            e.preventDefault();
            const payload = {
                type: "set_limits",
                linear_vel: parseFloat(document.getElementById("limit-linear-vel").value),
                linear_acc: parseFloat(document.getElementById("limit-linear-acc").value),
                angular_vel: parseFloat(document.getElementById("limit-angular-vel").value),
                angular_acc: parseFloat(document.getElementById("limit-angular-acc").value)
            };
            ws.send(JSON.stringify(payload));
            addLog("Enviando novos limites de vel/acel.");
        });

        // Malha Aberta
        document.getElementById("open-loop-check").addEventListener("change", (e) => {
            const payload = {
                type: "set_open_loop",
                enabled: e.target.checked
            };
            ws.send(JSON.stringify(payload));
            addLog(`Modo Malha Aberta: ${e.target.checked ? 'HABILITADO' : 'DESABILITADO'}`);
        });

        // Kalman
        initMatrix("kalman-model-matrix");
        initMatrix("kalman-measurement-matrix");
        document.getElementById("kalman-form").addEventListener("submit", (e) => {
            e.preventDefault();
            const payload = {
                type: "set_kalman_cov",
                model: readMatrix("kalman-model-matrix"),
                measurement: readMatrix("kalman-measurement-matrix")
            };
            ws.send(JSON.stringify(payload));
            addLog("Enviando matrizes de covariância.");
        });
    }

    function initMatrix(containerId) {
        const container = document.getElementById(containerId);
        const size = parseInt(container.dataset.size);
        container.style.gridTemplateColumns = `repeat(${size}, 1fr)`;
        for (let r = 0; r < size; r++) {
            for (let c = 0; c < size; c++) {
                const input = document.createElement('input');
                input.type = 'number';
                input.step = '0.001';
                input.id = `${containerId}-${r}-${c}`;
                input.value = (r === c) ? '1.0' : '0.0'; // Identidade por padrão
                container.appendChild(input);
            }
        }
    }

    function readMatrix(containerId) {
        const container = document.getElementById(containerId);
        const size = parseInt(container.dataset.size);
        const matrix = [];
        for (let r = 0; r < size; r++) {
            const row = [];
            for (let c = 0; c < size; c++) {
                row.push(parseFloat(document.getElementById(`${containerId}-${r}-${c}`).value));
            }
            matrix.push(row);
        }
        return matrix;
    }
    
    function writeMatrix(containerId, matrix) {
        const container = document.getElementById(containerId);

        if (!container) {
                console.error(`Elemento da matriz não encontrado: ${containerId}`);
                return;
        }

        const size = parseInt(container.dataset.size);

        if (!matrix || matrix.length !== size) return;

        for (let r = 0; r < size; r++) {
            if (!matrix[r] || matrix[r].length !== size) return;
            for (let c = 0; c < size; c++) {
                const el = document.getElementById(`${containerId}-${r}-${c}`);
                if(el) el.value = matrix[r][c];
            }
        }
    }
    
    // Recebe e popula os formulários com dados do robô
    function updateConfigForms(configs) {
        if (!configs) return;
        
        // PIDs
        if (configs.pid) {
            ['linear', 'angular', 'left', 'right'].forEach(target => {
                if (configs.pid[target]) {
                    document.getElementById(`pid-${target}-p`).value = configs.pid[target].p;
                    document.getElementById(`pid-${target}-i`).value = configs.pid[target].i;
                    document.getElementById(`pid-${target}-d`).value = configs.pid[target].d;
                }
            });
        }
        // Limites
        if (configs.limits) {
            document.getElementById("limit-linear-vel").value = configs.limits.linear_vel;
            document.getElementById("limit-linear-acc").value = configs.limits.linear_acc;
            document.getElementById("limit-angular-vel").value = configs.limits.angular_vel;
            document.getElementById("limit-angular-acc").value = configs.limits.angular_acc;
        }
        // Malha Aberta
        if (configs.open_loop !== undefined) {
            document.getElementById("open-loop-check").checked = configs.open_loop;
        }
        // Kalman
        if (configs.kalman) {
            writeMatrix("kalman-model-matrix", configs.kalman.model);
            writeMatrix("kalman-measurement-matrix", configs.kalman.measurement);
        }
    }

    // --- Toggles dos Gráficos ---
    function initGraphToggles() {
        document.getElementById("vel-toggles").addEventListener("change", (e) => {
            if (e.target.type !== 'checkbox') return;
            const datasetLabel = e.target.dataset.dataset;
            const chart = charts.velocity;
            
            const datasetMap = {
                'vel_linear_fusion': 0,
                'vel_angular_fusion': 1,
                'vel_linear_encoder': 2,
                'vel_angular_encoder': 3,
                'vel_angular_imu': 4,
                'setpoint_linear': 5,
                'setpoint_angular': 6,
            };
            
            const index = datasetMap[datasetLabel];
            if (index !== undefined) {
                chart.data.datasets[index].hidden = !e.target.checked;
                chart.update();
            }
        });
    }

    // --- Iniciar Aplicação ---
    init();
});
