document.addEventListener("DOMContentLoaded", () => {
    // --- Constantes ---
    const MAX_CHART_POINTS = 300; // ~30s a 10Hz, ~60s a 5Hz
    const MAX_POSE_POINTS = 150;  // Metade dos pontos do gráfico
    const POSE_CANVAS_SIZE = 300; // Deve bater com o <canvas>
    const MAX_LOG_ENTRIES = 200;
    let joystickVel = { linear: 0, angular: 0 };
    let joystickTimer = null;
    const JOYSTICK_SEND_INTERVAL = 100; // Envia comandos 10x por segundo
    const JOYSTICK_MAX_LINEAR = 0.5; // m/s
    const JOYSTICK_MAX_ANGULAR = 1.0; // rad/s
    const STACK_WARNING_THRESHOLD = 512; // bytes

    // --- Elementos da UI ---
    const statusDiv = document.getElementById("connection-status");
    const logContainer = document.getElementById("log-messages");
    const ecuInfoDiv = document.getElementById("ecu-info");
    const navInfoDiv = document.getElementById("nav-info");

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
        initJoystick();
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
        // const isScrolledToBottom = logContainer.scrollHeight - logContainer.clientHeight <= logContainer.scrollTop + 10;
        // if (isScrolledToBottom) {
        logContainer.scrollTop = logContainer.scrollHeight;
        // }
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
            case "nav_info":
                updateNavInfo(data.info);
                break;
            case "rtos_tasks":
                updateRTOSCard(data.info);
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
                    { label: 'Encoder Linear', data: [], borderColor: '#2ecc71', tension: 0.1, hidden: false },
                    { label: 'Encoder Angular', data: [], borderColor: '#f1c40f', tension: 0.1, hidden: false },
                    { label: 'IMU Angular', data: [], borderColor: '#9b59b6', tension: 0.1, hidden: false },
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
                    { label: 'Vel. Esq.', data: [], borderColor: '#3498db', tension: 0.1 },
                    { label: 'Vel. Dir.', data: [], borderColor: '#e74c3c', tension: 0.1 }
                ]
            },
            options: chartOptions('Velocidade (rad/s)', true) // Mostra legenda
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

        let tempOpts = {
            angle: -0.25,
            lineWidth: 0.2,
            radiusScale: 0.9,
            pointer: { length: 0.6, strokeWidth: 0.035, color: '#e0e0e0' },
            // staticLabels: { font: "12px sans-serif", labels: [0, 50, 100], color: "#e0e0e0" },
            staticZones: [
                {strokeStyle: "#f44336", min: 80, max: 100},
                {strokeStyle: "#f1c40f", min: 50, max: 80},
                {strokeStyle: "#4caf50", min: 0, max: 50}
            ],
            limitMax: true, limitMin: true,
            colorStart: '#f44336', colorStop: '#4caf50',
            strokeColor: '#444', generateGradient: true
        };

        gauges.battery = new Gauge(document.getElementById('gauge-battery')).setOptions(gaugeOpts);
        gauges.battery.maxValue = 100; gauges.battery.set(0);

        gauges.motorCurrent = new Gauge(document.getElementById('gauge-motor')).setOptions(gaugeOpts);
        gauges.motorCurrent.maxValue = 10; gauges.motorCurrent.set(0);

        gauges.chargeCurrent = new Gauge(document.getElementById('gauge-charge-current')).setOptions(gaugeOpts);
        gauges.chargeCurrent.maxValue = 5; gauges.chargeCurrent.set(0);

        gauges.tempIMU = new Gauge(document.getElementById('gauge-temp-imu')).setOptions(tempOpts);
        gauges.tempIMU.maxValue = 100; gauges.tempIMU.set(0);

        gauges.tempECU = new Gauge(document.getElementById('gauge-temp-ecu')).setOptions(tempOpts);
        gauges.tempECU.maxValue = 100; gauges.tempECU.set(0);

        gauges.tempMCU = new Gauge(document.getElementById('gauge-temp-mcu')).setOptions(tempOpts);
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
        
        if (charts.encoders && data.velocity && data.velocity.encoder) {
            charts.encoders.data.labels.push(now);
            charts.encoders.data.datasets[0].data.push(data.velocity.encoder.left || 0);
            charts.encoders.data.datasets[1].data.push(data.velocity.encoder.right || 0);
            trimChartHistory(charts.encoders);
            charts.encoders.update('none');
        }
        
        // Atualiza os labels de contagem de pulsos
        if (data.encoders) {
            document.getElementById('encoder-val-left').textContent = data.encoders.left_pulses.toFixed(2);
            ;
            document.getElementById('encoder-val-right').textContent = data.encoders.right_pulses.toFixed(2);
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

        gauges.motorCurrent.set(data.gauges.motor_current);
        document.getElementById('gauge-val-motor').textContent = data.gauges.motor_current.toFixed(1);

        gauges.chargeCurrent.set(data.gauges.charging_current);
        document.getElementById('gauge-val-charge-current').textContent = data.gauges.charging_current.toFixed(1);

        gauges.tempIMU.set(data.gauges.temp_imu);
        document.getElementById('gauge-val-temp-imu').textContent = data.gauges.temp_imu.toFixed(1);

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

    function updateRTOSCard(json) {
      const container = document.getElementById("tasks-container");
      const title = document.getElementById("rtos-title");

      container.innerHTML = "";

      if (!json || json.length === 0) {
        title.textContent = "RTOS Tasks (0)";
        container.innerHTML = "<em>Nenhuma task ativa</em>";
        return;
      }

      title.textContent = `RTOS Tasks (${json.length})`;

      json.forEach(task => {
        const taskDiv = document.createElement("div");
        taskDiv.className = "task";

        const warning =
          task.stack_free < STACK_WARNING_THRESHOLD ? "warning" : "";

        const cpu = Math.min(task.cpu ?? 0, 100).toFixed(2);
        const state = (task.state || "INVALID").toUpperCase();

        taskDiv.innerHTML = `
      <div class="task-header">
        <span class="task-name">${task.name}</span>

        <div class="task-right">
          <span class="task-cpu">${cpu}%</span>
          <span class="task-status status-${state}">${state}</span>
        </div>
      </div>

      <div class="cpu-bar">
        <div class="cpu-fill" style="width: ${cpu}%;"></div>
      </div>

      <div class="stack-info ${warning}">
        Stack livre: <strong>${task.stack_free} bytes</strong>
      </div>
    `;

        container.appendChild(taskDiv);
      });
    }

    function updateEcuInfo(info) {
        info.wheel_distance = info.wheel_distance.toFixed(3);
        info.wheel_diameter = info.wheel_diameter.toFixed(3);
        info.pkt_frequency = info.pkt_frequency.toFixed(1);
        
        ecuInfoDiv.textContent = `ECU:   ${info.ecu_connected ? 'Conectada' : 'Desconectada'}
Taxa de Atualização:   ${info.pkt_frequency || 'N/A'} Hz
Nome do Robô:   ${info.robot_name || 'N/A'}
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

    function updateNavInfo(info) {
        info.last_pose.x = info.last_pose.x.toFixed(3);
        info.last_pose.y = info.last_pose.y.toFixed(3);
        info.last_pose.yaw = info.last_pose.yaw.toFixed(3);

        let last_pose = `(${info.last_pose.x} ; ${info.last_pose.y} @ ${info.last_pose.yaw})`

        info.eta = info.eta.toFixed(2);
        info.rem_distance = info.rem_distance.toFixed(2);
        
        navInfoDiv.textContent = `Status:   ${info.status || 'Desativado'}
Última Pose:        ${last_pose    || '(x;y @ yaw)'}
Navegação:          ${info.navigation_status || 'Desconhecido'}
Localização:        ${info.localization_status  || 'Desconhecido'}
Feedback:           ${info.nav_feedback || 'Desconhecido'}
Poses Restantes:    ${info.rem_poses || '0'}
ETA:                ${info.eta       || '0'} s
Distância Restante: ${info.rem_distance || '0'} m
Tempo Total:        ${info.total_time   || '0'} s
Recuperações:       ${info.recoveries   || '0'}`;
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

        if (poseHistory.length === 0) return;

        // Acha o centro da trajetória
        let avgX = 0, avgY = 0;
        poseHistory.forEach(p => { avgX += p.x; avgY += p.y; });
        avgX /= poseHistory.length;
        avgY /= poseHistory.length;
        
        // Fator de zoom (provisório)
        const zoom = 40; // 40 pixels por metro

        ctx.save();
        // Centraliza o canvas na trajetória média e inverte o eixo Y
        ctx.translate(w / 2, h / 2);
        ctx.scale(1, -1); // Inverte Y (para Y positivo ser para cima)
        ctx.translate(-avgX * zoom, -avgY * zoom);
        
        // Desenha a Grade e os Eixos
        drawPoseGrid(ctx, avgX, avgY, zoom, w, h);

        //  Desenha trajetória
        ctx.beginPath();
        ctx.strokeStyle = "#aaa"; // Cor mais clara
        ctx.lineWidth = 3; // MODIFICADO: Mais largo
        const firstPose = poseHistory[0];
        ctx.moveTo(firstPose.x * zoom, firstPose.y * zoom);
        poseHistory.forEach(p => {
            ctx.lineTo(p.x * zoom, p.y * zoom);
        });
        ctx.stroke();

        // Desenha o robô (posição atual)
        const currentPose = poseHistory[poseHistory.length - 1];
        ctx.translate(currentPose.x * zoom, currentPose.y * zoom);
        ctx.rotate(currentPose.theta); // Theta já deve estar em radianos
        
        // Inverte o robô de volta, já que invertemos o canvas
        ctx.scale(1, -1);

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
    
    function drawPoseGrid(ctx, centerX, centerY, zoom, width, height) {
        ctx.strokeStyle = "#2a2a2a"; // Cor da grade
        ctx.lineWidth = 1;

        const meterSize = 1 * zoom; // 1 metro em pixels
        const viewWidthMeters = (width / zoom) / 2;
        const viewHeightMeters = (height / zoom) / 2;
        
        const startX = Math.floor(centerX - viewWidthMeters);
        const endX = Math.ceil(centerX + viewWidthMeters);
        const startY = Math.floor(centerY - viewHeightMeters);
        const endY = Math.ceil(centerY + viewHeightMeters);
        
        // Linhas verticais
        for (let x = startX; x <= endX; x++) {
            ctx.beginPath();
            ctx.moveTo(x * zoom, startY * zoom - meterSize);
            ctx.lineTo(x * zoom, endY * zoom + meterSize);
            ctx.stroke();
        }
        // Linhas horizontais
        for (let y = startY; y <= endY; y++) {
            ctx.beginPath();
            ctx.moveTo(startX * zoom - meterSize, y * zoom);
            ctx.lineTo(endX * zoom + meterSize, y * zoom);
            ctx.stroke();
        }
        
        // Eixos X e Y
        ctx.strokeStyle = "#444"; // Cor dos eixos
        ctx.lineWidth = 2;
        
        // Eixo X (Y=0)
        ctx.beginPath();
        ctx.moveTo(startX * zoom - meterSize, 0);
        ctx.lineTo(endX * zoom + meterSize, 0);
        ctx.stroke();
        
        // Eixo Y (X=0)
        ctx.beginPath();
        ctx.moveTo(0, startY * zoom - meterSize);
        ctx.lineTo(0, endY * zoom + meterSize);
        ctx.stroke();
    }
    
    // --- Lógica da Aba de Configuração ---
    function initForms() {
        // PIDs
        document.getElementById("pid-form").addEventListener("submit", (e) => {
            e.preventDefault();
            const target = e.submitter.dataset.target;
            if (!target) return;
            
            // Pega valores dos inputs, se estiverem vazios, não envia
            const p_val = document.getElementById(`pid-${target}-p`).value;
            const i_val = document.getElementById(`pid-${target}-i`).value;
            const d_val = document.getElementById(`pid-${target}-d`).value;
            
            const payload = {
                type: "set_pid",
                target: target,
                // NOVO: Envia o estado do checkbox
                enabled: document.getElementById(`pid-${target}-enable`).checked,
                // Envia o valor apenas se não for nulo
                p: (p_val !== "") ? parseFloat(p_val) : null,
                i: (i_val !== "") ? parseFloat(i_val) : null,
                d: (d_val !== "") ? parseFloat(d_val) : null
            };
            
            ws.send(JSON.stringify(payload));
            addLog(`Enviando PID para: ${target}`);

            // Limpa os campos de input após o envio
            document.getElementById(`pid-${target}-p`).value = "";
            document.getElementById(`pid-${target}-i`).value = "";
            document.getElementById(`pid-${target}-d`).value = "";
        });
        
        // Adiciona listener para os checkboxes de PID (envia imediatamente)
        const pidCheckboxes = document.querySelectorAll('#pid-form input[type="checkbox"]');
        pidCheckboxes.forEach(cb => {
            cb.addEventListener('change', (e) => {
                const target = e.target.id.replace('pid-', '').replace('-enable', '');
                const payload = {
                    type: "set_pid_enabled", // Um novo tipo de mensagem
                    target: target,
                    enabled: e.target.checked
                };
                ws.send(JSON.stringify(payload));
                addLog(`PID ${target} ${e.target.checked ? 'Habilitado' : 'Desabilitado'}`);
            });
        });

        document.getElementById("nav-poses-form").addEventListener("submit", (e) => {
            e.preventDefault();
            const target = e.submitter.dataset.target;
            if (!target) return;

            const payload = {
                type: "test_nav",
                button: target
            }
            ws.send(JSON.stringify(payload));
            addLog(`Enviando Nav-${target}`);
        });

        document.getElementById("record-poses-check").addEventListener('change', (e) => {

            const payload = {
                type: "record_poses",
                enabled: e.target.checked
            }
            ws.send(JSON.stringify(payload));
            addLog(`Gravar Poses: ${e.target.checked? 'Habilitado' : 'Desabilitado' }`);
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
                    // Atualiza os LABELS de valor atual
                    document.getElementById(`current-pid-${target}-p`).textContent = configs.pid[target].p.toFixed(3);
                    document.getElementById(`current-pid-${target}-i`).textContent = configs.pid[target].i.toFixed(3);
                    document.getElementById(`current-pid-${target}-d`).textContent = configs.pid[target].d.toFixed(3);
                    // Atualiza o CHECKBOX
                    document.getElementById(`pid-${target}-enable`).checked = configs.pid[target].enabled;
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

    // Lógica do Joystick 
    function initJoystick() {
        const options = {
            zone: document.getElementById('joystick-container'),
            mode: 'semi', // 'static' ou 'semi' ou 'dynamic'
            catchDistance: 150,
            color: 'white',
            size: 200 // Tamanho do joystick
        };
        const manager = nipplejs.create(options);
        
        const linearVal = document.getElementById('joystick-val-linear');
        const angularVal = document.getElementById('joystick-val-angular');

        manager.on('move', (evt, data) => {
            if (!data.vector) return;
            
            // Mapeia o joystick (Y para linear, X para angular)
            // Normaliza a distância (0 a 1)
            const distance = Math.min(data.distance / (options.size / 2), 1.0);
            const angleRad = data.angle.radian;
            
            // Calcula vetores x e y normalizados
            const vectorX = distance * Math.cos(angleRad);
            const vectorY = distance * Math.sin(angleRad);
            
            // Mapeia para velocidades do robô
            const linear = vectorY * JOYSTICK_MAX_LINEAR;
            const angular = -vectorX * JOYSTICK_MAX_ANGULAR; // Negativo para (esquerda -> +angular)
            
            joystickVel = { linear, angular };
            
            // Atualiza a UI
            linearVal.textContent = linear.toFixed(2);
            angularVal.textContent = angular.toFixed(2);
            
            // Inicia o timer para enviar os dados, se não estiver rodando
            if (!joystickTimer) {
                // Envia o primeiro comando imediatamente
                sendJoystickVelocity(); 
                joystickTimer = setInterval(sendJoystickVelocity, JOYSTICK_SEND_INTERVAL);
            }
        });

        manager.on('end', () => {
            // Para o timer
            clearInterval(joystickTimer);
            joystickTimer = null;
            
            // Reseta e envia o comando de parada
            joystickVel = { linear: 0, angular: 0 };
            sendJoystickVelocity();
            
            // Atualiza a UI
            linearVal.textContent = "0.00";
            angularVal.textContent = "0.00";
        });
    }
    
    // Função que envia o comando de velocidade
    function sendJoystickVelocity() {
        if (!ws || ws.readyState !== WebSocket.OPEN) return;
        
        ws.send(JSON.stringify({
            type: "set_velocity_command",
            linear: joystickVel.linear,
            angular: joystickVel.angular
        }));
    }

    // --- Iniciar Aplicação ---
    init();
});
