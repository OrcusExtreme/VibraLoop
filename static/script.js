// 소켓 연결 설정 (initSocketListeners()에서 한 번만 연결)
let socket = null;

// 차트 설정
let mainChart, zChart;
let xVals = [0];
let yVals = [0];
let zVals = [0];
let timeVals = [0]; // 각 데이터 포인트의 경과 시간 저장
let materialWidth = 0;
let materialHeight = 0;
let materialDepth = 0;

// 가공 시간 관련 변수
let estimatedProcessingTime = "00:00:00";
let processingStartTime = null;
let processingTimer = null;
let totalSeconds = 0;

// 센서 데이터 관련 변수
let sensorDataArray = []; // 센서 데이터 저장 배열
let isRecordingSensorData = false; // 센서 데이터 기록 상태

// 공구 정보 관련 변수
let toolDiameter = null;  // 공구 직경 (초기값 없음)
let toolFlutes = null;    // 공구 날 수 (초기값 없음)

// 전역 상태 변수
let currentProcessingState = 'idle'; // 서버 state_update 기준: idle, running, paused, emergency, alarm, alarm_unlocked
let controllerConnected = false;     // CNC 컨트롤러(GRBL) 포트 연결 여부

// 허용 공차 관련 변수 (서버 초기값 0mm와 일치, 접속 시 서버 값으로 갱신)
let toleranceValue = 0;

// 센서 데이터 로그 관련 변수 및 함수
const sensorLogText = document.getElementById('sensor-log-text');
let sensorLogAutoScroll = true;
const MAX_SENSOR_LOG_LINES = 500; // 센서는 약 100Hz로 들어오므로 오래된 로그는 삭제

// 센서 데이터 로그에 메시지 추가
function addSensorLog(message, type = 'info') {
    if (!sensorLogText) return;
    
    const logEntry = document.createElement('div');
    logEntry.className = `log-${type}`;
    
    // 타임스탬프 추가
    const now = new Date();
    const timestamp = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}.${now.getMilliseconds().toString().padStart(3, '0')}`;
    
    // 가속도계, 자이로스코프 데이터 추출 시도
    const accGyroMatch = message.match(/가속도\(X:([-\d\.]+), Y:([-\d\.]+), Z:([-\d\.]+)\), 자이로\(X:([-\d\.]+), Y:([-\d\.]+), Z:([-\d\.]+)\)/);
    
    if (accGyroMatch) {
        // 가속도계, 자이로스코프 데이터 파싱
        const accX = parseFloat(accGyroMatch[1]);
        const accY = parseFloat(accGyroMatch[2]);
        const accZ = parseFloat(accGyroMatch[3]);
        const gyroX = parseFloat(accGyroMatch[4]);
        const gyroY = parseFloat(accGyroMatch[5]);
        const gyroZ = parseFloat(accGyroMatch[6]);

        // 센서 데이터 배열에 저장
        const sensorData = {
            timestamp: now.toISOString(),
            accX: accX,
            accY: accY,
            accZ: accZ,
            gyroX: gyroX,
            gyroY: gyroY,
            gyroZ: gyroZ
        };
        sensorDataArray.push(sensorData);
    }
    
    logEntry.textContent = `[${timestamp}] ${message}`;
    sensorLogText.appendChild(logEntry);
    while (sensorLogText.children.length > MAX_SENSOR_LOG_LINES) {
        sensorLogText.removeChild(sensorLogText.firstChild);
    }

    if (sensorLogAutoScroll) {
        sensorLogText.scrollTop = sensorLogText.scrollHeight;
    }
}

// 페이지 로드 시 초기화 (DOMContentLoaded는 여기서 한 번만 등록)
document.addEventListener('DOMContentLoaded', function() {
    try {
        console.log("페이지 로드됨, 초기화 시작");
        initPage();
    } catch (error) {
        console.error("초기화 중 오류 발생:", error);
    }
});

// 페이지 초기화 함수
function initPage() {
    // 로그 초기화
    console.log("페이지 초기화 시작");
    
    initCharts();
    initSocketListeners();
    
    const connectBtn = document.getElementById('connect-btn');
    const disconnectBtn = document.getElementById('disconnect-btn');
    
    // 초기 버튼 상태 설정
    if (connectBtn) connectBtn.style.display = 'inline-block';
    if (disconnectBtn) disconnectBtn.style.display = 'none';
    
    // 시간 정보 초기화
    updateTimeInfo();
    
    // 초기 포트 상태 설정
    updatePortStatus('disconnected');
    
    // 초기 상태 라벨 설정
    const statusLabel = document.getElementById('status-label');
    if (statusLabel) statusLabel.innerHTML = 'Spindle: - RPM <br> Feed Rate: - mm/min';
    
    initLogControls();
    initSensorLogControls();
    initToolInfo();
    
    // 이벤트 리스너 초기화는 다른 초기화 후에 실행
    setTimeout(() => {
        initEventListeners();
    }, 500);
    
    // 포트 목록 가져오기
    fetchPorts();
    
    // 센서 포트 목록 가져오기
    fetchSensorPorts();

    // 진동 감지 모델 학습 상태와 데이터 수집 기록 상태 표시
    refreshVibrationModelStatus();
    refreshRecordingStatus(false);

    // 센서 로그 초기화
    addSensorLog('센서 데이터 로깅 시작됨', 'info');
}

// 센서 로그 컨트롤 초기화
function initSensorLogControls() {
    const clearLogBtn = document.getElementById('clear-sensor-log-btn');
    const autoScrollCheckbox = document.getElementById('auto-scroll-sensor-log');
    const saveDataBtn = document.getElementById('save-sensor-data-btn');

    if (clearLogBtn) {
        clearLogBtn.addEventListener('click', () => {
            if (sensorLogText) {
                sensorLogText.innerHTML = '';
                addSensorLog('로그가 초기화되었습니다.', 'info');
            }
        });
    }

    if (autoScrollCheckbox) {
        autoScrollCheckbox.addEventListener('change', (e) => {
            sensorLogAutoScroll = e.target.checked;
        });
    }

    if (saveDataBtn) {
        saveDataBtn.addEventListener('click', saveSensorDataToCSV);
    }
}

// 차트 초기화
function initCharts() {
    try {
        // Chart.js 디바이스 픽셀 비율 설정 - 고화질 차트를 위해
        Chart.defaults.devicePixelRatio = Math.max(window.devicePixelRatio || 1, 2);
        
        // 차트에 원점 라인을 표시하는 플러그인 설정
        const zeroLinePlugin = {
            id: 'zeroLine',
            beforeDraw: function(chart, args, options) {
                if (!options.display) return;
                
                const ctx = chart.ctx;
                const yAxis = chart.scales.y;
                
                if (!yAxis) return;
                
                const yPixel = yAxis.getPixelForValue(0);
                const chartArea = chart.chartArea;
                
                // 원점(0) 위치에 강조선 그리기
                ctx.save();
                ctx.beginPath();
                ctx.moveTo(chartArea.left, yPixel);
                ctx.lineTo(chartArea.right, yPixel);
                ctx.lineWidth = options.lineWidth || 2;
                ctx.strokeStyle = options.color || 'rgba(255, 255, 255, 0.5)';
                ctx.setLineDash(options.dash || [5, 5]);
                ctx.stroke();
                ctx.restore();
            }
        };
        
        // 메인 차트 (XY 평면)
        const mainCtx = document.getElementById('main-chart').getContext('2d');
        mainChart = new Chart(mainCtx, {
            type: 'scatter',
            data: {
                datasets: [
                    {
                        // 경로 선
                        label: '가공 경로',
                        data: [{x: 0, y: 0}],
                        showLine: true,
                        backgroundColor: 'rgba(0, 173, 181, 0.1)',
                        borderColor: 'rgba(0, 173, 181, 1)',
                        borderWidth: 2,
                        pointRadius: 0,
                        fill: false,
                        tension: 0
                    },
                    {
                        // 현재 위치 점
                        label: '현재 위치',
                        data: [{x: 0, y: 0}],
                        backgroundColor: 'rgba(255, 82, 82, 1)',
                        borderColor: 'rgba(255, 255, 255, 0.8)',
                        borderWidth: 2,
                        pointRadius: 5,
                        pointHoverRadius: 7
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                devicePixelRatio: Math.max(window.devicePixelRatio || 1, 2), // 고화질
                scales: {
                    x: {
                        type: 'linear',
                        position: 'bottom',
                        title: {
                            display: true,
                            text: 'X (mm)',
                            color: '#EEEEEE'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#EEEEEE'
                        }
                    },
                    y: {
                        title: {
                            display: true,
                            text: 'Y (mm)',
                            color: '#EEEEEE'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#EEEEEE'
                        }
                    }
                },
                plugins: {
                    legend: {
                        labels: {
                            color: '#EEEEEE'
                        }
                    }
                }
            }
        });
        
        // Z 차트 (Z 높이 그래프)
        const zCtx = document.getElementById('z-chart').getContext('2d');
        zChart = new Chart(zCtx, {
            type: 'line',
            data: {
                labels: [0],
                datasets: [
                    {
                        label: 'Z 경로',
                        data: [0],
                        backgroundColor: 'rgba(0, 173, 181, 0.2)',
                        borderColor: 'rgba(0, 173, 181, 1)',
                        borderWidth: 2,
                        pointRadius: 0,
                        pointHoverRadius: 5,
                        fill: true,
                        tension: 0.1
                    },
                    {
                        label: '현재 위치',
                        data: [0],
                        backgroundColor: 'rgba(255, 82, 82, 1)',
                        borderColor: 'rgba(255, 255, 255, 0.8)',
                        borderWidth: 2,
                        pointRadius: 5,
                        pointHoverRadius: 7,
                        fill: false
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                devicePixelRatio: Math.max(window.devicePixelRatio || 1, 2), // 고화질
                scales: {
                    x: {
                        title: {
                            display: true,
                            text: '가공 단계',
                            color: '#EEEEEE'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#EEEEEE'
                        }
                    },
                    y: {
                        title: {
                            display: true,
                            text: 'Z (mm)',
                            color: '#EEEEEE'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)',
                            drawBorder: true,
                            lineWidth: 1
                        },
                        ticks: {
                            color: '#EEEEEE'
                        },
                        // 원점을 가운데로 설정하기 위한 시작점
                        beginAtZero: false
                    }
                },
                plugins: {
                    legend: {
                        labels: {
                            color: '#EEEEEE'
                        }
                    },
                    // 원점 라인 표시 플러그인 설정
                    zeroLine: {
                        display: true,
                        lineWidth: 2,
                        color: 'rgba(255, 255, 255, 0.8)',
                        dash: [0, 0] // 실선
                    }
                }
            },
            plugins: [zeroLinePlugin] // 플러그인 추가
        });

        console.log("차트 초기화 성공");
        // 차트 크기 초기 설정
        resizeCharts();
    } catch (error) {
        console.error("차트 초기화 오류:", error);
        addLog(`차트 초기화 중 오류가 발생했습니다: ${error.message}`, 'error');
    }
}

// 배열의 최솟값/최댓값 (Math.min(...arr)는 점이 많으면 호출 스택을 넘어 오류가 나므로 직접 계산)
function minMax(values, extra) {
    let min = Infinity;
    let max = -Infinity;
    for (const v of values) {
        if (v < min) min = v;
        if (v > max) max = v;
    }
    if (extra !== undefined && extra !== null) {
        min = Math.min(min, extra);
        max = Math.max(max, extra);
    }
    return [min, max];
}

// 서버가 보낸 경로 조각을 로컬 경로 배열에 반영 (path_start: 조각이 시작하는 인덱스, 0이면 전체 경로)
function mergePathUpdate(data) {
    if (!data.path_x) return true;
    const start = data.path_start || 0;
    if (start > xVals.length) {
        // 중간 조각을 놓친 경우 전체 경로를 다시 요청
        socket.emit('request_status');
        return false;
    }
    xVals.length = start;
    yVals.length = start;
    zVals.length = start;
    timeVals.length = Math.min(timeVals.length, start);
    for (let i = 0; i < data.path_x.length; i++) {
        xVals.push(data.path_x[i]);
        yVals.push(data.path_y[i]);
        zVals.push(data.path_z[i]);
        timeVals.push(totalSeconds);
    }
    return true;
}

// 차트 업데이트 (경로는 전역 xVals/yVals/zVals, 현재 위치는 data.x/y/z)
function updateCharts(data) {
    if (!mainChart || !zChart) return;

    // 전체 경로 데이터 표시
    const pathData = xVals.map((x, i) => ({x: x, y: yVals[i]}));
    mainChart.data.datasets[0].data = pathData;
    mainChart.data.datasets[1].data = [{x: data.x, y: data.y}];

    // Z 차트 업데이트
    const zLabels = Array.from({length: zVals.length}, (_, i) => i);
    zChart.data.labels = zLabels;
    zChart.data.datasets[0].data = zVals.slice();

    // 현재 Z 위치를 데이터셋에 업데이트
    // 현재 위치를 포인트로 표시하기 위해 가장 마지막 인덱스에 표시
    const currentZIndex = zVals.length - 1;
    zChart.data.datasets[1].data = Array(zVals.length).fill(null);
    if (currentZIndex >= 0) {
        zChart.data.datasets[1].data[currentZIndex] = data.z;
    }

    // Z 축 스케일 동적 조정
    if (zVals.length > 1) {
        const [minZ, maxZ] = minMax(zVals, data.z);

        // 최대 절대값 계산 (양수/음수 중 절대값이 더 큰 것)
        const absMaxZ = Math.max(Math.abs(minZ), Math.abs(maxZ));
        const zRange = Math.max(absMaxZ, 5); // 최소 5mm 범위 보장
        
        // 최대값과 최소값의 비율이 동일하게 설정
        const zPadding = zRange * 0.1; // 10% 패딩
        zChart.options.scales.y.min = -(zRange + zPadding);
        zChart.options.scales.y.max = zRange + zPadding;
    }
    
    // 동적 축 범위 설정 (material size 없을 때)
    if (!materialWidth || !materialHeight) {
        const [minX, maxX] = minMax(xVals);
        const [minY, maxY] = minMax(yVals);
        const padX = (maxX - minX) * 0.1 || 10;
        const padY = (maxY - minY) * 0.1 || 10;
        mainChart.options.scales.x.min = minX - padX;
        mainChart.options.scales.x.max = maxX + padX;
        mainChart.options.scales.y.min = minY - padY;
        mainChart.options.scales.y.max = maxY + padY;
    }
    // 차트 업데이트 (실시간 갱신이므로 애니메이션 없이)
    mainChart.update('none');
    zChart.update('none');
}

// 차트 데이터 초기화
function resetChartData() {
    // 가공 경로 초기화
    xVals = [0];
    yVals = [0];
    zVals = [0];
    timeVals = [0];
    
    // 차트 데이터 초기화
    mainChart.data.datasets[0].data = [{x: 0, y: 0}];
    mainChart.data.datasets[1].data = [{x: 0, y: 0}];
    zChart.data.labels = [0];
    zChart.data.datasets[0].data = [0];
    zChart.data.datasets[1].data = [0];
    
    // 차트 스케일 초기화
    mainChart.options.scales.x.min = -100;
    mainChart.options.scales.x.max = 100;
    mainChart.options.scales.y.min = -100;
    mainChart.options.scales.y.max = 100;
    
    // Z축 차트 스케일 초기화 - 비율 동일하게 설정
    zChart.options.scales.y.min = -30;
    zChart.options.scales.y.max = 30;
    
    // 차트 업데이트
    mainChart.update();
    zChart.update();
    
    // 좌표 라벨 초기화
    document.getElementById('coordinate-label').textContent = 
        '현재 위치: X: 0.00, Y: 0.00, Z: 0.00';
    
    // 진행률 초기화
    document.getElementById('progress-bar').style.width = '0%';
    document.getElementById('progress-text').textContent = '0%';
    
    // 시간 정보 업데이트
    updateTimeInfo();
}

// 차트 크기 조절
function resizeCharts() {
    if (!mainChart || !zChart) {
        console.warn("차트가 초기화되지 않았습니다.");
        return;
    }
    
    // 차트 컨테이너 크기에 맞춰 차트 크기를 동적으로 조정
    const mainContainer = document.querySelector('.graph-container');
    const zContainer = document.querySelector('.mini-graph-container');
    
    // XY 차트 범위 설정
    if (materialWidth && materialHeight) {
        const padding = 0.1;
        const maxDimension = Math.max(materialWidth, materialHeight);
        const paddedHalfSize = (maxDimension / 2) * (1 + padding);

        mainChart.options.scales.x.min = -paddedHalfSize;
        mainChart.options.scales.x.max = paddedHalfSize;
        mainChart.options.scales.y.min = -paddedHalfSize;
        mainChart.options.scales.y.max = paddedHalfSize;
    } else {
        // 기존 데이터 범위에 따라 동적 조정
        if (xVals.length > 1) {
            const [minX, maxX] = minMax(xVals);
            const [minY, maxY] = minMax(yVals);
            const padX = Math.max((maxX - minX) * 0.1, 10);
            const padY = Math.max((maxY - minY) * 0.1, 10);
            
            mainChart.options.scales.x.min = minX - padX;
            mainChart.options.scales.x.max = maxX + padX;
            mainChart.options.scales.y.min = minY - padY;
            mainChart.options.scales.y.max = maxY + padY;
        } else {
            mainChart.options.scales.x.min = -100;
            mainChart.options.scales.x.max = 100;
            mainChart.options.scales.y.min = -100;
            mainChart.options.scales.y.max = 100;
        }
    }
    
    // Z 차트 범위 설정
    if (materialDepth) {
        // 원점(0)을 기준으로 위/아래 범위 균등하게 설정
        const maxAbsZ = Math.max(materialDepth, 5); // 최소 5mm 표시
        zChart.options.scales.y.min = -maxAbsZ;
        zChart.options.scales.y.max = maxAbsZ; // 최대값과 최소값 비율 동일하게 설정
    } else if (zVals.length > 1) {
        const [minZ, maxZ] = minMax(zVals);
        
        // 최대 절대값 계산 (양수/음수 중 절대값이 더 큰 것)
        const absMaxZ = Math.max(Math.abs(minZ), Math.abs(maxZ));
        const zRange = Math.max(absMaxZ, 5); // 최소 5mm 범위 보장
        
        // 최대값과 최소값의 비율이 동일하게 설정
        zChart.options.scales.y.min = -zRange;
        zChart.options.scales.y.max = zRange;
        
        // 워크 영역이 보이도록 약간의 패딩 추가 (동일한 비율로)
        const zPadding = zRange * 0.1; // 10% 패딩
        zChart.options.scales.y.min -= zPadding;
        zChart.options.scales.y.max += zPadding;
    } else {
        // 기본값 (데이터가 없을 때) - 비율 동일하게 설정
        zChart.options.scales.y.min = -30;
        zChart.options.scales.y.max = 30;
    }
    
    // 가로선 간격 조정 (원점 강조)
    zChart.options.scales.y.grid.z = 1; // 그리드를 위에 표시
    
    // 차트 업데이트
    mainChart.update();
    zChart.update();
}

// 로그 컨트롤 초기화
function initLogControls() {
    const clearLogBtn = document.getElementById('clear-log-btn');
    const autoScrollCheckbox = document.getElementById('auto-scroll-log');
    
    // 로그 지우기 버튼
    clearLogBtn.addEventListener('click', function() {
        const logText = document.getElementById('log-text');
        logText.innerHTML = '';
        addLog('로그를 지웠습니다.', 'info');
    });
    
    // 자동 스크롤 설정 변경되면 localStorage에 저장
    autoScrollCheckbox.addEventListener('change', function() {
        localStorage.setItem('autoScrollLog', this.checked);
    });
    
    // localStorage에서 자동 스크롤 설정 불러오기
    const savedAutoScroll = localStorage.getItem('autoScrollLog');
    if (savedAutoScroll !== null) {
        autoScrollCheckbox.checked = savedAutoScroll === 'true';
    }
}

// 로그 메시지 추가 함수
function addLog(message, type = '') {
    const logText = document.getElementById('log-text');
    if (!logText) return;
    
    // 센서 로그인 경우, 일반 로그에 표시하지 않음
    const isSensorLog = type === 'sensor' || message.startsWith('[센서]') || 
                        message.includes('센서') || message.includes('진동:') || 
                        message.includes('온도:') || message.includes('아두이노');
    
    if (isSensorLog) {
        // 센서 로그는 센서 로그 영역으로 보내기
        addSensorLog(message, type);
        return;
    }
    
    const logLine = document.createElement('div');
    
    // 시간 추가
    const now = new Date();
    const timeStr = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
    
    // 메시지 너무 길면 축소
    if (message.length > 150) {
        const shortMessage = message.substring(0, 147) + '...';
        logLine.textContent = `${timeStr} | ${shortMessage}`;
        logLine.title = message;  // 전체 메시지는 툴팁으로 표시
        logLine.style.cursor = 'pointer';
        
        // 클릭하면 전체/축소 전환
        let expanded = false;
        logLine.addEventListener('click', function() {
            expanded = !expanded;
            logLine.textContent = expanded ? `${timeStr} | ${message}` : `${timeStr} | ${shortMessage}`;
        });
    } else {
        logLine.textContent = `${timeStr} | ${message}`;
    }
    
    // 로그 타입에 따른 스타일 적용
    if (type) {
        logLine.classList.add(`log-${type}`);
    }
    
    logText.appendChild(logLine);
    
    // 자동 스크롤이 활성화된 경우에만 스크롤
    const autoScroll = document.getElementById('auto-scroll-log').checked;
    if (autoScroll) {
        logText.scrollTop = logText.scrollHeight;
    }
    
    // 로그가 1000줄을 넘으면 오래된 로그 삭제 (메모리 관리)
    if (logText.children.length > 1000) {
        logText.removeChild(logText.firstChild);
    }
}

// GRBL 콘솔 명령 전송
document.getElementById('grbl-command-send').onclick = function() {
    const cmd = document.getElementById('grbl-command-input').value.trim();
    if (cmd) {
        socket.emit('grbl_command', { command: cmd });
        document.getElementById('grbl-command-input').value = '';
    }
};
document.getElementById('grbl-command-input').addEventListener('keydown', function(e) {
    if (e.key === 'Enter') {
        document.getElementById('grbl-command-send').click();
    }
});

// 포트 상태 업데이트
function updatePortStatus(status, portName = '') {
    const statusElem = document.getElementById('port-status');
    
    // 기존 클래스 제거
    statusElem.classList.remove('connected', 'disconnected', 'connecting');
    
    // 포트 상태에 따른 처리
    switch(status) {
        case 'disconnected':
            statusElem.textContent = '포트 상태: 연결되지 않음';
            statusElem.classList.add('disconnected');
            break;
        case 'connecting':
            statusElem.textContent = `포트 상태: ${portName} 연결 중...`;
            statusElem.classList.add('connecting');
            break;
        case 'connected':
            statusElem.textContent = `포트 상태: ${portName} 연결됨`;
            statusElem.classList.add('connected');
            break;
        default:
            statusElem.textContent = '컨트롤러 상태: 알 수 없음';
    }
}

// 소켓 이벤트 리스너 초기화
function initSocketListeners() {
    try {
        // 이전 소켓 연결이 있으면 정리
        if (socket) {
            try {
                socket.disconnect();
                socket.off();
            } catch (e) {
                console.warn("이전 소켓 연결 정리 중 오류:", e);
            }
        }
        
        // 소켓 연결 옵션 설정 - 안정적인 연결을 위한 설정 강화
        const socketOptions = {
            reconnection: true,
            reconnectionAttempts: 5,
            reconnectionDelay: 1000,
            reconnectionDelayMax: 5000,
            timeout: 15000,             // 타임아웃 증가
            transports: ['websocket', 'polling'],
            forceNew: true,             // 새로운 연결 강제
            autoConnect: true           // 자동 연결 (웹소켓 연결에만 적용, 포트 연결 관련 없음)
        };
        
        // 서버에 연결
        console.log("소켓 연결 시도...", window.location.origin);
        socket = io(window.location.origin, socketOptions);
        
        // 소켓 연결 상태 추적 변수
        let isConnected = false;
        
        // 서버 연결 이벤트
        socket.on('connect', function() {
            console.log('서버에 연결되었습니다');
            isConnected = true;
            addLog('서버에 연결되었습니다.', 'success');
            socket.emit('request_status')
            // 페이지 로드 후, 또는 재연결 이후 포트 목록 갱신
            fetchPorts();
            // 센서 포트 목록도 갱신
            fetchSensorPorts();
        });
        
        // 연결 끊김 이벤트
        socket.on('disconnect', function(reason) {
            console.log('서버 연결 끊김:', reason);
            isConnected = false;
            addLog(`서버 연결이 끊어졌습니다: ${reason}`, 'error');
        });
        
        // 소켓 오류 이벤트
        socket.on('error', function(error) {
            console.error('소켓 오류:', error);
            addLog(`소켓 통신 오류: ${error}`, 'error');
        });
        
        // 로그 업데이트 수신
        socket.on('log_update', function(data) {
            // 이동, 속도, 예상 시간 관련 메시지는 표시하지 않음
            if (data.message.includes('이동:') && data.message.includes('속도:')) {
                return;
            }
            
            // 가공 진행률 메시지도 표시하지 않음
            if (data.message.includes('가공 진행률:')) {
                return;
            }
            
            // 센서 데이터 로그 특별 처리
            const isSensorLog = data.message.startsWith('[센서]') || data.color === 'sensor' || 
                                data.message.includes('센서') || data.message.includes('진동:') || 
                                data.message.includes('온도:') || data.message.includes('아두이노');
            
            // 로그 타입 결정
            let logType = '';
            
            if (data.color === 'red') {
                logType = 'error';
            } else if (data.color === 'orange') {
                logType = 'warning';
            } else if (data.color === 'green') {
                logType = 'success';
            } else if (data.color === 'sensor' || isSensorLog) {
                logType = 'sensor'; // 센서 로그에 특별한 클래스 추가
            } else if (data.message.includes('응답 없음') || data.message.includes('통신 시간 초과')) {
                logType = 'warning';
            } else if (data.message.includes('성공') || data.message.includes('연결됨')) {
                logType = 'success';
            } else if (data.message.includes('오류') || data.message.includes('실패')) {
                logType = 'error';
            } else if (data.message.includes('시도')) {
                logType = 'info';
            }
            
            // 센서 관련 로그는 센서 로그에 추가하고, 나머지는 일반 로그에 추가
            if (isSensorLog) {
                // 센서 로그는 항상 센서 로그 영역에 표시 (더 이상 체크박스로 제어하지 않음)
                addSensorLog(data.message, logType);
            } else {
                // 일반 로그 추가
                addLog(data.message, logType);
            }
        });
        
        // 디버그 메시지 수신
        socket.on('debug_message', function(data) {
            console.log('서버 디버그 메시지:', data.message);
            
            // 디버그 메시지를 일반 로그에 추가
            addLog('[디버그] ' + data.message, 'debug');
        });
        
        // 재연결 시도 이벤트
        socket.on('reconnect_attempt', function(attemptNumber) {
            console.log(`재연결 시도 #${attemptNumber}`);
            showNotification(`서버 연결 재시도 중... (${attemptNumber}/5)`, 'info');
        });
        
        // 재연결 성공 이벤트
        socket.on('reconnect', function(attemptNumber) {
            console.log(`${attemptNumber}번 시도 후 서버에 재연결됨`);
            showNotification('서버에 다시 연결되었습니다', 'success');
        });
        
        // 재연결 실패 이벤트
        socket.on('reconnect_failed', function() {
            console.log('모든 재연결 시도 실패');
            showNotification('서버와의 연결이 끊어졌습니다. 페이지를 새로고침하세요.', 'error');
        });
        
        // 위치 업데이트 수신
        socket.on('position_update', function(data) {
            // 좌표 업데이트
            document.getElementById('coordinate-label').textContent = 
                `현재 위치: X: ${data.x.toFixed(2)}, Y: ${data.y.toFixed(2)}, Z: ${data.z.toFixed(2)}`;
            
            // 상태 업데이트
            document.getElementById('status-label').innerHTML = 
                `Spindle: ${data.spindle || '-'} RPM <br> Feed Rate: ${data.feedrate || '-'} mm/min`;
            
            // 진행률 바 업데이트
            document.getElementById('progress-bar').style.width = `${data.progress}%`;
            document.getElementById('progress-text').textContent = `${data.progress}%`;
            
            // 경과 시간 처리 - 서버에서 보낸 정확한 경과 시간 사용
            if (data.elapsed_time) {
                // 서버의 경과 시간이 있는 경우 이를 사용
                totalSeconds = Math.floor(data.elapsed_time);
                updateTimeInfo();
            } else if (processingStartTime) {
                // 서버 시간이 없는 경우 클라이언트 시간 계산 사용 (기존 방식)
                const now = new Date();
                totalSeconds = Math.floor((now - processingStartTime) / 1000);
                updateTimeInfo();
            }
            
            // 경로 데이터 업데이트 (서버는 새로 추가된 점만 보내므로 기존 경로에 이어 붙임)
            if (mergePathUpdate(data)) {
                // 차트 업데이트
                updateCharts(data);
            }
        });
        
        // 데이터 수신 이벤트
        socket.on('data', function(data) {
            // 데이터 처리 및 차트 업데이트
            handleReceivedData(data);
        });
        
        // 알람 상태 업데이트 수신 (서버가 보낸 문구와 색상을 그대로 표시)
        // updateAlarmStatus()는 {alarm, state} 형식용이라 {text, color}를 넘기면 알람 중에도 "알람 없음"으로 표시됨
        socket.on('alarm_update', function(data) {
            const alarmLabel = document.getElementById('alarm-label');
            if (alarmLabel && data && data.text) {
                alarmLabel.textContent = data.text;
                alarmLabel.style.color = data.color || '';
            }
        });

        // 서버 가공 상태 수신 (일시정지/시작 버튼 판단의 기준)
        socket.on('state_update', function(data) {
            if (!data || !data.state) return;
            currentProcessingState = data.state;
            if (data.state !== 'running') {
                stopProcessingTimer();
            }
            if (data.state !== 'running' && data.state !== 'stopping') {
                renderVibrationLive(null);
            }
        });

        // 실시간 진동 지표와 데이터 수집 상태
        socket.on('vibration_status', function(data) {
            renderVibrationLive(data);
        });
        socket.on('recording_status', function(data) {
            renderRecordingStatus(data);
        });

        // 허용 공차 수신 (입력창을 서버 값과 일치)
        socket.on('tolerance_update', function(data) {
            if (!data || typeof data.tolerance !== 'number') return;
            toleranceValue = data.tolerance;
            const toleranceInput = document.getElementById('tolerance');
            if (toleranceInput && document.activeElement !== toleranceInput) {
                toleranceInput.value = data.tolerance;
            }
        });

        // 오버라이드 비율 수신 (GRBL 상태 보고 기준)
        socket.on('override_status', function(data) {
            if (!data) return;
            const spindleLabel = document.getElementById('spindle-override-value');
            if (spindleLabel && data.spindle !== undefined) {
                spindleLabel.textContent = `${data.spindle}%`;
            }
            const feedLabel = document.getElementById('feed-override-value');
            const feedSlider = document.getElementById('feed-override');
            if (data.feed !== undefined) {
                if (feedLabel) feedLabel.textContent = `${data.feed}%`;
                if (feedSlider && document.activeElement !== feedSlider) feedSlider.value = data.feed;
            }
        });
        
        // 상태 업데이트 수신
        socket.on('status_update', function(data) {

            // 상태 플래그 갱신
            if (data.text.includes('가공 재개') || data.text.includes('가공 시작')) {
                currentProcessingState = 'running';
            } else if (data.text.includes('일시정지')) {
                currentProcessingState = 'paused';
            } else if (data.text.includes('긴급정지')) {
                currentProcessingState = 'emergency';
            } else if (data.text.includes('대기') || data.text.includes('가공 완료')) {
                currentProcessingState = 'idle';
            }
            
            // 가공 완료 확인
            if (data.text.includes('가공 완료')) {
                handleProcessingComplete();
            }
        });
        
        // 연결 상태 이벤트
        socket.on('connection_status', function(data) {
            console.log('연결 상태 변경:', data);
            updateConnectionStatus(data);
        });
        
        // 포트 목록 업데이트 이벤트 (새로운 이벤트)
        socket.on('ports_updated', function(data) {
            console.log('포트 목록 자동 업데이트:', data);
            
            if (data && data.ports) {
                updatePortSelect(data.ports);
                addLog('포트 목록이 자동으로 업데이트되었습니다.', 'info');
            }
        });
        
        // 아두이노 센서 상태 업데이트
        socket.on('arduino_status', function(data) {
            console.log('아두이노 센서 상태 업데이트:', data);
            updateArduinoStatus(data.connected, data.port);
            
            // 연결 상태에 따라 UI 업데이트
            const connectBtn = document.getElementById('connect-sensor-btn');
            const disconnectBtn = document.getElementById('disconnect-sensor-btn');
            const portSelect = document.getElementById('sensor-port-select');
            
            if (data.connected) {
                // 연결됨
                if (connectBtn) connectBtn.style.display = 'none';
                if (disconnectBtn) {
                    disconnectBtn.style.display = 'inline-block';
                    disconnectBtn.disabled = false;
                }
                if (portSelect) portSelect.disabled = true;
            } else {
                // 연결 해제됨
                if (connectBtn) {
                    connectBtn.style.display = 'inline-block';
                    connectBtn.disabled = false;
                    connectBtn.textContent = '연결';
                }
                if (disconnectBtn) disconnectBtn.style.display = 'none';
                if (portSelect) portSelect.disabled = false;
            }
        });
        
        // 아두이노 센서 데이터 업데이트 (약 100Hz로 들어오므로 콘솔 출력은 하지 않음)
        socket.on('arduino_data', function(data) {
            handleArduinoSensorData(data.data);
        });
        
        // 서버 연결 확인 및 재연결 메커니즘
        let connectionCheckInterval;
        function startConnectionCheck() {
            // 이전 인터벌 정리
            if (connectionCheckInterval) {
                clearInterval(connectionCheckInterval);
            }
            
            // 30초마다 서버 연결 확인
            connectionCheckInterval = setInterval(() => {
                if (socket && !isConnected) {
                    console.log("서버 연결 끊김 감지, 재연결 시도...");
                    try {
                        socket.connect();
                    } catch (e) {
                        console.error("재연결 시도 중 오류:", e);
                    }
                }
            }, 30000);
        }
        
        // 연결 확인 시작
        startConnectionCheck();
        
        // NC 코드 파일 업로드 비활성화 이벤트
        socket.on('disable_nc_upload', function() {
            // Removed
        });
        
        // NC 코드 파일 업로드 활성화 이벤트
        socket.on('enable_nc_upload', function() {
            // Removed
        });
        
        // 이상 진동 알림 처리
        socket.on('vibration_alert', function(data) {
            console.log("vibration_alert 이벤트 수신:", data); // 추가된 로그
            const alertPopup = document.getElementById('vibration-alert');
            
            if (data.show) {
                if (alertPopup) {
                    const detail = alertPopup.querySelector ? alertPopup.querySelector('p') : null;
                    if (detail && data.message) detail.textContent = data.message;
                    alertPopup.classList.add('show');
                    // 3초 후 자동으로 알림 닫기
                    setTimeout(() => {
                        alertPopup.classList.remove('show');
                    }, 3000);
                } else {
                    console.error("vibration-alert 엘리먼트를 찾을 수 없습니다.");
                }
            } else {
                if (alertPopup) {
                    alertPopup.classList.remove('show');
                } else {
                    console.error("vibration-alert 엘리먼트를 찾을 수 없습니다.");
                }
            }
        });
        
    } catch (error) {
        console.error("소켓 초기화 오류:", error);
        addLog(`소켓 연결 중 오류가 발생했습니다: ${error.message}`, 'error');
    }
}

// 포트 목록 가져오기
function fetchPorts() {
    console.log("사용 가능한 포트 목록 요청 중...");
    
    addLog('사용 가능한 포트 목록을 검색 중입니다...', 'info');
    
    // 소켓 연결 상태 확인
    if (!socket || !socket.connected) {
        addLog('서버 연결이 없어 포트 목록을 가져올 수 없습니다.', 'error');
        return;
    }
    
    // 포트 목록 요청
    socket.emit('get_ports', {}, function(response) {
        if (!response || !response.hasOwnProperty('ports')) {
            addLog('포트 검색 중 오류: 서버 응답 형식이 잘못되었습니다.', 'error');
            return;
        }
        
        // 포트 목록 업데이트
        updatePortSelect(response.ports);
    });
}

// 포트 선택 드롭다운 업데이트
function updatePortSelect(ports) {
    const portSelect = document.getElementById('port-select');
    const selectedPort = portSelect.value;
    
    // 포트 목록 초기화
    portSelect.innerHTML = '';
    
    if (ports && ports.length > 0) {
        // 이전에 연결된 포트
        const lastConnectedPort = localStorage.getItem('lastConnectedPort');
        
        // 포트 옵션 추가
        ports.forEach(port => {
            const option = document.createElement('option');
            option.value = port;
            option.textContent = port;
            
            // 선택 우선순위: 이전 선택 > 마지막 연결 > 첫 번째 포트
            if (port === selectedPort || 
                (!selectedPort && port === lastConnectedPort) || 
                (!selectedPort && !lastConnectedPort && port === ports[0])) {
                option.selected = true;
            }
            
            portSelect.appendChild(option);
        });
        
        // 연결 버튼 활성화
        document.getElementById('connect-btn').disabled = false;
        addLog(`사용 가능한 포트 ${ports.length}개를 찾았습니다: ${ports.join(', ')}`, 'success');
    } else {
        // 포트가 없는 경우
        const option = document.createElement('option');
        option.disabled = true;
        option.textContent = '사용 가능한 포트가 없습니다';
        portSelect.appendChild(option);
        
        // 연결 버튼 비활성화
        document.getElementById('connect-btn').disabled = true;
        addLog('사용 가능한 포트가 없습니다. 장치 연결 상태를 확인하세요.', 'warning');
    }
}

// 포트 연결 요청
function connectPort() {
    const port = document.getElementById('port-select').value;
    if (!port) {
        showNotification('포트를 선택해주세요', 'error');
        return;
    }
    
    // 연결 버튼 상태 변경
    const connectBtn = document.getElementById('connect-btn');
    connectBtn.textContent = '연결 중...';
    connectBtn.disabled = true;
    
    // 포트 상태 표시
    updatePortStatus('connecting', port);
    addLog(`컨트롤러 ${port} 연결 시도 중...`);
    
    // 알람 상태 초기화
    const alarmLabel = document.getElementById('alarm-label');
    if (alarmLabel) {
        alarmLabel.textContent = "연결 중...";
        alarmLabel.style.color = 'orange';
    }
    
    // 연결 요청 - 최대 10번의 시도를 통한 연결성 확인 요청
    socket.emit('connect_port', {
        port: port,
        attempts: 10 // 10번의 시도 요청
    }, function(response) {
        if (response && response.status === 'success') {
            showNotification(response.message, 'success');
            
            // 시작 버튼 및 알람 해제 버튼 초기 상태 설정
            const startBtn = document.getElementById('start-btn');
            const unlockBtn = document.getElementById('unlock-btn');
            
            // 알람 상태 확인 및 UI 업데이트
            if (alarmLabel) {
                if (response.alarm_status === 'alarm') {
                    // 알람 상태 - 알람 해제 버튼 활성화, 시작 버튼 비활성화
                    alarmLabel.textContent = "알람 상태: 해제가 필요합니다";
                    alarmLabel.style.color = 'red';
                    addLog("컨트롤러가 알람 상태입니다. 알람 해제가 필요합니다.", 'warning');
                    
                } else if (response.alarm_status === 'none') {
                    // 알람 없음 - 알람 해제 없이 바로 가공 가능 상태
                    updateAlarmStatus({alarm: false, state: 'alarm_unlocked'});
                    addLog("컨트롤러가 정상 상태입니다. 바로 가공 시작 가능합니다.", 'success');
                    
                } else {
                    // 상태 알 수 없음 - 추가 확인 필요
                    alarmLabel.textContent = "알람 상태 확인 중...";
                    alarmLabel.style.color = 'orange';
                    
                    // 추가 알람 상태 확인을 위한 명시적 요청
                    socket.emit('check_alarm_status', {}, function(alarmResponse) {
                        if (alarmResponse && alarmResponse.status === 'success') {
                            updateAlarmStatus(alarmResponse);
                        }
                    });
                }
            }
            
            // 연결 상태 저장
            controllerConnected = true;
            localStorage.setItem('lastConnectedPort', port);
        } else {
            // 연결 실패 처리
            const errorMsg = response ? response.message : '서버로부터 응답이 없습니다';
            showNotification(errorMsg, 'error');
            updatePortStatus('disconnected');
            addLog(`컨트롤러 ${port} 연결 실패: ${errorMsg}`, 'error');
            
            // 알람 상태 초기화
            if (alarmLabel) {
                alarmLabel.textContent = "연결 실패";
                alarmLabel.style.color = 'red';
            }
            
            // 연결 상태 변수 초기화
            controllerConnected = false;

            // 버튼 상태 복원
            connectBtn.textContent = '연결';
            connectBtn.disabled = false;
        }
    });
}

// 연결 상태 업데이트
function updateConnectionStatus(data) {
    // 데이터 유효성 확인
    if (!data || !('connected' in data)) {
        return;
    }
    
    const connectBtn = document.getElementById('connect-btn');
    const disconnectBtn = document.getElementById('disconnect-btn');
    const portSelect = document.getElementById('port-select');
    const alarmLabel = document.getElementById('alarm-label');
    controllerConnected = !!data.connected;

    if (data.connected) {
        // 연결 상태 처리
        updatePortStatus('connected', data.port);
        
        // 알람 상태 처리
        if (data.warning) {
            alarmLabel.textContent = "알람 상태 확인중";
            alarmLabel.style.color = 'orange';
            addLog("컨트롤러가 응답하지 않습니다. 연결을 다시 확인하세요.", 'warning');
        } else {
            addLog(`컨트롤러 ${data.port} 연결 성공!`, 'success');
            socket.emit('check_alarm_status', {}, function(response) {
                if (response.status === 'success') {
                    alarmLabel.textContent = response.alarm ? "알람 상태: 해제가 필요합니다" : "알람 없음";
                    alarmLabel.style.color = response.alarm ? 'red' : 'green';
                } else {
                    alarmLabel.textContent = "알람 상태 확인중";
                    alarmLabel.style.color = 'orange';
                }
            });
        }
        
        // UI 업데이트
        connectBtn.style.display = 'none';
        disconnectBtn.style.display = 'inline-block';
        disconnectBtn.disabled = false;
        portSelect.disabled = true;
        
        // 상태 저장
        try {
            localStorage.setItem('lastConnectedPort', data.port);
            localStorage.setItem('connectionTimestamp', Date.now().toString());
        } catch (e) {}
    } else {
        // 연결 해제 상태 처리
        updatePortStatus('disconnected');
        alarmLabel.textContent = "연결 해제됨";
        alarmLabel.style.color = 'blue';
        addLog('컨트롤러 연결이 해제되었습니다.', 'info');
        
        // UI 업데이트
        connectBtn.style.display = 'inline-block';
        connectBtn.disabled = false;
        connectBtn.textContent = '연결';
        disconnectBtn.style.display = 'none';
        portSelect.disabled = false;
        
        // 저장된 상태 제거
        try {
            localStorage.removeItem('lastConnectedPort');
            localStorage.removeItem('connectionTimestamp');
        } catch (e) {}
        
        // 연결 해제 후 포트 목록 갱신
        fetchPorts();
    }
}

// 이벤트 리스너 초기화
function initEventListeners() {
    console.log("이벤트 리스너 초기화 시작");

    // Override 컨트롤 이벤트 리스너
    const spindleOverride = document.getElementById('spindle-override');
    const feedOverride = document.getElementById('feed-override');
    const spindleOverrideValue = document.getElementById('spindle-override-value');
    const feedOverrideValue = document.getElementById('feed-override-value');

    if (spindleOverride && spindleOverrideValue) {
        spindleOverride.addEventListener('input', function(e) {
            spindleOverrideValue.textContent = e.target.value + '%';
        });
    }

    if (feedOverride && feedOverrideValue) {
        feedOverride.addEventListener('input', function(e) {
            feedOverrideValue.textContent = e.target.value + '%';
        });
    }

    // Override 설정 버튼 이벤트 리스너
    // const setSpindleOverrideBtn = document.getElementById('set-spindle-override');
    const setFeedOverrideBtn = document.getElementById('set-feed-override');

    // // Spindle Override 버튼 이벤트 리스너
    // const spindleOverrideUpBtn = document.getElementById('spindle-override-up');
    // const spindleOverrideDownBtn = document.getElementById('spindle-override-down');
    // const spindleOverrideResetBtn = document.getElementById('spindle-override-reset');

    // if (spindleOverrideUpBtn) {
    //     spindleOverrideUpBtn.addEventListener('click', function() {
    //         socket.emit('spindle_override_command', { cmd: '0x99' });
    //         addLog('Spindle Override +10% (0x99)', 'info');
    //     });
    // }
    // if (spindleOverrideDownBtn) {
    //     spindleOverrideDownBtn.addEventListener('click', function() {
    //         socket.emit('spindle_override_command', { cmd: '0x9A' });
    //         addLog('Spindle Override -10% (0x9A)', 'info');
    //     });
    // }
    // if (spindleOverrideResetBtn) {
    //     spindleOverrideResetBtn.addEventListener('click', function() {
    //         socket.emit('spindle_override_command', { cmd: '0x9C' });
    //         addLog('Spindle Override Reset (0x9C)', 'info');
    //     });
    // }

    
    // Spindle Override 버튼 클릭 시 Socket 이벤트 전송 (GRBL 1.1 실시간 명령: 0x9A +10%, 0x9B -10%, 0x99 100% 복귀)
    document.getElementById("spindle-override-up").addEventListener("click", () => {
        socket.emit("spindle_override_command", { cmd: "0x9A" });
    });

    document.getElementById("spindle-override-down").addEventListener("click", () => {
        socket.emit("spindle_override_command", { cmd: "0x9B" });
    });

    document.getElementById("spindle-override-reset").addEventListener("click", () => {
        socket.emit("spindle_override_command", { cmd: "0x99" });
    });

    // 실시간 스핀들 RPM 값 수신해서 화면에 반영
    socket.on("spindle_rpm_update", function (data) {
        const rpm = data.rpm ?? "N/A";
        const override = data.override !== undefined ? `${data.override}% ` : "";
        document.getElementById("spindle-override-value").innerText = `${override}(${rpm} RPM)`;
    });

    if (setFeedOverrideBtn) {
        setFeedOverrideBtn.addEventListener('click', function() {
            const value = feedOverride.value;
            socket.emit('set_feed_override', { value: parseInt(value) });
            addLog(`Feed Override 설정: ${value}%`, 'info');
        });
    }

    // Jog 컨트롤 버튼 이벤트 리스너
    const jogButtons = document.querySelectorAll('.jog-btn');
    jogButtons.forEach(button => {
        button.addEventListener('click', function() {
            const axis = this.dataset.axis;
            const direction = this.dataset.direction;
            let stepXY = parseFloat(document.getElementById('step-xy-input').value) || 1.0;
            let stepZ = parseFloat(document.getElementById('step-z-input').value) || 0.2;
            let feed = 100; // 기본 피드값
            let dx = 0, dy = 0, dz = 0;
            
            if (axis === 'x') {
                dx = direction === '+' ? stepXY : -stepXY;
            } else if (axis === 'y') {
                dy = direction === '+' ? stepXY : -stepXY;
            } else if (axis === 'z') {
                dz = direction === '+' ? stepZ : -stepZ;
            }

            // jog 이벤트로 전송 (position_update를 emit하는 핸들러 사용)
            socket.emit('jog', { dx: dx, dy: dy, dz: dz, feed: feed });
            addLog(`조그 이동: X${dx} Y${dy} Z${dz} F${feed}`, 'info');
        });
    });

    // 모든 DOM 요소에 대해 안전하게 이벤트 리스너 추가하는 함수
    function safeAddEventListener(id, event, handler) {
        const element = document.getElementById(id);
        if (element) {
            element.addEventListener(event, handler);
            return true;
        } else {
            console.warn(`${id} 엘리먼트를 찾을 수 없습니다.`);
            return false;
        }
    }
    
    // 포트 새로고침
    safeAddEventListener('refresh-btn', 'click', fetchPorts);
    
    // 센서 포트 새로고침
    safeAddEventListener('refresh-sensor-port-btn', 'click', function() {
        fetchSensorPorts();
    });
    
    // 센서 연결 버튼
    safeAddEventListener('connect-sensor-btn', 'click', function() {
        connectSensorPort();
    });
    
    // 센서 연결 해제 버튼
    safeAddEventListener('disconnect-sensor-btn', 'click', function() {
        disconnectSensorPort();
    });
    
    // 컨트롤러 연결
    safeAddEventListener('connect-btn', 'click', connectPort);
    
    // 컨트롤러 연결 해제
    safeAddEventListener('disconnect-btn', 'click', function() {
        // 해제 중 상태 표시
        this.disabled = true;
        this.textContent = '해제 중...';
        addLog('컨트롤러 연결 해제 중...', 'info');
        
        // 해제 요청
        socket.emit('disconnect_port', {}, function(response) {
            const disconnectBtn = document.getElementById('disconnect-btn');
            if (disconnectBtn) {
                disconnectBtn.textContent = '해제';
                
                if (response.status === 'error') {
                    disconnectBtn.disabled = false;
                    showNotification(response.message, 'error');
                    addLog(`컨트롤러 연결 해제 실패: ${response.message}`, 'error');
                }
            }
        });
    });
    
    // NC 코드 로드
    safeAddEventListener('load-nc-btn', 'click', function() {
        const fileInput = document.getElementById('nc-file-input');
        if (!fileInput || !fileInput.files.length) {
            showNotification('파일을 선택해주세요', 'error');
            return;
        }
        
        // 파일 정보 가져오기
        const file = fileInput.files[0];
        const fileName = file.name;
        const fileSize = (file.size / 1024).toFixed(2); // KB 단위로 변환
        
        // 로딩 시작 표시
        addLog(`NC 파일 로드 시작: ${fileName} (${fileSize} KB)`, 'info');
        
        const reader = new FileReader();
        reader.onload = function(e) {
            socket.emit('load_nc_file', { content: e.target.result, fileName: fileName }, function(response) {
                showNotification(response.message, response.status);
                
                // 로딩 완료 로그 추가
                if (response.status === 'success') {
                    addLog(`NC 파일 로드 완료: ${fileName}, ${response.lines}줄`, 'success');
                } else {
                    addLog(`NC 파일 로드 실패: ${response.message}`, 'error');
                }
                
                // 예상 시간 업데이트
                if (response.estimated_time) {
                    estimatedProcessingTime = response.estimated_time.text;
                    // 시간 정보 업데이트
                    updateTimeInfo();
                    addLog(`예상 가공 시간: ${response.estimated_time.text}`, 'info');
                }
                
                // G코드 내용에서 S값과 F값 추출
                if (response.status === 'success' && response.gcode_lines) {
                    // G코드 내용에서 S값과 F값 추출
                    let spindle = '-';
                    let feedrate = '-';
                    
                    for (let line of response.gcode_lines) {
                        // 스핀들 속도(S) 추출
                        const spindleMatch = line.match(/S(\d+)/);
                        if (spindleMatch && spindleMatch[1]) {
                            spindle = spindleMatch[1];
                        }
                        
                        // 이송 속도(F) 추출
                        const feedrateMatch = line.match(/F(\d+(\.\d+)?)/);
                        if (feedrateMatch && feedrateMatch[1]) {
                            feedrate = feedrateMatch[1];
                        }
                    }
                    
                    // 상태 라벨 업데이트
                    const statusLabel = document.getElementById('status-label');
                    if (statusLabel) {
                        statusLabel.innerHTML = `Spindle: ${spindle} RPM <br> Feed Rate: ${feedrate} mm/min`;
                    }
                }
                
                // NC 코드 로그 표시
                const logText = document.getElementById('log-text');
                if (logText && response.gcode_lines) {
                    logText.innerHTML = '';
                    response.gcode_lines.forEach(line => {
                        const logLine = document.createElement('div');
                        logLine.textContent = line;
                        logText.appendChild(logLine);
                    });
                    logText.scrollTop = logText.scrollHeight;
                }
            });
        };
        reader.readAsText(fileInput.files[0]);
    });
    
    // NC 파일 입력 변경 시 자동 로드
    safeAddEventListener('nc-file-input', 'change', function() {
        if (this.files.length) {
            const loadBtn = document.getElementById('load-nc-btn');
            if (loadBtn) loadBtn.click();
        }
    });
    
    // 가공 데이터 저장 버튼
    safeAddEventListener('save-data-btn', 'click', function() {
        saveProcessingDataToCSV();
    });
    
    // 창 크기 변경 이벤트
    window.addEventListener('resize', function() {
        // 차트 크기 조정
        if (mainChart && zChart) {
            // 디바운스 처리
            clearTimeout(window.resizeTimer);
            window.resizeTimer = setTimeout(function() {
                console.log("화면 크기 변경됨, 차트 리사이즈");
                resizeCharts();
            }, 250);
        }
    });
    
    // OrientationChange 이벤트 (모바일 기기 회전 감지)
    window.addEventListener('orientationchange', function() {
        // 가로/세로 모드 전환 시 차트 크기 조정
        setTimeout(function() {
            console.log("기기 회전 감지, 차트 리사이즈");
            resizeCharts();
        }, 300);
    });
    


    // (로그 지우기 버튼과 자동 스크롤 체크박스는 initLogControls()에서 등록)

    // 모달 닫기 버튼
    const closeButtons = document.querySelectorAll('.close-modal');
    closeButtons.forEach(function(btn) {
        btn.addEventListener('click', function() {
            const modal = this.closest('.modal');
            if (modal) modal.style.display = 'none';
        });
    });

    // 진동 감지 모델 관리 창
    safeAddEventListener('vibration-settings-btn', 'click', function() {
        openModal('vibration-settings-modal');
        refreshVibrationModelStatus();
    });
    safeAddEventListener('cancel-vibration-settings', 'click', function() {
        closeModal('vibration-settings-modal');
    });
    safeAddEventListener('train-model-btn', 'click', trainVibrationModel);
    safeAddEventListener('reset-vibration-analyzer', 'click', resetVibrationAnalyzer);
    safeAddEventListener('mitigation-enabled', 'change', function() {
        saveVibrationSettings({ mitigation_enabled: this.checked });
    });
    safeAddEventListener('decision-mode', 'change', function() {
        saveVibrationSettings({ decision_mode: this.value });
    });

    // 데이터 수집 창
    safeAddEventListener('recording-btn', 'click', function() {
        openModal('recording-modal');
        refreshRecordingStatus(true);
    });
    safeAddEventListener('rec-close-btn', 'click', function() {
        closeModal('recording-modal');
    });
    safeAddEventListener('rec-start-btn', 'click', startRecording);
    safeAddEventListener('rec-stop-btn', 'click', stopRecording);
    safeAddEventListener('rec-mark-start-btn', 'click', function() { markRecording('start'); });
    safeAddEventListener('rec-mark-end-btn', 'click', function() { markRecording('end'); });

    // 센서 통신 속도: 마지막으로 연결한 값 복원 (115200 기존 펌웨어, 500000 고속 펌웨어)
    const baudSelect = document.getElementById('sensor-baud-select');
    let savedBaud = null;
    try { savedBaud = localStorage.getItem('lastSensorBaud'); } catch (e) { savedBaud = null; }
    if (baudSelect && savedBaud) baudSelect.value = savedBaud;


    
    // 가공 시작 버튼 (일시정지 상태에서는 중단된 지점부터 재개)
    safeAddEventListener('start-btn', 'click', function() {
        if (currentProcessingState === 'running') {
            showNotification('이미 가공이 진행 중입니다.', 'warning');
            return;
        }
        socket.emit('start_gcode', {}, function(response) {
            if (response.status === 'success') {
                currentProcessingState = 'running';
                // 재개일 때는 경과 시간과 수집 중인 센서 데이터를 초기화하지 않음
                if (!response.resumed) {
                    startProcessingTimer();
                }
                showNotification(response.message, 'success');
                addLog(response.resumed ? '가공을 재개합니다.' : '가공을 시작합니다.', 'success');
                if (response.estimated_time) {
                    estimatedProcessingTime = response.estimated_time.text;
                }
                updateTimeInfo();
            } else {
                showNotification(response.message, 'error');
                addLog(response.message, 'error');
            }
        });
    });

    // 일시정지 버튼 (가공 여부는 서버가 판단: 화면 상태가 늦게 갱신되어도 일시정지가 막히지 않도록)
    const pauseButton = document.getElementById('pause-btn');
    if (pauseButton) {
        pauseButton.addEventListener('click', function() {
            socket.emit('toggle_pause', {}, function(response) {
                if (response.status === 'success') {
                    currentProcessingState = 'paused';
                    showNotification(response.message, 'success');
                    addLog(response.message, 'success');
                    stopProcessingTimer();
                } else {
                    showNotification(response.message, 'error');
                    addLog(response.message, 'error');
                }
            });
        });
    }

    // 긴급정지 버튼
    safeAddEventListener('emergency-btn', 'click', function() {
        socket.emit('emergency_stop', {}, function(response) {
            if (response.status === 'success') {
                currentProcessingState = 'emergency';
                showNotification(response.message, 'success');
                addLog(response.message, 'success');
                stopProcessingTimer();
                const emergencyBtn = document.getElementById('emergency-btn');
                if (emergencyBtn && response.button_text) {
                    emergencyBtn.textContent = response.button_text;
                }
            } else {
                showNotification(response.message, 'error');
                addLog(response.message, 'error');
            }
        });
    });

    // 알람 해제 버튼
    safeAddEventListener('unlock-btn', 'click', function() {
        const unlockBtn = this;
        unlockBtn.textContent = '해제 중...';
        unlockBtn.classList.remove('attention');
        socket.emit('unlock_alarm', {}, function(response) {
            unlockBtn.textContent = response.button_text || '알람해제';
            if (response.status === 'success') {
                showNotification(response.message, 'success');
                addLog(response.message, 'success');
            } else {
                showNotification(response.message, 'error');
                addLog(response.message, 'error');
                unlockBtn.classList.add('attention');
            }
        });
    });


    // XY 원점 버튼
    document.getElementById('zero-xy-btn').addEventListener('click', function () {
        fetch('/api/set-zero', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: 'axis=xy'
        })
        .then(res => res.json())
        .then(data => {
            addLog(data.message, data.status === 'success' ? 'success' : 'error');
            // 원점 설정 후 G54 활성화 및 G0 X0 Y0 이동
            // (새 원점 좌표는 서버가 position_update로 보내므로 여기서 경로 배열을 직접 바꾸지 않음)
            if (data.status === 'success') {
                socket.emit('grbl_command', { command: 'G54\nG0 X0 Y0' });
            }
        });
    });

    // Z 원점 버튼
    document.getElementById('zero-z-btn').addEventListener('click', function () {
        fetch('/api/set-zero', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: 'axis=z'
        })
        .then(res => res.json())
        .then(data => {
            addLog(data.message, data.status === 'success' ? 'success' : 'error');
            // 원점 설정 후 G54 활성화 및 G0 Z0 이동
            // (새 원점 좌표는 서버가 position_update로 보내므로 여기서 경로 배열을 직접 바꾸지 않음)
            if (data.status === 'success') {
                socket.emit('grbl_command', { command: 'G54\nG0 Z0' });
            }
        });
    });

   
    console.log("이벤트 리스너 초기화 완료");

    // 허용 공차 확인 버튼 이벤트 리스너
    document.getElementById('tolerance-confirm-btn').addEventListener('click', function() {
        const toleranceInput = document.getElementById('tolerance');
        const value = parseFloat(toleranceInput.value);
        
        console.log('입력된 허용 공차 값:', value);
        
        if (isNaN(value) || value < 0) {
            alert('유효한 허용 공차 값을 입력하세요 (0 이상의 숫자)');
            return;
        }
        
        fetch('/set_tolerance', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/x-www-form-urlencoded',
            },
            body: `tolerance=${encodeURIComponent(value)}`
        })
        .then(response => response.json())
        .then(data => {
            console.log('서버 응답:', data);
            if (data.status === 'success') {
                toleranceValue = data.tolerance;
                console.log('설정된 허용 공차 값:', toleranceValue);
                alert(data.message);
            } else {
                // 서버가 거부한 값이면 입력창을 현재 서버 값으로 되돌림
                toleranceInput.value = toleranceValue;
                alert('오류: ' + data.message);
            }
        })
        .catch(error => {
            console.error('오류:', error);
            alert('서버 통신 중 오류가 발생했습니다.');
        });
    });
}


// 알림 표시
function showNotification(message, status) {
    // 간단한 알림 표시 (실제 프로덕션에서는 토스트 라이브러리 사용 권장)
    const notification = document.createElement('div');
    notification.className = `notification ${status}`;
    notification.textContent = message;
    
    document.body.appendChild(notification);
    
    // 스타일 적용
    notification.style.position = 'fixed';
    notification.style.bottom = '20px';
    notification.style.right = '20px';
    notification.style.padding = '10px 15px';
    notification.style.borderRadius = '4px';
    notification.style.color = 'white';
    notification.style.zIndex = '1000';
    
    if (status === 'success') {
        notification.style.backgroundColor = 'var(--success-color)';
    } else if (status === 'error') {
        notification.style.backgroundColor = 'var(--emergency-color)';
    } else {
        notification.style.backgroundColor = 'var(--accent-color)';
    }
    
    // 3초 후 제거
    setTimeout(() => {
        notification.style.opacity = '0';
        notification.style.transition = 'opacity 0.5s ease';
        setTimeout(() => {
            document.body.removeChild(notification);
        }, 500);
    }, 3000);
}

// 시간 정보 업데이트
function updateTimeInfo() {
    // 경과 시간 업데이트
    const elapsedTimeElement = document.getElementById('elapsed-time');
    if (elapsedTimeElement) {
        elapsedTimeElement.textContent = `경과 시간: ${formatTimeHMS(totalSeconds)}`;
    }
    
    // 예상 가공 시간 업데이트
    const estimatedTimeElement = document.getElementById('estimated-time');
    if (estimatedTimeElement) {
        estimatedTimeElement.textContent = `예상 가공 시간: ${estimatedProcessingTime || "00:00:00"}`;
    }
}

// 시간 형식 변환 (초 -> HH:MM:SS)
function formatTimeHMS(seconds) {
    seconds = Math.max(0, Math.floor(seconds));
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

// 가공 데이터를 CSV로 저장
function saveProcessingDataToCSV() {
    // 데이터가 없는 경우
    if (xVals.length <= 1 && yVals.length <= 1 && zVals.length <= 1) {
        showNotification('저장할 가공 데이터가 없습니다', 'error');
        return;
    }
    
    // BOM(Byte Order Mark) 추가로 한글 인코딩 문제 해결
    let csvContent = '\ufeff';
    
    // CSV 헤더 - 서버와 동일한 형식으로 변경
    csvContent += 'X,Y,Z,ProcessTime\n';
    
    // 데이터 행 추가
    for (let i = 0; i < xVals.length; i++) {
        const x = xVals[i] || 0;
        const y = yVals[i] || 0;
        const z = zVals[i] || 0;
        let timeSeconds = timeVals[i] || 0;
        
        // 시간을 정수로 변환
        timeSeconds = Math.round(timeSeconds);
        
        // 서버 로깅과 동일한 순서로 데이터 저장
        csvContent += `${x},${y},${z},${timeSeconds}\n`;
    }
    
    // 현재 날짜와 시간으로 파일명 생성
    const now = new Date();
    const dateString = [
    now.getFullYear(),
    (now.getMonth() + 1).toString().padStart(2, '0'),
    now.getDate().toString().padStart(2, '0')
    ].join('-') + '_' +
    [
    now.getHours().toString().padStart(2, '0'),
    now.getMinutes().toString().padStart(2, '0'),
    now.getSeconds().toString().padStart(2, '0')
    ].join('-');
    const fileName = `가공_데이터_${dateString}.csv`;
    
    // 파일 다운로드
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = fileName;
    link.click();
    URL.revokeObjectURL(link.href);
    
    showNotification('가공 데이터가 성공적으로 저장되었습니다', 'success');
}

// 센서 데이터 CSV 저장 함수
function saveSensorDataToCSV() {
    if (sensorDataArray.length === 0) {
        showNotification('저장할 센서 데이터가 없습니다', 'error');
        return;
    }
    let csvContent = '\ufeff';
    csvContent += 'Timestamp,ProcessTime,AccX,AccY,AccZ,GyroX,GyroY,GyroZ\n';
    sensorDataArray.forEach(data => {
        csvContent += [
            data.timestamp ?? '',
            data.processTime ?? '',
            data.accX ?? '',
            data.accY ?? '',
            data.accZ ?? '',
            data.gyroX ?? '',
            data.gyroY ?? '',
            data.gyroZ ?? ''
        ].join(',') + '\n';
    });
    const now = new Date();
    const dateString = [
        now.getFullYear(),
        (now.getMonth() + 1).toString().padStart(2, '0'),
        now.getDate().toString().padStart(2, '0')
    ].join('-') + '_' +
    [
        now.getHours().toString().padStart(2, '0'),
        now.getMinutes().toString().padStart(2, '0'),
        now.getSeconds().toString().padStart(2, '0')
    ].join('-');
    const fileName = `센서_데이터_${dateString}.csv`;
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = fileName;
    link.click();
    URL.revokeObjectURL(link.href);
    showNotification('센서 데이터가 성공적으로 저장되었습니다', 'success');
}

// 가공 완료 처리 함수 (누락된 함수 추가)
function handleProcessingComplete() {
    stopProcessingTimer();
    showNotification('가공이 완료되었습니다!', 'success');
    
    // 가공 상태 업데이트
    currentProcessingState = 'idle';
}

// 가공 타이머 시작 함수 (누락된 함수 추가)
function startProcessingTimer() {
    // 이미 타이머가 실행 중이면 중지
    if (processingTimer) {
        clearInterval(processingTimer);
    }
    
    // 처리 시작 시간 기록
    processingStartTime = new Date();
    totalSeconds = 0;
    
    // 센서 데이터 수집 시작
    sensorDataArray = [];
    isRecordingSensorData = true;
    
    // 타이머 시작 (1초마다 업데이트)
    processingTimer = setInterval(function() {
        totalSeconds++;
        updateTimeInfo();
    }, 1000);
}

// 가공 타이머 중지 함수 (누락된 함수 추가)
function stopProcessingTimer() {
    if (processingTimer) {
        clearInterval(processingTimer);
        processingTimer = null;
    }
    
    // 센서 데이터 수집 중지
    isRecordingSensorData = false;
}

// 수신 데이터 처리 함수 (누락된 함수 추가)
function handleReceivedData(data) {
    // 데이터가 있으면 차트에 표시
    if (data && data.x !== undefined && data.y !== undefined) {
        updateCharts(data);
    }
}

// Jog 버튼 설정 함수 (누락된 함수 추가)
function setupJogButtons() {
    // X+ 버튼
    document.getElementById('jog-x-plus').addEventListener('click', function() {
        const stepSize = parseFloat(document.getElementById('step-xy-input').value);
        const feedrate = parseInt(document.getElementById('jog-feedrate-input').value);
        socket.emit('jog', {dx: stepSize, dy: 0, dz: 0, feed: feedrate});
    });
    
    // X- 버튼
    document.getElementById('jog-x-minus').addEventListener('click', function() {
        const stepSize = parseFloat(document.getElementById('step-xy-input').value);
        const feedrate = parseInt(document.getElementById('jog-feedrate-input').value);
        socket.emit('jog', {dx: -stepSize, dy: 0, dz: 0, feed: feedrate});
    });
    
    // Y+ 버튼
    document.getElementById('jog-y-plus').addEventListener('click', function() {
        const stepSize = parseFloat(document.getElementById('step-xy-input').value);
        const feedrate = parseInt(document.getElementById('jog-feedrate-input').value);
        socket.emit('jog', {dx: 0, dy: stepSize, dz: 0, feed: feedrate});
    });
    
    // Y- 버튼
    document.getElementById('jog-y-minus').addEventListener('click', function() {
        const stepSize = parseFloat(document.getElementById('step-xy-input').value);
        const feedrate = parseInt(document.getElementById('jog-feedrate-input').value);
        socket.emit('jog', {dx: 0, dy: -stepSize, dz: 0, feed: feedrate});
    });
    
    // Z+ 버튼
    document.getElementById('jog-z-plus').addEventListener('click', function() {
        const stepSize = parseFloat(document.getElementById('step-z-input').value);
        const feedrate = parseInt(document.getElementById('jog-feedrate-input').value);
        socket.emit('jog', {dx: 0, dy: 0, dz: stepSize, feed: feedrate});
    });
    
    // Z- 버튼
    document.getElementById('jog-z-minus').addEventListener('click', function() {
        const stepSize = parseFloat(document.getElementById('step-z-input').value);
        const feedrate = parseInt(document.getElementById('jog-feedrate-input').value);
        socket.emit('jog', {dx: 0, dy: 0, dz: -stepSize, feed: feedrate});
    });
}

// 공구 정보 초기화
function initToolInfo() {
    // 항상 공구 정보를 초기화하여 "공구 정보를 설정해주세요" 메시지가 나오도록 함
    toolDiameter = null;
    toolFlutes = null;
    
    // 초기 값 표시
    updateToolInfoDisplay();
}

// 공구 정보 표시 업데이트
function updateToolInfoDisplay() {
    const toolInfoEl = document.getElementById('current-tool-info');
    if (toolInfoEl) {
        if (toolDiameter !== null && toolFlutes !== null) {
            toolInfoEl.textContent = `현재 공구: 직경 ${toolDiameter}mm, 날 수 ${toolFlutes}개`;
        } else {
            toolInfoEl.textContent = '공구 정보를 설정해주세요';
        }
    }
}

// 공구 정보 모달 열기
function openToolModal() {
    const toolModal = document.getElementById('tool-modal');
    const toolDiameterInput = document.getElementById('tool-diameter');
    const toolFlutesInput = document.getElementById('tool-flutes');
    
    if (toolModal && toolDiameterInput && toolFlutesInput) {
        // 현재 값이 설정되어 있으면 해당 값으로, 없으면 기본값으로 입력 필드 초기화
        toolDiameterInput.value = toolDiameter !== null ? toolDiameter : 0;
        toolFlutesInput.value = toolFlutes !== null ? toolFlutes : 0;
        
        // 모달 표시
        toolModal.style.display = 'block';
        
        // 첫 번째 입력 필드에 포커스
        toolDiameterInput.focus();
    }
}

// 공구 정보 모달 닫기
function closeToolModal() {
    const toolModal = document.getElementById('tool-modal');
    if (toolModal) {
        toolModal.style.display = 'none';
    }
}

// 공구 정보 저장
function saveToolInfo() {
    const toolDiameterInput = document.getElementById('tool-diameter');
    const toolFlutesInput = document.getElementById('tool-flutes');
    
    if (toolDiameterInput && toolFlutesInput) {
        // 입력값 검증
        const newDiameter = parseFloat(toolDiameterInput.value);
        const newFlutes = parseInt(toolFlutesInput.value);
        
        if (isNaN(newDiameter) || newDiameter <= 0) {
            addLog('공구 직경은 0보다 큰 숫자여야 합니다.', 'error');
            return;
        }
        
        if (isNaN(newFlutes) || newFlutes <= 0) {
            addLog('공구 날 수는  1개 이상이어야 합니다.', 'error');
            return;
        }
        
        // 서버로 공구 정보 전송
        socket.emit('set_tool_info', {
            diameter: newDiameter,
            flutes: newFlutes
        }, function(response) {
            if (response.status === 'success') {
                // 성공 시 로컬 변수 업데이트
                toolDiameter = newDiameter;
                toolFlutes = newFlutes;
                
                // 화면에 표시 업데이트
                updateToolInfoDisplay();
                
                // 알림 표시
                showNotification('공구 정보가 저장되었습니다.', 'success');
                
                // 모달 닫기
                closeToolModal();
            } else {
                // 오류 메시지 표시
                addLog(response.message, 'error');
            }
        });
    }
}

// 모달 열기
function openModal(modalId) {
    const modal = document.getElementById(modalId);
    if (modal) {
        modal.style.display = 'block';

    }
}

// 모달 닫기
function closeModal(modalId) {
    const modal = document.getElementById(modalId);
    if (modal) {
        modal.style.display = 'none';
    }
}

// 진동 감지 모델 상태 조회 (서버 시작 직후 모델을 학습하는 동안은 잠시 뒤 다시 조회)
let vibrationModelRetryTimer = null;
function refreshVibrationModelStatus() {
    clearTimeout(vibrationModelRetryTimer);
    fetch('/api/vibration/model')
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success') {
                renderVibrationModelStatus(data.model);
            } else {
                renderVibrationModelStatus(null, data.message);
                vibrationModelRetryTimer = setTimeout(refreshVibrationModelStatus, 2000);
            }
        })
        .catch(error => {
            console.error('진동 감지 모델 상태 조회 실패:', error);
            renderVibrationModelStatus(null, '모델 상태를 불러오지 못했습니다.');
        });
}

// 제어 영역의 상태 문구와 모델 관리 창 내용을 갱신 (model이 null이면 message만 표시)
function renderVibrationModelStatus(model, message) {
    const summary = document.getElementById('vibration-status-text');
    const detail = document.getElementById('vibration-model-status');
    const fileList = document.getElementById('vibration-model-files');
    const untrained = !!model && !model.is_trained;
    let summaryText = `진동 감지 모델: ${message}`;
    let detailText = message;
    let files = [];

    if (model && model.is_trained) {
        summaryText = `진동 감지 모델: 학습됨 (정상 데이터 ${model.files.length}개 파일)`;
        const thresholds = model.thresholds || {};
        const lines = [
            '상태: 학습됨',
            `학습 데이터: ${model.training_dir} (파일 ${model.files.length}개, 샘플 ${model.samples.toLocaleString()}개, 가공 윈도우 ${(model.windows || 0).toLocaleString()}개)`
        ];
        if (thresholds.rms_g !== null && thresholds.rms_g !== undefined) {
            lines.push(`기준: 진동 크기 ${thresholds.rms_g.toFixed(3)} g, LOF 점수 ${thresholds.lof.toFixed(2)}` +
                (thresholds.calibration === 'session_holdout' ? ' (파일 단위로 검증한 값)' : ''));
        }
        const win = model.window || {};
        if (win.label) {
            lines.push(`판정 창: ${win.label}` +
                (win.fallback ? ' (센서 샘플링에 맞는 창의 학습 데이터가 없어 이 모델로 감지 중 - 이 센서로 기록한 정상 데이터를 추가하세요)' : ''));
        }
        const optimizerInfo = model.optimizer || {};
        lines.push('재가공·온라인 대응 조건: ' + (optimizerInfo.model ? '채터 확률 모델' : '채터 확률 모델 없음 → 채터 주파수 또는 기본 규칙') +
            (model.condition_model ? ', 조건 모델로 진동 크기 비교' : ''));
        if (model.sensor && model.sensor.fs) {
            lines.push(`센서: ${model.sensor.device || ''} ${Math.round(model.sensor.fs)} Hz, ±${model.sensor.range_g} g`);
        }
        detailText = lines.join('\n');
        files = model.files;
    } else if (model) {
        summaryText = '진동 감지 모델: 학습 안 됨 (이상 진동 감지 꺼짐)';
        detailText = `상태: 학습 안 됨 - 이상 진동 감지가 동작하지 않습니다.\n` +
            `${model.training_dir} 폴더에 정상 가공 센서 CSV를 넣거나 아래에서 파일을 올려 학습하세요.`;
    }
    if (model && model.is_training) {
        detailText += '\n지금 모델을 다시 학습하는 중입니다.';
    }

    if (summary) {
        summary.textContent = summaryText;
        summary.title = summaryText;
        summary.classList.toggle('untrained', untrained);
    }
    if (detail) {
        detail.textContent = detailText;
        detail.classList.toggle('untrained', untrained);
    }
    if (fileList) {
        fileList.textContent = files.join('\n');
    }
    if (model && model.settings) {
        const mitigation = document.getElementById('mitigation-enabled');
        if (mitigation && document.activeElement !== mitigation) mitigation.checked = !!model.settings.mitigation_enabled;
        const mode = document.getElementById('decision-mode');
        if (mode && document.activeElement !== mode) mode.value = model.settings.decision_mode;
    }
}

// 감지 설정 저장 (온라인 대응 켜기/끄기, 판정 방식). 가공 중에도 다음 판정부터 적용됩니다.
function saveVibrationSettings(updates) {
    fetch('/api/vibration/settings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(updates)
    })
        .then(response => response.json())
        .then(data => {
            showNotification(data.message, data.status === 'success' ? 'success' : 'error');
            if (data.model) renderVibrationModelStatus(data.model);
        })
        .catch(error => {
            console.error('감지 설정 저장 실패:', error);
            showNotification('감지 설정을 저장하지 못했습니다.', 'error');
            refreshVibrationModelStatus();
        });
}

// 실시간 진동 지표 (서버가 판정할 때마다 최대 초당 2번 보냄, data가 없으면 표시를 지움)
function renderVibrationLive(data) {
    const el = document.getElementById('vibration-live-text');
    if (!el) return;
    let text = '';
    let level = '';
    if (data) {
        if (data.stage === 'mitigating') {
            text = '· 온라인 대응 중';
            level = 'warn';
        } else if (data.stage === 'stopping' || data.stage === 'detected') {
            text = '· 이상 진동: 정지 중';
            level = 'alarm';
        } else if (!data.gate) {
            text = '· 판정 구간 밖 (소재 밖·급속 이동)';
        } else if (data.evidence === null || data.evidence === undefined) {
            text = '· 판정 준비 중';
        } else {
            text = `· 지표 ${Number(data.evidence).toFixed(2)}`;
            level = data.alarm ? 'alarm' : (data.evidence >= 1 ? 'warn' : '');
        }
    }
    el.textContent = text;
    el.title = data && data.parts ? Object.entries(data.parts).map(([k, v]) => `${k} ${v}`).join(', ') : '';
    el.classList.toggle('alarm', level === 'alarm');
    el.classList.toggle('warn', level === 'warn');
}

// ===== 데이터 수집 기록 (data/raw/) =====
const RECORDING_FIELDS = ['run_id', 'S_rpm', 'F_mm_min', 'ap_mm', 'ae_mm', 'tool_id', 'tool_diameter_mm',
    'flutes', 'stickout_mm', 'material', 'photo', 'notes'];
const RECORDING_LABELS = { chatter: '채터', transition: '전이', air: '비가공', stable: '안정' };
let recordingPollTimer = null;

function refreshRecordingStatus(fillForm) {
    fetch('/api/recording/status')
        .then(response => response.json())
        .then(data => {
            if (data.recording) renderRecordingStatus(data.recording, fillForm);
        })
        .catch(error => console.error('데이터 수집 상태 조회 실패:', error));
}

function renderRecordingStatus(rec, fillForm) {
    if (!rec) return;
    const statusEl = document.getElementById('recording-status');
    const button = document.getElementById('recording-btn');
    let text;
    if (rec.active) {
        const openLabels = (rec.open_labels || []).map(label => RECORDING_LABELS[label] || label);
        text = `● 기록 중: ${rec.run_id} (${Math.round(rec.elapsed)}초, 센서 ${rec.sensor_samples.toLocaleString()}샘플` +
            (rec.sensor_dropped ? `, 누락 ${rec.sensor_dropped}` : '') +
            `, 가공 로그 ${rec.machine_rows}줄, 라벨 구간 ${rec.labels}개)` +
            (openLabels.length ? `\n열린 구간: ${openLabels.join(', ')}` : '');
    } else {
        text = '기록하지 않는 중';
        if (rec.last_run) {
            text += `\n마지막 기록: ${rec.last_run.run_id} (${rec.last_run.duration_s}초, 센서 ${rec.last_run.sensor_samples}샘플, 라벨 구간 ${rec.last_run.labels}개)`;
        }
    }
    let sensorText = '연결되지 않음';
    if (rec.sensor_connected) {
        sensorText = rec.sensor && rec.sensor.fs
            ? `${rec.sensor.device} ${Math.round(rec.sensor.fs)} Hz, ±${rec.sensor.range_g} g`
            : '연결됨 (기존 펌웨어: 장치 시각 없음, 고속 펌웨어 권장)';
    }
    text += `\n센서: ${sensorText}`;
    if (statusEl) statusEl.textContent = text;
    if (button) {
        button.classList.toggle('active', !!rec.active);
        button.textContent = rec.active ? '기록 중' : '데이터 수집';
    }
    const startBtn = document.getElementById('rec-start-btn');
    if (startBtn) startBtn.disabled = !!rec.active;
    ['rec-stop-btn', 'rec-mark-start-btn', 'rec-mark-end-btn'].forEach(id => {
        const btn = document.getElementById(id);
        if (btn) btn.disabled = !rec.active;
    });
    if (fillForm && !rec.active) fillRecordingForm(rec);

    // 기록 중에는 2초마다 진행 상황 갱신
    clearTimeout(recordingPollTimer);
    if (rec.active) recordingPollTimer = setTimeout(() => refreshRecordingStatus(false), 2000);
}

// 빈 칸만 NC 파일의 META 주석(실험 조건)과 공구 정보로 채움
function fillRecordingForm(rec) {
    const meta = rec.program_meta || {};
    const tool = rec.tool || {};
    const defaults = {
        run_id: meta.run_id, S_rpm: meta.S, F_mm_min: meta.F, ap_mm: meta.ap, ae_mm: meta.ae,
        tool_id: meta.tool, material: meta.material,
        tool_diameter_mm: meta.tool_d || tool.diameter, flutes: meta.flutes || tool.flutes
    };
    Object.entries(defaults).forEach(([key, value]) => {
        const input = document.getElementById(`rec-${key}`);
        if (input && !input.value && value !== undefined && value !== null && value !== '') input.value = value;
    });
}

function recordingFormValues() {
    const values = {};
    RECORDING_FIELDS.forEach(key => {
        const input = document.getElementById(`rec-${key}`);
        if (input && input.value !== '') values[key] = input.value;
    });
    return values;
}

function postRecording(url, body) {
    return fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body || {})
    })
        .then(response => response.json())
        .then(data => {
            showNotification(data.message, data.status === 'success' ? 'success' : 'error');
            if (data.recording) renderRecordingStatus(data.recording);
            return data;
        })
        .catch(error => {
            console.error('데이터 수집 요청 실패:', error);
            showNotification('데이터 수집 요청 중 오류가 발생했습니다.', 'error');
            return null;
        });
}

function startRecording() {
    postRecording('/api/recording/start', recordingFormValues());
}

function stopRecording() {
    postRecording('/api/recording/stop').then(data => {
        // 다음 실행은 새로 불러온 NC의 실행 ID를 쓰도록 실행별 값은 비움
        if (data && data.status === 'success') {
            ['run_id', 'photo', 'notes'].forEach(key => {
                const input = document.getElementById(`rec-${key}`);
                if (input) input.value = '';
            });
        }
    });
}

function markRecording(edge) {
    const select = document.getElementById('rec-label-select');
    postRecording('/api/recording/mark', { label: select ? select.value : 'chatter', edge: edge });
}

// 진동 감지 모델 학습/초기화 요청 (응답이 올 때까지 버튼을 막고 진행 중임을 표시)
function requestVibrationModelChange(url, body, busyMessage, onSuccess) {
    const buttons = ['train-model-btn', 'reset-vibration-analyzer']
        .map(id => document.getElementById(id))
        .filter(Boolean);
    buttons.forEach(btn => { btn.disabled = true; });
    const detail = document.getElementById('vibration-model-status');
    if (detail) detail.textContent = busyMessage;

    fetch(url, { method: 'POST', body: body })
        .then(response => response.json())
        .then(data => {
            // 시스템 로그는 서버의 log_update 이벤트로 남으므로 여기서는 알림만 표시
            showNotification(data.message, data.status === 'success' ? 'success' : 'error');
            if (data.status === 'success' && onSuccess) onSuccess();
            if (data.model) {
                renderVibrationModelStatus(data.model);
            } else {
                refreshVibrationModelStatus();
            }
        })
        .catch(error => {
            console.error('진동 감지 모델 요청 실패:', error);
            showNotification('진동 감지 모델 요청 중 오류가 발생했습니다.', 'error');
            refreshVibrationModelStatus();
        })
        .finally(() => {
            buttons.forEach(btn => { btn.disabled = false; });
        });
}

// 선택한 정상 가공 CSV를 학습 데이터에 추가하고 모델 재학습
function trainVibrationModel() {
    const input = document.getElementById('train-model-file');
    const file = input && input.files[0];
    if (!file) {
        showNotification('학습할 CSV 파일을 선택하세요.', 'warning');
        return;
    }
    if (!confirm(`'${file.name}' 파일을 학습 데이터에 추가하고 모델을 다시 학습할까요?`)) {
        return;
    }
    const formData = new FormData();
    formData.append('train_file', file);
    requestVibrationModelChange('/api/vibration/train', formData,
        '모델을 학습하는 중입니다... 잠시 기다려 주세요.',
        () => { input.value = ''; });
}

// 학습 데이터 폴더의 파일로 모델을 다시 학습하고 감지 상태 초기화
function resetVibrationAnalyzer() {
    if (!confirm('학습 데이터 폴더의 파일로 모델을 다시 학습하고 감지 버퍼를 비울까요?')) {
        return;
    }
    requestVibrationModelChange('/api/vibration/reset', null, '학습 데이터 폴더로 모델을 다시 학습하는 중입니다...');
}

// 소켓 이벤트 핸들러 등록
function registerSocketEvents() {
    // 이 함수는 사용하지 않음 - 모든 소켓 이벤트 핸들러는 initSocketListeners에서 처리
    console.warn("registerSocketEvents 함수는 더 이상 사용되지 않습니다. initSocketListeners 함수를 사용하세요.");
}

// 아두이노 포트 목록 가져오기
function fetchArduinoPorts() {
    console.log("아두이노 포트 목록 가져오기");
    try {
        // 아두이노 포트 드롭다운 존재 확인
        const arduinoPortSelect = document.getElementById('arduino-port-select');
        if (!arduinoPortSelect) {
            console.error("arduino-port-select 엘리먼트를 찾을 수 없습니다.");
            return;
        }
        
        // 초기 상태로 "로딩 중" 표시
        const oldHTML = arduinoPortSelect.innerHTML;
        arduinoPortSelect.innerHTML = '<option disabled selected>로딩 중...</option>';
        
        // API를 통한 포트 목록 요청
        fetch('/api/refresh_ports')
            .then(response => {
                if (!response.ok) {
                    throw new Error(`HTTP error ${response.status}`);
                }
                return response.json();
            })
            .then(data => {
                if (data.status === 'success') {
                    updateArduinoPortSelect(data.ports);
                } else {
                    console.error("포트 목록 가져오기 실패:", data.message);
                    // 실패 시 원래 상태로 복원
                    arduinoPortSelect.innerHTML = oldHTML;
                    showNotification('포트 목록을 가져오는데 실패했습니다', 'error');
                }
            })
            .catch(error => {
                console.error("아두이노 포트 목록 요청 오류:", error);
                // 오류 시 원래 상태로 복원
                arduinoPortSelect.innerHTML = oldHTML;
                showNotification(`포트 목록 요청 오류: ${error.message}`, 'error');
            });
    } catch (error) {
        console.error("fetchArduinoPorts 실행 중 오류:", error);
    }
}

// 아두이노 포트 선택 드롭다운 업데이트
function updateArduinoPortSelect(ports) {
    console.log("아두이노 포트 목록 업데이트:", ports);
    const portSelect = document.getElementById('arduino-port-select');
    if (!portSelect) {
        console.error("arduino-port-select 엘리먼트를 찾을 수 없습니다.");
        return;
    }
    
    // 현재 선택된 포트 저장
    const selectedPort = portSelect.value;
    
    // 포트 목록 초기화
    portSelect.innerHTML = '';
    
    if (ports && ports.length > 0) {
        // 이전에 연결된 포트
        const lastConnectedArduinoPort = localStorage.getItem('lastConnectedArduinoPort');
        console.log("마지막 연결 포트:", lastConnectedArduinoPort);
        
        // 포트 옵션 추가
        ports.forEach(port => {
            const option = document.createElement('option');
            option.value = port;
            option.textContent = port;
            
            // 선택 우선순위: 이전 선택 > 마지막 연결 > 첫 번째 포트
            if (port === selectedPort || 
                (!selectedPort && port === lastConnectedArduinoPort) || 
                (!selectedPort && !lastConnectedArduinoPort && port === ports[0])) {
                option.selected = true;
            }
            
            portSelect.appendChild(option);
        });
        
        // 연결 버튼 활성화
        const connectBtn = document.getElementById('connect-arduino-btn');
        if (connectBtn) connectBtn.disabled = false;
        
        const disconnectBtn = document.getElementById('disconnect-arduino-btn');
        if (disconnectBtn) disconnectBtn.disabled = true;
        
        addLog(`사용 가능한 아두이노 포트 ${ports.length}개를 찾았습니다: ${ports.join(', ')}`, 'success');
    } else {
        // 포트가 없는 경우
        const option = document.createElement('option');
        option.disabled = true;
        option.textContent = '사용 가능한 포트가 없습니다';
        portSelect.appendChild(option);
        
        // 연결 버튼 비활성화
        const connectBtn = document.getElementById('connect-arduino-btn');
        if (connectBtn) connectBtn.disabled = true;
        
        const disconnectBtn = document.getElementById('disconnect-arduino-btn');
        if (disconnectBtn) disconnectBtn.disabled = true;
        
        addLog('사용 가능한 아두이노 포트가 없습니다. 장치 연결 상태를 확인하세요.', 'warning');
    }
}

// 아두이노 상태 업데이트 함수
function updateArduinoStatus(connected, port) {
    const statusIndicator = document.getElementById('arduino-status-indicator');
    const statusText = document.getElementById('arduino-status-text');
    
    if (connected) {
        statusIndicator.classList.remove('off');
        statusIndicator.classList.add('on');
        statusText.textContent = `아두이노 센서: ${port || '연결됨'}`;
        addSensorLog('아두이노 센서가 연결되었습니다.', 'success');
    } else {
        statusIndicator.classList.remove('on');
        statusIndicator.classList.add('off');
        statusText.textContent = '아두이노 센서: 연결되지 않음';
        addSensorLog('아두이노 센서 연결이 해제되었습니다.', 'warning');
    }
}

// 아두이노 센서 데이터를 처리하고 sensorDataArray에 저장하는 함수
function handleArduinoSensorData(jsonData) {
    try {
        const data = JSON.parse(jsonData);

        // {"x":..,"y":..,"z":..} 형식을 표준 구조로 변환
        if (!data.accel && ['x', 'y', 'z'].every(axis => axis in data)) {
            const accelData = {
                x: Number(data.x) || 0,
                y: Number(data.y) || 0,
                z: Number(data.z) || 0
            };
            const gyroData = (typeof data.gyro === 'object' && data.gyro !== null) ? data.gyro : { x: 0, y: 0, z: 0 };
            data.accel = accelData;
            data.gyro = {
                x: Number(gyroData.x) || 0,
                y: Number(gyroData.y) || 0,
                z: Number(gyroData.z) || 0
            };
        }
        const now = new Date();
        const sensorData = {
            timestamp: now.toISOString(),
            processTime: totalSeconds, // 현재 가공 경과 시간을 센서 데이터와 함께 저장
            accX: data.accel?.x ?? null,
            accY: data.accel?.y ?? null,
            accZ: data.accel?.z ?? null,
            gyroX: data.gyro?.x ?? null,
            gyroY: data.gyro?.y ?? null,
            gyroZ: data.gyro?.z ?? null
        };
        sensorDataArray.push(sensorData);
        // 선택적으로 센서 로그 UI에도 표시 (자이로가 없는 고속 펌웨어는 '-')
        const fmt = value => (value === null || value === undefined) ? '-' : Number(value).toFixed(3);
        const logMessage = `[센서] ACC(X:${fmt(sensorData.accX)}, Y:${fmt(sensorData.accY)}, Z:${fmt(sensorData.accZ)}) GYRO(X:${fmt(sensorData.gyroX)}, Y:${fmt(sensorData.gyroY)}, Z:${fmt(sensorData.gyroZ)})`;
        addSensorLog(logMessage, 'sensor');
    } catch (e) {
        // JSON 파싱 실패 시 텍스트 형식 파싱 시도 (예: "x: 5319 y: 5824 z: 2801")
        try {
            const textMatch = jsonData.match(/x:\s*(\d+)\s+y:\s*(\d+)\s+z:\s*(\d+)/i);
            if (textMatch) {
                const x = parseFloat(textMatch[1]);
                const y = parseFloat(textMatch[2]);
                const z = parseFloat(textMatch[3]);
                const now = new Date();
                const sensorData = {
                    timestamp: now.toISOString(),
                    processTime: totalSeconds,
                    accX: x,
                    accY: y,
                    accZ: z,
                    gyroX: null,
                    gyroY: null,
                    gyroZ: null
                };
                sensorDataArray.push(sensorData);
                const logMessage = `[센서] x: ${x} y: ${y} z: ${z}`;
                addSensorLog(logMessage, 'sensor');
            } else {
                // 파싱할 수 없는 형식인 경우에만 오류 메시지 출력
                // 서버에서 이미 로그를 출력한 경우가 많으므로 조용히 무시
                console.warn('센서 데이터 파싱 실패 (무시):', jsonData.substring(0, 100));
            }
        } catch (parseError) {
            // 텍스트 파싱도 실패한 경우에만 오류 메시지 출력
            console.error('센서 데이터 파싱 오류:', parseError);
            // 서버에서 이미 로그를 출력했을 가능성이 높으므로 오류 메시지는 출력하지 않음
        }
    }
}

// =================== 센서 포트 제어 기능 ===================

// 센서 포트 목록 가져오기
function fetchSensorPorts() {
    console.log("사용 가능한 센서 포트 목록 요청 중...");
    
    addLog('사용 가능한 센서 포트 목록을 검색 중입니다...', 'info');
    
    // 소켓 연결 상태 확인
    if (!socket || !socket.connected) {
        addLog('서버 연결이 없어 센서 포트 목록을 가져올 수 없습니다.', 'error');
        return;
    }
    
    // 포트 목록 요청 - 일반 포트와 동일한 API 사용 (get_ports)
    socket.emit('get_ports', {}, function(response) {
        if (!response || !response.hasOwnProperty('ports')) {
            addLog('센서 포트 검색 중 오류: 서버 응답 형식이 잘못되었습니다.', 'error');
            return;
        }
        
        // 센서 포트 목록 업데이트
        updateSensorPortSelect(response.ports);
    });
}

// 센서 포트 선택 드롭다운 업데이트
function updateSensorPortSelect(ports) {
    const portSelect = document.getElementById('sensor-port-select');
    if (!portSelect) {
        console.error("sensor-port-select 엘리먼트를 찾을 수 없습니다.");
        return;
    }
    
    const selectedPort = portSelect.value;
    
    // 포트 목록 초기화
    portSelect.innerHTML = '';
    
    if (ports && ports.length > 0) {
        // 이전에 연결된 포트
        const lastConnectedSensorPort = localStorage.getItem('lastConnectedSensorPort');
        
        // 포트 옵션 추가
        ports.forEach(port => {
            const option = document.createElement('option');
            option.value = port;
            option.textContent = port;
            
            // 선택 우선순위: 이전 선택 > 마지막 연결 > 첫 번째 포트
            if (port === selectedPort || 
                (!selectedPort && port === lastConnectedSensorPort) || 
                (!selectedPort && !lastConnectedSensorPort && port === ports[0])) {
                option.selected = true;
            }
            
            portSelect.appendChild(option);
        });
        
        // 연결 버튼 활성화
        const connectBtn = document.getElementById('connect-sensor-btn');
        if (connectBtn) connectBtn.disabled = false;
        
        addLog(`사용 가능한 센서 포트 ${ports.length}개를 찾았습니다: ${ports.join(', ')}`, 'success');
    } else {
        // 포트가 없는 경우
        const option = document.createElement('option');
        option.disabled = true;
        option.textContent = '사용 가능한 포트가 없습니다';
        portSelect.appendChild(option);
        
        // 연결 버튼 비활성화
        const connectBtn = document.getElementById('connect-sensor-btn');
        if (connectBtn) connectBtn.disabled = true;
        
        addLog('사용 가능한 센서 포트가 없습니다. 장치 연결 상태를 확인하세요.', 'warning');
    }
}

// 센서 포트 연결 요청
function connectSensorPort() {
    const port = document.getElementById('sensor-port-select').value;
    if (!port) {
        showNotification('센서 포트를 선택해주세요', 'error');
        return;
    }
    
    // 연결 버튼 상태 변경
    const connectBtn = document.getElementById('connect-sensor-btn');
    connectBtn.textContent = '연결 중...';
    connectBtn.disabled = true;
    
    // 포트 상태 표시
    addLog(`센서 ${port} 연결 시도 중...`, 'info');
    
    // 통신 속도: 115200(기존 펌웨어) 또는 500000(고속 펌웨어 MPU6050_fast)
    const baudSelect = document.getElementById('sensor-baud-select');
    const baud = baudSelect && baudSelect.value ? Number(baudSelect.value) : 115200;

    // 연결 요청 - routes.py의 handle_connect_sensor 핸들러 호출
    socket.emit('connect_sensor', {port: port, baud: baud}, function(response) {
        if (response && response.status === 'success') {
            showNotification(response.message, 'success');

            // 상태 저장
            localStorage.setItem('lastConnectedSensorPort', port);
            localStorage.setItem('lastSensorBaud', String(baud));
        } else {
            const errorMsg = response ? response.message : '서버로부터 응답이 없습니다';
            showNotification(errorMsg, 'error');
            addLog(`센서 ${port} 연결 실패: ${errorMsg}`, 'error');
            
            // 버튼 상태 복원
            connectBtn.textContent = '연결';
            connectBtn.disabled = false;
        }
    });
}

// 센서 포트 연결 해제 요청
function disconnectSensorPort() {
    // 해제 중 상태 표시
    const disconnectBtn = document.getElementById('disconnect-sensor-btn');
    disconnectBtn.textContent = '해제 중...';
    disconnectBtn.disabled = true;
    
    addLog('센서 연결 해제 중...', 'info');
    
    // 해제 요청 - routes.py의 handle_disconnect_sensor 핸들러 호출
    socket.emit('disconnect_sensor', {}, function(response) {
        if (response && response.status === 'success') {
            // 연결 해제 성공
            showNotification(response.message || '센서 연결 해제 완료', 'success');
            
            // 버튼 상태 변경
            document.getElementById('connect-sensor-btn').style.display = 'inline-block';
            document.getElementById('connect-sensor-btn').disabled = false;
            document.getElementById('connect-sensor-btn').textContent = '연결';
            disconnectBtn.style.display = 'none';
            
            // 포트 선택 활성화
            document.getElementById('sensor-port-select').disabled = false;
        } else {
            // 연결 해제 실패
            const errorMsg = response ? response.message : '서버로부터 응답이 없습니다';
            showNotification(errorMsg, 'error');
            
            // 버튼 상태 복원
            disconnectBtn.textContent = '연결 해제';
            disconnectBtn.disabled = false;
        }
    });
}

// 알람 상태 확인 함수
function checkAlarmStatus() {
    // 컨트롤러가 연결된 상태일 때만 확인
    if (controllerConnected && socket) {
        // 알람 상태 확인 전 UI 업데이트
        const alarmLabel = document.getElementById('alarm-label');
        if (alarmLabel && alarmLabel.textContent === "연결 실패") {
            // 이미 연결 실패 상태면 확인 중단
            return;
        }
        
        // 상태 확인 요청 시간 기록
        const requestTime = Date.now();
        
        // 알람 상태 확인 요청
        socket.emit('check_alarm_status', {}, function(response) {
            // 응답 시간 측정 (너무 오래 걸리면 연결 문제일 수 있음)
            const responseTime = Date.now() - requestTime;
            
            if (response && response.status === 'success') {
                // 성공적인 응답 - 알람 상태 UI 업데이트
                updateAlarmStatus(response);
            } else {
                // 오류 응답 처리
                const errorMsg = response ? response.message : '서버로부터 응답이 없습니다';
                console.warn(`알람 상태 확인 오류: ${errorMsg}`);
                
                if (alarmLabel) {
                    if (responseTime > 3000) {  // 3초 이상 걸리면 통신 문제로 간주
                        alarmLabel.textContent = "통신 지연: 연결 상태 확인 필요";
                        alarmLabel.style.color = 'orange';
                        addLog("컨트롤러 응답 지연이 감지되었습니다. 연결 상태를 확인하세요.", 'warning');
                    } else {
                        alarmLabel.textContent = "상태 확인 오류";
                        alarmLabel.style.color = 'orange';
                    }
                }
                
                // 연결이 끊어진 것으로 보이면 재연결 유도
                if (!socket.connected || responseTime > 5000) {
                    updatePortStatus('disconnected');
                    controllerConnected = false;
                    
                    if (alarmLabel) {
                        alarmLabel.textContent = "연결 끊김: 재연결 필요";
                        alarmLabel.style.color = 'red';
                    }
                    
                    addLog("컨트롤러 연결이 끊어진 것으로 보입니다. 재연결이 필요합니다.", 'error');
                    showNotification('컨트롤러 연결이 끊어졌습니다. 재연결해주세요.', 'error');
                }
            }
        });
    }
}

// 알람 상태 UI 업데이트 함수
function updateAlarmStatus(data) {
    const alarmLabel = document.getElementById('alarm-label');
    if (!alarmLabel) return;

    const alarmFrame = document.querySelector('.alarm-section');
    const startBtn = document.getElementById('start-btn');
    const unlockBtn = document.getElementById('unlock-btn');
    const fileInput = document.getElementById('nc-file-input');
    const ncLoaded = fileInput && fileInput.files && fileInput.files.length > 0;

    if (data.alarm) {
        let alarmText = data.message || "알람 상태: 해제가 필요합니다";
        if (alarmText.length > 50) {
            alarmText = alarmText.substring(0, 47) + "...";
        }
        if (alarmText.includes("ALARM:")) {
            const alarmCode = alarmText.split(":")[1]?.trim();
            if (alarmCode) {
                switch(alarmCode) {
                    case "1": alarmText = "알람 상태: 하드 리미트 작동"; break;
                    case "2": alarmText = "알람 상태: G-코드 이동 범위 초과"; break;
                    case "3": alarmText = "알람 상태: 리셋 중 위치 오류"; break;
                    case "4": alarmText = "알람 상태: 프로브 초기 실패"; break;
                    case "5": alarmText = "알람 상태: 프로브 실패"; break;
                    case "6": alarmText = "알람 상태: 원점 복귀 작업 실패"; break;
                    case "7": alarmText = "알람 상태: 원점 복귀 안전 도어 열림"; break;
                    case "8": alarmText = "알람 상태: 원점 복귀 풀 사이클 필요"; break;
                    case "9": alarmText = "알람 상태: 원점 복귀 이동 오류"; break;
                    default: alarmText = `알람 상태: 코드 ${alarmCode}`;
                }
            }
        }
        alarmLabel.textContent = alarmText;
        alarmLabel.style.color = 'red';

        if (alarmFrame) {
            alarmFrame.style.backgroundColor = 'rgba(255, 0, 0, 0.1)';
            if (!alarmFrame.classList.contains('alarm-blink')) {
                alarmFrame.classList.add('alarm-blink');
            }
        }

        currentProcessingState = 'alarm';
        setTimeout(checkAlarmStatus, 5000);
    } else {
        if ((data.state === "alarm_unlocked" || data.state === "idle") && ncLoaded) {
            alarmLabel.textContent = "알람 없음: 가공 준비 완료";
            alarmLabel.style.color = 'green';
            currentProcessingState = 'idle';
        } else if (data.state === "running") {
            alarmLabel.textContent = "알람 없음: 가공 중";
            alarmLabel.style.color = 'green';
            currentProcessingState = 'running';
        } else if (data.state === "paused") {
            alarmLabel.textContent = "알람 없음: 일시정지 상태";
            alarmLabel.style.color = 'orange';
            currentProcessingState = 'paused';
        } else if (data.state === "emergency") {
            alarmLabel.textContent = "긴급정지 상태";
            alarmLabel.style.color = 'red';
            if (alarmFrame) {
                alarmFrame.style.backgroundColor = 'rgba(255, 0, 0, 0.1)';
                if (!alarmFrame.classList.contains('alarm-blink')) {
                    alarmFrame.classList.add('alarm-blink');
                }
            }
            currentProcessingState = 'emergency';
        } else {
            alarmLabel.textContent = "알람 없음";
            alarmLabel.style.color = 'green';
            currentProcessingState = 'idle';
        }
        if (alarmFrame && data.state !== "emergency") {
            alarmFrame.style.backgroundColor = '';
            alarmFrame.classList.remove('alarm-blink');
        }
    }
}

// 제어 프레임 버튼 이벤트 핸들러
function initControlButtons() {
    // 가공 시작 버튼
    const startButton = document.getElementById('start-button');
    if (startButton) {
        startButton.addEventListener('click', function() {
            if (currentProcessingState === 'running') {
                showNotification('이미 가공이 진행 중입니다.', 'warning');
                return;
            }
            startProcessingTimer();
            socket.emit('start_gcode', {}, function(response) {
                if (response.status === 'success') {
                    currentProcessingState = 'running';
                    showNotification(response.message, 'success');
                    addLog('가공을 시작합니다.', 'success');
                    if (response.estimated_time) {
                        estimatedProcessingTime = response.estimated_time.text;
                    }
                    updateTimeInfo();
                } else {
                    showNotification(response.message, 'error');
                    addLog(response.message, 'error');
                    stopProcessingTimer();
                }
            });
        });
    }

    // 일시정지 버튼
    const pauseButton = document.getElementById('pause-button');
    if (pauseButton) {
        pauseButton.addEventListener('click', function() {
            if (currentProcessingState !== 'running') {
                showNotification('가공 중이 아닐 때는 일시정지를 할 수 없습니다.', 'warning');
                return;
            }
            socket.emit('toggle_pause', {}, function(response) {
                if (response.status === 'success') {
                    currentProcessingState = 'paused';
                    showNotification(response.message, 'success');
                    addLog(response.message, 'success');
                    stopProcessingTimer();
                } else {
                    showNotification(response.message, 'error');
                    addLog(response.message, 'error');
                }
            });
        });
    }

    // 긴급정지 버튼
    const emergencyButton = document.getElementById('emergency-button');
    if (emergencyButton) {
        emergencyButton.addEventListener('click', function() {
            socket.emit('emergency_stop', {}, function(response) {
                if (response.status === 'success') {
                    currentProcessingState = 'emergency';
                    showNotification(response.message, 'success');
                    addLog(response.message, 'success');
                    stopProcessingTimer();
                    const emergencyBtn = document.getElementById('emergency-btn');
                    if (emergencyBtn && response.button_text) {
                        emergencyBtn.textContent = response.button_text;
                    }
                } else {
                    showNotification(response.message, 'error');
                    addLog(response.message, 'error');
                }
            });
        });
    }

    // 알람 해제 버튼
    const alarmButton = document.getElementById('alarm-button');
    if (alarmButton) {
        alarmButton.addEventListener('click', function() {
            socket.emit('unlock_alarm', {}, function(response) {
                if (response.status === 'success') {
                    showNotification(response.message, 'success');
                    addLog(response.message, 'success');
                    if (response.button_text) {
                        alarmButton.textContent = response.button_text;
                    }
                } else {
                    showNotification(response.message, 'error');
                    addLog(response.message, 'error');
                }
            });
        });
    }
}

// NC 파일 로드 비활성화 함수
function disableNCFileLoading() {
    // 파일 업로드 관련 요소들
    const fileInput = document.getElementById('nc-file');
    const fileLoadBtn = document.getElementById('load-file-btn');
    const fileDropZone = document.getElementById('file-drop-zone');
    
    // 요소 비활성화
    if (fileInput) fileInput.disabled = true;
    if (fileLoadBtn) fileLoadBtn.disabled = true;
    if (fileDropZone) {
        fileDropZone.classList.add('disabled');
        fileDropZone.draggable = false;
    }
    
    // 상태 표시
    if (fileDropZone) {
        const dropText = fileDropZone.querySelector('span');
        if (dropText) {
            dropText.textContent = "가공 중에는 파일을 로드할 수 없습니다";
        }
    }
}

// NC 파일 업로드 활성화 함수
function enableNCFileLoading() {
    // 파일 업로드 관련 요소들
    const fileInput = document.getElementById('nc-file');
    const fileLoadBtn = document.getElementById('load-file-btn');
    const fileDropZone = document.getElementById('file-drop-zone');
    
    // 요소 활성화
    if (fileInput) fileInput.disabled = false;
    if (fileLoadBtn) fileLoadBtn.disabled = false;
    if (fileDropZone) {
        fileDropZone.classList.remove('disabled');
        fileDropZone.draggable = true;
    }
    
    // 상태 표시 복원
    if (fileDropZone) {
        const dropText = fileDropZone.querySelector('span');
        if (dropText) {
            dropText.textContent = "NC 코드 파일 선택 또는 드래그 앤 드롭";
        }
    }
}