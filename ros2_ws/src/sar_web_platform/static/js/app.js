// Global variables
let isConnected = false;
let statusUpdateInterval = null;

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    updateTime();
    setInterval(updateTime, 1000);
    
    // Start status updates
    startStatusUpdates();
    
    // Setup NLP input enter key
    document.getElementById('nlp-command').addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            sendNLPCommand();
        }
    });
});

// Update current time
function updateTime() {
    const now = new Date();
    document.getElementById('current-time').textContent = now.toLocaleTimeString();
}

// Start periodic status updates
function startStatusUpdates() {
    updateStatus();
    statusUpdateInterval = setInterval(updateStatus, 500); // Update every 500ms
}

// Fetch and update drone status
async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        // Update connection status
        if (!isConnected) {
            isConnected = true;
            document.getElementById('connection-status').textContent = 'Connected';
            addLog('Connected to drone system', 'success');
        }
        
        // Update drone status
        document.getElementById('armed-status').textContent = data.armed ? 'YES' : 'NO';
        document.getElementById('armed-status').style.color = data.armed ? '#00ff88' : '#ff3366';
        
        document.getElementById('mode-status').textContent = data.mode;
        document.getElementById('battery-status').textContent = data.battery + '%';
        document.getElementById('gps-status').textContent = data.gps_fix ? 'OK' : 'NO FIX';
        document.getElementById('gps-status').style.color = data.gps_fix ? '#00ff88' : '#ff3366';
        
        // Update position
        document.getElementById('pos-x').textContent = data.position.x;
        document.getElementById('pos-y').textContent = data.position.y;
        document.getElementById('pos-z').textContent = data.position.z;
        
        // Update home position
        if (data.home_set) {
            document.getElementById('home-status').textContent = 
                `(${data.home_position.x}, ${data.home_position.y}, ${data.home_position.z})`;
        }
        
        // Update mission status
        document.getElementById('mission-status').textContent = data.mission_status;
        
        // Update active controller
        document.getElementById('controller-source').textContent = data.velocity_source;
        
        // Update detection count
        if (data.detections && data.detections.length > 0) {
            document.getElementById('detection-count').textContent = 
                `${data.detections.length} object(s) detected`;
        } else {
            document.getElementById('detection-count').textContent = 'No detections';
        }
        
        // Color code mission status
        const missionElement = document.getElementById('mission-status');
        switch(data.mission_status) {
            case 'SEARCHING':
                missionElement.style.color = '#00d9ff';
                break;
            case 'RETURNING':
                missionElement.style.color = '#ffaa00';
                break;
            case 'EMERGENCY':
                missionElement.style.color = '#ff3366';
                break;
            default:
                missionElement.style.color = '#e0e0e0';
        }
        
    } catch (error) {
        if (isConnected) {
            isConnected = false;
            document.getElementById('connection-status').textContent = 'Disconnected';
            addLog('Lost connection to drone system', 'error');
        }
    }
}

// Send command to drone
async function sendCommand(command) {
    try {
        const response = await fetch('/api/command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ command: command })
        });
        
        const result = await response.json();
        
        if (result.success) {
            addLog(result.message, 'success');
        } else {
            addLog('Error: ' + result.message, 'error');
        }
        
    } catch (error) {
        addLog('Failed to send command: ' + error.message, 'error');
    }
}

// Send natural language command
function sendNLPCommand() {
    const input = document.getElementById('nlp-command');
    const command = input.value.trim();
    
    if (!command) return;
    
    sendCommand('nlp:' + command);
    addLog('NLP: ' + command, 'info');
    
    // Clear input
    input.value = '';
}

// Add log entry
function addLog(message, type = 'info') {
    const logContainer = document.getElementById('log-container');
    const entry = document.createElement('div');
    entry.className = 'log-entry ' + type;
    
    const timestamp = new Date().toLocaleTimeString();
    entry.textContent = `[${timestamp}] ${message}`;
    
    logContainer.appendChild(entry);
    
    // Auto-scroll to bottom
    logContainer.scrollTop = logContainer.scrollHeight;
    
    // Limit log entries to 100
    while (logContainer.children.length > 100) {
        logContainer.removeChild(logContainer.firstChild);
    }
}

// Keyboard shortcuts
document.addEventListener('keydown', function(e) {
    // Ctrl+E for emergency stop
    if (e.ctrlKey && e.key === 'e') {
        e.preventDefault();
        sendCommand('emergency_stop');
    }
    
    // Arrow keys for manual control (when not in input field)
    if (document.activeElement.tagName !== 'INPUT') {
        switch(e.key) {
            case 'ArrowUp':
                e.preventDefault();
                sendCommand('move_forward');
                break;
            case 'ArrowDown':
                e.preventDefault();
                sendCommand('move_back');
                break;
            case 'ArrowLeft':
                e.preventDefault();
                sendCommand('move_left');
                break;
            case 'ArrowRight':
                e.preventDefault();
                sendCommand('move_right');
                break;
            case ' ':
                e.preventDefault();
                sendCommand('move_stop');
                break;
        }
    }
});

// Add initial log
addLog('SAR Platform initialized', 'success');
addLog('Press Ctrl+E for emergency stop', 'warning');
addLog('Use arrow keys for manual control', 'info');