// ESP32 Motor Control Web Interface
let autoUpdateInterval = null;
let isAutoUpdating = false;

function updateMotorData() {
    fetch('/motor/status')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                document.getElementById('speed').textContent = data.speed || '--';
                document.getElementById('direction').textContent = data.direction || '--';
                document.getElementById('pulses').textContent = data.pulses || '0';
                document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();

                // Update status
                updateMotorStatus(data);
            } else {
                document.getElementById('speed').textContent = 'Error';
                document.getElementById('direction').textContent = 'Error';
                document.getElementById('status').textContent = 'Motor Error';
                document.getElementById('status').className = 'status error';
            }
        })
        .catch(error => {
            console.error('Error fetching motor data:', error);
            document.getElementById('speed').textContent = 'Error';
            document.getElementById('direction').textContent = 'Error';
            document.getElementById('status').textContent = 'Connection Error';
            document.getElementById('status').className = 'status error';
        });
}

function updateMotorStatus(data) {
    const statusElement = document.getElementById('status');

    if (data.running) {
        statusElement.textContent = `Motor Running - ${data.direction || 'Unknown'} at ${data.speed || 0}%`;
        statusElement.className = 'status active';
    } else {
        statusElement.textContent = 'Motor Stopped';
        statusElement.className = 'status stopped';
    }
}

function startMotor() {
    const speed = document.getElementById('speedInput').value;
    fetch(`/motor/start?speed=${speed}`)
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                updateMotorData();
            } else {
                alert('Failed to start motor');
            }
        })
        .catch(error => {
            console.error('Error starting motor:', error);
            alert('Error starting motor');
        });
}

function stopMotor() {
    fetch('/motor/stop')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                updateMotorData();
            } else {
                alert('Failed to stop motor');
            }
        })
        .catch(error => {
            console.error('Error stopping motor:', error);
            alert('Error stopping motor');
        });
}

function setDirection(direction) {
    fetch(`/motor/direction?dir=${direction}`)
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                updateMotorData();
            } else {
                alert('Failed to set direction');
            }
        })
        .catch(error => {
            console.error('Error setting direction:', error);
            alert('Error setting direction');
        });
}

function resetEncoder() {
    fetch('/motor/reset')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                document.getElementById('pulses').textContent = '0';
                updateMotorData();
            } else {
                alert('Failed to reset encoder');
            }
        })
        .catch(error => {
            console.error('Error resetting encoder:', error);
            alert('Error resetting encoder');
        });
}

function toggleAutoUpdate() {
    const buttonText = document.getElementById('autoUpdateText');

    if (isAutoUpdating) {
        // Stop auto update
        clearInterval(autoUpdateInterval);
        autoUpdateInterval = null;
        isAutoUpdating = false;
        buttonText.textContent = 'Start Auto Update';
    } else {
        // Start auto update
        autoUpdateInterval = setInterval(updateMotorData, 500); // Update every 500ms
        isAutoUpdating = true;
        buttonText.textContent = 'Stop Auto Update';
        updateMotorData(); // Update immediately
    }
}

function brakeMotor() {
    fetch('/motor/brake')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                updateMotorData();
            } else {
                alert('Failed to brake motor');
            }
        })
        .catch(error => {
            console.error('Error braking motor:', error);
            alert('Error braking motor');
        });
}

// Initialize the page
document.addEventListener('DOMContentLoaded', function() {
    console.log('ESP32 Motor Control interface loaded');

    // Get initial motor data
    updateMotorData();

    // Set up keyboard shortcuts
    document.addEventListener('keydown', function(event) {
        if (event.key === 's' || event.key === 'S') {
            startMotor();
        } else if (event.key === 'x' || event.key === 'X') {
            stopMotor();
        } else if (event.key === 'b' || event.key === 'B') {
            brakeMotor();
        } else if (event.key === 'r' || event.key === 'R') {
            resetEncoder();
        } else if (event.key === 'a' || event.key === 'A') {
            toggleAutoUpdate();
        } else if (event.key === 'f' || event.key === 'F') {
            setDirection('forward');
        } else if (event.key === 'v' || event.key === 'V') {
            setDirection('reverse');
        } else if (event.key === 'Escape') {
            if (isAutoUpdating) {
                toggleAutoUpdate();
            }
        }
    });

    // Add some visual feedback
    console.log('Motor Control Keyboard shortcuts:');
    console.log('S - Start motor');
    console.log('X - Stop motor');
    console.log('B - Brake motor');
    console.log('R - Reset encoder');
    console.log('F - Forward direction');
    console.log('V - Reverse direction');
    console.log('A - Toggle auto update');
    console.log('ESC - Stop auto update');
});