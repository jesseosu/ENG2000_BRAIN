// ESP32 Ultrasonic Sensor Web Interface
let autoUpdateInterval = null;
let isAutoUpdating = false;

function updateDistance() {
    fetch('/distance')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                document.getElementById('distance').textContent = data.distance.toFixed(1);
                document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();

                // Update status based on distance
                updateStatus(data.distance);
            } else {
                document.getElementById('distance').textContent = 'Error';
                document.getElementById('status').textContent = 'Sensor Error';
                document.getElementById('status').className = 'status error';
            }
        })
        .catch(error => {
            console.error('Error fetching distance:', error);
            document.getElementById('distance').textContent = 'Error';
            document.getElementById('status').textContent = 'Connection Error';
            document.getElementById('status').className = 'status error';
        });
}

function updateStatus(distance) {
    const statusElement = document.getElementById('status');

    if (distance < 0) {
        statusElement.textContent = 'Sensor Error';
        statusElement.className = 'status error';
    } else if (distance < 10) {
        statusElement.textContent = 'Object Very Close!';
        statusElement.className = 'status warning';
    } else if (distance > 400) {
        statusElement.textContent = 'Out of Range';
        statusElement.className = 'status warning';
    } else {
        statusElement.textContent = 'Sensor Active';
        statusElement.className = 'status normal';
    }
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
        autoUpdateInterval = setInterval(updateDistance, 1000); // Update every second
        isAutoUpdating = true;
        buttonText.textContent = 'Stop Auto Update';
        updateDistance(); // Update immediately
    }
}

// Initialize the page
document.addEventListener('DOMContentLoaded', function() {
    console.log('ESP32 Ultrasonic Sensor interface loaded');

    // Get initial reading
    updateDistance();

    // Set up keyboard shortcuts
    document.addEventListener('keydown', function(event) {
        if (event.key === 'r' || event.key === 'R') {
            updateDistance();
        } else if (event.key === 'a' || event.key === 'A') {
            toggleAutoUpdate();
        } else if (event.key === 'Escape') {
            if (isAutoUpdating) {
                toggleAutoUpdate();
            }
        }
    });

    // Add some visual feedback
    console.log('Keyboard shortcuts:');
    console.log('R - Refresh distance');
    console.log('A - Toggle auto update');
    console.log('ESC - Stop auto update');
});