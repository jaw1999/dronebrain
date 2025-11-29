// DroneBrain Web UI v2 - Main Application

// ===== Tab Switching =====

function switchTab(tabName) {
    // Hide all tabs
    document.querySelectorAll('.tab-content').forEach(tab => {
        tab.classList.remove('active');
    });

    // Deactivate all tab buttons
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.remove('active');
    });

    // Show selected tab
    document.getElementById(`tab-${tabName}`).classList.add('active');

    // Activate clicked button
    event.target.classList.add('active');
}

// Global state
let ws = null;
let pcAnnotated = null;  // WebRTC peer connection for annotated stream
let pcRaw = null;         // WebRTC peer connection for raw stream
let reconnectInterval = null;

// Configuration
const WS_URL = `ws://${window.location.hostname}:${window.location.port}/ws/telemetry`;
const API_BASE = `/api`;
const WEBRTC_URL = `http://${window.location.hostname}:8889`;

// Initialize application
document.addEventListener('DOMContentLoaded', function() {
    console.log('DroneBrain Web UI initializing...');

    initVideoStreams();
    connectWebSocket();
});

// ===== WebSocket Connection =====

function connectWebSocket() {
    console.log('Connecting to WebSocket...');

    ws = new WebSocket(WS_URL);

    ws.onopen = function() {
        console.log('WebSocket connected');
        document.getElementById('connection-status').textContent = 'Connected';
        document.getElementById('connection-status').classList.remove('disconnected');
        document.getElementById('connection-status').classList.add('connected');

        if (reconnectInterval) {
            clearInterval(reconnectInterval);
            reconnectInterval = null;
        }
    };

    ws.onmessage = function(event) {
        const data = JSON.parse(event.data);

        if (data.type === 'telemetry') {
            updateTelemetry(data);
        }
    };

    ws.onerror = function(error) {
        console.error('WebSocket error:', error);
    };

    ws.onclose = function() {
        console.log('WebSocket disconnected');
        document.getElementById('connection-status').textContent = 'Disconnected';
        document.getElementById('connection-status').classList.remove('connected');
        document.getElementById('connection-status').classList.add('disconnected');

        // Attempt to reconnect
        if (!reconnectInterval) {
            reconnectInterval = setInterval(connectWebSocket, 5000);
        }
    };
}

// ===== Telemetry Updates =====

function updateTelemetry(data) {
    // System status
    document.getElementById('system-status').textContent = data.status || 'UNKNOWN';

    // Drone telemetry
    if (data.drone) {
        const drone = data.drone;

        // Basic status
        document.getElementById('drone-mode').textContent = drone.mode || '-';
        document.getElementById('drone-armed').textContent = drone.armed ? 'YES' : 'NO';
        document.getElementById('drone-battery').textContent =
            drone.battery_percent >= 0 ? `${drone.battery_percent}%` : '-';
        document.getElementById('drone-gps').textContent =
            drone.gps_fix_type > 0 ? `${drone.satellites_visible} sats` : 'No Fix';

        // Attitude
        document.getElementById('drone-roll').textContent =
            drone.roll !== undefined ? `${drone.roll.toFixed(1)}°` : '-';
        document.getElementById('drone-pitch').textContent =
            drone.pitch !== undefined ? `${drone.pitch.toFixed(1)}°` : '-';

        // Velocity
        if (drone.vx !== undefined && drone.vy !== undefined) {
            const groundSpeed = Math.sqrt(drone.vx * drone.vx + drone.vy * drone.vy);
            document.getElementById('drone-velocity').textContent =
                `${groundSpeed.toFixed(1)} m/s`;
        } else {
            document.getElementById('drone-velocity').textContent = '-';
        }

        document.getElementById('drone-vz').textContent =
            drone.vz !== undefined ? `${drone.vz.toFixed(1)} m/s` : '-';

        // Position info
        if (drone.latitude && drone.longitude) {
            // Individual lat/lon displays
            document.getElementById('drone-lat').textContent =
                drone.latitude.toFixed(6);
            document.getElementById('drone-lon').textContent =
                drone.longitude.toFixed(6);

            // Altitude displays
            document.getElementById('drone-altitude').textContent =
                `${drone.altitude_relative.toFixed(1)} m`;
            document.getElementById('drone-altitude-msl').textContent =
                `${drone.altitude.toFixed(1)} m`;

            // Heading
            document.getElementById('drone-heading').textContent =
                `${drone.yaw.toFixed(1)}°`;
        }
    }

    // Gimbal telemetry
    if (data.gimbal) {
        const gimbal = data.gimbal;

        document.getElementById('gimbal-yaw').textContent =
            gimbal.yaw !== null ? `${gimbal.yaw.toFixed(1)}°` : '-';
        document.getElementById('gimbal-pitch').textContent =
            gimbal.pitch !== null ? `${gimbal.pitch.toFixed(1)}°` : '-';
    }

    // Camera info
    if (data.camera) {
        const camera = data.camera;

        document.getElementById('camera-fps').textContent =
            camera.fps ? camera.fps.toFixed(1) : '0';
        document.getElementById('camera-resolution').textContent =
            camera.resolution || '-';
    }

    // Detection count
    document.getElementById('detection-count').textContent = data.detections || 0;

    // Update tracks list
    if (data.tracks && Array.isArray(data.tracks)) {
        updateTracksList(data.tracks);
    }

    // Tracking status
    if (data.tracking) {
        const tracking = data.tracking;

        document.getElementById('tracking-state').textContent = tracking.state || 'IDLE';

        if (tracking.target) {
            document.getElementById('tracking-target-info').style.display = 'block';
            document.getElementById('tracking-target').textContent =
                `ID ${tracking.target.track_id} (${tracking.target.class_name})`;

            // Display target location if available
            if (tracking.target.location) {
                const loc = tracking.target.location;
                document.getElementById('target-location-section').style.display = 'block';
                document.getElementById('target-lat').textContent = loc.latitude.toFixed(6);
                document.getElementById('target-lon').textContent = loc.longitude.toFixed(6);
                document.getElementById('target-distance').textContent =
                    loc.distance ? `${loc.distance.toFixed(1)} m` : '-';
            } else {
                document.getElementById('target-location-section').style.display = 'none';
            }
        } else {
            document.getElementById('tracking-target-info').style.display = 'none';
            document.getElementById('target-location-section').style.display = 'none';
        }
    }

    // TAK status
    if (data.tak) {
        const tak = data.tak;

        if (tak.enabled) {
            const statusText = tak.connected ? 'Connected' : 'Disconnected';
            document.getElementById('tak-status').textContent = statusText;
            document.getElementById('tak-messages').textContent = tak.messages_sent || 0;
            document.getElementById('tak-messages-row').style.display = 'flex';
        } else {
            document.getElementById('tak-status').textContent = 'Disabled';
            document.getElementById('tak-messages-row').style.display = 'none';
        }
    }

    // Autonomous mode status
    if (data.autonomous) {
        const autonomous = data.autonomous;
        const modeElement = document.getElementById('autonomous-mode');
        const loiterBtn = document.getElementById('btn-loiter');
        const followBtn = document.getElementById('btn-follow');
        const stopBtn = document.getElementById('btn-autonomous-stop');
        const requirementsInfo = document.getElementById('autonomous-requirements');
        const standoffSlider = document.getElementById('standoff-distance');
        const standoffValue = document.getElementById('standoff-distance-value');

        if (autonomous.available) {
            // Update mode display
            modeElement.textContent = autonomous.mode.toUpperCase();

            // Update standoff distance from server
            if (autonomous.standoff_distance && standoffSlider) {
                standoffSlider.value = autonomous.standoff_distance;
                standoffValue.textContent = `${autonomous.standoff_distance}m`;
            }

            // Update button states based on mode
            if (autonomous.enabled) {
                modeElement.classList.add('active');
                loiterBtn.style.display = 'none';
                followBtn.style.display = 'none';
                stopBtn.style.display = 'block';
                requirementsInfo.style.display = 'none';
            } else {
                modeElement.classList.remove('active');
                loiterBtn.style.display = 'block';
                followBtn.style.display = 'block';
                stopBtn.style.display = 'none';

                // Show requirements info if prerequisites not met
                const droneArmed = data.drone && data.drone.armed;
                const guidedMode = data.drone && data.drone.mode === 'GUIDED';
                const targetLocked = data.tracking && data.tracking.target;

                if (!droneArmed || !guidedMode || !targetLocked) {
                    requirementsInfo.style.display = 'block';
                    loiterBtn.disabled = true;
                    followBtn.disabled = true;
                } else {
                    requirementsInfo.style.display = 'none';
                    loiterBtn.disabled = false;
                    followBtn.disabled = false;
                }
            }
        } else {
            modeElement.textContent = 'NOT AVAILABLE';
            loiterBtn.disabled = true;
            followBtn.disabled = true;
        }
    }
}

// ===== Video Streaming (WebRTC) =====

async function createWebRTCConnection(streamName, videoElement) {
    console.log(`Creating WebRTC connection for ${streamName}...`);

    const pc = new RTCPeerConnection({
        iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
    });

    // Handle incoming tracks
    pc.ontrack = (event) => {
        console.log(`${streamName}: Received track`, event.track.kind);
        videoElement.srcObject = event.streams[0];
        videoElement.play().catch(e => console.error(`${streamName}: Play error:`, e));
        document.getElementById('video-status').textContent = 'Live';
    };

    // Handle connection state changes
    pc.onconnectionstatechange = () => {
        console.log(`${streamName}: Connection state:`, pc.connectionState);
        if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
            console.log(`${streamName}: Connection lost, reconnecting...`);
            setTimeout(() => initVideoStreams(), 3000);
        }
    };

    // Request offer from MediaMTX
    try {
        const response = await fetch(`${WEBRTC_URL}/${streamName}/whep`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/sdp'
            },
            body: await pc.createOffer({
                offerToReceiveVideo: true,
                offerToReceiveAudio: false
            }).then(offer => {
                pc.setLocalDescription(offer);
                return offer.sdp;
            })
        });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }

        const answerSdp = await response.text();
        await pc.setRemoteDescription(new RTCSessionDescription({
            type: 'answer',
            sdp: answerSdp
        }));

        console.log(`${streamName}: WebRTC connection established`);
        return pc;

    } catch (error) {
        console.error(`${streamName}: WebRTC connection failed:`, error);
        document.getElementById('video-status').textContent = `Error: ${error.message}`;
        pc.close();
        return null;
    }
}

async function initVideoStreams() {
    const videoAnnotated = document.getElementById('video-annotated');
    const videoRaw = document.getElementById('video-raw');

    console.log('Initializing WebRTC video streams...');

    // Close existing connections
    if (pcAnnotated) {
        pcAnnotated.close();
        pcAnnotated = null;
    }
    if (pcRaw) {
        pcRaw.close();
        pcRaw = null;
    }

    // Create WebRTC connections
    pcAnnotated = await createWebRTCConnection('annotated', videoAnnotated);
    pcRaw = await createWebRTCConnection('raw', videoRaw);

    if (!pcAnnotated && !pcRaw) {
        document.getElementById('video-status').textContent = 'Failed to connect';
    }
}

function switchVideo(stream) {
    const videoAnnotated = document.getElementById('video-annotated');
    const videoRaw = document.getElementById('video-raw');
    const buttons = document.querySelectorAll('.video-tab-btn');

    if (stream === 'annotated') {
        videoAnnotated.style.display = 'block';
        videoAnnotated.classList.add('active');
        videoRaw.style.display = 'none';
        videoRaw.classList.remove('active');

        buttons[0].classList.add('active');
        buttons[1].classList.remove('active');
    } else {
        videoRaw.style.display = 'block';
        videoRaw.classList.add('active');
        videoAnnotated.style.display = 'none';
        videoAnnotated.classList.remove('active');

        buttons[1].classList.add('active');
        buttons[0].classList.remove('active');
    }
}

// ===== Control Functions =====

async function centerGimbal() {
    try {
        const response = await fetch(`${API_BASE}/gimbal/center`, {
            method: 'POST'
        });

        const data = await response.json();

        if (data.success) {
            console.log('Gimbal centered');
        } else {
            console.error('Failed to center gimbal');
        }
    } catch (error) {
        console.error('Error centering gimbal:', error);
    }
}

async function startTracking() {
    const trackIdInput = document.getElementById('track-id-input');
    const trackId = parseInt(trackIdInput.value);

    if (!trackId || trackId < 1) {
        alert('Please enter a valid track ID');
        return;
    }

    try {
        const response = await fetch(`${API_BASE}/tracking/start/${trackId}`, {
            method: 'POST'
        });

        const data = await response.json();

        if (data.success) {
            console.log(`Started tracking target ${trackId}`);
            trackIdInput.value = '';
        } else {
            alert('Failed to start tracking');
        }
    } catch (error) {
        console.error('Error starting tracking:', error);
        alert('Error starting tracking');
    }
}

async function stopTracking() {
    try {
        const response = await fetch(`${API_BASE}/tracking/stop`, {
            method: 'POST'
        });

        const data = await response.json();

        if (data.success) {
            console.log('Stopped tracking');
        } else {
            alert('Failed to stop tracking');
        }
    } catch (error) {
        console.error('Error stopping tracking:', error);
        alert('Error stopping tracking');
    }
}

// ===== Autonomous Flight Control =====

// Initialize standoff distance slider
document.addEventListener('DOMContentLoaded', function() {
    const standoffSlider = document.getElementById('standoff-distance');
    const standoffValue = document.getElementById('standoff-distance-value');

    if (standoffSlider) {
        standoffSlider.addEventListener('input', function() {
            standoffValue.textContent = `${this.value}m`;
        });

        standoffSlider.addEventListener('change', async function() {
            await updateStandoffDistance(parseFloat(this.value));
        });
    }
});

async function updateStandoffDistance(distance) {
    try {
        const response = await fetch(`${API_BASE}/autonomous/standoff?distance=${distance}`, {
            method: 'POST'
        });

        const data = await response.json();

        if (data.success) {
            console.log(`Standoff distance updated to ${distance}m`);
        } else {
            console.error('Failed to update standoff distance');
        }
    } catch (error) {
        console.error('Error updating standoff distance:', error);
    }
}

async function enableLoiterMode() {
    try {
        const response = await fetch(`${API_BASE}/autonomous/loiter`, {
            method: 'POST'
        });

        if (!response.ok) {
            const error = await response.json();
            alert(`Failed to enable loiter mode:\n${error.detail}`);
            return;
        }

        const data = await response.json();

        if (data.success) {
            console.log('Loiter mode enabled');
        }
    } catch (error) {
        console.error('Error enabling loiter mode:', error);
        alert('Error enabling loiter mode');
    }
}

async function enableFollowMode() {
    try {
        const response = await fetch(`${API_BASE}/autonomous/follow`, {
            method: 'POST'
        });

        if (!response.ok) {
            const error = await response.json();
            alert(`Failed to enable follow mode:\n${error.detail}`);
            return;
        }

        const data = await response.json();

        if (data.success) {
            console.log('Follow mode enabled');
        }
    } catch (error) {
        console.error('Error enabling follow mode:', error);
        alert('Error enabling follow mode');
    }
}

async function disableAutonomousMode() {
    try {
        const response = await fetch(`${API_BASE}/autonomous/disable`, {
            method: 'POST'
        });

        const data = await response.json();

        if (data.success) {
            console.log('Autonomous mode disabled');
        } else {
            alert('Failed to disable autonomous mode');
        }
    } catch (error) {
        console.error('Error disabling autonomous mode:', error);
        alert('Error disabling autonomous mode');
    }
}

// ===== Configuration Management =====

function openConfigModal() {
    document.getElementById('config-modal').classList.add('active');
    loadConfig();
}

function closeConfigModal() {
    document.getElementById('config-modal').classList.remove('active');
}

// Close modal when clicking outside of it
window.onclick = function(event) {
    const modal = document.getElementById('config-modal');
    if (event.target === modal) {
        closeConfigModal();
    }
}


async function loadConfig() {
    try {
        const response = await fetch(`${API_BASE}/config`);

        if (!response.ok) {
            console.error('Failed to load configuration');
            return;
        }

        const config = await response.json();

        // Populate TAK settings
        if (config.tak) {
            document.getElementById('config-tak-enabled').checked = config.tak.enabled || false;
            document.getElementById('config-tak-ip').value = config.tak.server_ip || '';
            document.getElementById('config-tak-port').value = config.tak.server_port || 8087;
            document.getElementById('config-tak-callsign').value = config.tak.callsign || '';
        }

        // Populate detector settings
        if (config.vision && config.vision.detector) {
            document.getElementById('config-detection-model').value = config.vision.detector.model_size || 'm';
            document.getElementById('config-detection-confidence').value = config.vision.detector.confidence_threshold || 0.25;
        }

        // Populate tracking settings
        if (config.vision && config.vision.tracking) {
            const yawGains = config.vision.tracking.pid_yaw_gains || [1.5, 0.0, 0.1];
            const pitchGains = config.vision.tracking.pid_pitch_gains || [1.5, 0.0, 0.1];

            document.getElementById('config-tracking-yaw-kp').value = yawGains[0];
            document.getElementById('config-tracking-yaw-kd').value = yawGains[2];
            document.getElementById('config-tracking-pitch-kp').value = pitchGains[0];
            document.getElementById('config-tracking-pitch-kd').value = pitchGains[2];
            document.getElementById('config-tracking-max-speed').value = config.vision.tracking.max_gimbal_speed || 30.0;
        }

        console.log('Configuration loaded');
    } catch (error) {
        console.error('Error loading configuration:', error);
    }
}

async function saveConfig() {
    try {
        // Fetch current config first
        const response = await fetch(`${API_BASE}/config`);
        if (!response.ok) {
            alert('Failed to load current configuration');
            return;
        }

        const config = await response.json();

        // Update TAK settings
        config.tak = config.tak || {};
        config.tak.enabled = document.getElementById('config-tak-enabled').checked;
        config.tak.server_ip = document.getElementById('config-tak-ip').value;
        config.tak.server_port = parseInt(document.getElementById('config-tak-port').value);
        config.tak.protocol = 'udp';  // TAK is UDP only
        config.tak.callsign = document.getElementById('config-tak-callsign').value;

        // Update detector settings
        config.vision = config.vision || {};
        config.vision.detector = config.vision.detector || {};
        config.vision.detector.model_size = document.getElementById('config-detection-model').value;
        config.vision.detector.confidence_threshold = parseFloat(document.getElementById('config-detection-confidence').value);

        // Update tracking settings
        config.vision.tracking = config.vision.tracking || {};
        const yawKp = parseFloat(document.getElementById('config-tracking-yaw-kp').value);
        const yawKd = parseFloat(document.getElementById('config-tracking-yaw-kd').value);
        const pitchKp = parseFloat(document.getElementById('config-tracking-pitch-kp').value);
        const pitchKd = parseFloat(document.getElementById('config-tracking-pitch-kd').value);

        config.vision.tracking.pid_yaw_gains = [yawKp, 0.0, yawKd];
        config.vision.tracking.pid_pitch_gains = [pitchKp, 0.0, pitchKd];
        config.vision.tracking.max_gimbal_speed = parseFloat(document.getElementById('config-tracking-max-speed').value);

        // Save configuration
        const saveResponse = await fetch(`${API_BASE}/config`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(config)
        });

        const result = await saveResponse.json();

        if (result.success) {
            alert('Configuration saved successfully! Restart DroneBrain for changes to take effect.');
            closeConfigModal();
        } else {
            alert('Failed to save configuration');
        }
    } catch (error) {
        console.error('Error saving configuration:', error);
        alert('Error saving configuration');
    }
}

// ===== Tracks List Management =====

// Track elements cache to avoid recreating DOM elements
let trackElementsCache = {};

function updateTracksList(tracks) {
    const tracksList = document.getElementById('tracks-list');

    if (!tracks || tracks.length === 0) {
        tracksList.innerHTML = '<p class="tracks-empty">No active tracks</p>';
        trackElementsCache = {}; // Clear cache
        return;
    }

    // Track IDs in current update
    const currentTrackIds = new Set(tracks.map(t => t.track_id));

    // Remove tracks that no longer exist
    Object.keys(trackElementsCache).forEach(trackId => {
        if (!currentTrackIds.has(parseInt(trackId))) {
            const element = trackElementsCache[trackId];
            if (element && element.parentNode) {
                element.remove();
            }
            delete trackElementsCache[trackId];
        }
    });

    // Update or create track elements
    tracks.forEach(track => {
        const trackId = track.track_id;
        const hasGPS = track.latitude && track.longitude;
        const gpsClass = hasGPS ? 'track-gps-valid' : 'track-gps-invalid';

        let trackElement = trackElementsCache[trackId];

        if (!trackElement) {
            // Create new track element
            trackElement = document.createElement('div');
            trackElement.className = `track-item ${gpsClass}`;
            trackElement.onclick = () => trackTarget(trackId);
            tracksList.appendChild(trackElement);
            trackElementsCache[trackId] = trackElement;
        } else {
            // Update existing element's class
            trackElement.className = `track-item ${gpsClass}`;
        }

        // Build inner HTML
        let html = `
            <div class="track-header">
                <span class="track-id">ID: ${trackId}</span>
                <span class="track-class">${track.class_name}</span>
                <span class="track-confidence">${(track.confidence * 100).toFixed(0)}%</span>
            </div>
        `;

        if (hasGPS) {
            html += `
                <div class="track-location">
                    <div class="track-coords">
                        <span class="coords-label">GPS:</span>
                        <span class="coords-value">${track.latitude.toFixed(6)}, ${track.longitude.toFixed(6)}</span>
                    </div>
            `;

            if (track.distance) {
                html += `
                    <div class="track-distance">
                        <span class="distance-label">Dist:</span>
                        <span class="distance-value">${track.distance.toFixed(0)}m</span>
                    </div>
                `;
            }

            html += '</div>';
        } else {
            html += '<div class="track-no-gps">Calculating GPS...</div>';
        }

        trackElement.innerHTML = html;
    });
}

function trackTarget(trackId) {
    console.log(`Tracking target ID: ${trackId}`);
    document.getElementById('track-id-input').value = trackId;
    startTracking();
}

// ===== Camera Control Functions =====

// Search mode state
let searchModeActive = false;
let searchDirection = 1; // 1 for right, -1 for left

// Gimbal manual control
async function gimbalControl(direction) {
    const speed = parseInt(document.getElementById('gimbal-speed').value);
    let yaw_speed = 0;
    let pitch_speed = 0;

    switch(direction) {
        case 'up':
            pitch_speed = speed;
            break;
        case 'down':
            pitch_speed = -speed;
            break;
        case 'left':
            yaw_speed = -speed;
            break;
        case 'right':
            yaw_speed = speed;
            break;
    }

    try {
        const response = await fetch(`${API_BASE}/gimbal/move?yaw_speed=${yaw_speed}&pitch_speed=${pitch_speed}`, {
            method: 'POST'
        });
        const data = await response.json();
        if (!data.success) {
            console.error('Failed to move gimbal');
        }
    } catch (error) {
        console.error('Error moving gimbal:', error);
    }
}

// Stop gimbal movement
async function gimbalStop() {
    try {
        const response = await fetch(`${API_BASE}/gimbal/move?yaw_speed=0&pitch_speed=0`, {
            method: 'POST'
        });
    } catch (error) {
        console.error('Error stopping gimbal:', error);
    }
}

// Center gimbal
async function gimbalCenter() {
    try {
        const response = await fetch(`${API_BASE}/gimbal/center`, {
            method: 'POST'
        });
        const data = await response.json();
        if (data.success) {
            console.log('Gimbal centered');
        }
    } catch (error) {
        console.error('Error centering gimbal:', error);
    }
}

// Zoom control
async function zoomControl(direction) {
    const level = direction === 'in' ? 1 : -1;

    try {
        const response = await fetch(`${API_BASE}/gimbal/zoom?level=${level}`, {
            method: 'POST'
        });
    } catch (error) {
        console.error('Error controlling zoom:', error);
    }
}

// Stop zoom
async function zoomStop() {
    try {
        const response = await fetch(`${API_BASE}/gimbal/zoom?level=0`, {
            method: 'POST'
        });
    } catch (error) {
        console.error('Error stopping zoom:', error);
    }
}

// Toggle search mode
async function toggleSearchMode() {
    const btn = document.getElementById('search-mode-btn');

    searchModeActive = !searchModeActive;

    if (searchModeActive) {
        btn.textContent = 'Stop Search';
        btn.classList.add('active');
        searchDirection = 1; // Reset to start right
        startSearchPattern();
    } else {
        btn.textContent = 'Start Search';
        btn.classList.remove('active');
        await gimbalStop();
    }
}

// Search pattern - oscillating sweep within ±135° range
async function startSearchPattern() {
    if (!searchModeActive) return;

    const searchSpeed = 12; // Slow speed for sweep

    // Move gimbal in current direction
    await fetch(`${API_BASE}/gimbal/move?yaw_speed=${searchSpeed * searchDirection}&pitch_speed=0`, {
        method: 'POST'
    });

    // Reverse direction every 18 seconds (sweeps ~270° at 12°/s)
    setTimeout(() => {
        if (searchModeActive) {
            searchDirection *= -1;
            startSearchPattern();
        }
    }, 18000);
}

// ===== Utility Functions =====

// Initialize UI elements and event listeners
document.addEventListener('DOMContentLoaded', function() {
    // Gimbal speed slider
    const speedSlider = document.getElementById('gimbal-speed');
    const speedValue = document.getElementById('gimbal-speed-value');
    if (speedSlider && speedValue) {
        speedSlider.addEventListener('input', function() {
            speedValue.textContent = `${this.value}°/s`;
        });
    }

    // Track ID input - Enter key to start tracking
    const trackIdInput = document.getElementById('track-id-input');
    if (trackIdInput) {
        trackIdInput.addEventListener('keypress', function(event) {
            if (event.key === 'Enter') {
                startTracking();
            }
        });
    }
});
