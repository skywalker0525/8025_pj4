const api = '';
const wsUrl = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;

const els = {
  connection: document.getElementById('connection'),
  missionState: document.getElementById('mission-state'),
  pointA: document.getElementById('point-a'),
  pointB: document.getElementById('point-b'),
  target: document.getElementById('target'),
  pose: document.getElementById('pose'),
  velocity: document.getElementById('velocity'),
  cmdVelocity: document.getElementById('cmd-velocity'),
  obstacle: document.getElementById('obstacle'),
  orbitProgress: document.getElementById('orbit-progress'),
  radiusError: document.getElementById('radius-error'),
  safetyPanel: document.getElementById('safety-panel'),
  warnings: document.getElementById('warnings'),
  recording: document.getElementById('recording'),
  videoPath: document.getElementById('video-path'),
  events: document.getElementById('events'),
  pan: document.getElementById('pan'),
  tilt: document.getElementById('tilt'),
  panValue: document.getElementById('pan-value'),
  tiltValue: document.getElementById('tilt-value'),
};

function fmt(value, digits = 2) {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--';
}

async function post(path, payload) {
  await fetch(`${api}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: payload ? JSON.stringify(payload) : undefined,
  });
}

function renderList(container, items, emptyText) {
  container.innerHTML = '';
  if (!items || items.length === 0) {
    const p = document.createElement('p');
    p.textContent = emptyText;
    container.appendChild(p);
    return;
  }
  items.forEach((item) => {
    const p = document.createElement('p');
    p.textContent = item;
    container.appendChild(p);
  });
}

function updateTelemetry(data) {
  const pose = data.pose || {};
  const velocity = data.velocity || {};
  const commandVelocity = data.command_velocity || {};
  const orbit = data.orbit || {};
  const video = data.video || {};
  const points = data.points || {};
  const target = data.target_object || {};
  const warnings = data.warnings || [];
  const events = data.events || [];
  const joints = data.joints || {};

  els.missionState.textContent = data.mission_state || 'WAITING';
  els.pointA.textContent = `A (${fmt(points.A?.x)}, ${fmt(points.A?.y)})`;
  els.pointB.textContent = `B (${fmt(points.B?.x)}, ${fmt(points.B?.y)})`;
  els.target.textContent = `Target (${fmt(target.x)}, ${fmt(target.y)})`;
  els.pose.textContent = `${fmt(pose.x)} / ${fmt(pose.y)} / ${fmt(pose.yaw)}`;
  els.velocity.textContent = `${fmt(velocity.linear)} m/s · ${fmt(velocity.angular)} rad/s`;
  els.cmdVelocity.textContent = `${fmt(commandVelocity.linear)} m/s · ${fmt(commandVelocity.angular)} rad/s`;
  els.obstacle.textContent = `${fmt(data.nearest_obstacle)} m`;
  els.orbitProgress.textContent = `${Math.round((orbit.progress || 0) * 100)}%`;
  els.radiusError.textContent = `${fmt(orbit.radius_error)} m`;
  els.recording.textContent = video.recording ? 'Yes' : 'No';
  els.videoPath.textContent = video.path || 'Not saved yet';

  if (typeof joints.camera_pan_joint === 'number') {
    els.pan.value = joints.camera_pan_joint;
    els.panValue.textContent = fmt(joints.camera_pan_joint);
  }
  if (typeof joints.camera_tilt_joint === 'number') {
    els.tilt.value = joints.camera_tilt_joint;
    els.tiltValue.textContent = fmt(joints.camera_tilt_joint);
  }

  els.safetyPanel.classList.toggle('danger', warnings.length > 0);
  els.safetyPanel.classList.toggle('ok', warnings.length === 0);
  renderList(els.warnings, warnings, 'All monitored constraints nominal.');
  renderList(els.events, events.slice(-10).reverse(), 'No events yet.');
}

function connect() {
  const socket = new WebSocket(wsUrl);
  socket.onopen = () => {
    els.connection.textContent = 'Bridge online';
    els.connection.classList.remove('offline');
    els.connection.classList.add('online');
  };
  socket.onclose = () => {
    els.connection.textContent = 'Bridge offline';
    els.connection.classList.remove('online');
    els.connection.classList.add('offline');
    setTimeout(connect, 1000);
  };
  socket.onmessage = (event) => {
    updateTelemetry(JSON.parse(event.data || '{}'));
  };
}

document.querySelectorAll('[data-command]').forEach((button) => {
  button.addEventListener('click', () => post(`/api/command/${button.dataset.command}`));
});

const driveCommands = {
  forward: [0.22, 0.0],
  reverse: [-0.18, 0.0],
  left: [0.0, 0.75],
  right: [0.0, -0.75],
  stop: [0.0, 0.0],
};

document.querySelectorAll('[data-drive]').forEach((button) => {
  const send = () => {
    const [linear, angular] = driveCommands[button.dataset.drive];
    post('/api/manual/cmd_vel', { linear, angular });
  };
  button.addEventListener('mousedown', send);
  button.addEventListener('touchstart', (event) => {
    event.preventDefault();
    send();
  });
  button.addEventListener('mouseup', () => post('/api/manual/cmd_vel', { linear: 0, angular: 0 }));
  button.addEventListener('mouseleave', () => post('/api/manual/cmd_vel', { linear: 0, angular: 0 }));
  button.addEventListener('touchend', () => post('/api/manual/cmd_vel', { linear: 0, angular: 0 }));
});

function sendGimbal() {
  const pan = Number(els.pan.value);
  const tilt = Number(els.tilt.value);
  els.panValue.textContent = fmt(pan);
  els.tiltValue.textContent = fmt(tilt);
  post('/api/manual/gimbal', { pan, tilt });
}

els.pan.addEventListener('input', sendGimbal);
els.tilt.addEventListener('input', sendGimbal);

connect();
