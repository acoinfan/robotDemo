const CM_TO_M = 0.01;
const APPROACH_HEIGHT_M = 0.1;
const GRIPPER_MAX_WIDTH_M = 0.08;

const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");
const taskSelect = document.getElementById("taskSelect");
const playButton = document.getElementById("playButton");
const resetButton = document.getElementById("resetButton");
const timeline = document.getElementById("timeline");

let tasks = [];
let current = null;
let progress = 0;
let playing = false;
let lastFrame = 0;

init().catch((error) => {
  console.error(error);
  drawMessage(`Failed to load demo data: ${error.message}`);
});

async function init() {
  if (window.DEMO_PAYLOAD) {
    initStandalone(window.DEMO_PAYLOAD);
    requestAnimationFrame(tick);
    return;
  }

  const index = await fetchJson("../data/tasks/index.json");
  tasks = index.tasks;
  for (const task of tasks) {
    const option = document.createElement("option");
    option.value = task.task;
    option.textContent = `${task.task}  ${task.variant}`;
    taskSelect.appendChild(option);
  }

  taskSelect.addEventListener("change", () => loadTask(taskSelect.value));
  playButton.addEventListener("click", togglePlay);
  resetButton.addEventListener("click", reset);
  timeline.addEventListener("input", () => {
    progress = Number(timeline.value) / 1000;
    playing = false;
    playButton.textContent = "Play";
    render();
  });
  window.addEventListener("resize", render);

  await loadTask(tasks[0].task);
  requestAnimationFrame(tick);
}

function initStandalone(payload) {
  tasks = [{ task: payload.task, variant: "standalone" }];
  const option = document.createElement("option");
  option.value = payload.task;
  option.textContent = payload.task;
  taskSelect.appendChild(option);
  taskSelect.disabled = true;

  playButton.addEventListener("click", togglePlay);
  resetButton.addEventListener("click", reset);
  timeline.addEventListener("input", () => {
    progress = Number(timeline.value) / 1000;
    playing = false;
    playButton.textContent = "Play";
    render();
  });
  window.addEventListener("resize", render);

  current = buildDemoState(tasks[0], payload.origin, payload.end);
  reset();
}

async function loadTask(taskName) {
  const meta = tasks.find((item) => item.task === taskName);
  const [origin, end] = await Promise.all([
    fetchJson(`../data/tasks/${taskName}/origin.json`),
    fetchJson(`../data/tasks/${taskName}/end.json`),
  ]);
  current = buildDemoState(meta, origin, end);
  reset();
}

function buildDemoState(meta, origin, end) {
  const moved = findMovedObject(origin, end);
  const originObj = moved.origin;
  const endObj = moved.end;
  const pick = cmToM(originObj.position);
  const place = cmToM(endObj.position);
  const poses = [
    addZ(pick, APPROACH_HEIGHT_M),
    pick,
    addZ(pick, APPROACH_HEIGHT_M),
    addZ(place, APPROACH_HEIGHT_M),
    place,
    addZ(place, APPROACH_HEIGHT_M),
  ];
  return {
    meta,
    origin,
    end,
    moved,
    poses,
    gripWidth: Math.min(Math.max(Math.min(originObj.size[0], originObj.size[1]) * CM_TO_M * 0.8, 0.005), GRIPPER_MAX_WIDTH_M),
  };
}

function findMovedObject(origin, end) {
  const endByUid = new Map(end.objects.map((object) => [uid(object), object]));
  const diffs = [];
  for (const object of origin.objects) {
    const target = endByUid.get(uid(object));
    if (!target) continue;
    const positionDelta = distance(object.position, target.position);
    const rotationDelta = quatAngle(object.rotation, target.rotation);
    diffs.push({
      score: positionDelta + rotationDelta * 10,
      positionDelta,
      rotationDelta,
      origin: object,
      end: target,
    });
  }
  diffs.sort((a, b) => b.score - a.score);
  return diffs[0];
}

function tick(timestamp) {
  if (!lastFrame) lastFrame = timestamp;
  const dt = Math.min(timestamp - lastFrame, 48);
  lastFrame = timestamp;

  if (playing) {
    progress += dt / 6500;
    if (progress >= 1) {
      progress = 1;
      playing = false;
      playButton.textContent = "Play";
    }
    timeline.value = String(Math.round(progress * 1000));
    render();
  }
  requestAnimationFrame(tick);
}

function togglePlay() {
  if (!current) return;
  if (progress >= 1) progress = 0;
  playing = !playing;
  playButton.textContent = playing ? "Pause" : "Play";
}

function reset() {
  progress = 0;
  playing = false;
  timeline.value = "0";
  playButton.textContent = "Play";
  render();
}

function render() {
  resizeCanvas();
  if (!current) {
    drawMessage("Loading...");
    return;
  }

  const bounds = current.origin.item_placement_zone;
  const view = tableView(bounds);
  drawBackground(view);
  drawObjects(current, view);
  drawPath(current, view);
  drawGripper(current, view);
  updateStats(current);
}

function drawBackground(view) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#d7dde3";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = "#f4f0e7";
  ctx.strokeStyle = "#596675";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.rect(view.x, view.y, view.w, view.h);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "#596675";
  ctx.font = "14px system-ui, sans-serif";
  ctx.fillText("table frame, cm", view.x, view.y - 12);
}

function drawObjects(state, view) {
  const movedUid = uid(state.moved.origin);
  const animated = animatedObjectPosition(state);

  for (const object of state.origin.objects) {
    if (uid(object) === movedUid) continue;
    drawObject(object, object.position, view, "#9aa8b6", "#697887", 0.9);
  }

  for (const object of state.end.objects) {
    if (uid(object) !== movedUid) continue;
    drawObject(object, object.position, view, "rgba(47,111,237,0.08)", "#2f6fed", 1, true);
  }

  drawObject(state.moved.origin, animated, view, "#f2b84b", "#9a6400", 1);
}

function drawObject(object, position, view, fill, stroke, alpha = 1, dashed = false) {
  const center = toCanvas(position, view);
  const width = Math.max(object.size[0] * view.scale, 6);
  const height = Math.max(object.size[1] * view.scale, 6);

  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.translate(center.x, center.y);
  ctx.rotate(-(object.z_rotation || 0));
  ctx.fillStyle = fill;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = dashed ? 2 : 1.5;
  ctx.setLineDash(dashed ? [7, 6] : []);
  ctx.beginPath();
  roundRect(-width / 2, -height / 2, width, height, 5);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawPath(state, view) {
  const points = state.poses.map((pose) => toCanvas(mToCm(pose), view));
  ctx.save();
  ctx.strokeStyle = "#2f6fed";
  ctx.lineWidth = 3;
  ctx.setLineDash([10, 8]);
  ctx.beginPath();
  for (let i = 0; i < points.length; i += 1) {
    if (i === 0) ctx.moveTo(points[i].x, points[i].y);
    else ctx.lineTo(points[i].x, points[i].y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawGripper(state, view) {
  const position = gripperPosition(state);
  const point = toCanvas(mToCm(position), view);
  const zCm = position[2] * 100;
  const open = progress < 0.18 || progress > 0.82;
  const span = open ? 22 : 12;

  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.strokeStyle = "#16202b";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(0, -18);
  ctx.lineTo(0, 18);
  ctx.moveTo(-span, -12);
  ctx.lineTo(0, -12);
  ctx.moveTo(-span, 12);
  ctx.lineTo(0, 12);
  ctx.moveTo(span, -12);
  ctx.lineTo(0, -12);
  ctx.moveTo(span, 12);
  ctx.lineTo(0, 12);
  ctx.stroke();

  ctx.fillStyle = "#16202b";
  ctx.font = "12px system-ui, sans-serif";
  ctx.fillText(`${zCm.toFixed(1)} cm`, 16, -18);
  ctx.restore();
}

function animatedObjectPosition(state) {
  if (progress < 0.22) return state.moved.origin.position;
  if (progress > 0.82) return state.moved.end.position;
  const gripper = gripperPosition(state);
  return mToCm(gripper);
}

function gripperPosition(state) {
  const segments = state.poses.length - 1;
  const scaled = progress * segments;
  const index = Math.min(Math.floor(scaled), segments - 1);
  const local = scaled - index;
  return lerpVec(state.poses[index], state.poses[index + 1], ease(local));
}

function updateStats(state) {
  document.getElementById("objectName").textContent = state.moved.origin.instance;
  document.getElementById("moveDelta").textContent = `${state.moved.positionDelta.toFixed(2)} cm`;
  document.getElementById("gripWidth").textContent = `${state.gripWidth.toFixed(3)} m`;
  document.getElementById("variantName").textContent = state.meta.variant;
}

function tableView(bounds) {
  const margin = Math.max(36, Math.min(canvas.width, canvas.height) * 0.08);
  const minX = bounds[0];
  const maxX = bounds[1];
  const minY = bounds[2];
  const maxY = bounds[3];
  const tableW = maxX - minX;
  const tableH = maxY - minY;
  const scale = Math.min((canvas.width - margin * 2) / tableW, (canvas.height - margin * 2) / tableH);
  const w = tableW * scale;
  const h = tableH * scale;
  return {
    minX,
    maxY,
    scale,
    x: (canvas.width - w) / 2,
    y: (canvas.height - h) / 2,
    w,
    h,
  };
}

function toCanvas(positionCm, view) {
  return {
    x: view.x + (positionCm[0] - view.minX) * view.scale,
    y: view.y + (view.maxY - positionCm[1]) * view.scale,
  };
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.round(rect.width));
  canvas.height = Math.max(1, Math.round(rect.height));
}

function drawMessage(message) {
  resizeCanvas();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#17202a";
  ctx.font = "18px system-ui, sans-serif";
  ctx.fillText(message, 32, 48);
}

async function fetchJson(path) {
  const response = await fetch(path);
  if (!response.ok) throw new Error(`${path}: ${response.status}`);
  return response.json();
}

function uid(object) {
  return object.selected_uid || object.retrieved_uid;
}

function cmToM(position) {
  return position.map((value) => value * CM_TO_M);
}

function mToCm(position) {
  return position.map((value) => value / CM_TO_M);
}

function addZ(position, dz) {
  return [position[0], position[1], position[2] + dz];
}

function distance(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function quatAngle(a, b) {
  const dot = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(Math.min(1, Math.max(-1, dot)));
}

function lerpVec(a, b, t) {
  return [
    a[0] + (b[0] - a[0]) * t,
    a[1] + (b[1] - a[1]) * t,
    a[2] + (b[2] - a[2]) * t,
  ];
}

function ease(t) {
  return t * t * (3 - 2 * t);
}

function roundRect(x, y, w, h, r) {
  const radius = Math.min(r, Math.abs(w) / 2, Math.abs(h) / 2);
  ctx.moveTo(x + radius, y);
  ctx.lineTo(x + w - radius, y);
  ctx.quadraticCurveTo(x + w, y, x + w, y + radius);
  ctx.lineTo(x + w, y + h - radius);
  ctx.quadraticCurveTo(x + w, y + h, x + w - radius, y + h);
  ctx.lineTo(x + radius, y + h);
  ctx.quadraticCurveTo(x, y + h, x, y + h - radius);
  ctx.lineTo(x, y + radius);
  ctx.quadraticCurveTo(x, y, x + radius, y);
}
