// Mission Planner GUI
// Vanilla JS, no framework/build step -- this needs to run on a laptop at
// poolside with no reliable internet.

const state = {
  positions: {},     // named waypoint -> [x,y,z,roll,pitch,yaw]
  positionFiles: [], // positions file names on disk (without .json)
  positionsFile: "", // currently active positions file, "" if unsaved/new
  steps: [],         // current mission's steps, in order
  selected: null,    // index into steps, or null
  missionName: "",
  view: "sequence", // "sequence" | "positions"
};

const el = (id) => document.getElementById(id);

const toast = el("toast");
function showToast(message, isError = false) {
  toast.textContent = message;
  toast.classList.toggle("error", isError);
  toast.classList.add("show");
  clearTimeout(showToast._t);
  showToast._t = setTimeout(() => toast.classList.remove("show"), 3200);
}

// ---------- API helpers ----------

async function api(path, options) {
  const res = await fetch(path, options);
  let body = null;
  try { body = await res.json(); } catch (_) { /* no body */ }
  if (!res.ok) {
    const msg = (body && body.description) || res.statusText || "Request failed";
    throw new Error(msg);
  }
  return body;
}

// ---------- default step shapes ----------

function defaultMoveToStep() {
  return {
    type: "move_to",
    label: "New waypoint",
    reference: "start",
    offset: [0, 0, 0, 0, 0, 0],
    target_pose: [0, 0, 0, 0, 0, 0],
    target_velocity: [0, 0, 0, 0, 0, 0],
  };
}

function defaultActuateStep() {
  return {
    type: "actuate",
    label: "New actuation",
    mechanism: "dropper",
    setpoint: "RESET",
  };
}

function defaultToggleWatchdogStep() {
  return {
    type: "toggle_watchdog",
    label: "Toggle watchdog",
    enabled: true,
  };
}

function defaultWaitStep() {
  return {
    type: "wait",
    label: "Wait",
    seconds: 5,
  };
}

function defaultCliStep() {
  return {
    type: "cli",
    label: "CLI command",
    command: "",
  };
}

function defaultCliStopStep() {
  return {
    type: "cli_stop",
    label: "Stop CLI",
    target: "",
  };
}

// ---------- rendering: step list ----------

function stepDetailText(step) {
  if (step.type === "move_to") {
    const p = step.target_pose.map((v) => Number(v).toFixed(2)).join(", ");
    return `move_to  (${p})`;
  }
  if (step.type === "toggle_watchdog") {
    return `toggle_watchdog  ${step.enabled ? "ENABLE" : "DISABLE"}`;
  }
  if (step.type === "wait") {
    return `wait  ${step.seconds}s`;
  }
  if (step.type === "cli") {
    return `cli  ${step.command || "—"}`;
  }
  if (step.type === "cli_stop") {
    return `cli_stop  ${step.target || "—"}`;
  }
  return `actuate  ${step.mechanism || "—"} / ${step.setpoint || "—"}`;
}

function renderStepList() {
  const list = el("step-list");
  list.innerHTML = "";

  state.steps.forEach((step, i) => {
    const li = document.createElement("li");
    li.className = "step-item" + (state.selected === i ? " selected" : "");

    li.innerHTML = `
      <div class="step-node">${i + 1}</div>
      <div class="step-body">
        <div class="step-label">${escapeHtml(step.label || step.type)}</div>
        <div class="step-detail">${escapeHtml(stepDetailText(step))}</div>
      </div>
      <div class="step-reorder">
        <button type="button" class="reorder-btn" data-action="up" data-i="${i}" title="Move up">▲</button>
        <button type="button" class="reorder-btn" data-action="down" data-i="${i}" title="Move down">▼</button>
      </div>
    `;

    li.addEventListener("click", (e) => {
      if (e.target.closest(".reorder-btn")) return;
      selectStep(i);
    });

    list.appendChild(li);
  });

  el("step-count").textContent = `${state.steps.length} step${state.steps.length === 1 ? "" : "s"}`;

  list.querySelectorAll(".reorder-btn").forEach((btn) => {
    btn.addEventListener("click", (e) => {
      e.stopPropagation();
      const i = Number(btn.dataset.i);
      const dir = btn.dataset.action === "up" ? -1 : 1;
      const j = i + dir;
      if (j < 0 || j >= state.steps.length) return;
      [state.steps[i], state.steps[j]] = [state.steps[j], state.steps[i]];
      if (state.selected === i) state.selected = j;
      else if (state.selected === j) state.selected = i;
      renderStepList();
    });
  });
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  }[c]));
}

// ---------- editor ----------

const STEP_HEADINGS = {
  move_to: "Move To",
  actuate: "Actuate",
  toggle_watchdog: "Toggle Watchdog",
  wait: "Wait",
  cli: "CLI",
  cli_stop: "Stop CLI",
};

function selectStep(i) {
  switchView("sequence");
  state.selected = i;
  renderStepList();
  showEditor(state.steps[i]);
}

function showEditor(step) {
  el("editor-empty").classList.add("hidden");
  el("editor-form").classList.remove("hidden");

  el("edit-index").value = state.selected;
  el("editor-heading").textContent = `Step ${state.selected + 1} \u00b7 ${STEP_HEADINGS[step.type] || step.type}`;
  el("f-label").value = step.label || "";

  el("fields-move_to").classList.toggle("hidden", step.type !== "move_to");
  el("fields-actuate").classList.toggle("hidden", step.type !== "actuate");
  el("fields-toggle_watchdog").classList.toggle("hidden", step.type !== "toggle_watchdog");
  el("fields-wait").classList.toggle("hidden", step.type !== "wait");
  el("fields-cli").classList.toggle("hidden", step.type !== "cli");
  el("fields-cli_stop").classList.toggle("hidden", step.type !== "cli_stop");

  if (step.type === "move_to") {
    el("f-reference").value = step.reference || "start";
    const [x, y, z, roll, pitch, yaw] = step.offset || [0, 0, 0, 0, 0, 0];
    el("f-x").value = x; el("f-y").value = y; el("f-z").value = z;
    el("f-roll").value = roll; el("f-pitch").value = pitch; el("f-yaw").value = yaw;

    const [vx, vy, vz, vroll, vpitch, vyaw] = step.target_velocity || [0, 0, 0, 0, 0, 0];
    el("f-vx").value = vx; el("f-vy").value = vy; el("f-vz").value = vz;
    el("f-vroll").value = vroll; el("f-vpitch").value = vpitch; el("f-vyaw").value = vyaw;

    updatePoseLabelAndResolved();
  } else if (step.type === "actuate") {
    el("f-mechanism").value = step.mechanism || "dropper";
    el("f-setpoint").value = step.setpoint || "RESET";
  } else if (step.type === "toggle_watchdog") {
    el("f-watchdog-enabled").value = step.enabled ? "true" : "false";
  } else if (step.type === "wait") {
    el("f-wait-seconds").value = step.seconds ?? 5;
  } else if (step.type === "cli") {
    el("f-cli-command").value = step.command || "";
  } else if (step.type === "cli_stop") {
    populateCliStopTargets(step);
    el("f-cli-stop-target").value = step.target || "";
  }
}

// The dropdown only offers labels of cli steps *earlier* in the sequence --
// stopping something that hasn't been launched yet makes no sense. If the
// step's saved target no longer matches any of those (the cli step was
// removed/relabeled/moved), it's kept as an explicit "(missing)" entry
// instead of being silently swapped for something else; save will reject it.
function populateCliStopTargets(step) {
  const select = el("f-cli-stop-target");
  select.innerHTML = "";

  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = "— pick a CLI step —";
  select.appendChild(placeholder);

  const seen = new Set();
  state.steps.slice(0, state.selected).forEach((s) => {
    if (s.type !== "cli") return;
    const label = s.label || s.type;
    if (seen.has(label)) return;
    seen.add(label);
    const opt = document.createElement("option");
    opt.value = label;
    opt.textContent = label;
    select.appendChild(opt);
  });

  if (step.target && !seen.has(step.target)) {
    const opt = document.createElement("option");
    opt.value = step.target;
    opt.textContent = `${step.target} (missing)`;
    select.appendChild(opt);
  }
}

// ---------- pose composition (mirrors compose_pose() in mission_planner.py) ----------
// A move_to's offset is expressed in the *reference task's* FLU frame, not
// world frame -- e.g. offset (-1,0,0) means "1m behind the task, along
// whichever way it's facing," not "1m in world -X". So resolving the
// preview pose needs the same quaternion rotation the robot does at
// runtime, not a plain addition.

function eulerToQuat(roll, pitch, yaw) {
  const cy = Math.cos(yaw * 0.5), sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5), sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5), sr = Math.sin(roll * 0.5);
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

function quatToEuler(w, x, y, z) {
  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(sinp);

  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  return [roll, pitch, yaw];
}

function quatMultiply(q1, q2) {
  const [w1, x1, y1, z1] = q1;
  const [w2, x2, y2, z2] = q2;
  return [
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  ];
}

function quatConjugate(q) {
  const [w, x, y, z] = q;
  return [w, -x, -y, -z];
}

function rotateVectorByQuat(q, v) {
  const qv = [0, v[0], v[1], v[2]];
  const r = quatMultiply(quatMultiply(q, qv), quatConjugate(q));
  return [r[1], r[2], r[3]];
}

function composePose(taskPose, relativePose) {
  const [tx, ty, tz, troll, tpitch, tyaw] = taskPose;
  const [rx, ry, rz, rroll, rpitch, ryaw] = relativePose;

  const qTask = eulerToQuat(troll, tpitch, tyaw);
  const qRel = eulerToQuat(rroll, rpitch, ryaw);

  const relWorld = rotateVectorByQuat(qTask, [rx, ry, rz]);
  const globalPos = [tx + relWorld[0], ty + relWorld[1], tz + relWorld[2]];
  const globalQuat = quatMultiply(qTask, qRel);

  return [globalPos, globalQuat];
}

function currentOffsetFields() {
  return [
    parseFloat(el("f-x").value) || 0,
    parseFloat(el("f-y").value) || 0,
    parseFloat(el("f-z").value) || 0,
    parseFloat(el("f-roll").value) || 0,
    parseFloat(el("f-pitch").value) || 0,
    parseFloat(el("f-yaw").value) || 0,
  ];
}

function resolveAbsolutePose() {
  const ref = el("f-reference").value || "start";
  const offset = currentOffsetFields();
  const taskPose = state.positions[ref] || [0, 0, 0, 0, 0, 0];
  const [globalPos, globalQuat] = composePose(taskPose, offset);
  const [roll, pitch, yaw] = quatToEuler(...globalQuat);
  return [...globalPos, roll, pitch, yaw];
}

function updatePoseLabelAndResolved() {
  el("pose-offset-label").textContent = "Offset from reference (task frame, m / rad)";
  const resolved = resolveAbsolutePose();
  el("resolved-pose-value").textContent = resolved.map((v) => v.toFixed(3)).join(", ");
}

// ---------- populate reference/mechanism dropdowns ----------

function populateReferenceOptions() {
  const select = el("f-reference");
  const previous = select.value;
  select.innerHTML = "";
  Object.keys(state.positions).forEach((name) => {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name.toUpperCase();
    select.appendChild(opt);
  });
  if (Object.prototype.hasOwnProperty.call(state.positions, previous)) {
    select.value = previous;
  } else if (Object.prototype.hasOwnProperty.call(state.positions, "start")) {
    select.value = "start";
  }
}

// ---------- view switching (Sequence <-> Positions) ----------

function switchView(view) {
  state.view = view;

  el("nav-positions").classList.toggle("active", view === "positions");

  el("sequence-toolbar").classList.toggle("hidden", view === "positions");
  el("positions-toolbar").classList.toggle("hidden", view !== "positions");

  el("positions-editor").classList.toggle("hidden", view !== "positions");

  if (view === "positions") {
    el("editor-empty").classList.add("hidden");
    el("editor-form").classList.add("hidden");
    renderPositionsList();
  } else if (state.selected !== null && state.steps[state.selected]) {
    el("editor-empty").classList.add("hidden");
    el("editor-form").classList.remove("hidden");
  } else {
    el("editor-empty").classList.remove("hidden");
    el("editor-form").classList.add("hidden");
  }
}

// ---------- Positions config editor ----------

function renderPositionsList() {
  const container = el("positions-list");
  container.innerHTML = "";

  const names = Object.keys(state.positions);
  if (names.length === 0) {
    container.innerHTML = '<p class="dim" style="color: var(--text-dim); font-size: 13.5px;">No waypoints yet -- add one below.</p>';
    return;
  }

  names.forEach((name) => {
    const pose = state.positions[name];
    const card = document.createElement("div");
    card.className = "editor-section position-card";
    card.dataset.name = name;

    const axisLabels = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"];
    const fields = axisLabels.map((axis, i) => `
      <label class="field-mini">
        <span>${axis}</span>
        <input type="number" step="any" data-axis="${i}" value="${pose[i]}">
      </label>
    `).join("");

    const isStart = name === "start";
    card.innerHTML = `
      <div class="position-card-head">
        <span class="position-name">${escapeHtml(name.toUpperCase())}${isStart ? ' <span class="position-name-hint">(origin, fixed reference)</span>' : ""}</span>
        ${isStart ? "" : '<button type="button" class="btn btn-danger-ghost remove-position-btn">Remove</button>'}
      </div>
      <div class="position-pose-grid">${fields}</div>
    `;

    card.querySelectorAll("input[data-axis]").forEach((input) => {
      input.addEventListener("input", () => {
        const axis = Number(input.dataset.axis);
        state.positions[name][axis] = parseFloat(input.value) || 0;
      });
    });

    const removeBtn = card.querySelector(".remove-position-btn");
    if (removeBtn) {
      removeBtn.addEventListener("click", () => {
        delete state.positions[name];
        populateReferenceOptions();
        renderPositionsList();
      });
    }

    container.appendChild(card);
  });
}

function addPosition() {
  const input = el("new-position-name");
  const name = input.value.trim().toLowerCase();
  if (!name) {
    showToast("Give the waypoint a name first.", true);
    return;
  }
  if (!/^[a-z0-9_-]+$/.test(name)) {
    showToast("Waypoint names can only use letters, numbers, \"_\" and \"-\".", true);
    return;
  }
  if (state.positions[name]) {
    showToast(`"${name.toUpperCase()}" already exists.`, true);
    return;
  }
  state.positions[name] = [0, 0, 0, 0, 0, 0];
  input.value = "";
  populateReferenceOptions();
  renderPositionsList();
}

// ---------- positions files (there can be several, e.g. one per pool) ----------

// The top-bar dropdown showing every positions file on disk, plus a
// "(unsaved)" entry when the working set hasn't been written anywhere yet.
function renderPositionsSelect() {
  const select = el("positions-select");
  select.innerHTML = "";

  const names = [...state.positionFiles];
  if (!names.includes(state.positionsFile)) names.push(state.positionsFile);

  names.sort().forEach((name) => {
    const opt = document.createElement("option");
    opt.value = name;
    if (name === "") opt.textContent = "— unsaved —";
    else opt.textContent = state.positionFiles.includes(name) ? name : `${name} (unsaved)`;
    select.appendChild(opt);
  });

  select.value = state.positionsFile;
}

async function loadPositionFileList() {
  state.positionFiles = await api("/api/positions");
  renderPositionsSelect();
}

// Everything that depends on the active waypoint set: the reference
// dropdown, the positions editor, and (if a move_to step is open) its
// resolved-pose preview + stored target_pose.
function refreshPositionDependents() {
  populateReferenceOptions();
  if (state.view === "positions") renderPositionsList();
  if (state.selected !== null && state.steps[state.selected]
      && state.steps[state.selected].type === "move_to"
      && !el("editor-form").classList.contains("hidden")) {
    syncFormToStep();
  }
}

async function loadPositionsFile(name) {
  try {
    state.positions = await api(`/api/positions/${encodeURIComponent(name)}`);
  } catch (e) {
    showToast(e.message, true);
    renderPositionsSelect(); // snap the dropdown back to the current file
    return;
  }
  state.positionsFile = name;
  el("positions-name").value = name;
  renderPositionsSelect();
  refreshPositionDependents();
}

function newPositionsFile() {
  switchView("positions");
  state.positions = { start: [0, 0, 0, 0, 0, 0] };
  state.positionsFile = "";
  el("positions-name").value = "";
  renderPositionsSelect();
  refreshPositionDependents();
}

async function savePositions() {
  const name = el("positions-name").value.trim();
  if (!name) {
    showToast("Give the positions file a name before saving.", true);
    return;
  }
  try {
    await api(`/api/positions/${encodeURIComponent(name)}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(state.positions),
    });
    state.positionsFile = name;
    showToast(`Saved "${name}.json"`);
    await loadPositionFileList();
  } catch (e) {
    showToast(e.message, true);
  }
}

async function deletePositionsFile() {
  const name = state.positionsFile;
  if (!name) return;
  if (!confirm(`Delete positions file "${name}.json"? This can't be undone.`)) return;
  try {
    await api(`/api/positions/${encodeURIComponent(name)}`, { method: "DELETE" });
    showToast(`Deleted "${name}.json"`);
    await loadPositionFileList();
    if (state.positionFiles.length > 0) {
      await loadPositionsFile(state.positionFiles[0]);
    } else {
      newPositionsFile();
    }
  } catch (e) {
    showToast(e.message, true);
  }
}



function newMission() {
  switchView("sequence");
  state.steps = [];
  state.selected = null;
  el("mission-name").value = "";
  el("mission-select").value = "";
  el("editor-form").classList.add("hidden");
  el("editor-empty").classList.remove("hidden");
  renderStepList();
}

async function loadMissionList() {
  const names = await api("/api/missions");
  const select = el("mission-select");
  select.innerHTML = '<option value="">— load a mission —</option>';
  names.forEach((name) => {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    select.appendChild(opt);
  });
}

async function loadMission(name) {
  switchView("sequence");
  const data = await api(`/api/missions/${encodeURIComponent(name)}`);
  state.steps = data.steps || [];
  state.selected = null;
  el("mission-name").value = name;
  el("editor-form").classList.add("hidden");
  el("editor-empty").classList.remove("hidden");
  renderStepList();

  // Switch to the positions file this mission was built against, if it
  // recorded one (missions saved before multi-file support won't have it).
  if (data.positions_file && data.positions_file !== state.positionsFile) {
    if (state.positionFiles.includes(data.positions_file)) {
      await loadPositionsFile(data.positions_file);
    } else {
      showToast(`Mission uses positions file "${data.positions_file}", which doesn't exist -- keeping "${state.positionsFile}".`, true);
    }
  }

  showToast(`Loaded "${name}" (${state.steps.length} steps)`);
}

async function saveMission() {
  const name = el("mission-name").value.trim();
  if (!name) {
    showToast("Give the mission a name before saving.", true);
    return;
  }
  if (state.steps.length === 0) {
    showToast("Add at least one step before saving.", true);
    return;
  }
  const body = { steps: state.steps };
  // Record which positions file the mission was built against so the
  // planner loads the same one at launch. Skipped while the working set
  // is unsaved (positionsFile === "") -- there's no file to point at yet.
  if (state.positionsFile) body.positions_file = state.positionsFile;
  try {
    await api(`/api/missions/${encodeURIComponent(name)}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    showToast(`Saved "${name}"`);
    await loadMissionList();
    el("mission-select").value = name;
  } catch (e) {
    showToast(e.message, true);
  }
}

async function deleteMission() {
  const name = el("mission-name").value.trim();
  if (!name) return;
  if (!confirm(`Delete mission "${name}"? This can't be undone.`)) return;
  try {
    await api(`/api/missions/${encodeURIComponent(name)}`, { method: "DELETE" });
    showToast(`Deleted "${name}"`);
    newMission();
    await loadMissionList();
  } catch (e) {
    showToast(e.message, true);
  }
}

// ---------- editor actions ----------

// Writes the current form values straight into the selected step. Called on
// every keystroke/change -- there's no separate "commit" step. Save is the
// only action that touches disk.
function syncFormToStep() {
  const i = Number(el("edit-index").value);
  const step = state.steps[i];
  if (!step) return;

  step.label = el("f-label").value.trim() || step.type;

  if (step.type === "move_to") {
    step.reference = el("f-reference").value || "start";
    step.offset = currentOffsetFields();
    step.target_pose = resolveAbsolutePose();
    step.target_velocity = [
      parseFloat(el("f-vx").value) || 0,
      parseFloat(el("f-vy").value) || 0,
      parseFloat(el("f-vz").value) || 0,
      parseFloat(el("f-vroll").value) || 0,
      parseFloat(el("f-vpitch").value) || 0,
      parseFloat(el("f-vyaw").value) || 0,
    ];
    updatePoseLabelAndResolved();
  } else if (step.type === "actuate") {
    step.mechanism = el("f-mechanism").value;
    step.setpoint = el("f-setpoint").value;
  } else if (step.type === "toggle_watchdog") {
    step.enabled = el("f-watchdog-enabled").value === "true";
  } else if (step.type === "wait") {
    step.seconds = Math.max(0, parseFloat(el("f-wait-seconds").value) || 0);
  } else if (step.type === "cli") {
    step.command = el("f-cli-command").value;
  } else if (step.type === "cli_stop") {
    step.target = el("f-cli-stop-target").value;
  }

  renderStepList();
}

function removeStep() {
  const i = Number(el("edit-index").value);
  state.steps.splice(i, 1);
  state.selected = null;
  el("editor-form").classList.add("hidden");
  el("editor-empty").classList.remove("hidden");
  renderStepList();
}

// ---------- wiring ----------

function addStep(type) {
  switchView("sequence");
  let step;
  if (type === "move_to") step = defaultMoveToStep();
  else if (type === "toggle_watchdog") step = defaultToggleWatchdogStep();
  else if (type === "wait") step = defaultWaitStep();
  else if (type === "cli") step = defaultCliStep();
  else if (type === "cli_stop") step = defaultCliStopStep();
  else step = defaultActuateStep();

  state.steps.push(step);
  selectStep(state.steps.length - 1);
}

async function init() {
  try {
    await loadPositionFileList();
    if (state.positionFiles.includes("positions")) {
      await loadPositionsFile("positions");
    } else if (state.positionFiles.length > 0) {
      await loadPositionsFile(state.positionFiles[0]);
    } else {
      state.positions = { start: [0, 0, 0, 0, 0, 0] };
      renderPositionsSelect();
    }
  } catch (e) {
    showToast("Couldn't load positions config: " + e.message, true);
  }
  populateReferenceOptions();

  try {
    await loadMissionList();
  } catch (e) {
    showToast("Couldn't reach the backend: " + e.message, true);
  }

  renderStepList();

  // Scoped to the sequence pane's add-row -- "+ Add Waypoint" in the
  // positions editor shares the .btn-add class for styling, and a bare
  // .btn-add selector used to catch it too, so adding a waypoint also
  // appended a phantom actuate step to the mission.
  document.querySelectorAll(".add-row .btn-add").forEach((btn) => {
    btn.addEventListener("click", () => addStep(btn.dataset.type));
  });

  el("btn-new").addEventListener("click", newMission);
  el("btn-save").addEventListener("click", () => {
    if (state.view === "positions") savePositions();
    else saveMission();
  });
  el("btn-delete").addEventListener("click", () => {
    if (state.view === "positions") deletePositionsFile();
    else deleteMission();
  });

  el("nav-positions").addEventListener("click", () => switchView("positions"));
  el("btn-add-position").addEventListener("click", addPosition);
  el("btn-new-positions").addEventListener("click", newPositionsFile);
  el("positions-select").addEventListener("change", (e) => {
    if (e.target.value) loadPositionsFile(e.target.value);
    else renderPositionsSelect(); // "— unsaved —" isn't loadable; snap back
  });

  el("mission-select").addEventListener("change", (e) => {
    if (e.target.value) loadMission(e.target.value);
  });

  el("btn-remove").addEventListener("click", removeStep);

  [
    "f-label",
    "f-reference", "f-x", "f-y", "f-z", "f-roll", "f-pitch", "f-yaw",
    "f-vx", "f-vy", "f-vz", "f-vroll", "f-vpitch", "f-vyaw",
    "f-mechanism", "f-setpoint", "f-watchdog-enabled", "f-wait-seconds", "f-cli-command",
    "f-cli-stop-target",
  ].forEach((id) => {
    el(id).addEventListener("input", syncFormToStep);
    el(id).addEventListener("change", syncFormToStep);
  });
}

init();
