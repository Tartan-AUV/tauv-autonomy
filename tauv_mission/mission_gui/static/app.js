// Mission Planner GUI
// Vanilla JS, no framework/build step -- this needs to run on a laptop at
// poolside with no reliable internet.

const state = {
  positions: {},   // named waypoint -> [x,y,z,roll,pitch,yaw]
  steps: [],       // current mission's steps, in order
  selected: null,  // index into steps, or null
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
    reference: "",
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
  };
}

function defaultToggleWatchdogStep() {
  return {
    type: "toggle_watchdog",
    label: "Toggle watchdog",
    enabled: true,
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
  return `actuate  ${step.mechanism || "—"}`;
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

  if (step.type === "move_to") {
    el("f-reference").value = step.reference || "";
    const [x, y, z, roll, pitch, yaw] = step.offset || [0, 0, 0, 0, 0, 0];
    el("f-x").value = x; el("f-y").value = y; el("f-z").value = z;
    el("f-roll").value = roll; el("f-pitch").value = pitch; el("f-yaw").value = yaw;

    const [vx, vy, vz, vroll, vpitch, vyaw] = step.target_velocity || [0, 0, 0, 0, 0, 0];
    el("f-vx").value = vx; el("f-vy").value = vy; el("f-vz").value = vz;
    el("f-vroll").value = vroll; el("f-vpitch").value = vpitch; el("f-vyaw").value = vyaw;

    updatePoseLabelAndResolved();
  } else if (step.type === "actuate") {
    el("f-mechanism").value = step.mechanism || "dropper";
  } else if (step.type === "toggle_watchdog") {
    el("f-watchdog-enabled").value = step.enabled ? "true" : "false";
  }
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
  const ref = el("f-reference").value;
  const offset = currentOffsetFields();
  if (ref && state.positions[ref]) {
    return state.positions[ref].map((v, idx) => v + offset[idx]);
  }
  return offset; // manual mode: the fields ARE the absolute pose
}

function updatePoseLabelAndResolved() {
  const ref = el("f-reference").value;
  el("pose-offset-label").textContent = ref ? "Offset from reference (m / rad)" : "Absolute pose (m / rad)";
  const resolved = resolveAbsolutePose();
  el("resolved-pose-value").textContent = resolved.map((v) => v.toFixed(3)).join(", ");
}

// ---------- populate reference/mechanism dropdowns ----------

function populateReferenceOptions() {
  const select = el("f-reference");
  const previous = select.value;
  select.innerHTML = '<option value="">— manual pose (no reference) —</option>';
  Object.keys(state.positions).forEach((name) => {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name.toUpperCase();
    select.appendChild(opt);
  });
  if (Object.prototype.hasOwnProperty.call(state.positions, previous)) {
    select.value = previous;
  }
}

// ---------- view switching (Sequence <-> Positions) ----------

function switchView(view) {
  state.view = view;

  el("nav-positions").classList.toggle("active", view === "positions");

  el("sequence-toolbar").classList.toggle("hidden", view === "positions");
  el("positions-toolbar").classList.toggle("hidden", view !== "positions");
  el("btn-delete").classList.toggle("hidden", view === "positions");

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

    card.innerHTML = `
      <div class="position-card-head">
        <span class="position-name">${escapeHtml(name.toUpperCase())}</span>
        <button type="button" class="btn btn-danger-ghost remove-position-btn">Remove</button>
      </div>
      <div class="position-pose-grid">${fields}</div>
    `;

    card.querySelectorAll("input[data-axis]").forEach((input) => {
      input.addEventListener("input", () => {
        const axis = Number(input.dataset.axis);
        state.positions[name][axis] = parseFloat(input.value) || 0;
      });
    });

    card.querySelector(".remove-position-btn").addEventListener("click", () => {
      delete state.positions[name];
      populateReferenceOptions();
      renderPositionsList();
    });

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

async function savePositions() {
  try {
    await api("/api/positions", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(state.positions),
    });
    showToast("Saved positions.json");
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
  try {
    await api(`/api/missions/${encodeURIComponent(name)}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ steps: state.steps }),
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
    step.reference = el("f-reference").value;
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
  } else if (step.type === "toggle_watchdog") {
    step.enabled = el("f-watchdog-enabled").value === "true";
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
  else step = defaultActuateStep();

  state.steps.push(step);
  selectStep(state.steps.length - 1);
}

async function init() {
  try {
    state.positions = await api("/api/positions");
  } catch (e) {
    showToast("Couldn't load config/positions.json: " + e.message, true);
  }
  populateReferenceOptions();

  try {
    await loadMissionList();
  } catch (e) {
    showToast("Couldn't reach the backend: " + e.message, true);
  }

  renderStepList();

  document.querySelectorAll(".btn-add").forEach((btn) => {
    btn.addEventListener("click", () => addStep(btn.dataset.type));
  });

  el("btn-new").addEventListener("click", newMission);
  el("btn-save").addEventListener("click", () => {
    if (state.view === "positions") savePositions();
    else saveMission();
  });
  el("btn-delete").addEventListener("click", deleteMission);

  el("nav-positions").addEventListener("click", () => switchView("positions"));
  el("btn-add-position").addEventListener("click", addPosition);

  el("mission-select").addEventListener("change", (e) => {
    if (e.target.value) loadMission(e.target.value);
  });

  el("btn-remove").addEventListener("click", removeStep);

  [
    "f-label",
    "f-reference", "f-x", "f-y", "f-z", "f-roll", "f-pitch", "f-yaw",
    "f-vx", "f-vy", "f-vz", "f-vroll", "f-vpitch", "f-vyaw",
    "f-mechanism", "f-watchdog-enabled",
  ].forEach((id) => {
    el(id).addEventListener("input", syncFormToStep);
    el(id).addEventListener("change", syncFormToStep);
  });
}

init();
