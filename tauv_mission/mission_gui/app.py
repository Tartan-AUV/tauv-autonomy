"""
Local mission-plan editor.

Run with:  python3 app.py
Then open: http://127.0.0.1:5050

Missions are saved as plain JSON files under ../missions/, in the same
format mission_planner.py loads at launch. Nothing here talks to ROS,
the sub, or the network beyond localhost -- it's just a form that writes
a file.
"""

import json
import re
from pathlib import Path

from flask import Flask, abort, jsonify, render_template, request

BASE_DIR = Path(__file__).resolve().parent.parent
MISSIONS_DIR = BASE_DIR / "missions"
CONFIG_DIR = BASE_DIR / "config"

MISSIONS_DIR.mkdir(parents=True, exist_ok=True)
CONFIG_DIR.mkdir(parents=True, exist_ok=True)

app = Flask(__name__)

NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def safe_mission_path(name: str) -> Path:
    """Keeps mission names to a safe charset so this can't be tricked into
    reading/writing outside the missions directory."""
    if not NAME_RE.match(name or ""):
        abort(400, description='Mission name may only contain letters, numbers, "_" and "-".')
    return MISSIONS_DIR / f"{name}.json"


def safe_positions_path(name: str) -> Path:
    """Same charset restriction as mission names, for config/<name>.json."""
    if not NAME_RE.match(name or ""):
        abort(400, description='Positions file name may only contain letters, numbers, "_" and "-".')
    return CONFIG_DIR / f"{name}.json"


def validate_steps(steps):
    if not isinstance(steps, list) or not steps:
        abort(400, description="Mission must have at least one step.")

    # Labels of cli steps seen so far, so cli_stop steps can be checked
    # against them -- a cli_stop only makes sense after the cli it targets.
    cli_labels = set()

    for i, step in enumerate(steps):
        step_type = step.get("type")

        if step_type == "move_to":
            offset = step.get("offset")
            vel = step.get("target_velocity")
            if not step.get("reference"):
                abort(400, description=f'Step {i + 1} ("move_to") needs a reference position (use "start" for absolute).')
            if not (isinstance(offset, list) and len(offset) == 6 and all(isinstance(v, (int, float)) for v in offset)):
                abort(400, description=f'Step {i + 1} ("move_to") needs a 6-value offset.')
            if not (isinstance(vel, list) and len(vel) == 6 and all(isinstance(v, (int, float)) for v in vel)):
                abort(400, description=f'Step {i + 1} ("move_to") needs a 6-value target_velocity.')

        elif step_type == "actuate":
            if not step.get("mechanism"):
                abort(400, description=f'Step {i + 1} ("actuate") needs a mechanism name.')
            if not step.get("setpoint"):
                abort(400, description=f'Step {i + 1} ("actuate") needs a setpoint.')

        elif step_type == "toggle_watchdog":
            if not isinstance(step.get("enabled"), bool):
                abort(400, description=f'Step {i + 1} ("toggle_watchdog") needs a boolean "enabled" value.')

        elif step_type == "cli":
            command = step.get("command")
            if not isinstance(command, str) or not command.strip():
                abort(400, description=f'Step {i + 1} ("cli") needs a non-empty command.')
            label = step.get("label")
            if isinstance(label, str) and label.strip():
                if label in cli_labels:
                    abort(400, description=f'Step {i + 1} ("cli") reuses the label "{label}" -- cli labels must be unique so cli_stop can target them.')
                cli_labels.add(label)

        elif step_type == "cli_stop":
            target = step.get("target")
            if not isinstance(target, str) or not target.strip():
                abort(400, description=f'Step {i + 1} ("cli_stop") needs the label of the "cli" step to stop.')
            if target not in cli_labels:
                abort(400, description=f'Step {i + 1} ("cli_stop") targets "{target}", but no earlier "cli" step has that label.')

        elif step_type == "wait":
            seconds = step.get("seconds")
            if not isinstance(seconds, (int, float)) or isinstance(seconds, bool) or seconds <= 0:
                abort(400, description=f'Step {i + 1} ("wait") needs a positive number of seconds.')

        else:
            abort(400, description=f'Step {i + 1} has an unrecognized type: "{step_type}".')


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/positions")
def list_position_files():
    return jsonify(sorted(p.stem for p in CONFIG_DIR.glob("*.json")))


@app.route("/api/positions/<name>", methods=["GET"])
def get_positions(name):
    path = safe_positions_path(name)
    if not path.is_file():
        abort(404, description=f'No positions file named "{name}".')
    try:
        return jsonify(json.loads(path.read_text()))
    except json.JSONDecodeError:
        abort(500, description=f'config/{name}.json is not valid JSON.')


@app.route("/api/positions/<name>", methods=["POST"])
def save_positions(name):
    path = safe_positions_path(name)
    body = request.get_json(silent=True)
    if not isinstance(body, dict) or not body:
        abort(400, description="Request body must be a JSON object of named waypoints.")

    for wp_name, pose in body.items():
        if not NAME_RE.match(wp_name):
            abort(400, description=f'Waypoint name "{wp_name}" may only contain letters, numbers, "_" and "-".')
        if not (isinstance(pose, list) and len(pose) == 6 and all(isinstance(v, (int, float)) for v in pose)):
            abort(400, description=f'Waypoint "{wp_name}" needs a 6-value pose [x, y, z, roll, pitch, yaw].')

    path.write_text(json.dumps(body, indent=2))
    return jsonify({"saved": name, "count": len(body)})


@app.route("/api/positions/<name>", methods=["DELETE"])
def delete_positions(name):
    path = safe_positions_path(name)
    if not path.is_file():
        abort(404, description=f'No positions file named "{name}".')
    path.unlink()
    return jsonify({"deleted": name})


@app.route("/api/missions")
def list_missions():
    return jsonify(sorted(p.stem for p in MISSIONS_DIR.glob("*.json")))


@app.route("/api/missions/<name>", methods=["GET"])
def get_mission(name):
    path = safe_mission_path(name)
    if not path.is_file():
        abort(404, description=f'No mission named "{name}".')
    try:
        return jsonify(json.loads(path.read_text()))
    except json.JSONDecodeError:
        abort(500, description=f'"{name}.json" on disk is not valid JSON.')


@app.route("/api/missions/<name>", methods=["POST"])
def save_mission(name):
    path = safe_mission_path(name)
    body = request.get_json(silent=True)
    if not isinstance(body, dict) or "steps" not in body:
        abort(400, description='Request body must be JSON with a "steps" array.')

    validate_steps(body["steps"])

    positions_file = body.get("positions_file")
    if positions_file is not None and (
            not isinstance(positions_file, str) or not NAME_RE.match(positions_file)):
        abort(400, description='"positions_file" must be a positions config name (letters, numbers, "_" and "-").')

    body["name"] = name
    path.write_text(json.dumps(body, indent=2))
    return jsonify({"saved": name, "steps": len(body["steps"]), "path": str(path)})


@app.route("/api/missions/<name>", methods=["DELETE"])
def delete_mission(name):
    path = safe_mission_path(name)
    if not path.is_file():
        abort(404, description=f'No mission named "{name}".')
    path.unlink()
    return jsonify({"deleted": name})


if __name__ == "__main__":
    # Bound to localhost only -- this is a local dev tool, not a service.
    app.run(host="0.0.0.0", port=5050, debug=True)
