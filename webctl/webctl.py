#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""
webctl.py - lightweight web control panel for the smart parcel bench.

Runs as an INDEPENDENT process alongside the core `locker` controller.
It never modifies the locker binary or its logic. It only:

  - reads the servo's present_position from sysfs (safe, non-exclusive)
  - reports whether the locker service is running (via systemctl)
  - offers a manual sweep that briefly stops the locker, drives the paddle,
    then restarts it -- a borrow-and-return of servo control, so the two
    never write to the servo at the same time

The servo sysfs directory is discovered at runtime rather than hard-coded,
so the UART3 -> UART5 migration required no change here either.

Served on :5000. With the board hosting its own access point the address is
fixed at http://10.42.0.1:5000, which is what the printed QR code points to.
"""
import glob
import os
import subprocess
import time

from flask import Flask, jsonify, render_template_string

app = Flask(__name__)

# ---- servo geometry (must match integration/sensors.h) ----
SERVO_HOME = 1024          # idle / home position
SERVO_OPEN = 1706          # action position: +60 deg (682 counts @ ~11.38/deg)
HOLD_S     = 3.0           # dwell at the action position, seconds

LOCKER_SERVICE = "smartbench-locker.service"


# ---- servo sysfs helpers (path discovered at runtime) ----
def servo_dir():
    hits = glob.glob("/sys/bus/serial/drivers/sts3215/*/goal_position")
    return os.path.dirname(hits[0]) if hits else None


def servo_read(attr):
    d = servo_dir()
    if not d:
        return None
    try:
        with open(os.path.join(d, attr)) as f:
            return int(f.read().strip())
    except Exception:
        return None


def servo_write(attr, val):
    d = servo_dir()
    if not d:
        return False
    try:
        with open(os.path.join(d, attr), "w") as f:
            f.write(str(val))
        return True
    except Exception:
        return False


# ---- locker service control ----
def locker_active():
    r = subprocess.run(["systemctl", "is-active", LOCKER_SERVICE],
                       capture_output=True, text=True)
    return r.stdout.strip() == "active"


def locker_stop():
    subprocess.run(["systemctl", "stop", LOCKER_SERVICE])


def locker_start():
    subprocess.run(["systemctl", "start", LOCKER_SERVICE])


# ---- API ----
@app.route("/api/status")
def api_status():
    return jsonify({
        "servo_pos": servo_read("present_position"),
        "servo_home": SERVO_HOME,
        "servo_open": SERVO_OPEN,
        "locker_running": locker_active(),
    })


@app.route("/api/sweep", methods=["POST"])
def api_sweep():
    """Borrow servo control from the auto controller, sweep, hand it back."""
    was_running = locker_active()
    if was_running:
        locker_stop()
        time.sleep(0.5)              # let it release torque cleanly

    servo_write("torque_enable", 1)
    servo_write("goal_position", SERVO_OPEN)
    time.sleep(HOLD_S)
    servo_write("goal_position", SERVO_HOME)
    time.sleep(0.8)                  # let it seat before handing back

    if was_running:
        locker_start()

    return jsonify({"ok": True, "resumed_auto": was_running})


# ---- page ----
PAGE = """
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Smart Parcel Bench</title>
<style>
  * { box-sizing: border-box; }
  body { margin:0; font-family:-apple-system,Segoe UI,Roboto,sans-serif;
         background:#0f1115; color:#e8eaed; padding:24px; }
  h1 { font-size:20px; font-weight:600; margin:0 0 20px; }
  .card { background:#1a1d24; border:1px solid #2a2e37; border-radius:14px;
          padding:18px; margin-bottom:16px; }
  .row { display:flex; justify-content:space-between; align-items:center;
         padding:10px 0; border-bottom:1px solid #22262f; }
  .row:last-child { border-bottom:none; }
  .label { color:#9aa0aa; font-size:14px; }
  .value { font-size:18px; font-weight:600; font-variant-numeric:tabular-nums; }
  .dot { display:inline-block; width:10px; height:10px; border-radius:50%;
         margin-right:8px; vertical-align:middle; }
  .on { background:#34d399; } .off { background:#f87171; }
  button { width:100%; padding:16px; font-size:16px; font-weight:600;
           border:none; border-radius:12px; color:#0f1115; background:#60a5fa;
           cursor:pointer; margin-top:4px; }
  button:active { transform:scale(0.98); }
  button:disabled { background:#3a3f4a; color:#7a808a; }
  .hint { color:#7a808a; font-size:12px; margin-top:10px; text-align:center; }
</style>
</head>
<body>
  <h1>Smart Parcel Bench</h1>
  <div class="card">
    <div class="row">
      <span class="label">Auto controller</span>
      <span class="value"><span id="dot" class="dot off"></span><span id="locker">--</span></span>
    </div>
    <div class="row">
      <span class="label">Servo position</span>
      <span class="value"><span id="pos">--</span></span>
    </div>
    <div class="row">
      <span class="label">Home / Action</span>
      <span class="value" id="range">--</span>
    </div>
  </div>
  <div class="card">
    <button id="sweep" onclick="sweep()">Sweep paddle (manual)</button>
    <div class="hint">Briefly pauses auto mode, sweeps the paddle, then resumes.</div>
  </div>

<script>
async function refresh() {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    document.getElementById('pos').textContent = s.servo_pos ?? '--';
    document.getElementById('range').textContent = s.servo_home + ' / ' + s.servo_open;
    const running = s.locker_running;
    document.getElementById('locker').textContent = running ? 'running' : 'stopped';
    document.getElementById('dot').className = 'dot ' + (running ? 'on' : 'off');
  } catch (e) {}
}
async function sweep() {
  const b = document.getElementById('sweep');
  b.disabled = true; b.textContent = 'Sweeping...';
  try { await fetch('/api/sweep', {method:'POST'}); } catch(e) {}
  b.textContent = 'Sweep paddle (manual)'; b.disabled = false;
  refresh();
}
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(PAGE)


if __name__ == "__main__":
    # 0.0.0.0 so the page is reachable from phones on the board's access point
    app.run(host="0.0.0.0", port=5000)
