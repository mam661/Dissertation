# ArduPilot + Gazebo Dev Environment — Setup & Testing Guide

## Windows 10 Setup (VcXsrv)

### 1. Install VcXsrv
Download from: https://sourceforge.net/projects/vcxsrv/

### 2. Launch VcXsrv with the correct settings
Run **XLaunch** with these settings:
- Display number: **0**
- Start no client: ✅
- Clipboard: ✅
- Native opengl: ✅ (try unchecking this if Gazebo crashes)
- **Disable access control: ✅** ← this is critical, allows Docker to connect

Save the config as a `.xlaunch` file so you can relaunch it easily.

### 3. Allow VcXsrv through Windows Firewall
When prompted by Windows Defender, allow VcXsrv on **both private and public** networks.
If you missed the prompt, go to:
Windows Defender Firewall → Allow an app → vcxsrv.exe → check both boxes.

### 4. Open the devcontainer in VS Code
The `DISPLAY=host.docker.internal:0.0` in devcontainer.json will point to your
running VcXsrv instance automatically.

---

## Windows 11 Setup (WSLg — no extra software needed)

WSLg is built into Windows 11's WSL2 and Docker Desktop uses it automatically.

### 1. Make sure WSL2 backend is enabled in Docker Desktop
Docker Desktop → Settings → General → "Use the WSL 2 based engine" ✅

### 2. Override DISPLAY in your container shell if needed
WSLg typically uses `:0`, but if Gazebo doesn't open a window, run:
```bash
export DISPLAY=:0
```

---

## Testing the Environment

### Step 1 — Verify install (no display needed)
Open a terminal in the devcontainer and run:
```bash
# Gazebo Classic should be version 11.x
gazebo --version

# ArduPilot SITL should print usage
sim_vehicle.py --help

# The ArduPilot Gazebo plugin should be installed
ls /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/ | grep ardupilot
```

### Step 2 — Test ArduPilot SITL alone (no Gazebo, no display needed)
```bash
cd /ardupilot
sim_vehicle.py -v ArduCopter --console
```
You should get a MAVProxy prompt. Try:
```
mode guided
arm throttle
takeoff 10
```
The drone should respond in the MAVProxy console. Press Ctrl+C to stop.

### Step 3 — Test Gazebo GUI (display required)
Open two terminals in the devcontainer.

**Terminal 1 — Launch Gazebo:**
```bash
gazebo --verbose /ardupilot_gazebo/worlds/iris_arducopter_runway.world
```
A Gazebo window should open on your Windows desktop showing a runway with a drone.

**Terminal 2 — Connect SITL to Gazebo:**
```bash
cd /ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
The drone in Gazebo should become active. You can then fly it via MAVProxy:
```
mode guided
arm throttle
takeoff 10
```

---

## Multi-Drone Testing

Each SITL instance needs a unique `--instance` number and its own port.
Open one terminal per drone:

```bash
# Drone 1
sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 0 -I0

# Drone 2
sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 1 -I1

# Drone 3
sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 2 -I2
```

Default MAVLink ports per instance:
| Instance | TCP (SITL) | UDP (GCS) |
|----------|-----------|-----------|
| 0        | 5760      | 14550     |
| 1        | 5770      | 14560     |
| 2        | 5780      | 14570     |

Connect a GCS (e.g. Mission Planner or QGroundControl) to `localhost:14550`
for drone 1, `localhost:14560` for drone 2, etc.

---

## Troubleshooting

**Gazebo opens but crashes immediately (Windows 10)**
- Uncheck "Native opengl" in VcXsrv/XLaunch and restart it.
- Try adding `export LIBGL_ALWAYS_SOFTWARE=1` in the container before launching.

**"cannot open display" error**
- Make sure VcXsrv is running (Windows 10) with "Disable access control" checked.
- Check the firewall is allowing VcXsrv.
- Verify the DISPLAY variable: `echo $DISPLAY` should print `host.docker.internal:0.0`.

**Gazebo opens but the drone doesn't move**
- Make sure SITL was started with `-f gazebo-iris` so it outputs to the Gazebo plugin port.
- Check SITL output for "Waiting for Gazebo" — Gazebo must be running first.
