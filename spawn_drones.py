#!/usr/bin/env python3
"""
spawn_drones.py - Spawn N ArduPilot iris drones into a running Gazebo world.

Usage:
    python3 spawn_drones.py --drones 4
    python3 spawn_drones.py --drones 4 --spacing 5.0
    python3 spawn_drones.py --drones 4 --spacing 5.0 --model-path /workspaces/Dissertation/models

Each drone gets:
    - Unique model name: iris_0, iris_1, ...
    - FDM ports:  instance 0 -> 9002/9003, instance 1 -> 9012/9013, etc.
    - MAVLink ports: instance 0 -> 5760/14550, instance 1 -> 5770/14560, etc.
    - Positions arranged in a grid with configurable spacing
"""

import argparse
import os
import subprocess
import sys
import time

TEMPLATE_SDF = os.path.join(os.path.dirname(__file__), "models", "iris_ardupilot", "model.sdf")


def get_spawn_position(index, spacing):
    """Arrange drones in a row along the x-axis."""
    x = index * spacing
    y = 0.0
    z = 0.0
    return x, y, z


def build_sdf(instance, spacing):
    """Read the template SDF and substitute instance-specific values."""
    model_name = f"iris_{instance}"
    fdm_port_in  = 9002 + instance * 10
    fdm_port_out = 9003 + instance * 10

    with open(TEMPLATE_SDF, "r") as f:
        sdf = f.read()

    sdf = sdf.replace("__MODEL_NAME__", model_name)
    sdf = sdf.replace("__FDM_PORT_IN__",  str(fdm_port_in))
    sdf = sdf.replace("__FDM_PORT_OUT__", str(fdm_port_out))
    # Update model name attribute in the SDF
    sdf = sdf.replace('name="iris_ardupilot"', f'name="{model_name}"')

    return sdf, model_name


def spawn_drone(instance, spacing):
    """Spawn a single drone into Gazebo using gz model spawn service."""
    sdf, model_name = build_sdf(instance, spacing)
    x, y, z = get_spawn_position(instance, spacing)

    # Write SDF to a temp file
    tmp_path = f"/tmp/{model_name}.sdf"
    with open(tmp_path, "w") as f:
        f.write(sdf)

    cmd = [
        "gz", "model",
        "--spawn-file", tmp_path,
        "--model-name", model_name,
        "-x", str(x), "-y", str(y), "-z", str(z)
    ]

    print(f"Spawning {model_name} at ({x}, {y}, {z})  FDM ports: {9002 + instance*10}/{9003 + instance*10}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"  ERROR: {result.stderr.strip()}")
        return False
    else:
        print(f"  OK")
        return True


def print_sitl_commands(n_drones):
    """Print the sim_vehicle.py commands needed for each drone."""
    print("\n" + "="*60)
    print("Now start SITL instances - one terminal per drone:")
    print("="*60)
    for i in range(n_drones):
        mavlink_port = 14551 + i * 10
        gcs_port     = 15001 + i * 10
        print(f"\n# Drone {i}  (MAVLink: {mavlink_port}, GCS: {gcs_port})")
        print(f"cd /ardupilot && sim_vehicle.py -v ArduCopter -f gazebo-iris -I{i} --console --add-param-file=/workspaces/Dissertation/sitl_params.parm --out=udp:127.0.0.1:{mavlink_port}")

        
    print()


def main():
    parser = argparse.ArgumentParser(description="Spawn N ArduPilot iris drones into Gazebo.")
    parser.add_argument("--drones",     type=int,   default=4,    help="Number of drones to spawn (default: 4)")
    parser.add_argument("--spacing",    type=float, default=5.0,  help="Spacing between drones in metres (default: 5.0)")
    parser.add_argument("--model-path", type=str,   default=None, help="Extra path to prepend to GAZEBO_MODEL_PATH")
    args = parser.parse_args()

    if not os.path.exists(TEMPLATE_SDF):
        print(f"ERROR: Template SDF not found at {TEMPLATE_SDF}")
        print("Make sure you run this script from the project root or set --model-path correctly.")
        sys.exit(1)

    if args.model_path:
        current = os.environ.get("GAZEBO_MODEL_PATH", "")
        os.environ["GAZEBO_MODEL_PATH"] = f"{args.model_path}:{current}"

    print(f"Spawning {args.drones} drones with {args.spacing}m spacing...\n")

    success = 0
    for i in range(args.drones):
        if spawn_drone(i, args.spacing):
            success += 1
        time.sleep(0.5)  # Small delay between spawns

    print(f"\n{success}/{args.drones} drones spawned successfully.")

    if success > 0:
        print_sitl_commands(success)


if __name__ == "__main__":
    main()