# mam661 Search and Rescue Drone Dissertation Code

## Prerequisites (All in the docker file)
- WSL
- ROS2
- Gazebo
- Ardupilot
- Ardupilot_gazebo plugin
- Docker
- Mavproxy
- MAVROS


## SETUP (Windows)

- Have docker running, and open a WSL terminal. Navigate to the project folder, and open with "code ."

- This will ensure all of the mounting is done properly, and things should render properly. Other methods may work, but were inconsistent and this way worked all the time.

- When prompted, or just otherwise, open the project in a docker container. Expect this to take a long time, as there are lots of dependencies

- Once everything is ready in the docker container, open a terminal in the source directory of the project and run: colcon build --symlink-install   Followed by sourcing the terminal with:    Source install/local_setup.bash

- With that ready, you can launch the file using: ros2 launch drone_controller diss_launch.py drones:=[num drones]

- This should fire up the simulation, after which point in the terminal it will prompt you to run something like:
  [python3-3 ] # Drone 0  (MAVLink: 14551, GCS: 15001)
  [python3-3 ] cd /ardupilot && sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console --add-param-file=/workspaces/Dissertation/sitl_params.parm --out=udp:127.0.0.1:14551


- Run this is a new terminal (that doesn't need to be sourced)

- Then, open a third terminal, source it as before, and run: ros2 run drone_controller droneNode [droneNum - same as the -I argument above]

- When the drone has properly configured, the drone should take off, the central controller will start instructing it, and it'll be on its way






This project uses some assets from, and was initially based off:
https://github.com/monemati/multiuav-gazebo-simulation

Though I think all that I use from it now is the drone which was from ardupilot and has been customised.