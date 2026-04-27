import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths
    world_path = '/workspaces/Dissertation/models/map3.world'
    model_path = '/workspaces/Dissertation/models'
    spawn_script = '/workspaces/Dissertation/spawn_drones.py'

    # 2. Launch Arguments (so you can do: ros2 launch ... drones:=5)
    num_drones_arg = DeclareLaunchArgument(
        'drones', default_value='0', description='Number of drones to spawn'
    )
    num_drones = LaunchConfiguration('drones')

    # 3. Set Environment Variable for Gazebo
    # This replaces your 'export GAZEBO_MODEL_PATH' command
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    # 4. Action: Start Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path],
        output='screen'
    )

    
    # 5. Action: Run the Spawn Script (Wait 10 seconds for Gazebo)
    # We pass the 'drones' argument directly to your python script
    start_spawn_script = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', spawn_script, '--drones', num_drones],
                output='screen'
            )
        ]
    )

    # 6. Action: Start Central Controller
    start_controller = Node(
        package='drone_controller',
        executable='centralController',
        parameters=[{'num_drones': num_drones}],
        output='screen'
    )

    return LaunchDescription([
        num_drones_arg,
        set_gazebo_model_path,
        start_gazebo,
        start_spawn_script,
        start_controller
    ])