import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    world_path = '/workspaces/Dissertation/models/map3.world'
    model_path = '/workspaces/Dissertation/models'
    spawn_script = '/workspaces/Dissertation/spawn_drones.py'


    num_drones_arg = DeclareLaunchArgument(
        'drones', default_value='0', description='Number of drones to spawn'
    )
    num_drones = LaunchConfiguration('drones')


    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )


    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path],
        output='screen'
    )

    
    start_spawn_script = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', spawn_script, '--drones', num_drones],
                output='screen'
            )
        ]
    )


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