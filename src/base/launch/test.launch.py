import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

package_name = 'ros2_swervebot_sim_base'

def generate_launch_description():

    gazebo_launch_file = os.path.join(get_package_share_directory("ros_gz_sim"),'launch/gz_sim.launch.py')
    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gazebo_launch_file),
      launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    ros_gz_bridge = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      arguments=[
        '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/empty/control@ros_gz_interfaces/srv/ControlWorld'
      ],
      remappings=[("/world/empty/clock", "/clock")]
    )

    gazebo_unpause = ExecuteProcess(
      cmd=[[
        "ros2",
        " service call ",
        " /world/empty/control ",
        " ros_gz_interfaces/srv/ControlWorld ",
        " '{world_control: {pause: false}}' "
      ]],
      shell=True
    )

    robot_desc = xacro.process_file(
      os.path.join(get_package_share_directory(package_name), 'robot_desc/swervebot.xacro')
      ).toprettyxml(indent='  ')

    robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      parameters=[{
       'use_sim_time': True, 
       'robot_description': robot_desc
      }]
    )

    bot_spawner = Node(
      package='ros_gz_sim',
      executable='create',
      parameters=[{
        'name': package_name,
        'world': 'empty',
        'topic': '/robot_description',
        'x': 0.0,
        'y': 0.0,
        'z': 0.5,
        'Y': 0.0
      }]
    )

    ros2c_activate = Node(
      package="controller_manager",
      executable="spawner",
      arguments=
      [
        "JSB",
        "velo_c"
      ]
    )

    rviz_config_file = os.path.join(
      get_package_share_directory(package_name), 'rviz2', 'main_view.rviz')
      
    rviz2 = Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=['-d', rviz_config_file],
      parameters=[{ 'use_sim_time': True }]
    )

    return LaunchDescription([

      gazebo_launch,
      ros_gz_bridge,
      robot_state_publisher,
      gazebo_unpause,

      RegisterEventHandler(
        OnProcessExit(
          target_action=gazebo_unpause,
          on_exit=bot_spawner
        )
      ),

      RegisterEventHandler(
        OnProcessExit(
          target_action=bot_spawner,
          on_exit=[ros2c_activate, rviz2]
        )
      ),

    ])