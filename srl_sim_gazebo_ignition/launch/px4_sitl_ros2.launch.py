import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
  airframe_launch_arg = DeclareLaunchArgument(
    'airframe', default_value='gz_x500'
    )
  ddsport_launch_arg = DeclareLaunchArgument(
    'port', default_value='8888'
  )

  airframe_value = LaunchConfiguration("airframe")
  ddsport_value = LaunchConfiguration("port")
  px4_package_dir = get_package_prefix("px4") # /home/docker/colcon_ws/install/px4
  install_dir = os.path.split(px4_package_dir)[0]
  workspace_dir = os.path.split(install_dir)[0]
  px4_src_dir = os.path.join(workspace_dir, "src/srl-mav-sim/PX4-Autopilot/")

  px4_node = Node(
      package='srl_sim_gazebo_ignition',
      executable='px4_sitl.sh',
      name='px4_sitl',
      arguments=[px4_src_dir, airframe_value],
      output="screen",
      )

  dds_cmd = ExecuteProcess(
    cmd=[["MicroXRCEAgent tcp4 -p ", ddsport_value]],
    shell=True
  )

  return LaunchDescription(
    [
    airframe_launch_arg,
    ddsport_launch_arg,
    px4_node,
    dds_cmd
    ]
  )    
