import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    #  寻找文件----------------------------------------------------------------------------------
    urdf_file = os.path.join(
        get_package_share_directory('robot_describtion'),
        'urdf',
        'robot.urdf'
    )
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"World file not found: {world_file}")
    
    #  配置节点----------------------------------------------------------------------------------
    robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # arguments可以直接传递给节点的命令行参数,一次性使用；parameters是通过ROS2参数服务器传递给节点的参数可以在运行时动态修改
        arguments=[urdf_file]
        # parameters=[{'robot_description': open(urdf_file).read()}]
        
    )
    
    #  添加节点----------------------------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    
    return ld