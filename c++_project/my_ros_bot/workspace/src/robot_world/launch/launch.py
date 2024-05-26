import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    #  定义变量----------------------------------------------------------------------------------
    robot_name_in_model='my_robot'
    #  寻找文件----------------------------------------------------------------------------------
    world_file = os.path.join(
        get_package_share_directory('robot_world'),
        'world',
        'world.sdf'
    )
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}")
    
    robot_file=os.path.join(
        get_package_share_directory('robot_world'),
        'world',
        'robot.urdf'
    )
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file not found: {robot_file}")
    
    #  配置节点----------------------------------------------------------------------------------
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[robot_file]
        # parameters=[{'robot_description': open(robot_file).read()}]
    )
    
    #  生成实体脚本
    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        output='log',
        arguments=['-entity', robot_name_in_model,  '-file', robot_file ]
    )

    #  添加节点----------------------------------------------------------------------------------
    ld=LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    

    return ld
