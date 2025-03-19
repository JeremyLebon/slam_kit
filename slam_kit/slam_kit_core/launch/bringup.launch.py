import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource
)
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description() -> LaunchDescription:
    """ Launch discription to bringup ROS2 nodes """
    ld = LaunchDescription()

    agv_pkg_path = FindPackageShare(
        package='slam_kit_core').find('slam_kit_core')
  
   

    model_path = os.getenv('AGV_MODEL_PATH',
                           os.path.join(agv_pkg_path,
                                        'urdf/bringup.urdf.xacro'))

    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
   
    declare_use_robot_state_pub = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value=os.getenv('USE_ROBOT_STATE_PUB',
                                'True'),
        description='Whether to start the robot state publisher')

    declare_model_path = DeclareLaunchArgument(
        name='model',
        default_value=model_path,
        description='Absolute path to robot urdf file')

 
    robot_desc_doc = xacro.process_file(
        model_path,
        mappings={
            # 'simulation_mode': str(
            #    simulation_mode_arg).lower()
                }
        )
    
    
    
    robot_desc = robot_desc_doc.toprettyxml(indent='  ')
    robot_desc_params = {'robot_description': robot_desc, 
                         }

    robot_state_publisher_node = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
       # namespace=namespace,
        log_cmd=True,
        output='screen',
        parameters=[{ 
        'robot_description': xacro.process_file(model_path).toxml(),
        'update_rate': 10}
                    ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
    )

    
    ld.add_action(declare_model_path)
    ld.add_action(declare_use_robot_state_pub)
    
    ld.add_action(robot_state_publisher_node)


    return ld
