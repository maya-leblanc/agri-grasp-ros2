import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('robotiq_description')
    fruit_models_share = get_package_share_directory('fruit_models')
    
    # Cleanly define target model directories 
    fruit_models_dir = os.path.join(fruit_models_share, 'models')
    robotiq_models_dir = os.path.join(pkg_share, 'models')

    # Get the parent share directories so Gazebo can resolve package:// URIs
    ros2_workspace_share_dir = os.path.dirname(pkg_share)
    fruit_workspace_share_dir = os.path.dirname(fruit_models_share)

    # Path to your custom world file
    world_path = os.path.join(pkg_share, 'worlds', 'agri_grasp.world')

    # CORRECT GAZEBO LAUNCH (Points directly to your custom world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'robotiq_2f_85_gripper.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        'sim_gazebo': 'true',
        'use_fake_hardware': 'true'
    }).toxml()

    # Set model path so Gazebo Harmonic knows where to find all custom meshes/models
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{fruit_models_dir}:{robotiq_models_dir}:{ros2_workspace_share_dir}:{fruit_workspace_share_dir}"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Spawns your gripper robot into the loaded world
    spawn_gripper = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robotiq_gripper',
            '-x', '0',
            '-y', '0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # Spawns the joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawns the main gripper action/command controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller'], 
        output='screen'
    )
    
    # Network communication bridge between Gazebo Harmonic and ROS 2
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_model_path,
        robot_state_publisher,
        gazebo,
        gz_ros_bridge,
        spawn_gripper,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_gripper,
                # Spawns both controllers once the model exists in the physics world
                on_exit=[
                    joint_state_broadcaster,
                    gripper_controller_spawner
                ]
            )
        )
    ])