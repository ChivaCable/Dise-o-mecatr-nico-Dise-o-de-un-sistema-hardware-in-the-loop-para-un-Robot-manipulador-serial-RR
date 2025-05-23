import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
###########################################################################################################################
def generate_launch_description():
    # Rutas de archivos
    urdf_file_name = 'robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_planar'),
        'urdf',
        urdf_file_name)
    
    sdf_world_path = '/home/chiva/ros2_ws/src/my_robot_planar/urdf/mi_mundo.sdf'
    sdf_robot_path = '/home/chiva/ros2_ws/src/my_robot_planar/urdf/robot_con_plugin.sdf'

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Lanzar Gazebo con el mundo
    world_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', sdf_world_path],
        output='screen'
    )

    # Spawn del robot en Gazebo
    spawn_robot = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/default/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000',
             '--req', 'sdf_filename: "' + sdf_robot_path + '" name: "my_robot"'],
        output='screen'
    )

    # Nodos ROS 2
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
        ]
    )
    
    # Bridge configurado correctamente
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/topic_1@std_msgs/msg/Float64@gz.msgs.Double",
            "/topic_2@std_msgs/msg/Float64@gz.msgs.Double",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    real_to_sim_node = Node(
        package='my_robot_planar',
        executable='real_to_sim',
        name='real_to_sim_bridge',
        output='screen'
    )
    '''
    sim_to_real_node = Node(
        package='my_robot_planar',
        executable='sim_to_real',
        name='sim_to_real_bridge',
        output='screen'
    )
    '''

    # Configuración del LaunchDescription con eventos
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Usar GUI para el joint_state_publisher'
        ),
        world_cmd,
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=world_cmd,
                on_start=[spawn_robot]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=spawn_robot,
                on_start=[gz_bridge]
            )
        ),
        rviz_node,
        real_to_sim_node,  # ¡Aquí se lanza tu puente!
        #sim_to_real_node
    ])

    return launch_description
