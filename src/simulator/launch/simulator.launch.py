import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前package的目录，并以此得到地图文件的文件名
    package_dir = get_package_share_directory('simulator')
    yaml_filename = os.path.join(package_dir, 'maps', 'b.yaml')
    # 设置rviz的加载配置文件位置
    rviz_config = os.path.join(package_dir, 'rviz', 'planner.rviz')
    # 设置需要被lifecycle manager管理的lifecycle node
    lifecycle_nodes = ['map_server']
    return LaunchDescription([
        # 启动仿真时间节点，在/clock话题中发布时钟
        Node(
           package='simulator',
           executable='clock_node',
           name='clock',
           output='screen'
           ),
        # rviz控件指令桥接器，用于planner server指令翻译发送
        Node(
           package='simulator',
           executable='planner_bridge',
           name='planner_bridge',
           parameters=[{'use_sim_time': True},
                       {'planner_id': "GridBased"}],
           output='screen'
           ),           
        # 发布一个静态的坐标变换关系，用于描述map->base_link
        Node(
           package='simulator',
           executable='transform',
           parameters=[{'use_sim_time': True},
                       {'pose_x': 0.0},{'pose_y': 0.0}, {'pose_yaw': 0.0}]),
        # 打开一个用于显示的rviz2终端
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            ),
        # 启动map_server并加载上面变量中的map文件
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': yaml_filename},
                        {'use_sim_time': True}]),
        # 启动lifecycle manager节点，用于管理map_server的生命周期
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_simulator',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]) 
    ])
