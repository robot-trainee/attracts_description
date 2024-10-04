from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro

share_dir_path = os.path.join(get_package_share_directory('attracts_description'))
xacro_path = os.path.join(share_dir_path, 'urdf', 'proto1.xacro')
urdf_path = os.path.join(share_dir_path, 'urdf', 'proto1.urdf')

def generate_launch_description():
    ld = LaunchDescription()

    # xacroをロード
    doc = xacro.process_file(xacro_path)
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    # urdf_pathに対してurdfを書き出し
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    robot_state_pub_node  = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        # argumentsでURDFを出力したパスを指定
        arguments=[urdf_path])

    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', share_dir_path + '/rviz/display.rviz']
    )

    ld.add_action(robot_state_pub_node)
    ld.add_action(joint_state_pub_gui_node)
    ld.add_action(rviz_node)

    return ld
