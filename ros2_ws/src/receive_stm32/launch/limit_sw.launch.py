import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # config/parameter.yml の絶対パスを取得する
    config_file = os.path.join(
        get_package_share_directory("receive_stm32"), "config", "receive_stm32_test.yml"
    )

    return LaunchDescription(
        [
            Node(
                package="receive_stm32",
                executable="limit_sw_node",
                name="limit_sw_node",
                output="screen",
                parameters=[config_file],  # 👈 ここで yml ファイルを読み込ませる！
            )
        ]
    )
