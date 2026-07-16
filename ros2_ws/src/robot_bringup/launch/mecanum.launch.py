import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # joy -> twist 変換ノード
    base_controller = Node(
        package="mecanum_controller",  # 実行したいバイナリが含まれるパッケージ名
        executable="base_controller_node",  # 実行ファイル名 (CMakeやsetup.pyで指定したもの)
        name="base_controller_node",  # ノード名 (リマップして上書きする場合)
        output="screen",  # ログを標準出力（画面）に出す設定
        parameters=[
            os.path.join(
                get_package_share_directory("mecanum_controller"),
                "config",
                "params.yaml",
            )
        ],
        remappings=[  # トピック名などのリマップ（名前空間の変更）
            ("/cmd_vel", "/cmd_vel")
        ],
    )

    # twist から各車輪の回転速度に変換するノード
    mecanum_controller = Node(
        package="mecanum_controller",
        executable="mecanum_controller_node",
        name="mecanum_controller_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("mecanum_controller"),
                "config",
                "params.yaml",
            )
        ],
    )

    # can フレーム生成ノード
    wheel_to_can = Node(
        package="wheel_to_can",
        executable="wheel_to_can_node",
        name="wheel_to_can_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory('wheel_to_can'),'config','params.yaml'],
    )

    delayed_wheel_to_can = TimerAction(
        period=2.0, actions=[wheel_to_can]  # 遅延させる時間（秒、float型）
    )
    # 5. LaunchDescriptionに要素を詰め込んで返す
    return LaunchDescription(
        [
            base_controller,
            mecanum_controller,
            delayed_wheel_to_can,
        ]
    )
