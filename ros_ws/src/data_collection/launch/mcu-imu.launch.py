import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='mcu_interface',
            executable='mcu',
            name='mcu',
            namespace='',
            parameters=[{'port': '/dev/ttyUSB0'}]  # Если требуются параметры узла
        )
    ])

