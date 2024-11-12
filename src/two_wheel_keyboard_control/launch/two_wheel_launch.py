import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Узел `keyboard_talker` под sudo с привилегиями root
    keyboard_talker_cmd = ExecuteProcess(
        cmd=[
            'sudo', '-E', 'bash', '-c', 
            'source /home/inj/ros2_iron/install/setup.bash && ' +
            'source /home/inj/ros2_iron/src/two_wheel_keyboard_control/.venv/bin/activate && ' +
            'ros2 run two_wheel_keyboard_control keyboard_talker'
        ],
        shell=True
    )

    # Узел `com_listener` без sudo
    com_listener_cmd = ExecuteProcess(
        cmd=[
            'sudo', '-E', 'bash', '-c',
            'source /home/inj/ros2_iron/install/setup.bash && ' +
            'source /home/inj/ros2_iron/src/two_wheel_keyboard_control/.venv/bin/activate && ' +
            'ros2 run two_wheel_keyboard_control com_listener'
        ],
        shell=True
    )

    return LaunchDescription([
        keyboard_talker_cmd,
        com_listener_cmd
    ])
