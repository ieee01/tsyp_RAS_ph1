from launch import LaunchDescription
from launch.actions import LogInfo
def generate_launch_description():
    return LaunchDescription([LogInfo(msg='Navigation is included by demo.launch.py. This compatibility launch file is intentionally a no-op.')])
