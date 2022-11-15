import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rotate',
            executable='rotate_action',
            name='rotate_action'),
        launch_ros.actions.Node(
            package='face_recog',
            executable='recognize_face_action',
            name='face_recog',
        ),
        launch_ros.actions.Node(
            package='demoRunner',
            executable='demoRunner',
            name='demoRunner',
        ),
    ])