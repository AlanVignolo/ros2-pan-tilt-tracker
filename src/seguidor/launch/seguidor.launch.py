"""
Arranca el sistema completo del seguidor.

Uso:
    pixi run ros2 launch seguidor seguidor.launch.py clase:=person ganancia:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Rango útil del pan medido con el homing (±71°), dejando un margen.
PAN_MIN = -70.0
PAN_MAX = 70.0


def generate_launch_description():
    arg_clase = DeclareLaunchArgument(
        'clase',
        default_value='banana',
        description='Clase COCO a seguir, en inglés (cat, person, dog...)',
    )
    arg_ganancia = DeclareLaunchArgument(
        'ganancia',
        default_value='0.6',
        # Un valor entero ("1") llega como int y el nodo falla al declarar un parámetro double.
        description='Ganancia proporcional del controlador. Siempre con punto decimal',
    )

    camara = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 480,
            'framerate': 60.0,
        }],
    )

    detector = Node(
        package='seguidor',
        executable='nodo_detector',
        output='screen',
        parameters=[{
            'clase_objetivo': LaunchConfiguration('clase'),
            'umbral_confianza': 0.3,
        }],
    )

    puente = Node(
        package='seguidor',
        executable='nodo_puente_serie',
        output='screen',
    )

    control = Node(
        package='seguidor',
        executable='nodo_control',
        output='screen',
        parameters=[{
            'ganancia': LaunchConfiguration('ganancia'),
            'signo_pan': -1.0,
            'signo_tilt': -1.0,
            'pan_min': PAN_MIN,
            'pan_max': PAN_MAX,
        }],
    )

    joystick = Node(
        package='seguidor',
        executable='nodo_joystick',
        output='screen',
    )

    selector = Node(
        package='seguidor',
        executable='nodo_selector',
        output='screen',
        parameters=[{
            'pan_min': PAN_MIN,
            'pan_max': PAN_MAX,
        }],
    )

    voz = Node(
        package='seguidor',
        executable='nodo_voz',
        output='screen',
    )

    return LaunchDescription([
        arg_clase,
        arg_ganancia,
        camara,
        detector,
        puente,
        control,
        joystick,
        selector,
        voz,
    ])
