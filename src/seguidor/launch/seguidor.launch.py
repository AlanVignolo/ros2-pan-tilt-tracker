# Launch file del seguidor: arranca los cuatro nodos con un solo comando.
# Uso: pixi run ros2 launch seguidor seguidor.launch.py clase:=person ganancia:=0.2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ros2 launch busca y llama a esta función, con este nombre exacto.
# Devuelve la lista de todo lo que hay que arrancar.
def generate_launch_description():

    # Argumentos que se pueden cambiar desde la línea de comandos.
    # Si no los pasás, se usa el default_value.
    arg_clase = DeclareLaunchArgument(
        'clase',
        default_value='banana',
        description='Clase de YOLO a seguir, en inglés (cat, person, dog...)',
    )
    arg_ganancia = DeclareLaunchArgument(
        'ganancia',
        default_value='0.2',
        description='Ganancia proporcional del controlador. Escribirla siempre con punto decimal',
    )

    # Nodo de la cámara, del paquete usb_cam.
    camara = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen',  # Mostrar sus mensajes en esta terminal
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 480,
            'framerate': 60.0,
        }],
    )

    # Detector YOLO. LaunchConfiguration('clase') se reemplaza al lanzar
    # por el valor del argumento 'clase'.
    detector = Node(
        package='seguidor',
        executable='nodo_detector',
        output='screen',
        parameters=[{
            'clase_objetivo': LaunchConfiguration('clase'),
            'umbral_confianza': 0.3,
        }],
    )

    # Puente serie con la Nucleo. Usa los parámetros por defecto del código.
    puente = Node(
        package='seguidor',
        executable='nodo_puente_serie',
        output='screen',
    )

    # Controlador. Los signos en -1.0 son con este montaje mecánico.
    control = Node(
        package='seguidor',
        executable='nodo_control',
        output='screen',
        parameters=[{
            'ganancia': LaunchConfiguration('ganancia'),
            'signo_pan': -1.0,
            'signo_tilt': -1.0,
            # Rango útil del pan medido con el homing (±71°), con margen.
            'pan_min': -70.0,
            'pan_max': 70.0,
        }],
    )

    # Recibe el joystick del ESP32 por UDP y lo publica en /joy.
    joystick = Node(
        package='seguidor',
        executable='nodo_joystick',
        output='screen',
    )

    # Elige quién mueve los motores: el controlador (/cmd_auto) o el joystick (/joy),
    # según el interruptor del ESP32. Publica en /cmd_angulos, que escucha el puente.
    selector = Node(
        package='seguidor',
        executable='nodo_selector',
        output='screen',
        parameters=[{
            # Mismos límites que el controlador.
            'pan_min': -70.0,
            'pan_max': 70.0,
        }],
    )

    # Escucha el micrófono y cambia el objeto a seguir con /set_target.
    voz = Node(
        package='seguidor',
        executable='nodo_voz',
        output='screen',
    )

    # El orden de la lista no garantiza el orden de arranque.
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