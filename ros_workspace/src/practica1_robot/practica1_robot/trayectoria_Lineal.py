#!/usr/bin/python3
"""
Ejecuta una trayectoria en espacio de articulaciones (Joint Space) sobre el robot UR3e.

Se mueve el robot a través de tres configuraciones distintas y se regresa
a la posición de partida. Los ángulos están expresados en radianes.
Se apoya en la clase ControlRobot proporcionada.
"""

import copy
import sys
sys.path.insert(0, '/home/laboratorio/ros_workspace/src/pruebas/pruebas')
from math import pi
from control_robot import ControlRobot
import rclpy
from std_msgs.msg import String

# Ángulo de 90° en radianes, referencia para las articulaciones del UR3e
PI_MEDIOS: float = pi / 2
# Topic compartido con el suscriptor para sincronizar el guardado
TOPIC_TRAYECTORIA_ACTIVA: str = "/trayectoria_activa"

def ejecutar_trayectoria_lineal(control: ControlRobot) -> None:
    pose = control.pose_actual()
    pose.position.z -= 0.1
    control.mover_trayectoria([copy.deepcopy(pose)])
    pose.position.z += 0.1
    control.mover_trayectoria([copy.deepcopy(pose)])  

    msg = String()
    msg.data = "joint"
    publicador.publish(msg)
    control.mover_articulaciones(posicion_home)

    # Señalizar fin de trayectoria con cadena vacía para que el suscriptor cierre el CSV
    msg.data = ""
    publicador.publish(msg)

if __name__ == '__main__':
    import time
    rclpy.init()
    control = ControlRobot()
    control.mover_articulaciones([0, -PI_MEDIOS, -PI_MEDIOS, -PI_MEDIOS, PI_MEDIOS, 0])
    try:
        ejecutar_trayectoria_lineal(control)
    finally:
        control.ejecutor.shutdown()
        control.destroy_node()
        rclpy.shutdown()