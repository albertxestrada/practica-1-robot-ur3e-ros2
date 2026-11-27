#!/usr/bin/python3
"""
Trayectoria en espacio de articulaciones (Joint Space) para el robot UR3e.

Avisa al suscriptor por /trayectoria_activa antes y después del movimiento
para que guarde los datos en joint.csv. Los ángulos están en radianes.
"""

import sys
import time
from math import pi
from typing import List, Optional, Tuple

import rclpy
from std_msgs.msg import String

# El paquete "pruebas" no es instalable, se importa desde su carpeta
sys.path.insert(0, "/home/laboratorio/ros_workspace/src/pruebas/pruebas")
from control_robot import ControlRobot  # noqa: E402

NOMBRE_TRAYECTORIA: str = "joint"
TOPIC_TRAYECTORIA_ACTIVA: str = "/trayectoria_activa"
TAMANO_COLA: int = 10
ESPERA_MAXIMA_SUSCRIPTOR_S: float = 10.0
PERIODO_COMPROBACION_S: float = 0.1
PAUSA_SENAL_S: float = 0.5

ANGULO_RECTO: float = pi / 2
MEDIO_ANGULO_RECTO: float = pi / 4

POSICION_HOME: Tuple[float, ...] = (
    0.0, -ANGULO_RECTO, -ANGULO_RECTO,
    -ANGULO_RECTO, ANGULO_RECTO, 0.0)
CONFIGURACION_IZQUIERDA: Tuple[float, ...] = (
    MEDIO_ANGULO_RECTO, -MEDIO_ANGULO_RECTO, -ANGULO_RECTO,
    -ANGULO_RECTO, ANGULO_RECTO, MEDIO_ANGULO_RECTO)
CONFIGURACION_DERECHA: Tuple[float, ...] = (
    -MEDIO_ANGULO_RECTO, -MEDIO_ANGULO_RECTO, -ANGULO_RECTO,
    -ANGULO_RECTO, ANGULO_RECTO, -MEDIO_ANGULO_RECTO)
RECORRIDO: Tuple[Tuple[float, ...], ...] = (
    CONFIGURACION_IZQUIERDA, CONFIGURACION_DERECHA, POSICION_HOME)


class TrayectoriaJoint:
    """Ejecuta el recorrido articular y señaliza su inicio y su fin."""

    def __init__(self, control: ControlRobot) -> None:
        """Prepara el publicador sobre el nodo de control del robot."""
        self._control = control
        self._publicador = control.create_publisher(
            String, TOPIC_TRAYECTORIA_ACTIVA, TAMANO_COLA)

    def ejecutar(self) -> bool:
        """Devuelve True si el recorrido completo se ejecutó sin fallos."""
        # Ir a home antes de grabar asegura el mismo punto de partida
        if not self._mover(POSICION_HOME):
            return False
        if not self._esperar_suscriptor():
            return False

        self._senalizar(NOMBRE_TRAYECTORIA)
        try:
            return all(self._mover(objetivo) for objetivo in RECORRIDO)
        finally:
            self._senalizar("")

    def _mover(self, objetivo: Tuple[float, ...]) -> bool:
        """Mueve el robot a una configuración e informa si falla."""
        correcto = self._control.mover_articulaciones(list(objetivo))
        if not correcto:
            self._control.get_logger().error(
                f"No se pudo alcanzar la configuración {objetivo}")
        return correcto

    def _esperar_suscriptor(self) -> bool:
        """Espera a que el suscriptor esté conectado para no perder avisos."""
        limite = time.monotonic() + ESPERA_MAXIMA_SUSCRIPTOR_S
        while self._publicador.get_subscription_count() == 0:
            if time.monotonic() > limite:
                self._control.get_logger().error(
                    "No hay suscriptor: arranca suscriptor_articulaciones")
                return False
            time.sleep(PERIODO_COMPROBACION_S)
        return True

    def _senalizar(self, nombre: str) -> None:
        """Publica el nombre activo; una cadena vacía indica el final."""
        self._publicador.publish(String(data=nombre))
        # Margen para que el suscriptor abra o cierre el CSV
        time.sleep(PAUSA_SENAL_S)


def main(args: Optional[List[str]] = None) -> None:
    """Conecta con el robot y ejecuta la trayectoria."""
    rclpy.init(args=args)
    control = ControlRobot()
    try:
        TrayectoriaJoint(control).ejecutar()
    finally:
        control.ejecutor.shutdown()
        control.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
