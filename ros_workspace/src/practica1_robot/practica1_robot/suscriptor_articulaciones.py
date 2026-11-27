#!/usr/bin/python3
"""
Nodo ROS 2 que lee /joint_states del robot UR3e y guarda posición, velocidad
y aceleración de cada trayectoria en un archivo CSV propio.

La aceleración no viene en el mensaje JointState, así que se calcula derivando
la velocidad entre dos muestras consecutivas del brazo.
"""

import csv
import os
from typing import Any, List, Optional, TextIO

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from interfaces_practica1_robot.msg import EstadoArticulaciones
ARTICULACIONES: List[str] = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
TOPIC_ARTICULACIONES: str = "/joint_states"
TOPIC_TRAYECTORIA_ACTIVA: str = "/trayectoria_activa"
DIRECTORIO_SALIDA: str = "/tmp/datos_trayectorias"
UMBRAL_MOVIMIENTO: float = 0.001  # rad/s
TAMANO_COLA: int = 10
PERIODO_LOG_S: float = 0.5
DECIMALES: int = 6


class SuscriptorArticulaciones(Node):
    """Guarda los datos cinemáticos mientras hay una trayectoria en marcha."""

    def __init__(self) -> None:
        """Crea las suscripciones y la carpeta de salida."""
        super().__init__("suscriptor_articulaciones")
        self._velocidad_previa: Optional[List[float]] = None
        self._tiempo_previo: Optional[float] = None
        self._trayectoria_activa: str = ""
        self._archivo_csv: Optional[TextIO] = None
        self._escritor_csv: Optional[Any] = None

        os.makedirs(DIRECTORIO_SALIDA, exist_ok=True)
        self.create_subscription(
            JointState, TOPIC_ARTICULACIONES,
            self._callback_joint_states, TAMANO_COLA)
        self.create_subscription(
            String, TOPIC_TRAYECTORIA_ACTIVA,
            self._callback_trayectoria, TAMANO_COLA)

    def _callback_joint_states(self, mensaje: JointState) -> None:
        """Procesa una muestra del brazo solo si el robot se está moviendo."""
        # La pinza publica en el mismo topic y sin velocidades: se descarta
        if not self._es_mensaje_del_brazo(mensaje):
            return

        indices = [mensaje.name.index(nombre) for nombre in ARTICULACIONES]
        posicion = [mensaje.position[i] for i in indices]
        velocidad = [mensaje.velocity[i] for i in indices]
        tiempo = (mensaje.header.stamp.sec
                  + mensaje.header.stamp.nanosec * 1e-9)

        velocidad_previa = self._velocidad_previa
        tiempo_previo = self._tiempo_previo
        # Se actualiza siempre para que la derivada use la muestra anterior
        self._velocidad_previa = velocidad
        self._tiempo_previo = tiempo

        if not self._trayectoria_activa:
            return
        if not self._robot_en_movimiento(velocidad):
            return

        aceleracion = self._calcular_aceleracion(
            velocidad, velocidad_previa, tiempo, tiempo_previo)
        self._guardar_fila(tiempo, posicion, velocidad, aceleracion)
        self._publicar_estado_personalizado(tiempo, posicion, velocidad, aceleracion)
        self.get_logger().info(
            f"[{self._trayectoria_activa}] t={tiempo:.3f} "
            f"vel={[round(v, 3) for v in velocidad]}",
            throttle_duration_sec=PERIODO_LOG_S)

    def _publicar_estado_personalizado(
        self,
        tiempo: float,
        posicion: list[float],
        velocidad: list[float],
        aceleracion: list[float],
    ) -> None:
        """Empaqueta y publica los datos cinemáticos en el mensaje ROS 2 personalizado."""
        msg = EstadoArticulaciones()
        msg.timestamp = float(tiempo)
        msg.posicion = [float(p) for p in posicion]
        msg.velocidad = [float(v) for v in velocidad]
        msg.aceleracion = [float(a) for a in aceleracion]
        self._publicador_estado.publish(msg)

    def _callback_trayectoria(self, mensaje: String) -> None:
        """Abre o cierra el CSV según el nombre de trayectoria recibido."""
        nueva = mensaje.data.strip()
        if nueva == self._trayectoria_activa:
            return
        if nueva:
            self._abrir_csv(nueva)
            self.get_logger().info(f"Trayectoria iniciada: {nueva}")
        else:
            self.cerrar_csv()
            self.get_logger().info(
                f"Trayectoria finalizada: {self._trayectoria_activa}")
        self._trayectoria_activa = nueva

    @staticmethod
    def _es_mensaje_del_brazo(mensaje: JointState) -> bool:
        """Indica si el mensaje trae las seis articulaciones con velocidad."""
        return (len(mensaje.velocity) == len(mensaje.name)
                and all(n in mensaje.name for n in ARTICULACIONES))

    @staticmethod
    def _robot_en_movimiento(velocidad: List[float]) -> bool:
        """Indica si alguna articulación supera el umbral de velocidad."""
        return any(abs(v) > UMBRAL_MOVIMIENTO for v in velocidad)

    @staticmethod
    def _calcular_aceleracion(
        velocidad: List[float],
        velocidad_previa: Optional[List[float]],
        tiempo: float,
        tiempo_previo: Optional[float],
    ) -> List[float]:
        """Deriva la velocidad por diferencia finita hacia atrás."""
        sin_datos = [0.0] * len(velocidad)
        if velocidad_previa is None or tiempo_previo is None:
            return sin_datos
        intervalo = tiempo - tiempo_previo
        if intervalo <= 0:
            return sin_datos
        return [(actual - previa) / intervalo
                for actual, previa in zip(velocidad, velocidad_previa)]

    def _abrir_csv(self, nombre_trayectoria: str) -> None:
        """Crea el CSV de la trayectoria y escribe su cabecera."""
        self.cerrar_csv()
        ruta = os.path.join(DIRECTORIO_SALIDA, f"{nombre_trayectoria}.csv")
        self._archivo_csv = open(ruta, "w", newline="")
        self._escritor_csv = csv.writer(self._archivo_csv)
        cabecera = ["timestamp"]
        for magnitud in ("pos", "vel", "acel"):
            cabecera += [f"{magnitud}_{nombre}" for nombre in ARTICULACIONES]
        self._escritor_csv.writerow(cabecera)

    def _guardar_fila(
        self,
        tiempo: float,
        posicion: List[float],
        velocidad: List[float],
        aceleracion: List[float],
    ) -> None:
        """Escribe una muestra en el CSV abierto."""
        if self._escritor_csv is None:
            return
        valores = [tiempo, *posicion, *velocidad, *aceleracion]
        self._escritor_csv.writerow([round(v, DECIMALES) for v in valores])

    def cerrar_csv(self) -> None:
        """Cierra el CSV si hay uno abierto."""
        if self._archivo_csv is not None:
            self._archivo_csv.close()
            self._archivo_csv = None
            self._escritor_csv = None


def main(args: Optional[List[str]] = None) -> None:
    """Arranca el nodo y cierra el CSV al terminar."""
    rclpy.init(args=args)
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.cerrar_csv()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
