#!/usr/bin/python3

"""
Nodo ROS 2 que suscribe al topic /joint_states del robot UR3e y extrae
la posición y velocidad de cada articulación. La aceleración se calcula
numéricamente derivando la velocidad respecto al tiempo entre mensajes.
"""

import csv
import os
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

UMBRAL_MOVIMIENTO: float = 0.001
DIRECTORIO_SALIDA: str = "/tmp/datos_trayectorias"

class SuscriptorArticulaciones(Node):
    """
    Nodo suscriptor que lee el estado de las articulaciones del UR3e.

    Se suscribe a /joint_states y calcula la aceleración por diferencia
    finita porque el mensaje JointState no la incluye directamente.
    """
    def __init__(self):
        super().__init__("suscriptor_articulaciones")
        # Estado previo necesario para calcular la derivada numérica
        self._velocidad_previa = None
        self._tiempo_previo = None
        self._trayectoria_activa: str = ""
        self._archivo_csv = None
        self._escritor_csv = None
        self.create_subscription(JointState, "/joint_states", self._callback_joint_states, 10)
        self.create_subscription(String, "/trayectoria_activa", self._callback_trayectoria, 10)
        os.makedirs(DIRECTORIO_SALIDA, exist_ok=True)

    
    def _callback_joint_states(self, mensaje: JointState):
        """
        Procesa cada mensaje de /joint_states y registra posición,
        velocidad y aceleración. En el primer mensaje la aceleración
        es cero porque todavía no hay estado previo con el que derivar.
        """
        # El timestamp llega separado en segundos y nanosegundos; los unimos en un float
        tiempo_actual = mensaje.header.stamp.sec + mensaje.header.stamp.nanosec * 1e-9
        posicion = list(mensaje.position)
        velocidad = list(mensaje.velocity)

        if self._velocidad_previa is not None and self._tiempo_previo is not None:
            dt = tiempo_actual - self._tiempo_previo
            if dt > 0:
                # Diferencia finita hacia atrás: a ≈ (v_actual - v_previa) / Δt
                aceleracion = [(v - vp) / dt for v, vp in zip(velocidad, self._velocidad_previa)]
            else:
                # dt=0 ocurre si dos mensajes tienen el mismo timestamp (evitar división por cero)
                aceleracion = [0.0] * len(velocidad)
        else:
            aceleracion = [0.0] * len(velocidad)

        # Guardamos los valores actuales para usarlos como "previos" en la siguiente llamada
        self._velocidad_previa = velocidad
        self._tiempo_previo = tiempo_actual

        self.get_logger().info(
            f"t={tiempo_actual:.3f} | pos={[round(p,3) for p in posicion]} | "
            f"vel={[round(v,3) for v in velocidad]} | "
            f"acel={[round(a,3) for a in aceleracion]}"
        )
        if self._trayectoria_activa and self._robot_en_movimiento(velocidad):
            self._guardar_fila(tiempo_actual, posicion, velocidad, aceleracion)

    def _callback_trayectoria(self, mensaje: String) -> None:
        nueva = mensaje.data.strip()
        if nueva and nueva != self._trayectoria_activa:
            self._abrir_csv(nueva)
            self.get_logger().info(f"Trayectoria iniciada: {nueva}")
        elif not nueva and self._trayectoria_activa:
            self._cerrar_csv()
            self.get_logger().info(f"Trayectoria finalizada: {self._trayectoria_activa}")
        self._trayectoria_activa = nueva

    def _robot_en_movimiento(self, velocidad: list) -> bool:
        """Devuelve True si alguna articulación supera el umbral mínimo de velocidad."""
        return any(abs(v) > UMBRAL_MOVIMIENTO for v in velocidad)

    def _abrir_csv(self, nombre_trayectoria: str) -> None:
        """Crea y abre un archivo CSV nuevo para la trayectoria indicada."""
        self._cerrar_csv()
        ruta = os.path.join(DIRECTORIO_SALIDA, f"{nombre_trayectoria}.csv")
        self._archivo_csv = open(ruta, "w", newline="")
        self._escritor_csv = csv.writer(self._archivo_csv)
        self._escritor_csv.writerow([
            "timestamp",
            "pos_1", "pos_2", "pos_3", "pos_4", "pos_5", "pos_6",
            "vel_1", "vel_2", "vel_3", "vel_4", "vel_5", "vel_6",
            "acel_1", "acel_2", "acel_3", "acel_4", "acel_5", "acel_6"
        ])

    def _cerrar_csv(self) -> None:
        """Cierra el archivo CSV si hay uno abierto."""
        if self._archivo_csv:
            self._archivo_csv.close()
            self._archivo_csv = None
            self._escritor_csv = None

    def _guardar_fila(self, tiempo: float, posicion: list, velocidad: list, aceleracion: list) -> None:
        """Escribe una fila con los datos cinemáticos actuales en el CSV abierto."""
        if self._escritor_csv:
            self._escritor_csv.writerow(
                [round(tiempo, 6)] +
                [round(p, 6) for p in posicion] +
                [round(v, 6) for v in velocidad] +
                [round(a, 6) for a in aceleracion]
            )

if __name__ == '__main__':
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo._cerrar_csv()
    nodo.destroy_node()
    rclpy.try_shutdown()
