#!/usr/bin/python3

import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class Suscriptor(Node):

    def __init__(self):
        super().__init__("suscriptor_basico")
        self.create_subscription(JointState, "/joint_states", self.callback_articulaciones, 10)

        self.archivo_csv = open('datos_articulaciones.csv', mode='w', newline='')
        self.escritor_csv = csv.writer(self.archivo_csv)
        self.escritor_csv.writerow(['timestamp', 'nombre_articulacion', 'posicion', 'velocidad', 'aceleracion'])
    
    def callback_articulaciones(self, mensaje: JointState):
        segundos = mensaje.header.stamp.sec
        nanosegundos = mensaje.header.stamp.nanoseconds
        timeStamp = segundos + nanosegundos / 1e9

        for i, nombre in enumerate(mensaje.name):
            posicion = mensaje.position[i]
            velocidad = mensaje.velocity[i]
            aceleracion = mensaje.effort[i]
            print(f"Tiempo: {self.timeStamp:.4f}, Articulación: {nombre}, Posición: {posicion:.4f}, Velocidad: {velocidad:.4f}, Aceleración: {aceleracion:.4f}")
            self.escritor_csv.writerow([timeStamp, nombre, posicion, velocidad, aceleracion])
    def destroy_node(self):
            self.get_logger().info("Cerrando CSV")
            self.archivo_csv.close()
            super().destroy_node()

if __name__ == '__main__':
    rclpy.init()
    nodo = Suscriptor()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()
