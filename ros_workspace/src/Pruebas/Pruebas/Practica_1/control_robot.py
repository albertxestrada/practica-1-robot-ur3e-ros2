#!/usr/bin/python3
import csv
import copy
import threading
import time
import numpy as np
from math import pi
from typing import List

import rclpy
from rclpy.time import Time
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

SISTEMA_REFERENCIA = "world"  # En ROS 2 el sistema de referencia raíz del robot es "world"
GRUPO = "robot"
EFECTOR_FINAL = "wrist_3_link"
ARTICULACIONES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
FACTOR_ESCALA = 0.1  # Velocidad y aceleración máximas (fracción de las máximas del robot)

class Suscriptor(Node):

    def __init__(self):
        super().__init__("suscriptor_basico")
        self.create_subscription(JointState, "/joint_states", self.callback_articulaciones, 10)

        self.archivo_csv = open('datos_articulaciones.csv', mode='w', newline='')
        self.escritor_csv = csv.writer(self.archivo_csv)
        self.escritor_csv.writerow(['timestamp', 'nombre_articulacion', 'posicion', 'velocidad', 'aceleracion'])
    
    def callback_articulaciones(self, mensaje: JointState):
        t = Time.from_msg(mensaje.header.stamp)
        segundos, nanosegundos = t.seconds_nanoseconds()
        timeStamp = segundos + nanosegundos / 1e9

        for i, nombre in enumerate(mensaje.name):
            posicion = mensaje.position[i] if i < len(mensaje.position) else 0.0
            velocidad = mensaje.velocity[i] if i < len(mensaje.velocity) else 0.0
            aceleracion = mensaje.effort[i] if i < len(mensaje.effort) else 0.0 # Nota: 'effort' suele ser par motor/fuerza, no aceleración directa, pero se guarda igual
            
            print(f"Tiempo: {timeStamp:.4f}, Articulación: {nombre}, Posición: {posicion:.4f}, Velocidad: {velocidad:.4f}, Aceleración: {aceleracion:.4f}")
            self.escritor_csv.writerow([timeStamp, nombre, posicion, velocidad, aceleracion])
    def destroy_node(self):
            self.get_logger().info("Cerrando CSV")
            self.archivo_csv.close()
            super().destroy_node()

class ControlRobot(Node):
    """Control del robot con MoveIt 2 a través de sus acciones y servicios (move_group)."""

    def __init__(self, id_robot: str = "") -> None:
        """id_robot: "" para un robot individual, o "203", "204"... en las escenas con dos robots."""
        super().__init__("control_robot")
        if id_robot:
            self.group_name = f"robot_{id_robot}"
            self.efector_final = f"{id_robot}_{EFECTOR_FINAL}"
            self.articulaciones = [f"{id_robot}_{j}" for j in ARTICULACIONES]
            accion_pinza = f"robot_{id_robot}/rg2_action_server"
        else:
            self.group_name = GRUPO
            self.efector_final = EFECTOR_FINAL
            self.articulaciones = list(ARTICULACIONES)
            accion_pinza = "rg2_action_server"
        self.__estado_articulaciones = {}

        self.create_subscription(JointState, "/joint_states", self.__cb_joint_states, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.cartesian_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.gripper_action_client = ActionClient(self, GripperCommand, accion_pinza)

        # Se atienden los callbacks (joint_states, tf, ...) en un hilo aparte
        self.ejecutor = MultiThreadedExecutor()
        self.ejecutor.add_node(self)
        threading.Thread(target=self.ejecutor.spin, daemon=True).start()

        self.get_logger().info("Esperando a MoveIt (move_group)...")
        self.move_group_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.cartesian_client.wait_for_service()
        self.scene_client.wait_for_service()
        while not all(j in self.__estado_articulaciones for j in self.articulaciones):
            time.sleep(0.05)

        self.añadir_suelo()

    def __cb_joint_states(self, msg: JointState) -> None:
        for nombre, posicion in zip(msg.name, msg.position):
            self.__estado_articulaciones[nombre] = posicion

    @staticmethod
    def __esperar(future):
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def articulaciones_actuales(self) -> list:
        return [self.__estado_articulaciones[j] for j in self.articulaciones]

    def __ir_a(self, restricciones: Constraints, wait: bool) -> bool:
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = FACTOR_ESCALA
        goal.request.max_acceleration_scaling_factor = FACTOR_ESCALA
        goal.request.goal_constraints.append(restricciones)
        goal.request.start_state.is_diff = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False

        goal_handle = self.__esperar(self.move_group_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        if not wait:
            return True
        resultado = self.__esperar(goal_handle.get_result_async()).result
        return resultado.error_code.val == MoveItErrorCodes.SUCCESS

    def mover_articulaciones(self, joint_goal: List[float], wait: bool = True) -> bool:
        restricciones = Constraints()
        for nombre, valor in zip(self.articulaciones, joint_goal):
            restricciones.joint_constraints.append(JointConstraint(
                joint_name=nombre, position=float(valor),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        return self.__ir_a(restricciones, wait)

    def pose_actual(self) -> Pose:
        transformacion = self.tf_buffer.lookup_transform(
            SISTEMA_REFERENCIA, self.efector_final, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=5.0))
        pose = Pose()
        pose.position.x = transformacion.transform.translation.x
        pose.position.y = transformacion.transform.translation.y
        pose.position.z = transformacion.transform.translation.z
        pose.orientation = transformacion.transform.rotation
        return pose

    def pose_a_stamped(self, pose: Pose) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = SISTEMA_REFERENCIA
        pose_stamped.pose = pose
        return pose_stamped

    def mover_a_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        restriccion_posicion = PositionConstraint()
        restriccion_posicion.header.frame_id = SISTEMA_REFERENCIA
        restriccion_posicion.link_name = self.efector_final
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        restriccion_posicion.constraint_region.primitives.append(region)
        restriccion_posicion.constraint_region.primitive_poses.append(pose_goal)
        restriccion_posicion.weight = 1.0

        restriccion_orientacion = OrientationConstraint()
        restriccion_orientacion.header.frame_id = SISTEMA_REFERENCIA
        restriccion_orientacion.link_name = self.efector_final
        restriccion_orientacion.orientation = pose_goal.orientation
        restriccion_orientacion.absolute_x_axis_tolerance = 0.01
        restriccion_orientacion.absolute_y_axis_tolerance = 0.01
        restriccion_orientacion.absolute_z_axis_tolerance = 0.01
        restriccion_orientacion.weight = 1.0

        restricciones = Constraints()
        restricciones.position_constraints.append(restriccion_posicion)
        restricciones.orientation_constraints.append(restriccion_orientacion)
        return self.__ir_a(restricciones, wait)

    def añadir_caja_a_escena_de_planificacion(self, pose_caja: Pose, name: str,
                                              tamaño: tuple = (.1, .1, .1)) -> None:
        caja = CollisionObject()
        caja.id = name
        caja.header.frame_id = SISTEMA_REFERENCIA
        caja.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(tamaño)))
        caja.primitive_poses.append(pose_caja)
        caja.operation = CollisionObject.ADD

        peticion = ApplyPlanningScene.Request()
        peticion.scene = PlanningScene()
        peticion.scene.is_diff = True
        peticion.scene.robot_state = RobotState(is_diff=True)
        peticion.scene.world.collision_objects.append(caja)
        self.__esperar(self.scene_client.call_async(peticion))

    def mover_trayectoria(self, poses: List[Pose], wait: bool = True) -> bool:
        poses_aux = copy.deepcopy(poses)
        poses_aux.insert(0, self.pose_actual())

        peticion = GetCartesianPath.Request()
        peticion.header.frame_id = SISTEMA_REFERENCIA
        peticion.start_state.is_diff = True
        peticion.group_name = self.group_name
        peticion.link_name = self.efector_final
        peticion.waypoints = poses_aux
        peticion.max_step = 0.01
        peticion.jump_threshold = 0.0
        peticion.avoid_collisions = True
        peticion.max_velocity_scaling_factor = FACTOR_ESCALA
        peticion.max_acceleration_scaling_factor = FACTOR_ESCALA
        respuesta = self.__esperar(self.cartesian_client.call_async(peticion))

        if respuesta.fraction != 1.0:
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = respuesta.solution
        goal_handle = self.__esperar(self.execute_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        if not wait:
            return True
        resultado = self.__esperar(goal_handle.get_result_async()).result
        return resultado.error_code.val == MoveItErrorCodes.SUCCESS

    def añadir_suelo(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        pose_suelo.orientation.w = 1.0
        self.añadir_caja_a_escena_de_planificacion(pose_suelo, "suelo", (2, 2, .05))

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        self.gripper_action_client.wait_for_server()
        goal = GripperCommand.Goal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        goal_handle = self.__esperar(self.gripper_action_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        result = self.__esperar(goal_handle.get_result_async()).result
        return result.reached_goal
    def generar_circulo(self, radio: float = 0.08, puntos: int = 36) -> List[Pose]:
        p0 = self.pose_actual()
        cx, cy = p0.position.x - radio, p0.position.y
        poses = []
        for k in range(1, puntos + 1):
            ang = 2 * pi * k / puntos
            p = copy.deepcopy(p0)
            p.position.x = cx + radio * np.cos(ang)
            p.position.y = cy + radio * np.sin(ang)
            poses.append(p)
        return poses

def trayectoria_joint(control: ControlRobot):
    control.mover_articulaciones([pi / 4, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0]) 
    control.mover_articulaciones([-pi / 4, -2 * pi / 3, -pi / 3, -pi / 2, pi / 2, 0],) 
    #control.mover_articulaciones([-pi / 4, pi / 4, pi / 4, pi / 4, -pi / 4, -pi / 4]) 
    control.mover_articulaciones([0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0])

def trayectoria_lineal(control: ControlRobot):
    pose = control.pose_actual()
    pose.position.z -= 0.1
    control.mover_trayectoria([copy.deepcopy(pose)])
    pose.position.z += 0.1
    control.mover_trayectoria([copy.deepcopy(pose)])


def trayectoria_circular(control: ControlRobot):
    control.mover_trayectoria(control.generar_circulo(0.08))

if __name__ == '__main__':
    rclpy.init()

    subscriptor = Suscriptor()
    # try:
    #     rclpy.spin(nodo)
    # except KeyboardInterrupt:
    #     pass


    # Crear el objeto de tipo robot
    control = ControlRobot()
    control.ejecutor.add_node(subscriptor)
    pi_medios = pi / 2
    # Mover el robot a articulaciones iniciales
    control.mover_articulaciones([0, -pi_medios, -pi_medios, -pi_medios, pi_medios, 0])

    print("Ejecutando trayectorias de prueba...")
    print("Trayectoria joint")
    trayectoria_joint(control)
    print("Trayectoria lineal")
    trayectoria_lineal(control)
    print("Trayectoria circular")
    trayectoria_circular(control)
 
    control.ejecutor.shutdown()
    control.destroy_node()
    subscriptor.destroy_node()
    rclpy.shutdown()
