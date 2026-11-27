# Estado del proyecto: Práctica 1, datos cinemáticos del robot UR3e con ROS 2

**Asignatura:** Robótica y Automatización Inteligente, Universidad de Deusto
**Fecha de entrega:** 30 de noviembre de 2026
**Última actualización:** 27 de noviembre de 2026

**Equipo:**

- Alberto Estrada: suscriptor, cálculo de aceleración, guardado en CSV, ingesta inteligente y trayectoria Joint
- Juan Camilo Bolaños: trayectorias Lineal y Circular y mensaje ROS 2 personalizado

Las instrucciones de ejecución y comprobación están en [README.md](README.md).

## Estado por requisito

| Requisito | Parte | Responsable | Implementado | Probado en simulación |
|---|---|---|---|---|
| Suscriptor de `/joint_states` | Obligatoria | Alberto | Sí | Sí (antes de añadir el mensaje propio) |
| Trayectoria Joint | Obligatoria | Alberto | Sí | Sí |
| Posición, velocidad y aceleración en CSV | Obligatoria | Alberto | Sí | Sí, con Joint |
| Trayectoria Lineal | Obligatoria | Juan Camilo | Sí | Pendiente |
| Trayectoria Circular | Obligatoria | Juan Camilo | Sí | Pendiente |
| Ingesta inteligente | Opcional | Alberto | Sí | Pendiente de comprobar las filas en reposo |
| Mensaje ROS 2 personalizado | Opcional | Juan Camilo | Sí | Pendiente |

## Resultado de las pruebas

Prueba de la trayectoria Joint en simulación (27 de noviembre): `joint.csv` con 5461 líneas (cabecera y 5460 muestras), todas con las 19 columnas completas. La prueba se hizo antes de integrar la publicación del mensaje personalizado en el suscriptor.

## Componentes

### Suscriptor (`suscriptor_articulaciones.py`)

- Lee `/joint_states` y selecciona las seis articulaciones del brazo en un orden fijo.
- Calcula la aceleración por diferencia finita hacia atrás: `a = (v_actual - v_previa) / dt`. El mensaje `JointState` no la incluye.
- Guarda un CSV por trayectoria en `/tmp/datos_trayectorias/`, con la marca de tiempo del mensaje.
- Solo guarda si hay una trayectoria activa y alguna articulación supera 0,001 rad/s.
- Publica cada muestra guardada en `/estado_cinematico`.

### Trayectorias

| Archivo | Movimiento | Señal en `/trayectoria_activa` |
|---|---|---|
| `trayectoria_joint.py` | Home, izquierda, derecha y regreso | `joint` |
| `trayectoria_Lineal.py` | 10 cm hacia abajo, subida y vuelta a home | `lineal` |
| `trayectoria_Circular.py` | Círculo horizontal de 8 cm de radio | `circular` |

Todas envían una cadena vacía al terminar y usan la clase `ControlRobot` de `pruebas/pruebas/control_robot.py`.

### Mensaje personalizado (`interfaces_practica_1`)

`EstadoArticulaciones.msg` agrupa `timestamp`, `posicion`, `velocidad` y `aceleracion`. Está en un paquete `ament_cmake` propio porque la generación de interfaces lo requiere.

## Cambios sobre el ejemplo `control_robot.py`

Respecto a `ejemplos/control_robot.py`, la copia que usan las trayectorias añade:

- Una pausa de 0,5 s tras arrancar el hilo del ejecutor. Se añadió al investigar el error `cannot use Destroyable because destruction was requested`; no se confirmó que sea su única causa.
- Mensajes de registro durante la conexión con MoveIt.

La lógica de movimiento no cambia.

## Problemas encontrados y soluciones

| Problema | Causa | Solución |
|---|---|---|
| CSV solo con la cabecera | La trayectoria Joint solo movía el robot a home; si ya estaba allí no había movimiento | Recorrer dos configuraciones más y volver a home |
| Avisos de inicio que podían perderse | Se publicaban antes de que el suscriptor estuviera conectado | La trayectoria Joint espera a que haya un suscriptor |
| Filas sin aceleración | La pinza publica en `/joint_states` sin velocidades | El suscriptor descarta esos mensajes |
| El suscriptor no arrancaba tras añadir el mensaje | Nombre de paquete incorrecto en el `import` y publicador sin crear | Corregidos el `import` y la creación del publicador |

