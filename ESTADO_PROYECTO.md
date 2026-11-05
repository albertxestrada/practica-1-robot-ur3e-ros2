# Práctica 1 — Extracción y Análisis de Datos Cinemáticos del Robot UR3e con ROS 2

**Asignatura:** Robótica y Automatización Inteligente — Universidad de Deusto  
**Fecha de entrega:** 30 de noviembre de 2025  
**Equipo:**
- Alberto Estrada — suscriptor, trayectoria Joint, guardado de datos en CSV, ingesta inteligente
- Juan Camilo Bolaños — trayectorias Lineal y Circular, mensaje ROS 2 personalizado

---

## Estructura del repositorio

    ros_workspace/
    └── src/
        ├── practica1_robot/              ← paquete propio del equipo
        │   └── practica1_robot/
        │       ├── suscriptor_articulaciones.py   # Tarea 1 + guardado CSV + ingesta — Alberto
        │       └── trayectoria_joint.py           # Trayectoria Joint — Alberto
        ├── pruebas/                      ← ejemplos del profesor (referencia, no se modifica)
        │   └── pruebas/
        │       ├── control_robot.py      # clase base para mover el robot
        │       ├── suscriptor_basico.py
        │       └── publicador_basico.py
        └── launcher_robots_lab_robotica/ ← configuración del entorno (profesor)

---

## Cómo se creó el paquete

El paquete `practica1_robot` se generó desde dentro del contenedor Docker con el siguiente comando estándar de ROS 2:

    cd /home/laboratorio/ros_workspace/src
    ros2 pkg create --build-type ament_python practica1_robot

Esto genera automáticamente la estructura de carpetas, el `package.xml` y el `setup.py`.

---

## Cómo arrancar el entorno

1. Desde la carpeta raíz del repositorio, lanzar la simulación:

        ./lab.sh sim 203

2. Abrir el entorno visual en el navegador: `http://localhost:6081`

3. Abrir una terminal dentro del contenedor Docker:

        docker exec -it practica1-desktop-1 bash

---

## Parte 1 — Requisitos Mínimos

### Tarea 1 — Suscriptor de articulaciones (`suscriptor_articulaciones.py`) — Alberto ✅

Nodo ROS 2 que se suscribe al topic `/joint_states` y extrae en tiempo real la evolución temporal de:

- **Posición** de cada articulación (en radianes)
- **Velocidad** angular (en rad/s)
- **Aceleración** calculada numéricamente mediante diferencias finitas: `a ≈ Δv / Δt`

La aceleración no forma parte del mensaje `JointState`, por lo que se calcula derivando la velocidad entre dos mensajes consecutivos. En el primer mensaje se devuelve cero porque todavía no existe estado previo.

Además suscribe al topic `/trayectoria_activa` para saber cuándo guardar datos y para qué trayectoria.

**Cómo ejecutarlo** (siempre antes que la trayectoria):

    python3 /home/laboratorio/ros_workspace/src/practica1_robot/practica1_robot/suscriptor_articulaciones.py

---

### Tarea 2 — Trayectoria Joint (`trayectoria_joint.py`) — Alberto ✅

Script que mueve el robot UR3e a través de tres configuraciones articulares distintas y regresa a la posición de inicio. Utiliza la clase `ControlRobot` proporcionada por el profesor en el paquete `pruebas`.

Publica en `/trayectoria_activa` el nombre `"joint"` antes de ejecutar y una cadena vacía al terminar, para que el suscriptor sepa cuándo guardar los datos.

**Cómo ejecutarlo** (con el suscriptor ya corriendo):

    python3 /home/laboratorio/ros_workspace/src/practica1_robot/practica1_robot/trayectoria_joint.py

### Tarea 2 — Trayectoria Lineal (`trayectoria_lineal.py`) — Juan Camilo 🔄 Pendiente

Mover el efector final en línea recta a través de una serie de poses cartesianas usando `mover_trayectoria()` de `ControlRobot`. Debe publicar `"lineal"` en `/trayectoria_activa` antes de ejecutar y `""` al terminar.

---

### Tarea 2 — Trayectoria Circular (`trayectoria_circular.py`) — Juan Camilo 🔄 Pendiente

Mover el efector final describiendo un arco o círculo en el espacio cartesiano. Debe publicar `"circular"` en `/trayectoria_activa` antes de ejecutar y `""` al terminar.

---

### Tarea 3 — Guardado de datos por trayectoria — Alberto ✅

El suscriptor guarda los datos en archivos CSV dentro del contenedor en `/tmp/datos_trayectorias/`. Se genera un archivo por trayectoria:

- `joint.csv` — datos de la trayectoria Joint
- `lineal.csv` — datos de la trayectoria Lineal (cuando Juan Camilo la implemente)
- `circular.csv` — datos de la trayectoria Circular (cuando Juan Camilo la implemente)

Cada fila contiene: timestamp, posición, velocidad y aceleración de las 6 articulaciones.

#### Modificaciones a `control_robot.py` del profesor

El archivo original requirió dos ajustes para que el guardado de datos funcionara correctamente:

**1. `time.sleep(0.5)` tras arrancar el hilo del ejecutor**

```python
threading.Thread(target=self.ejecutor.spin, daemon=True).start()
time.sleep(0.5)  # Dar tiempo al ejecutor para inicializarse
```

`MultiThreadedExecutor` corre en un hilo daemon asíncrono. Sin esta pausa, `wait_for_server()` se llama antes de que el ejecutor esté procesando callbacks, provocando una condición de carrera (*race condition*) con el error `cannot use Destroyable because destruction was requested`. El robot nunca llega a moverse y el CSV queda vacío.

**Lección:** en ROS 2, siempre que se use un executor en hilo separado, hay que darle tiempo de arranque antes de llamar a cualquier servicio o acción.

**2. `get_logger().info()` en lugar de `print()`**

Los mensajes de estado del `__init__` se escriben a través del sistema de logging de ROS 2, garantizando que aparezcan con timestamp y nombre de nodo en los logs del sistema.

#### Errores encontrados durante el desarrollo

**CSV solo con encabezado (sin filas de datos)**

El archivo `joint.csv` se creaba correctamente pero sin ninguna fila de datos. La causa era la race condition descrita arriba: el robot no llegaba a moverse de verdad, así que el filtro de ingesta inteligente descartaba todos los mensajes por falta de velocidad. Al añadir el `time.sleep(0.5)`, el robot se movió y los datos llegaron correctamente.

---

## Parte 2 — Retos Opcionales

### Ingesta inteligente de datos — Alberto ✅

El suscriptor solo guarda datos cuando el robot está en movimiento, comparando la velocidad de cada articulación contra un umbral mínimo (`UMBRAL_MOVIMIENTO = 0.001 rad/s`). Los periodos estáticos se ignoran.

---

### Mensaje ROS 2 personalizado (.msg) — Juan Camilo 🔄 Pendiente

Crear un archivo `.msg` propio que empaquete en un único mensaje los datos de posición, velocidad y aceleración. El suscriptor lo publicará en un topic propio cada vez que procese un nuevo estado del robot.

---

## Arquitectura de comunicación entre nodos

    trayectoria_joint.py  ──publica──►  /trayectoria_activa  ──escucha──►  suscriptor_articulaciones.py
    robot UR3e            ──publica──►  /joint_states         ──escucha──►  suscriptor_articulaciones.py

El suscriptor combina ambas señales: solo guarda en CSV cuando `/trayectoria_activa` tiene un nombre activo y el robot está en movimiento.

---

## Estado actual

| Tarea | Parte | Responsable | Estado |
|---|---|---|---|
| Suscriptor /joint_states + aceleración | Obligatoria | Alberto | ✅ Implementado |
| Trayectoria Joint | Obligatoria | Alberto | ✅ Implementado |
| Guardado de datos en CSV por trayectoria | Obligatoria | Alberto | ✅ Implementado |
| Ingesta inteligente (solo al moverse) | Opcional | Alberto | ✅ Implementado |
| Trayectoria Lineal | Obligatoria | Juan Camilo | 🔄 Pendiente |
| Trayectoria Circular | Obligatoria | Juan Camilo | 🔄 Pendiente |
| Mensaje ROS 2 personalizado (.msg) | Opcional | Juan Camilo | 🔄 Pendiente |

---

## Prueba final

La prueba definitiva se ejecutó con éxito. El archivo `/tmp/datos_trayectorias/joint.csv` generado contiene **2718 filas de datos** (más el encabezado), con timestamp, posición, velocidad y aceleración de las 6 articulaciones durante toda la trayectoria Joint. La ingesta inteligente confirmó su funcionamiento al descartar los periodos en que el robot estaba estático.

---

## Notas

- El paquete `pruebas` es el proporcionado por el profesor; los cambios a `control_robot.py` descritos arriba son los únicos realizados sobre él.
- La clase `ControlRobot` se importa directamente desde su ruta dentro del contenedor usando `sys.path.insert`.
- Las tareas de Juan Camilo son completamente independientes y pueden desarrollarse en paralelo sin depender del trabajo de Alberto.
- Para que el sistema funcione completo, Juan Camilo debe publicar en `/trayectoria_activa` el nombre de cada trayectoria antes y después de ejecutarla, igual que hace `trayectoria_joint.py`.
- Los archivos `build/`, `install/`, `log/` y `__pycache__/` están excluidos del repositorio mediante `.gitignore`.
