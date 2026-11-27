# Práctica 1: Extracción y análisis de datos cinemáticos del robot UR3e con ROS 2

**Asignatura:** Robótica y Automatización Inteligente, Universidad de Deusto
**Equipo:** Alberto Estrada y Juan Camilo Bolaños

El proyecto ejecuta tres trayectorias en un UR3e (articular, lineal y circular) y registra la posición, velocidad y aceleración de sus seis articulaciones, con un archivo CSV por trayectoria. Se ejecuta sobre ROS 2 Jazzy dentro de Docker.

## Estructura del proyecto

```
.
├── lab.sh / lab.bat / lab.ps1       Scripts para arrancar y usar el entorno
├── docker-compose.yml, dockerfiles/ Imagen de ROS 2 Jazzy con escritorio web
├── ejemplos/                        Ejemplos originales de la asignatura
└── ros_workspace/src/
    ├── practica1_robot/             Paquete propio de la práctica
    │   ├── practica1_robot/
    │   │   ├── suscriptor_articulaciones.py   Lectura, cálculo, CSV y mensaje propio
    │   │   ├── trayectoria_joint.py           Trayectoria articular
    │   │   ├── trayectoria_Lineal.py          Trayectoria lineal
    │   │   └── trayectoria_Circular.py        Trayectoria circular
    │   ├── package.xml
    │   └── setup.py
    ├── interfaces_practica_1/       Paquete del mensaje personalizado
    │   └── msg/EstadoArticulaciones.msg
    ├── pruebas/pruebas/
    │   └── control_robot.py         Clase ControlRobot que usan las trayectorias
    ├── launcher_robots_lab_robotica/  Lanzadores de simulación y robot real
    └── rg2_driver/                    Driver de la pinza
```

`control_robot.py` es el ejemplo de la asignatura, con una pausa de 0,5 s tras arrancar el ejecutor y mensajes de registro durante la conexión con MoveIt. Las trayectorias lo importan desde esa ruta, por lo que la carpeta `pruebas` es necesaria.

## Cómo funciona

```
trayectoria_*.py ──/trayectoria_activa──► suscriptor_articulaciones.py ──► <nombre>.csv
robot UR3e       ──/joint_states───────► suscriptor_articulaciones.py ──► /estado_cinematico
```

1. Cada trayectoria publica su nombre (`joint`, `lineal` o `circular`) en `/trayectoria_activa` y una cadena vacía al terminar.
2. El suscriptor lee `/joint_states` y descarta los mensajes de la pinza, que comparten topic y no incluyen velocidades.
3. La aceleración no viene en el mensaje: se calcula como la diferencia de velocidad entre dos muestras dividida por el tiempo transcurrido.
4. **Ingesta inteligente:** solo se guardan datos si hay una trayectoria activa y alguna articulación supera 0,001 rad/s.
5. **Mensaje personalizado:** cada muestra guardada se publica también en `/estado_cinematico` con el tipo `interfaces_practica_1/msg/EstadoArticulaciones` (`timestamp`, `posicion`, `velocidad`, `aceleracion`).

| Trayectoria | Movimiento |
|---|---|
| Joint | Desde home, dos configuraciones simétricas a izquierda y derecha, y regreso |
| Lineal | El efector baja 10 cm en línea recta, sube y vuelve a home |
| Circular | El efector describe un círculo horizontal de 8 cm de radio |

## Requisitos

- Git
- Docker (Docker Desktop en Windows y macOS). Si no está instalado: `./lab.sh instalar-docker`

En Windows, sustituir `./lab.sh` por `.\lab.bat` en todos los comandos.

## Ejecución en simulación

Se necesitan cuatro terminales. El número de robot puede ser 203, 204, 205 o 206.

### Terminal 1: arrancar, compilar y simular

Abrir antes Docker Desktop (o el servicio de Docker).

```bash
git clone https://github.com/albertxestrada/practica-1-robot-ur3e-ros2.git
```

```bash
cd practica-1-robot-ur3e-ros2
```

```bash
./lab.sh iniciar
```

La primera vez se construye la imagen, lo que puede tardar varios minutos.

```bash
./lab.sh compilar
```

```bash
./lab.sh sim 206
```

El escritorio con la simulación está en `http://localhost:6081` (contraseña `laboratorio`). Dejar esta terminal abierta.

### Terminal 2: suscriptor

Desde la carpeta del proyecto:

```bash
./lab.sh terminal
```

Ya dentro del contenedor:

```bash
ros2 run practica1_robot suscriptor_articulaciones
```

No imprime datos hasta que empieza una trayectoria. Debe seguir abierto mientras se ejecutan las tres.

### Terminal 3: trayectorias

```bash
./lab.sh terminal
```

Ejecutar una cada vez, esperando a que termine:

```bash
ros2 run practica1_robot trayectoria_joint
```

```bash
python3 ~/ros_workspace/src/practica1_robot/practica1_robot/trayectoria_Lineal.py
```

```bash
python3 ~/ros_workspace/src/practica1_robot/practica1_robot/trayectoria_Circular.py
```

## Comprobación de resultados

### Terminal 4

```bash
./lab.sh terminal
```

**Archivos generados**, uno por trayectoria:

```bash
wc -l /tmp/datos_trayectorias/*.csv
```

Deben aparecer `joint.csv`, `lineal.csv` y `circular.csv`, todos con más de una línea.

**Columnas completas** (`timestamp` más posición, velocidad y aceleración de 6 articulaciones):

```bash
for f in /tmp/datos_trayectorias/*.csv; do echo "$f"; awk -F, '{print NF}' "$f" | sort | uniq -c; done
```

Cada archivo debe mostrar un único valor: `19`.

**Ingesta inteligente** (filas con el robot quieto):

```bash
awk -F, 'NR>1{m=0; for(i=8;i<=13;i++){v=$i+0; if(v<0)v=-v; if(v>0.001)m=1} if(!m)s++} END{print "filas quietas:", s+0}' /tmp/datos_trayectorias/joint.csv
```

El resultado esperado es 0. Un valor muy bajo puede deberse al redondeo a 6 decimales en el umbral.

**Mensaje personalizado.** Ejecutar este comando antes de lanzar una trayectoria; mostrará el primer mensaje publicado:

```bash
ros2 topic echo /estado_cinematico --once
```

**Copiar los CSV al equipo anfitrión** (desde una terminal fuera del contenedor, en la carpeta del proyecto):

```bash
docker compose cp desktop:/tmp/datos_trayectorias ./datos_trayectorias
```

Los CSV se sobrescriben en cada ejecución y se pierden si se elimina el contenedor.

## Parar el entorno

`Ctrl+C` detiene la simulación y el suscriptor. Para parar el contenedor:

```bash
./lab.sh parar
```

## Problemas frecuentes

| Problema | Solución |
|---|---|
| Docker no está en ejecución | Abrir Docker Desktop y esperar a que esté listo |
| `Package 'practica1_robot' not found` | Ejecutar `./lab.sh compilar` y abrir una terminal nueva |
| `No hay suscriptor` al lanzar Joint | Arrancar antes `suscriptor_articulaciones` || No hay ningún contenedor en marcha | Ejecutar `./lab.sh iniciar` |
| `lab.sh` no es ejecutable | Usar `bash lab.sh iniciar` |
| Permisos al compilar en Linux | `chmod -R a+rwX ros_workspace` |
| El contenedor se bloquea por memoria | `LAB_MEM=6g LAB_CPUS=2 ./lab.sh iniciar` |

En los equipos del laboratorio (Linux con X11) se puede usar `./lab.sh iniciar --lab` o `--lab-gpu`.
