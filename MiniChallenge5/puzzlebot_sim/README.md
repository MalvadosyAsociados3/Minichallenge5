# MiniChallenge 5 — Multi-Puzzlebot + Propagación de Covarianza (ROS 2 Humble)

Simulación y control simultáneo de **dos puzzlebots** en namespaces independientes (`/robot1` y `/robot2`). Cada robot tiene su propio simulador cinemático, localización con **propagación de covarianza por dead-reckoning**, generador de waypoints y controlador.

Sobre el multi-robot del reto anterior se añade:
- **Task 1** — relleno de la matriz de covarianza 3x3 (x, y, θ) en el mensaje `nav_msgs/Odometry` mediante el modelo `Σₖ = Hₖ Σₖ₋₁ Hₖᵀ + Qₖ`. RViz dibuja las elipses de confianza automáticamente.
- **Task 2** — parámetros `kr`, `kl` por robot (en YAML) para calibrar el ruido del modelo `Σ_Δ = diag(kr·|ωr|, kl·|ωl|)`.

Solo se usan NumPy + librería estándar.

---

## Arquitectura

```
                       /robot1/cmd_vel                /robot1/wr,/wl
   +-----------------+ -------------> +--------------+ -------------> +----------------+
   | /robot1/control |                | /robot1/sim  |                | /robot1/local. |
   | _node           | <------------- |              |                |                |
   +-----------------+ /robot1/odom   +--------------+                +----------------+
            ^                                                                  |
            | /robot1/current_goal                                              |
   +-----------------+                                                         |  TF
   | /robot1/point_  | <------------- /robot1/goal_reached --------------------+
   | generator       |
   +-----------------+
                              ... idem para /robot2 ...

                          (TF compartido)
                         world ─┬─ robot1/odom ─ robot1/base_footprint ─ robot1/base_link ─ ...
                                └─ robot2/odom ─ robot2/base_footprint ─ robot2/base_link ─ ...
```

Cada grupo (un robot) contiene **5 nodos**:

| Nodo | Responsabilidad |
|---|---|
| `robot_state_publisher` | URDF + `frame_prefix=robotN/` → publica TFs de los links |
| `puzzlebot_sim` | Cinemática diferencial: `cmd_vel → wr, wl, joint_states, sim_pose` |
| `localisation` | `wr, wl → /odom` (con covarianza 3x3 propagada), TF `world → robotN/odom → robotN/base_footprint` |
| `point_generator` | Publica waypoints en `/current_goal` y `/planned_path` |
| `control_node` | Go-to-goal difuso → `/cmd_vel`, confirma con `/goal_reached` |

Parámetros físicos:
- Radio de rueda `r = 0.05 m`
- Distancia entre ruedas `L = 0.19 m`

---

## Compilar

Una sola vez por sesión:

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_sim --symlink-install
source install/setup.bash
```



En **cada terminal nueva**:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Lanzar la simulación multi-robot

```bash
ros2 launch puzzlebot_sim multi_robot_launch.py
```



```bash


# YAML alternativo para uno de los robots
ros2 launch puzzlebot_sim multi_robot_launch.py \
  robot1_params:=/ruta/a/mi_robot1.yaml
```

---



## Verificación rápida (en otra terminal)

```bash
# Topics namespaced (deben aparecer ambos)
ros2 topic list | grep -E "robot1|robot2" | sort

# Pose estimada
ros2 topic echo /robot1/odom --once
ros2 topic echo /robot2/odom --once

# TF (debe existir y moverse)
ros2 run tf2_ros tf2_echo world robot1/base_footprint
ros2 run tf2_ros tf2_echo world robot2/base_footprint

# Árbol de TF a PDF
ros2 run tf2_tools view_frames
```

Topics esperados (10 por robot):
```
/robot1/cmd_vel        /robot2/cmd_vel
/robot1/current_goal   /robot2/current_goal
/robot1/goal_reached   /robot2/goal_reached
/robot1/joint_states   /robot2/joint_states
/robot1/odom           /robot2/odom
/robot1/planned_path   /robot2/planned_path
/robot1/robot_description  /robot2/robot_description
/robot1/sim_pose       /robot2/sim_pose
/robot1/wl             /robot2/wl
/robot1/wr             /robot2/wr
```

---

## Control manual independiente

Cada robot tiene su propio `/cmd_vel`. Para mover uno sin tocar al otro:

```bash
# Solo robot1: avanzar recto
ros2 topic pub /robot1/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Solo robot2: girar en el sitio
ros2 topic pub /robot2/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```



---

## Propagación de covarianza (Task 1)

El nodo `localisation` mantiene una matriz `Σ ∈ ℝ³ˣ³` para el estado `[x, y, θ]` y la propaga en cada paso de simulación:

```
Σ_k  = H_k · Σ_{k-1} · H_k^T + Q_k
Q_k  = ∇ω_k · Σ_Δ_k · ∇ω_k^T
Σ_Δ  = diag(kr·|ωr|, kl·|ωl|)

           ⎡ 1  0  −v·dt·sin(θ) ⎤              r·dt ⎡ cosθ   cosθ ⎤
H_k =      ⎢ 0  1   v·dt·cos(θ) ⎥     ∇ω_k =  ──── ⎢ sinθ   sinθ ⎥
           ⎣ 0  0        1      ⎦               2  ⎣  2/L  −2/L  ⎦
```

Los nueve elementos relevantes (`σxx, σxy, σxθ, σyx, σyy, σyθ, σθx, σθy, σθθ`) se acomodan en la 6x6 del `pose.covariance` del mensaje `Odometry` (índices 0, 1, 5, 6, 7, 11, 30, 31, 35), respetando el orden `[x, y, z, roll, pitch, yaw]`. RViz se encarga de dibujar la elipse de posición y el cono de orientación automáticamente — ya viene activado en `multi_robot.rviz` con `Keep: 50` para dejar un rastro de elipses crecientes.

Verificar que se publica con valores no triviales:

```bash
ros2 topic echo --once /robot1/odom | grep -A 36 covariance
```

Índices clave esperados (con `kr=kl=0.02` y robot moviéndose en +x):

| Índice | Significado | Valor típico tras ~5 s |
|---|---|---|
| 0  | σxx (eje del movimiento) | ~1e-5 |
| 7  | σyy (perpendicular)      | ~1e-4 (crece con el yaw) |
| 11 | σy·θ (correlación)       | ~3e-4 |
| 35 | σθθ                      | ~8e-4 (crece linealmente) |

---

## Calibración de `kr` / `kl` (Task 2)

`kr` y `kl` son los únicos parámetros libres del modelo de ruido. Están en cada YAML, bloque `localisation`:

```yaml
localisation:
  ros__parameters:
    ...
    kr: 0.02
    kl: 0.02
```

Cambiar en caliente sin relanzar:

```bash
ros2 param set /robot1/localisation kr 0.05
ros2 param set /robot1/localisation kl 0.05
```

> Nota: el parámetro se actualiza al instante en el callback declarado, pero la `Σ` ya acumulada no se reinicia; para ver el efecto desde cero, relanza el nodo o publica una nueva trayectoria.

**Procedimiento sugerido (test en línea recta)**:

1. Pose inicial conocida (ej. `(0,0,0)`), waypoint a `(1.0, 0.0)` y `loop_trajectory: false`.
2. Repetir N veces el experimento real y anotar la pose final medida (cinta + transportador).
3. Calcular la varianza empírica `s²_x`, `s²_y`, `s²_θ` de los N resultados.
4. Ajustar `kr`, `kl` para que la `Σ` final del simulador (índices 0, 7, 35) sea del mismo orden que las varianzas medidas. Empezar con `kr = kl` y desbalancear sólo si la rueda izquierda y derecha tienen comportamientos distintos.
5. Validar con un test de giro en sitio: `cmd_vel: {linear:0, angular:0.5}` durante un tiempo conocido — la elipse debe abrirse principalmente en θ, no en x/y.

---

## Cambiar pose inicial / waypoints


Cambio en caliente (sin reiniciar):

```bash
ros2 param set /robot1/point_generator waypoints_x [1.5, 1.5, 0.0, 0.0]
ros2 param set /robot1/point_generator waypoints_y [0.0, 1.5, 1.5, 0.0]
```

> Orden importa: primero `wx`, luego `wy`. El callback solo reconstruye la trayectoria cuando los tamaños de ambas listas coinciden.

---

## Estructura del paquete

```
puzzlebot_sim/
├── config/
│   ├── robot1_params.yaml      # robot1: pose inicial (0,0,0), triángulo
│   └── robot2_params.yaml      # robot2: pose inicial (2,0,π/2), cuadrado
├── launch/
│   └── multi_robot_launch.py   
├── meshes/                     
├── rviz/multi_robot.rviz      
├── urdf/puzzlebot.urdf         
└── puzzlebot_sim/
    ├── simulator.py            
    ├── localisation.py         
    ├── point_generator.py
    └── control.py
```



## Apagar



```bash
pkill -9 -f puzzlebot_sim
pkill -9 -f rviz2
pkill -9 -f rqt_graph
ros2 daemon stop && ros2 daemon start
ros2 node list   # debe estar vacío
```

---

## Troubleshooting

**RobotModel muestra "Status: Error" y un blob blanco.**
Falta `TF Prefix` en el RobotModel display. Click en el display → campo `TF Prefix` → escribe `robot1` (o `robot2`). Ya viene puesto en el `multi_robot.rviz` del repo, pero si lo modificas a mano se pierde.

**rqt_graph aparece vacío o solo con `/rosout`.**
Pulsa el botón **Refresh** y desmarca "Hide Dead sinks" / "Hide Leaf topics". El launch lo arranca con 3 s de retraso pero los nodos pueden tardar más en publicar.

**Un robot no se mueve.**
Verifica el lazo:
```bash
ros2 topic hz /robot1/odom         # ~50 Hz
ros2 topic echo --once /robot1/current_goal
ros2 topic hz /robot1/cmd_vel      # ~20 Hz cuando hay goal activo
```

**Los robots empiezan en el mismo sitio.**
Revisa que `initial_x/y` esté distinto en `robot1_params.yaml` vs `robot2_params.yaml`, y que coincida entre los bloques `puzzlebot_sim` y `localisation` del MISMO robot.
