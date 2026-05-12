"""
Nodo para correr experimentos cronometrados de Task 2 (calibracion kr, kl).

Envia cmd_vel con trayectorias predefinidas (recta, rotacion, cuadrado)
y se detiene automaticamente al terminar, dejando un hold_time para
visualizar el elipsoide final en RViz.

Uso:
  ros2 run puzzlebot_sim experiment_runner --ros-args -p experiment:=straight
  ros2 run puzzlebot_sim experiment_runner --ros-args -p experiment:=rotate
  ros2 run puzzlebot_sim experiment_runner --ros-args -p experiment:=square
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__('experiment_runner')

        self.declare_parameter('experiment', 'straight')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('distance', 1.0)
        self.declare_parameter('rotation', 2.0 * math.pi)
        self.declare_parameter('hold_time', 2.0)

        self.experiment = str(self.get_parameter('experiment').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.distance = float(self.get_parameter('distance').value)
        self.rotation = float(self.get_parameter('rotation').value)
        self.hold_time = float(self.get_parameter('hold_time').value)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Build phase list: [(v, w, duration_s), ...]
        self.phases = self._build_phases()
        self.phase_idx = 0
        self.phase_elapsed = 0.0

        self.dt = 1.0 / 50.0
        self.total_duration = sum(d for _, _, d in self.phases)

        self._log_start()
        self.timer = self.create_timer(self.dt, self._tick)

    def _build_phases(self):
        if self.experiment == 'straight':
            t_move = self.distance / self.linear_speed
            return [
                (self.linear_speed, 0.0, t_move),
                (0.0, 0.0, self.hold_time),
            ]

        if self.experiment == 'rotate':
            t_rot = self.rotation / self.angular_speed
            return [
                (0.0, self.angular_speed, t_rot),
                (0.0, 0.0, self.hold_time),
            ]

        if self.experiment == 'square':
            t_side = self.distance / self.linear_speed
            t_turn = (math.pi / 2.0) / self.angular_speed
            phases = []
            for i in range(4):
                phases.append((self.linear_speed, 0.0, t_side))
                phases.append((0.0, self.angular_speed, t_turn))
            phases.append((0.0, 0.0, self.hold_time))
            return phases

        self.get_logger().error(
            f"Experimento '{self.experiment}' no reconocido. "
            "Valores validos: straight, rotate, square"
        )
        return [(0.0, 0.0, self.hold_time)]

    def _log_start(self):
        move_duration = self.total_duration - self.hold_time
        self.get_logger().info(
            f"Iniciando experimento '{self.experiment}' — "
            f"duracion movimiento: {move_duration:.2f}s, "
            f"hold: {self.hold_time:.1f}s"
        )

    def _tick(self):
        if self.phase_idx >= len(self.phases):
            self._finish()
            return

        v, w, duration = self.phases[self.phase_idx]

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

        self.phase_elapsed += self.dt
        if self.phase_elapsed >= duration:
            self._log_phase_end(v, w, duration)
            self.phase_idx += 1
            self.phase_elapsed = 0.0

    def _log_phase_end(self, v, w, duration):
        if v == 0.0 and w == 0.0:
            self.get_logger().info(f"Hold terminado ({duration:.1f}s)")
        elif w == 0.0:
            self.get_logger().info(
                f"Tramo recto terminado: {v:.2f} m/s x {duration:.2f}s "
                f"= {v * duration:.2f}m"
            )
        else:
            self.get_logger().info(
                f"Rotacion terminada: {w:.2f} rad/s x {duration:.2f}s "
                f"= {math.degrees(w * duration):.1f} deg"
            )

    def _finish(self):
        self.timer.cancel()
        # Publish stop
        self.cmd_pub.publish(Twist())

        move_duration = self.total_duration - self.hold_time

        if self.experiment == 'straight':
            summary = (
                f"Experimento 'straight' terminado. "
                f"Duracion: {move_duration:.2f}s. "
                f"Velocidad: {self.linear_speed} m/s. "
                f"Distancia objetivo: {self.distance} m"
            )
        elif self.experiment == 'rotate':
            summary = (
                f"Experimento 'rotate' terminado. "
                f"Duracion: {move_duration:.2f}s. "
                f"Velocidad angular: {self.angular_speed} rad/s. "
                f"Rotacion objetivo: {math.degrees(self.rotation):.1f} deg"
            )
        elif self.experiment == 'square':
            summary = (
                f"Experimento 'square' terminado. "
                f"Duracion: {move_duration:.2f}s. "
                f"Lado: {self.distance} m a {self.linear_speed} m/s. "
                f"Giros: 90 deg a {self.angular_speed} rad/s"
            )
        else:
            summary = f"Experimento '{self.experiment}' terminado."

        self.get_logger().info(summary)
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
