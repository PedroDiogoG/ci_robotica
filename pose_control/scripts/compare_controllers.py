#!/usr/bin/env python3
"""
Script de comparacao entre Controle Continuo e Controle por Tres Manobras (Bonus 1.2).
Permite:
  1. Gravar os dados de execucao da missao (/odom e /cmd_vel)
  2. Gerar graficos comparativos (trajetoria no plano XY, velocidades no tempo, erros)
  3. Gerar tabela de metricas (tempo total, comprimento da trajetoria, erro medio)
"""

import os
import sys
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False


class MissionRecorder(Node if HAVE_ROS2 else object):
    def __init__(self, mode_name, output_file):
        super().__init__('mission_recorder')
        self.mode_name = mode_name
        self.output_file = output_file
        self.data = []  # [t, x, y, yaw, v, w]

        self.start_time = None
        self.current_v = 0.0
        self.current_w = 0.0
        self.completed = False

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_cb, 10
        )
        self.status_sub = self.create_subscription(
            String, '/manager/mission_status', self.status_cb, 10
        )

        self.get_logger().info(f'Gravando missao [{mode_name}] em {output_file}...')

    def cmd_cb(self, msg: Twist):
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z

    def odom_cb(self, msg: Odometry):
        if self.completed:
            return

        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = now_sec

        t = now_sec - self.start_time
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.data.append([t, x, y, yaw, self.current_v, self.current_w])

    def status_cb(self, msg: String):
        if 'COMPLETED' in msg.data:
            self.get_logger().info(f'Missao finalizada recebida via status: {msg.data}')
            self.completed = True
            self.save_csv()
            rclpy.shutdown()

    def save_csv(self):
        arr = np.array(self.data)
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        header = 'time,x,y,yaw,v,w'
        np.savetxt(self.output_file, arr, delimiter=',', header=header, comments='')
        print(f'[OK] Dados salvos com sucesso em: {self.output_file} ({len(self.data)} amostras)')


def record_mission(mode_name, output_file):
    if not HAVE_ROS2:
        print('Erro: rclpy nao disponivel.')
        return
    rclpy.init()
    recorder = MissionRecorder(mode_name, output_file)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.save_csv()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def plot_comparison(cont_file, three_man_file, output_img='mission_comparison.png'):
    data_cont = None
    data_3man = None

    if os.path.exists(cont_file):
        data_cont = np.loadtxt(cont_file, delimiter=',', skiprows=1)
    if os.path.exists(three_man_file):
        data_3man = np.loadtxt(three_man_file, delimiter=',', skiprows=1)

    # Waypoints de referencia da missao
    waypoints = np.array([
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 1.57],
        [0.0, 1.0, 3.14],
        [0.0, 0.0, 0.0]
    ])

    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(2, 2)

    # Subplot 1: Trajetoria XY
    ax_xy = fig.add_subplot(gs[:, 0])
    ax_xy.set_title('Comparação de Trajetória: Plano XY', fontsize=14, fontweight='bold')
    ax_xy.set_xlabel('X [m]', fontsize=12)
    ax_xy.set_ylabel('Y [m]', fontsize=12)
    ax_xy.grid(True, linestyle='--', alpha=0.6)

    # Plot waypoints
    ax_xy.scatter(waypoints[:, 0], waypoints[:, 1], color='red', s=100, zorder=5, label='Waypoints')
    for i, (wx, wy, wyaw) in enumerate(waypoints):
        ax_xy.annotate(f'WP{i+1}\n({wx:.1f}, {wy:.1f})', (wx, wy),
                       textcoords="offset points", xytext=(10, 10),
                       ha='center', fontsize=10, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='yellow', alpha=0.5))
        # Seta do yaw desejado no waypoint
        dx = 0.15 * math.cos(wyaw)
        dy = 0.15 * math.sin(wyaw)
        ax_xy.arrow(wx, wy, dx, dy, head_width=0.04, head_length=0.04, fc='darkred', ec='darkred', zorder=6)

    # Trajetoria ideal (linhas retas entre waypoints)
    start_pt = np.array([[0.0, 0.0, 0.0]])
    all_pts = np.vstack([start_pt, waypoints])
    ax_xy.plot(all_pts[:, 0], all_pts[:, 1], 'k--', alpha=0.3, label='Segmentos Nominais')

    metrics = {}

    if data_cont is not None and len(data_cont) > 0:
        t_c = data_cont[:, 0]
        x_c = data_cont[:, 1]
        y_c = data_cont[:, 2]
        v_c = data_cont[:, 4]
        w_c = data_cont[:, 5]

        dist_c = np.sum(np.hypot(np.diff(x_c), np.diff(y_c)))
        time_c = t_c[-1] - t_c[0]
        metrics['Continuo'] = {
            'tempo': time_c,
            'distancia': dist_c,
            'v_max': np.max(np.abs(v_c)),
            'w_max': np.max(np.abs(w_c)),
            'v_med': np.mean(np.abs(v_c))
        }

        ax_xy.plot(x_c, y_c, 'b-', linewidth=2.5, label=f'Contínuo (T={time_c:.1f}s, L={dist_c:.2f}m)')
        # Subamostragem de setas de orientação
        step = max(1, len(x_c) // 15)
        for idx in range(0, len(x_c), step):
            yaw = data_cont[idx, 3]
            ax_xy.arrow(x_c[idx], y_c[idx], 0.06 * math.cos(yaw), 0.06 * math.sin(yaw),
                        head_width=0.02, head_length=0.02, fc='blue', ec='blue', alpha=0.5)

    if data_3man is not None and len(data_3man) > 0:
        t_m = data_3man[:, 0]
        x_m = data_3man[:, 1]
        y_m = data_3man[:, 2]
        v_m = data_3man[:, 4]
        w_m = data_3man[:, 5]

        dist_m = np.sum(np.hypot(np.diff(x_m), np.diff(y_m)))
        time_m = t_m[-1] - t_m[0]
        metrics['3 Manobras'] = {
            'tempo': time_m,
            'distancia': dist_m,
            'v_max': np.max(np.abs(v_m)),
            'w_max': np.max(np.abs(w_m)),
            'v_med': np.mean(np.abs(v_m))
        }

        ax_xy.plot(x_m, y_m, 'g--', linewidth=2.5, label=f'3 Manobras (T={time_m:.1f}s, L={dist_m:.2f}m)')
        step = max(1, len(x_m) // 15)
        for idx in range(0, len(x_m), step):
            yaw = data_3man[idx, 3]
            ax_xy.arrow(x_m[idx], y_m[idx], 0.06 * math.cos(yaw), 0.06 * math.sin(yaw),
                        head_width=0.02, head_length=0.02, fc='green', ec='green', alpha=0.5)

    ax_xy.axis('equal')
    ax_xy.legend(loc='best', fontsize=11)

    # Subplot 2: Velocidade Linear v(t)
    ax_v = fig.add_subplot(gs[0, 1])
    ax_v.set_title('Velocidade Linear $v(t)$', fontsize=12, fontweight='bold')
    ax_v.set_xlabel('Tempo [s]', fontsize=10)
    ax_v.set_ylabel('$v$ [m/s]', fontsize=10)
    ax_v.grid(True, linestyle='--', alpha=0.6)

    if data_cont is not None and len(data_cont) > 0:
        ax_v.plot(data_cont[:, 0], data_cont[:, 4], 'b-', label='Contínuo')
    if data_3man is not None and len(data_3man) > 0:
        ax_v.plot(data_3man[:, 0], data_3man[:, 4], 'g--', label='3 Manobras')
    ax_v.legend(loc='best')

    # Subplot 3: Velocidade Angular w(t)
    ax_w = fig.add_subplot(gs[1, 1])
    ax_w.set_title('Velocidade Angular $\omega(t)$', fontsize=12, fontweight='bold')
    ax_w.set_xlabel('Tempo [s]', fontsize=10)
    ax_w.set_ylabel('$\omega$ [rad/s]', fontsize=10)
    ax_w.grid(True, linestyle='--', alpha=0.6)

    if data_cont is not None and len(data_cont) > 0:
        ax_w.plot(data_cont[:, 0], data_cont[:, 5], 'b-', label='Contínuo')
    if data_3man is not None and len(data_3man) > 0:
        ax_w.plot(data_3man[:, 0], data_3man[:, 5], 'g--', label='3 Manobras')
    ax_w.legend(loc='best')

    plt.tight_layout()
    plt.savefig(output_img, dpi=300)
    print(f'[OK] Gráfico comparativo gerado em: {output_img}')

    # Imprime tabela comparativa no terminal
    print('\n' + '=' * 65)
    print('          TABELA COMPARATIVA DE DESEMPENHO (BÔNUS 1.2)')
    print('=' * 65)
    print(f"{'Métrica':<30} | {'Contínuo':<15} | {'3 Manobras':<15}")
    print('-' * 65)
    if 'Continuo' in metrics and '3 Manobras' in metrics:
        c = metrics['Continuo']
        m = metrics['3 Manobras']
        print(f"{'Tempo Total (s)':<30} | {c['tempo']:<15.2f} | {m['tempo']:<15.2f}")
        print(f"{'Comprimento do Caminho (m)':<30} | {c['distancia']:<15.2f} | {m['distancia']:<15.2f}")
        print(f"{'Velocidade Linear Média (m/s)':<30} | {c['v_med']:<15.2f} | {m['v_med']:<15.2f}")
        print(f"{'Velocidade Linear Máx (m/s)':<30} | {c['v_max']:<15.2f} | {m['v_max']:<15.2f}")
        print(f"{'Velocidade Angular Máx (rad/s)':<30} | {c['w_max']:<15.2f} | {m['w_max']:<15.2f}")
    elif 'Continuo' in metrics:
        c = metrics['Continuo']
        print(f"{'Tempo Total (s)':<30} | {c['tempo']:<15.2f} | {'N/A':<15}")
        print(f"{'Comprimento do Caminho (m)':<30} | {c['distancia']:<15.2f} | {'N/A':<15}")
    elif '3 Manobras' in metrics:
        m = metrics['3 Manobras']
        print(f"{'Tempo Total (s)':<30} | {'N/A':<15} | {m['tempo']:<15.2f}")
        print(f"{'Comprimento do Caminho (m)':<30} | {'N/A':<15} | {m['distancia']:<15.2f}")
    print('=' * 65)


def main():
    parser = argparse.ArgumentParser(description='Comparador de controladores de pose')
    parser.add_argument('--record', choices=['continuous', 'three_maneuvers'],
                        help='Grava uma missao em tempo real com o modo indicado')
    parser.add_argument('--output', type=str, default=None,
                        help='Nome do arquivo CSV de saida para gravacao')
    parser.add_argument('--plot', action='store_true',
                        help='Gera grafico e metricas comparativas a partir de CSVs gravados')
    parser.add_argument('--continuous-csv', type=str, default='mission_continuous.csv',
                        help='Arquivo CSV do modo continuo')
    parser.add_argument('--three-maneuvers-csv', type=str, default='mission_three_maneuvers.csv',
                        help='Arquivo CSV do modo tres manobras')
    parser.add_argument('--img-out', type=str, default='mission_comparison.png',
                        help='Nome da imagem de saida do grafico')

    args = parser.parse_args()

    if args.record:
        out = args.output if args.output else f'mission_{args.record}.csv'
        record_mission(args.record, out)
    elif args.plot:
        plot_comparison(args.continuous_csv, args.three_maneuvers_csv, args.img_out)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
