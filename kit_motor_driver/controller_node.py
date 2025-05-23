import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Declara parâmetro do tamanho da base do robô (distância entre as rodas)
        self.declare_parameter('robot_dimensions.wheel_base', 0.15)    
        self.wheel_base = self.get_parameter('robot_dimensions.wheel_base').value
        
        # Declara parâmetro do ganho proporcional do robô
        self.declare_parameter('controller.proportional_gain', 1)    
        self.Kp = self.get_parameter('controller.proportional_gain').value

        # Declara as variáveis de estado
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        # Declara subscribers 
        # Precisamos nos inscrever no tópico de comandos de velocidade, para atualizar o setpoint desejado pelo usuário
        # Precisamos nos inscrever no tópico de odometria das rodas, para atualizar o feedback do sistema de controle 
        self.create_subscription(Twist, 'kit/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, 'kit/wheel/odom', self.odom_callback, 10)

        # Declara publishers
        # Publicaremos os dados de duty cycle para o nó dos motores
        self.left_pub = self.create_publisher(Float32, '/kit/wheels/left/duty_cycle', 10)
        self.right_pub = self.create_publisher(Float32, '/kit/wheels/right/duty_cycle', 10)

        # Loga os parâmetros iniciais para o usuario
        self.get_logger().info(
            'Motor controller initialized with ' 
            f'Wheel base: {self.wheel_base}, '
            f'Proportional gain: {self.Kp}'
        )

        # Timer para controlar a frequência do sistema de controle
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def cmd_callback(self, msg: Twist):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z

    def odom_callback(self, msg: Odometry):
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

    def control_loop(self):
        # Controle proporcional
        
        # Velocidade desejada na roda esquerda. 
        # Velocidade linear desejada menos a velocidade angular desejada * raio do robô
        v_l_target = self.target_v - self.wheel_base * self.target_w / 2.0         
        # Velocidade desejada na roda direita. 
        # Velocidade linear desejada mais a velocidade angular desejada * raio do robô
        v_r_target = self.target_v + self.wheel_base * self.target_w / 2.0 


        # Velocidade atual na roda esquerda.
        # Velocidade linear atual menos a velocidade angular atual * raio do robô
        v_l_current = self.current_v - self.wheel_base * self.current_w / 2.0 
        # Velocidade atual na roda direita.
        # Velocidade linear atual mais a velocidade angular atual * raio do robô
        v_r_current = self.current_v + self.wheel_base * self.current_w / 2.0

        # Saida do sistema de controle para a roda esquerda
        output_l = self.Kp * (v_l_target - v_l_current)
        # Saida do sistema de controle para a roda direita
        output_r = self.Kp * (v_r_target - v_r_current)

        self.left_pub.publish(Float32(data=output_l))
        self.right_pub.publish(Float32(data=output_r))

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
