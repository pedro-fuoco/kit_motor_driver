import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Float32

class MotorNode(Node):
    def __init__(self):
        # Inicializa o node ROS 2 chamado 'motor_node'
        super().__init__('motor_node')
        
        # Cria os subscritores para ler os comando de duty cycle recebidos para cada roda
        self.create_subscription(Float32, '/kit/wheels/right/duty_cycle', self.right_wheel_callback, 10)
        self.create_subscription(Float32, '/kit/wheels/left/duty_cycle', self.left_wheel_callback, 10)
        
        # Declara os ROS params com valores padrão
        self.declare_parameter('kit_motors.right_wheel.GPIOs.direction_pin', 6)
        self.declare_parameter('kit_motors.right_wheel.GPIOs.pwm_pin', 13)
        self.declare_parameter('kit_motors.left_wheel.GPIOs.direction_pin', 5)
        self.declare_parameter('kit_motors.left_wheel.GPIOs.pwm_pin', 12)
        self.declare_parameter('kit_motors.duty_cycle_frequency', 1000)

        # Obtém os valores dos ROS params, caso o node tenha sido inicializado com valores customizados
        self.RIGHT_DIR_PIN = self.get_parameter('kit_motors.right_wheel.GPIOs.direction_pin').value
        self.RIGHT_PWM_PIN = self.get_parameter('kit_motors.right_wheel.GPIOs.pwm_pin').value
        self.LEFT_DIR_PIN = self.get_parameter('kit_motors.left_wheel.GPIOs.direction_pin').value
        self.LEFT_PWM_PIN = self.get_parameter('kit_motors.left_wheel.GPIOs.pwm_pin').value
        self.DUTY_CYCLE_FREQUENCY = self.get_parameter('kit_motors.duty_cycle_frequency').value
        self.MAX_DUTY_CYCLE = 100
        
        # Loga os parâmetros iniciais dos motores para o usuario
        self.get_logger().info(
            'Motors initialized with ' 
            f'Right direction pin: {self.RIGHT_DIR_PIN}, '
            f'Right pwm pin: {self.RIGHT_PWM_PIN}, '
            f'Left direction pin: {self.LEFT_DIR_PIN}, '
            f'Left pwm pin: {self.LEFT_PWM_PIN}, '
            f'Duty cycle frequency: {self.DUTY_CYCLE_FREQUENCY}'
        )

        # Configura o modo de numeração dos pinos GPIO
        GPIO.setmode(GPIO.BCM)

        # Configura os pinos como saídas
        GPIO.setup(self.LEFT_DIR_PIN, GPIO.OUT)
        GPIO.setup(self.LEFT_PWM_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_DIR_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_PWM_PIN, GPIO.OUT)

        # Inicializa os GPIOs de PWM
        self.left_pwm = GPIO.PWM(self.LEFT_PWM_PIN, self.DUTY_CYCLE_FREQUENCY)
        self.right_pwm = GPIO.PWM(self.RIGHT_PWM_PIN, self.DUTY_CYCLE_FREQUENCY)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

        # Confere que os valores lógicos dos pinos não estão flutuando (RISCO DE SEGURANÇA)
        self.stop_wheels()

    def stop_wheels(self):
        # Para a rotação de ambas as rodas. É importante mudar o pino de direção antes do duty cycle
        # para garantir que as rodas não girem na velocidade máxima

        GPIO.output(self.LEFT_DIR_PIN, GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(0)

        GPIO.output(self.RIGHT_DIR_PIN, GPIO.LOW)
        self.right_pwm.ChangeDutyCycle(0)

    def set_wheel(self, duty_cycle, pwm, dir_pin):
        # direção é definida pelo sinal do duty cycle recebido
        direction = duty_cycle >= 0
        # Valor absoluto, maximizado em 100%
        abs_duty_cycle = min(abs(duty_cycle), self.MAX_DUTY_CYCLE)

        # Coloca os valores nos pinos de controle
        GPIO.output(dir_pin, GPIO.LOW if direction else GPIO.HIGH)
        pwm.ChangeDutyCycle(abs_duty_cycle if direction else self.MAX_DUTY_CYCLE - abs_duty_cycle)

    def left_wheel_callback(self, msg):
        # Envia a informação para o usuário e para a roda desejada
        self.set_wheel(msg.data, self.left_pwm, self.LEFT_DIR_PIN)

    def right_wheel_callback(self, msg):
        # Envia a informação para o usuário e para a roda desejada
        self.set_wheel(msg.data, self.right_pwm, self.RIGHT_DIR_PIN)
    
    def __del__(self):
        # Libera os recursos dos GPIOs caso esse objeto seja destruido
        # Seguindo principios de "Resource acquisition is initialization" (RAII)
        self.stop_wheels()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args) # Inicializa o cliente ROS 2
    node = MotorNode() # Cria o node do motor
    try:
        # Mantém o node ativo para continuar capturando comandos para os motores
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Loga uma mensagem ao encerrar o node
        node.get_logger().info('Shutting down motor driver...')
    finally:
        # Destrói o node e finaliza o cliente ROS 2
        node.destroy_node()
        rclpy.shutdown()

# Ponto de entrada do script
if __name__ == '__main__':
    main()
