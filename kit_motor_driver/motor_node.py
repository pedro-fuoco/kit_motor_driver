import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Float32

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        
        # Declare parameters
        self.declare_parameter('kit_motors.right_wheel.GPIOs.direction_pin', 6)
        self.declare_parameter('kit_motors.right_wheel.GPIOs.pwm_pin', 13)
        self.declare_parameter('kit_motors.left_wheel.GPIOs.direction_pin', 5)
        self.declare_parameter('kit_motors.left_wheel.GPIOs.pwm_pin', 12)
        self.declare_parameter('kit_motors.duty_cycle_frequency', 1000)

        # Retrieve parameters
        self.RIGHT_DIR_PIN = self.get_parameter('kit_motors.right_wheel.GPIOs.direction_pin').value
        self.RIGHT_PWM_PIN = self.get_parameter('kit_motors.right_wheel.GPIOs.pwm_pin').value
        self.LEFT_DIR_PIN = self.get_parameter('kit_motors.left_wheel.GPIOs.direction_pin').value
        self.LEFT_PWM_PIN = self.get_parameter('kit_motors.left_wheel.GPIOs.pwm_pin').value
        self.DUTY_CYCLE_FREQUENCY = self.get_parameter('kit_motors.duty_cycle_frequency').value
        self.MAX_DUTY_CYCLE = 100

        self.create_subscription(Float32, '/kit/wheels/right/duty_cycle', self.right_wheel_callback, 10)
        self.create_subscription(Float32, '/kit/wheels/left/duty_cycle', self.left_wheel_callback, 10)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LEFT_DIR_PIN, GPIO.OUT)
        GPIO.setup(self.LEFT_PWM_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_DIR_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_PWM_PIN, GPIO.OUT)

        self.left_pwm = GPIO.PWM(self.LEFT_PWM_PIN, self.DUTY_CYCLE_FREQUENCY)
        self.right_pwm = GPIO.PWM(self.RIGHT_PWM_PIN, self.DUTY_CYCLE_FREQUENCY)

        self.left_pwm.start(0)
        self.right_pwm.start(0)
        self.stop_wheels()

        self.get_logger().info("Node is ready")

    def stop_wheels(self):
        GPIO.output(self.LEFT_DIR_PIN, GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(0)

        GPIO.output(self.RIGHT_DIR_PIN, GPIO.LOW)
        self.right_pwm.ChangeDutyCycle(0)

    def set_wheel(self, duty_cycle, pwm, dir_pin):
        direction = duty_cycle >= 0
        abs_duty_cycle = min(abs(duty_cycle), self.MAX_DUTY_CYCLE)

        GPIO.output(dir_pin, GPIO.LOW if direction else GPIO.HIGH)
        pwm.ChangeDutyCycle(abs_duty_cycle if direction else self.MAX_DUTY_CYCLE - abs_duty_cycle)

    def left_wheel_callback(self, msg):
        self.get_logger().info(f"Received left wheel duty cycle: {msg.data}")
        self.set_wheel(msg.data, self.left_pwm, self.LEFT_DIR_PIN)

    def right_wheel_callback(self, msg):
        self.get_logger().info(f"Received right wheel duty cycle: {msg.data}")
        self.set_wheel(msg.data, self.right_pwm, self.RIGHT_DIR_PIN)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_wheels()
        node.left_pwm.stop()
        node.right_pwm.stop()
        GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
