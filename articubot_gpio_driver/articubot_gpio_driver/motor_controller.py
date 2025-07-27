#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

        # Motor GPIO pins
        self.motor_pins = {
            'M1_IN1': 17, 'M1_IN2': 27, 'M1_EN': 22,
            'M2_IN1': 23, 'M2_IN2': 24, 'M2_EN': 25,
            'M3_IN1': 5,  'M3_IN2': 6,  'M3_EN': 12,
            'M4_IN1': 13, 'M4_IN2': 19, 'M4_EN': 26,
        }

        GPIO.setmode(GPIO.BCM)
        for pin in self.motor_pins.values():
            GPIO.setup(pin, GPIO.OUT)

        self.pwms = {
            'M1': GPIO.PWM(self.motor_pins['M1_EN'], 100),
            'M2': GPIO.PWM(self.motor_pins['M2_EN'], 100),
            'M3': GPIO.PWM(self.motor_pins['M3_EN'], 100),
            'M4': GPIO.PWM(self.motor_pins['M4_EN'], 100),
        }

        for pwm in self.pwms.values():
            pwm.start(0)

        self.get_logger().info("Motor controller node started")

    def listener_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        self.get_logger().info(f"Received cmd_vel: linear={linear:.2f}, angular={angular:.2f}")

        # Calculate speed and direction for left and right wheels
        left_speed = linear - angular
        right_speed = linear + angular

        # Set motor directions and speeds
        self.set_motor_speed_and_direction('M1', left_speed)
        self.set_motor_speed_and_direction('M2', left_speed)
        self.set_motor_speed_and_direction('M3', right_speed)
        self.set_motor_speed_and_direction('M4', right_speed)

    def set_motor_speed_and_direction(self, motor, speed):
        # speed range is from -inf to +inf, but we clamp to -100 to 100 for PWM
        duty_cycle = min(abs(speed) * 100, 100)

        if speed > 0:
            # Forward
            GPIO.output(self.motor_pins[f'{motor}_IN1'], GPIO.HIGH)
            GPIO.output(self.motor_pins[f'{motor}_IN2'], GPIO.LOW)
        elif speed < 0:
            # Backward
            GPIO.output(self.motor_pins[f'{motor}_IN1'], GPIO.LOW)
            GPIO.output(self.motor_pins[f'{motor}_IN2'], GPIO.HIGH)
        else:
            # Stop motor
            GPIO.output(self.motor_pins[f'{motor}_IN1'], GPIO.LOW)
            GPIO.output(self.motor_pins[f'{motor}_IN2'], GPIO.LOW)

        self.pwms[motor].ChangeDutyCycle(duty_cycle)

    def stop_motors(self):
        for motor in ['M1', 'M2', 'M3', 'M4']:
            GPIO.output(self.motor_pins[f'{motor}_IN1'], GPIO.LOW)
            GPIO.output(self.motor_pins[f'{motor}_IN2'], GPIO.LOW)
            self.pwms[motor].ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.stop_motors()
        GPIO.cleanup()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
