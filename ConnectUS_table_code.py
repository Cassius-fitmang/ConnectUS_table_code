import time
import board
import busio
from adafruit_motorkit import MotorKit
import adafruit_vl53l0x
import analogio
from digitalio import DigitalInOut, Direction, Pull

# --- Setup I2C bus
i2c_port = busio.I2C(board.SCL, board.SDA)

# --- Setup for the MotorKit
kit = MotorKit(i2c=i2c_port, address=0x60)  # Single MotorKit at default address
dc_motor = kit.motor3  # DC motor connected to M3
actuator_motor = kit.motor1  # Actuator motor connected to M1

# --- Setup for the VL53L0X distance sensor
dist_sensor = adafruit_vl53l0x.VL53L0X(i2c_port)

# --- Setup for the potentiometer and light sensors
potentiometer = analogio.AnalogIn(board.A0)  # Potentiometer on A1
light_sensor1 = analogio.AnalogIn(board.A2)
light_sensor2 = analogio.AnalogIn(board.A3)

# --- Constants
EXTEND_TARGET = 645  # Stop extending at ~640 mm
RETRACT_TARGET = 412  # Stop retracting at ~420 mm
DC_MOTOR_SPEED = 1.0  # Speed of the DC motor (0.0 to 1.0)
ACTUATOR_SPEED = 0.8  # Speed of the actuator motor
MOTOR_RUNTIME = 3.5  # Duration the DC motor runs
SAFETY_DELAY = 0.5  # Delay for motor transitions
LIGHT_THRESHOLD = 600  # Threshold for low light
POTENTIOMETER_THRESHOLD = 0.1  # 10% of the potentiometer range

# --- Motor functions
def run_dc_motor(clockwise):
    print(f"Running DC motor {'clockwise' if clockwise else 'counterclockwise'}...")
    actuator_motor.throttle = 0  # Ensure actuator motor is stopped
    time.sleep(SAFETY_DELAY)

    dc_motor.throttle = DC_MOTOR_SPEED if clockwise else -DC_MOTOR_SPEED
    time.sleep(MOTOR_RUNTIME)
    dc_motor.throttle = 0  # Stop the DC motor
    print("DC motor stopped.")

def move_actuator(direction):
    print(f"Moving actuator {'up' if direction == 'extend' else 'down'}...")
    actuator_motor.throttle = ACTUATOR_SPEED if direction == 'extend' else -ACTUATOR_SPEED

    while True:
        distance = dist_sensor.range  # Read sensor distance
        print(f"Current distance: {distance} mm")

        # Stop based on target distance
        if direction == "extend" and distance >= EXTEND_TARGET:
            break
        elif direction == "retract" and distance <= RETRACT_TARGET:
            break
        time.sleep(0.1)

    actuator_motor.throttle = 0  # Stop actuator
    print(f"Actuator {'extended' if direction == 'extend' else 'retracted'}.")

# --- Main logic
actuator_extended = False  # Tracks actuator state

# --- Initial startup routine
print("Starting program...")
time.sleep(1)  # Small delay to ensure hardware settles

# Ensure actuator starts retracted to 420 mm
print("Moving actuator to retracted position (420 mm)...")
move_actuator("retract")
actuator_extended = False
print("Actuator retracted. Ready for operation.")

# --- Main loop
while True:
    # Read light sensors
    light_value1 = light_sensor1.value
    light_value2 = light_sensor2.value
    potentiometer_value = potentiometer.value
    print(f"Light sensor 1: {light_value1}, Light sensor 2: {light_value2}, Potentiometer: {potentiometer_value}")

    # Actuator and motor control based on light sensors (low light on both sensors)
    if light_value1 < LIGHT_THRESHOLD and light_value2 < LIGHT_THRESHOLD:
        if not actuator_extended:
            print("Low light detected on both sensors. Extending actuator...")
            run_dc_motor(clockwise=True)  # Run DC motor counterclockwise first
            move_actuator("extend")  # Then extend actuator
            actuator_extended = True

    # Check potentiometer for retraction (below 10% threshold)
    if potentiometer_value < POTENTIOMETER_THRESHOLD * 65535:  # Potentiometer value range (0 to 65535)
        if actuator_extended:
            print("Potentiometer below threshold. Retracting actuator...")
            move_actuator("retract")
            run_dc_motor(clockwise=False)  # Run DC motor clockwise after retraction
            actuator_extended = False

    time.sleep(0.1)
