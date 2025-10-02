from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
from ev3dev2.sensor.lego import TouchSensor
from math import cos, sin, radians, sqrt, atan2, degrees
import time, sys

L1 = 13
L2 = 9

# Motors
joint1 = LargeMotor(OUTPUT_A)
joint2 = LargeMotor(OUTPUT_B)

# Sensors
touch = TouchSensor()  # Touch sensor for recording points

# Speed DPS
SPEED = 90

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

def reset_angles():
    joint1.reset()
    joint2.reset()


def forward_kinematics(theta1, theta2):
    """Return (x,y) of end effector given angles in degrees"""
    t1 = radians(theta1)
    t2 = radians(theta2)
    x = L1*cos(t1) + L2*cos(t1+t2)
    y = L1*sin(t1) + L2*sin(t1+t2)
    return (x, y)

def move_and_measure(theta1_cmd, theta2_cmd):
    # Move to commanded angles
    joint1.on_to_position(SpeedDPS(90), theta1_cmd)
    joint2.on_to_position(SpeedDPS(90), theta2_cmd)

    # Read actual encoder positions
    theta1_actual = joint1.position
    theta2_actual = joint2.position

    # Compute expected vs actual end effector position
    expected = forward_kinematics(theta1_cmd, theta2_cmd)
    actual = forward_kinematics(theta1_actual, theta2_actual)

    # Compute Euclidean error
    error = sqrt((expected[0]-actual[0])**2 + (expected[1]-actual[1])**2)

    debug_print("Expected:", expected)
    debug_print("Actual:  ", actual)
    debug_print("Error (cm):", error)
    
    return actual


def get_current_position():
    """Get current end effector position"""
    theta1 = joint1.position
    theta2 = joint2.position
    return forward_kinematics(theta1, theta2)


def wait_for_touch():
    """Wait for touch sensor to be pressed"""
    debug_print("Press touch sensor...")
    while not touch.is_pressed:
        time.sleep(0.1)
    # Wait for release
    while touch.is_pressed:
        time.sleep(0.1)


def calculate_distance(p1, p2):
    """Calculate distance between two points"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return sqrt(dx**2 + dy**2)


def calculate_angle(p1, p2, p3):
    """Calculate angle at p1 between lines p1-p2 and p1-p3"""
    # Vectors from p1 to p2 and p1 to p3
    v1_x = p2[0] - p1[0]
    v1_y = p2[1] - p1[1]
    v2_x = p3[0] - p1[0]
    v2_y = p3[1] - p1[1]

    # Calculate angles of each vector
    angle1 = atan2(v1_y, v1_x)
    angle2 = atan2(v2_y, v2_x)

    # Calculate angle between vectors
    angle_diff = abs(degrees(angle2 - angle1))

    # Return the smaller angle
    if angle_diff > 180:
        angle_diff = 360 - angle_diff

    return angle_diff


def measure_distance():
    """Measure distance between two points"""
    debug_print("=== DISTANCE MEASUREMENT ===")

    # Record first point
    debug_print("Move to first point")
    wait_for_touch()
    p1 = get_current_position()
    debug_print(f"Point 1: ({p1[0]:.2f}, {p1[1]:.2f})")

    # Record second point
    debug_print("Move to second point")
    wait_for_touch()
    p2 = get_current_position()
    debug_print(f"Point 2: ({p2[0]:.2f}, {p2[1]:.2f})")

    # Calculate and display distance
    distance = calculate_distance(p1, p2)
    debug_print(f"DISTANCE: {distance:.2f} cm")
    debug_print("=" * 28)

    return distance


def measure_angle():
    """Measure angle between two lines"""
    debug_print("=== ANGLE MEASUREMENT ===")

    # Record intersection point
    debug_print("Move to intersection")
    wait_for_touch()
    p1 = get_current_position()
    debug_print(f"Intersection: ({p1[0]:.2f}, {p1[1]:.2f})")

    # Record second point (on first line)
    debug_print("Move to point on line 1")
    wait_for_touch()
    p2 = get_current_position()
    debug_print(f"Point 2: ({p2[0]:.2f}, {p2[1]:.2f})")

    # Record third point (on second line)
    debug_print("Move to point on line 2")
    wait_for_touch()
    p3 = get_current_position()
    debug_print(f"Point 3: ({p3[0]:.2f}, {p3[1]:.2f})")

    # Calculate and display angle
    angle = calculate_angle(p1, p2, p3)
    debug_print(f"ANGLE: {angle:.2f} degrees")
    debug_print("=" * 25)

    return angle


def main():
    reset_angles()
    theta1 = 30
    theta2 = 30
    
    x,y = move_and_measure(theta1, theta2)
    
    debug_print(f"Current position: x={x:.2f}, y={y:.2f}")
    
    pass
    
    

# Main program
if __name__ == "__main__":
    main()
