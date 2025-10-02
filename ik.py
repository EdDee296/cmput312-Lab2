from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.button import Button
from math import cos, sin, radians, sqrt, atan2, degrees, acos
import time, sys

from fk import reset_angles, debug_print

# Link lengths 
L1 = 13
L2 = 9

SPEED = 90

# Motors
joint1 = LargeMotor(OUTPUT_A)
joint2 = LargeMotor(OUTPUT_B)

def analytical_method(x, y):
    """Return (theta1, theta2) in degrees for given (x,y) position"""
    # Compute theta2 using the law of cosines
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if D < -1 or D > 1:
        raise ValueError("Point out of reach")
    theta2 = acos(D) # Elbow-up solution

    # Compute theta1
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))

    # Convert to degrees
    theta1 = degrees(theta1)
    theta2 = degrees(theta2)
    debug_print(f"Analytical IK: theta1={theta1}, theta2={theta2}")
    # Move joints to calculated positions
    joint1.on_to_position(SpeedDPS(SPEED), theta1)
    joint2.on_to_position(SpeedDPS(SPEED), theta2)

    return (theta1, theta2)

def newton_method(x, y, initial_guess=(0, 0), tol=1e-2, max_iter=100):
    """Return (theta1, theta2) in degrees for given (x,y) position using Newton's method"""
    theta1, theta2 = initial_guess
    debug_print(f"Initial guess: theta1={theta1}, theta2={theta2}")
    for _ in range(max_iter):
        # Forward kinematics
        t1 = radians(theta1)
        t2 = radians(theta2)
        fx = L1 * cos(t1) + L2 * cos(t1 + t2) - x
        fy = L1 * sin(t1) + L2 * sin(t1 + t2) - y

        # Jacobian
        j11 = -L1 * sin(t1) - L2 * sin(t1 + t2)
        j12 = -L2 * sin(t1 + t2)
        j21 = L1 * cos(t1) + L2 * cos(t1 + t2)
        j22 = L2 * cos(t1 + t2)

        # Determinant
        det = j11 * j22 - j12 * j21
        if abs(det) < 1e-6:
            raise ValueError("Jacobian is singular")

        # Inverse Jacobian
        inv_j11 = j22 / det
        inv_j12 = -j12 / det
        inv_j21 = -j21 / det
        inv_j22 = j11 / det

        # Update angles
        dtheta1 = inv_j11 * fx + inv_j12 * fy
        dtheta2 = inv_j21 * fx + inv_j22 * fy

        theta1 -= degrees(dtheta1)
        theta2 -= degrees(dtheta2)

        debug_print(f"Iter {_+1}: theta1={theta1}, theta2={theta2}")

        # Move joints to new positions
        joint1.on_to_position(SpeedDPS(SPEED), theta1)
        joint2.on_to_position(SpeedDPS(SPEED), theta2)

        # Check convergence
        if sqrt(fx**2 + fy**2) < tol:
            return (theta1, theta2)

    raise ValueError("Newton's method did not converge")

def midpoint(p1, p2):
    """Return midpoint between two points"""
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

def main():
    #reset_angles()
    pass

if __name__ == "__main__":
    main()
