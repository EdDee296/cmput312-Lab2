from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
from math import cos, sin, radians, sqrt, atan2, degrees, acos
import time

from fk import debug_print, wait_for_touch

# Link lengths
L1 = 13
L2 = 9
SPEED = 90

# Motors
joint1 = LargeMotor(OUTPUT_A)
joint2 = LargeMotor(OUTPUT_B)

# Global offsets
offset1 = 0
offset2 = 0


# ---------------- Calibration & Helpers ----------------
def calibrate_zero():
    """Record encoder offsets when arm is straight (end effector at (L1+L2,0))"""
    global offset1, offset2
    debug_print("Place arm in zero pose (straight). Press touch sensor...")
    wait_for_touch()
    offset1 = joint1.position
    offset2 = joint2.position
    debug_print("Offsets saved: joint1={}, joint2={}".format(offset1, offset2))


def get_joint_angles():
    """Return current math angles (deg) from encoders"""
    return (joint1.position - offset1, joint2.position - offset2)


def move_to_angles(theta1, theta2):
    """Move motors to math-space angles (deg)"""
    joint1.on_to_position(SpeedDPS(SPEED), theta1 + offset1)
    joint2.on_to_position(SpeedDPS(SPEED), -theta2 + offset2)
    time.sleep(1)


# ---------------- Inverse Kinematics ----------------
def analytical_method(x, y):
    """Analytical IK solution (elbow-down)"""
    calibrate_zero()
    # Law of cosines
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if D < -1 or D > 1:
        raise ValueError("Point out of reach")

    theta2 = -acos(D)
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))

    # Convert to degrees
    theta1_deg = degrees(theta1)
    theta2_deg = degrees(theta2)

    debug_print("Analytical IK: theta1={:.2f}, theta2={:.2f}".format(
        theta1_deg, theta2_deg))

    move_to_angles(theta1_deg, theta2_deg)

    return (theta1_deg, theta2_deg)

def normalize_angle(angle):
    """Normalize angle to [-180, 180] range"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def newton_method(x, y, initial_guess=(0, 0), tol=1e-2, max_iter=100):
    """Numerical IK using Newton's method"""
    calibrate_zero()
    theta1, theta2 = initial_guess
    debug_print("Initial guess: t1={}, t2={}".format(theta1, theta2))

    for i in range(max_iter):
        t1 = radians(theta1)
        t2 = radians(theta2)
        fx = L1*cos(t1) + L2*cos(t1+t2) - x
        fy = L1*sin(t1) + L2*sin(t1+t2) - y

        # Check convergence FIRST
        error = sqrt(fx**2 + fy**2)
        if error < tol:
            debug_print("Converged! t1={:.2f}, t2={:.2f}, Error: {:.4f}".format(theta1, theta2, error))
            move_to_angles(theta1, theta2)  # Move only once at the end
            return (theta1, theta2)

        # Jacobian
        j11 = -L1*sin(t1) - L2*sin(t1+t2)
        j12 = -L2*sin(t1+t2)
        j21 = L1*cos(t1) + L2*cos(t1+t2)
        j22 = L2*cos(t1+t2)

        det = j11*j22 - j12*j21
        if abs(det) < 1e-6:
            raise ValueError("Jacobian singular")

        # Inverse Jacobian
        inv_j11 = j22 / det
        inv_j12 = -j12 / det
        inv_j21 = -j21 / det
        inv_j22 = j11 / det

        dtheta1 = inv_j11*fx + inv_j12*fy
        dtheta2 = inv_j21*fx + inv_j22*fy

        theta1 -= degrees(dtheta1)
        theta2 -= degrees(dtheta2)
        
        # Normalize angles to prevent wrap-around
        theta1 = normalize_angle(theta1)
        theta2 = normalize_angle(theta2)

        debug_print("Iter {}: t1={:.2f}, t2={:.2f}, error={:.4f}".format(
            i+1, theta1, theta2, error))

    raise ValueError("Newton did not converge after {} iterations".format(max_iter))
# ---------------- Main ----------------
def main():
    #analytical_method(10, 10)
    newton_method(10,10, (10,10))

if __name__ == "__main__":
    main()
