import sys
sys.path.append('..')
from color_tracking import Tracker
from server import Server
from queue import Queue
import time
import numpy as np

LAMBDA = 0.1
ALPHA = 0.02
ERROR = 40  # pixels
MAX_ERROR = 200  # pixels - divergence threshold
MAX_ITERATIONS = 50

host = "169.254.207.188"
port = 9999

def estimate_jacobian(server, tracker, queue, delta_theta=10):
    """Estimate image Jacobian using orthogonal motions"""
    print("=== Estimating Initial Jacobian ===")
    
    server.sendEnableSafetyMode()
    time.sleep(0.5)
    
    x0, y0, r0 = tracker.point[0]
    print(f"Initial pixel position: ({x0}, {y0})")
    
    server.sendDisableSafetyMode()
    
    # Move joint 1
    server.sendAngles(delta_theta, 0, queue)
    queue.get()
    time.sleep(0.5)
    x1, y1, r1 = tracker.point[0]
    
    # Return to center
    server.sendAngles(-delta_theta, 0, queue)
    queue.get()
    time.sleep(0.5)
    
    # Move joint 2
    server.sendAngles(0, delta_theta, queue)
    queue.get()
    time.sleep(0.5)
    x2, y2, r2 = tracker.point[0]
    
    # Return to center
    server.sendAngles(0, -delta_theta, queue)
    queue.get()
    time.sleep(0.5)
    
    # Compute Jacobian columns
    j11 = (x1 - x0) / delta_theta
    j21 = (y1 - y0) / delta_theta
    j12 = (x2 - x0) / delta_theta
    j22 = (y2 - y0) / delta_theta
    
    J = np.array([[j11, j12],
                  [j21, j22]])
    
    print(f"Estimated Jacobian:\n{J}")
    
    server.sendEnableSafetyMode()
    return J

def visual_servo(server, tracker, queue):
    """Main visual servoing loop"""
    print("Starting UVS...")
    
    J = estimate_jacobian(server, tracker, queue)
    J_inv = np.linalg.pinv(J)
    
    server.sendDisableSafetyMode()
    
    for iteration in range(MAX_ITERATIONS):
        x, y, r = tracker.point[0]
        gx, gy, gr = tracker.goal[0]
        
        error_x = gx - x
        error_y = gy - y
        error = np.array([error_x, error_y])
        error_norm = np.linalg.norm(error)
        
        print(f"Iter {iteration}: Error=({error_x:.1f}, {error_y:.1f}), Norm={error_norm:.1f}")
        
        if error_norm > MAX_ERROR:
            print(f"DIVERGED! Error {error_norm:.1f} exceeds maximum {MAX_ERROR}")
            server.sendEnableSafetyMode()
            return False
        
        if error_norm < ERROR:
            print("Goal reached!")
            break
        
        delta_theta = LAMBDA * J_inv @ error
        theta1_delta, theta2_delta = delta_theta
        
        server.sendAngles(theta1_delta, theta2_delta, queue)
        reply = queue.get()
        
        if reply == "RESET":
            print("Resetting Jacobian")
            J = estimate_jacobian(server, tracker, queue)
            J_inv = np.linalg.pinv(J)
            server.sendDisableSafetyMode()
            continue
        
        time.sleep(0.5)
        
        x_new, y_new, r_new = tracker.point[0]
        gx_new, gy_new, gr_new = tracker.goal[0]
        
        error_new = np.linalg.norm([gx_new - x_new, gy_new - y_new])
        print(f"After move: Error={error_new:.1f}")
        
        if error_new > MAX_ERROR:
            print(f"DIVERGED after move! Error {error_new:.1f} exceeds maximum {MAX_ERROR}")
            server.sendEnableSafetyMode()
            return False
        
        if error_new < ERROR:
            print("Goal reached after move!")
            break
        
        delta_pixels = np.array([x_new - x, y_new - y])
        
        if np.linalg.norm(delta_theta) > 0.1:
            J += ALPHA * np.outer(delta_pixels - J @ delta_theta, delta_theta) / np.dot(delta_theta, delta_theta)
            J_inv = np.linalg.pinv(J)
    
    server.sendEnableSafetyMode()
    print("Visual servoing complete")
    return True

if __name__ == "__main__":
    print("Waiting for client connection...")
    server = Server(host, port)
    print("Client connected!")
    
    queue = Queue()
    
    print("Starting camera tracker...")
    tracker = Tracker('g', 'r')
    
    print("Waiting for camera to detect objects...")
    while True:
        try:
            point = tracker.point
            goal = tracker.goal

            if isinstance(point, tuple):
                px, py, pr = point
            else:
                px, py, pr = point[0]

            if isinstance(goal, tuple):
                gx, gy, gr = goal
            else:
                gx, gy, gr = goal[0]

            print(f"Tracker raw values: point=({px:.2f}, {py:.2f}), goal=({gx:.2f}, {gy:.2f})")

            if px != 0 and py != 0 and gx != 0 and gy != 0:
                print("Tracker ready! Detected both objects ✅")
                break

            time.sleep(0.5)

        except Exception as e:
            print(f"Waiting for tracker initialization... ({e})")
            time.sleep(0.5)

    
    print("Tracker ready! Detected both objects")
    time.sleep(1)
    
    try:
        success = visual_servo(server, tracker, queue)
        if not success:
            print("Visual servoing failed - system diverged")
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        server.sendTermination()