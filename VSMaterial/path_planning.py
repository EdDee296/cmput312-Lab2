import sys
sys.path.append('..')
from ik import analytical_method, newton_method
from fk import get_current_position, wait_for_touch, debug_print, calibrate_zero
import time

def draw_line(p1, p2, num_points=10):
    """Draw straight line from p1 to p2"""
    x1, y1 = p1
    x2, y2 = p2
    
    debug_print("Drawing line from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f})".format(x1, y1, x2, y2))
    
    for i in range(1, num_points + 1):
        t = i / num_points
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        
        try:
            analytical_method(x, y)
            a, b = get_current_position()
            debug_print("Expected to move to ({:.2f}, {:.2f}), actually at ({:.2f}, {:.2f})".format(x, y, a, b))
        except ValueError as e:
            debug_print("Point ({:.2f}, {:.2f}) unreachable: {}".format(x, y, e))
            break

def record_and_draw():
    from ik import joint1, joint2
    
    calibrate_zero()
    
    debug_print("Move to start point")
    wait_for_touch()
    p1 = get_current_position()
    debug_print("Start: ({:.2f}, {:.2f})".format(p1[0], p1[1]))
    
    debug_print("Move to end point")
    wait_for_touch()
    p2 = get_current_position()
    debug_print("End: ({:.2f}, {:.2f})".format(p2[0], p2[1]))
    
    draw_line(p1, p2)
    
    joint1.stop_action = 'coast'
    joint2.stop_action = 'coast'
    joint1.stop()
    joint2.stop()
    debug_print("Motors set to coast")

if __name__ == "__main__":
    record_and_draw()