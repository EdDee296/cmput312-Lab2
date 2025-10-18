import sys
sys.path.append('..')
from ik import analytical_method, newton_method
from fk import get_current_position, wait_for_touch, debug_print, calibrate_zero
import time

def draw_line(p1, p2, num_points=20):
    """Draw straight line from p1 to p2"""
    x1, y1 = p1
    x2, y2 = p2
    
    debug_print("Drawing line from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f})".format(x1, y1, x2, y2))
    
    for i in range(num_points + 1):
        t = i / num_points
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        
        try:
            analytical_method(x, y)
            time.sleep(0.1)
            debug_print("Moved to point ({:.2f}, {:.2f})".format(x, y))
        except ValueError as e:
            debug_print("Point ({:.2f}, {:.2f}) unreachable: {}".format(x, y, e))
            break

def record_and_draw():
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

if __name__ == "__main__":
    record_and_draw()