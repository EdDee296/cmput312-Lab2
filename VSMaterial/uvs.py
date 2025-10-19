from color_tracking import Tracker
import time

print("Starting tracker...")
print("Close any programs using the camera!")
time.sleep(2)

tracker = Tracker('g', 'r')
print("Tracker initialized. Green=end effector, Red=goal")
print("Look for the 'Result' window showing camera feed")
print("Press Ctrl+C to stop")

try:
    while True:
        x, y, r = tracker.point
        goal_x, goal_y, goal_r = tracker.goal
        
        print("End effector at: ({}, {})".format(x, y))
        print("Goal at: ({}, {})".format(goal_x, goal_y))
        print("Error: dx={}, dy={}".format(goal_x - x, goal_y - y))
        print("-" * 40)
        
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped")