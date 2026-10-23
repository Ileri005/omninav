#!/usr/bin/env python3

"""Autonomous door tour"""

import math 

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


STOPS = [
    ('door_7 (end wall, middle)', -0.70, 0.00, math.pi),
    ('door_8 (end wall, kitchen side)', -0.70, 1.30, math.pi),
    ('door_6 (end wall, rules side)', -0.70, -1.15, math.pi),
    ('door_5 (Office Rules wall)', 0.50, -1.40, -math.pi / 2),
    ('door_10 (far wall)', 12.50, 0.80, 0.0),
    ('door_9 (far wall, kitchen side)', 12.50, 1.40, 0.0),
    ('door_4 (lobby, upper)', 14.34, -1.10, math.pi / 2),
    ('door_3 (lobby end)',  15.70, -1.10, 0.0),
    ('door_2 (lobby end)', 15.70, -2.40, 0.0),
    ('door_1 (lobby, lower)', 14.34, -2.40,  -math.pi / 2),
    ('home (spawn point)', 0.00, 0.00, 0.0),
    
]

def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p



def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()          # waits for AMCL and the navigation servers
 
    results = []
    for name, x, y, yaw in STOPS:
        nav.info(f'Going to {name} at ({x:.2f}, {y:.2f})')
        start = nav.get_clock().now()
        nav.goToPose(make_pose(nav, x, y, yaw))
 
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and (nav.get_clock().now() - start).nanoseconds > 120e9:
                nav.warn('Taking longer than 120 s, cancelling this stop')
                nav.cancelTask()
 
        result = nav.getResult()
        secs = (nav.get_clock().now() - start).nanoseconds / 1e9
        ok = result == TaskResult.SUCCEEDED
        results.append((name, ok, secs))
        nav.info(f'  -> {"SUCCEEDED" if ok else "FAILED"} in {secs:.1f} s (sim time)')
 
    passed = sum(ok for _, ok, _ in results)
    print('\n===== Door tour summary =====')
    for name, ok, secs in results:
        print(f'  {"PASS" if ok else "FAIL"}  {name:36s} {secs:6.1f} s')
    print(f'  {passed}/{len(results)} stops reached')
 
    nav.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()