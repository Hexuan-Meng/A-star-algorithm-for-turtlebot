#!/usr/bin/env python

import numpy as np
import time
import math
import pygame
import vidmaker
from sortedcollections import OrderedSet
import heapdict
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



'''
Github repository - https://github.com/sandipsharan/A_star_algorithm
'''

'''
Video Link - https://drive.google.com/file/d/1vIZtnI60usW49peaX9DfhXH0A0Eim3Nj/view?usp=sharing
'''

start_time = time.time()

def coords_pygame(coords, height):
    return (coords[0], height - coords[1])


def rect_pygame(coords, height, obj_height):
    return (coords[0], height - coords[1] - obj_height)


def create_map(d, explored, optimal_path, path):
    pygame.init()
    size = [600, 200]
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("Visualization")
    # video = vidmaker.Video("anime.mp4", late_export=True)
    clock = pygame.time.Clock()
    running = True
    x1, y1 = rect_pygame([150-d, 75-d], 200, 125+d)
    x3, y3 = rect_pygame([250-d, 0], 200, 125+d)

    x2, y2 = rect_pygame([150, 75], 200, 125)
    x4, y4 = rect_pygame([250, 0], 200, 125)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pygame.draw.rect(screen, "teal", [x1, y1, 15+(2*d), 125+d], 0)
        pygame.draw.rect(screen, "skyblue", [x2, y2, 15, 125], 0)
        pygame.draw.rect(screen, "teal", [x3, y3, 15+(2*d), 125+d], 0)
        pygame.draw.rect(screen, "skyblue", [x4, y4, 15, 125], 0)
        pygame.draw.rect(screen, "teal", [0, 0, d, 200], 0)
        pygame.draw.rect(screen, "teal", [0, 0, 600, d], 0)
        pygame.draw.rect(screen, "teal", [0, 200-d, 600, d], 0)
        pygame.draw.rect(screen, "teal", [600-d, 0, d, 200], 0)
        pygame.draw.circle(screen, "teal", coords_pygame((400, 110), 200), 55)
        pygame.draw.circle(screen, "skyblue", coords_pygame((400, 110), 200), 50)

        for l in range(len(explored)):
            pygame.draw.lines(screen, "white", False, path[explored[l]][1], width=1)
            print(path[explored[l]][1])
            # video.update(pygame.surfarray.pixels3d(
            #     screen).swapaxes(0, 1), inverted=False)
            pygame.display.flip()
            clock.tick(3500)
        # for i in optimal_path:
        #     pygame.draw.circle(screen, "black", coords_pygame(i, 250), r)
        #     video.update(pygame.surfarray.pixels3d(
        #         screen).swapaxes(0, 1), inverted=False)
        #     pygame.display.flip()
        #     clock.tick(1)
        running = False
    pygame.display.flip()
    pygame.time.wait(3000)
    pygame.quit()
    # video.export(verbose=True)


def check_obstacles(d):
    obstacles = OrderedSet()
    for x in np.arange(0, 6.1, 0.1):
        for y in np.arange(0, 2.1, 0.1):
            if (x >= (1.5 - d) and y >= (0.75-d) and x <= (1.65 + d) and y <= 2):
                obstacles.add((x, y))
            if (x >= (2.5 - d) and y >= 0 and x <= (2.65 + d) and y <= (1.25 + d)):
                obstacles.add((x, y))
            if ((x-4)**2 + (y-1.1)**2 - 0.25) <= 0:
                obstacles.add((x, y))
    return obstacles


def input_start(str):
    while True:
        print("Enter", str, "node (Sample: 10, 10 ): ")
        A = [int(i) for i in input().split(', ')]
        A_1 = (A[0], A[1])
        if A_1 in obstacle_space:
            print(
                "The entered input lies on the obstacles (or) not valid, please try again")
        else:
            return A_1


def input_cdr(str):
    while True:
        if str == 'step size':
            print("Enter", str, "(Sample: Enter a number between 1 to 10): ")
            A = [int(i) for i in input().split(', ')]
            A_1 = A[0]
            if 1 <= A_1 <= 10:
                return int(A_1)
            else:
                print(
                    "The entered input does not lie between the range 1 to 10, please try again")
        if str == 'start point' or str == 'goal point':
            print("Enter orientation of the", str,
                  "(Sample: Angles in degrees in multiples of 30): ")
            A = [int(i) for i in input().split(', ')]
            A_1 = A[0]
            if A_1 % 30 == 0:
                return int(A_1)
            else:
                print(
                    "The entered input is not in the multiples of 30, please try again")
        if str == 'radius' or str == 'clearance':
            print("Enter", str, "(Sample: 5): ")
            A = [int(i) for i in input().split(', ')]
            return int(A[0])


def check_conditions(X_n, Y_n, X_i, Y_i, T_i, Thetan, cc, ls, vel):
    cost2_go = np.sqrt((node_state_g[0]-X_n)**2 +
                       (node_state_g[1]-Y_n)**2)
    final_cost = cc + cost2_go
    if Thetan > 360:
        Thetan = Thetan%360
    if Thetan < 0:
        Thetan = -Thetan%360
    current_pos = (np.round(X_n, 2), np.round(Y_n, 2), np.round(Thetan, 2))
    if current_pos not in visited_nodes and (int(X_n), int(Y_n)) not in obstacle_space:
        if current_pos in queue_nodes:
            if queue_nodes[current_pos][0] > final_cost:
                queue_nodes[current_pos] = final_cost, cost2_go, cc
                path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
                visited_nodes.add(current_pos)
                return
            else:
                return
        queue_nodes[current_pos] = final_cost, cost2_go, cc
        path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
        visited_nodes.add(current_pos)
    return


def Actions(ul, ur, pos, c2c):
    t = 0
    dt = 2.5
    Xn = pos[0]
    Yn = pos[1]
    Thetan = np.deg2rad(pos[2])
    ls = OrderedSet()
    ls.add(coords_pygame((Xn, Yn), 250))
    while t <= 10:
        t = t + dt
        Xn += 0.5*R * (ul + ur) * np.cos(Thetan) * dt
        Yn += 0.5*R * (ul + ur) * np.sin(Thetan) * dt
        Thetan += (R / L)*(ul-ur)*dt
        ls.add(coords_pygame((Xn, Yn),250))
    velocity = ((Xn/dt), (Yn/dt), (Thetan/dt))
    cc = c2c + math.sqrt((0.5*R*(ul + ur)*math.cos(Thetan)*t)
                         ** 2) + (0.5*R * (ul + ur) * math.sin(Thetan)*t)**2
    Thetan = np.rad2deg(Thetan)
    if 0 <= Xn <= 600 and 0 <= Yn <= 600:
        check_conditions(Xn, Yn, pos[0], pos[1], pos[2], Thetan, cc, ls, velocity)
    return 


def back_tracking(path, pre_queue):
    best_path = OrderedSet()
    path_vel = OrderedSet()
    best_path.add(pre_queue[0])    
    parent_node = path[pre_queue[0]][0]
    vel_parent = path.get(pre_queue[0])[2]
    path_vel.add(vel_parent)
    best_path.add(parent_node)
    while parent_node != initial_state:
        vel_parent = path.get(parent_node)
        path_vel.add(vel_parent[2])
        parent_node = path[parent_node][0]
        best_path.add(parent_node)
        if pre_queue[0] == initial_state:
            vel_parent = path.get(parent_node)
            path_vel.add(vel_parent[2])
            parent_node = path[pre_queue[0]][0]
            best_path.add(parent_node)
            break
    final_path = sorted(best_path, reverse=False)
    path_vel = sorted(path_vel, reverse=False)
    print("Path Taken: ")
    for i in final_path:
        print(i)
    return final_path, path_vel

# class follow_path(Node):

#     def __init__(self):
#         super().__init__("Path_planning")
#         self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
#         self.timer_ = self.create_timer(0.5, self.send_velocity_command)
#         self.get_logger().info("Following the path planned")
    
#     def send_velocity_command(self):
#         msg = Twist()
#         msg.linear.x = 2.0
#         # msg.linear.y = 2.0
#         msg.angular.z = 1.0
#         self.cmd_vel_pub_.publish(msg)

# def ros_move(args=None):
#     rclpy.init(args=args)
#     node = follow_path()
#     rclpy.spin(node)
#     rclpy.shutdown()

# def ros_move(velo):
#     cmd_vel_pub_ = Node.create_publisher(Twist, "/turtle1/cmd_vel", 10)
#     Node.get_logger().info("Path Planning")
#     msg = Twist()
#     i = 0
#     while not rclpy.is_shutdown():
#         msg.linear.x = (velo[i][0])**2
#         print(msg.linear.x)
#         msg.linear.y = (velo[i][1])**2
#         msg.linear.z = (velo[i][2])**2
#         cmd_vel_pub_.publish(msg)
#         time.sleep(0.1)
#     rclpy.shutdown()
#     return    


def A_star():
    while (len(queue_nodes) != 0):
        queue_pop = queue_nodes.popitem()
        position = queue_pop[0]
        x, y, theta = position
        cc = queue_pop[1][2]
        if position not in closed_list:
            closed_list.add(position)
            if np.sqrt((node_state_g[0]-x)**2 + (node_state_g[1]-y)**2) > 1.5:
                for i in action_set:
                    Actions(i[0], i[1], position, cc)
            else:
                print("Goal reached")
                back_track, velocity_path = back_tracking(path_dict, queue_pop)
                print("Goal reached")
                print(len(back_track))                
                print(len(velocity_path))
                end_time = time.time()
                path_time = end_time - start_time
                print('Time to calculate path:', path_time, 'seconds')
                create_map(d, visited_nodes, back_track, path_dict)
                # ros_move()
                return
    print("Path cannot be acheived")


RPM1, RPM2 = 0.5, 0.5
action_set = [0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2], [
    RPM2, 0], [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]

# r = input_cdr('radius')
r = 0.105
R = 0.33
L = 0.16
# d = input_cdr('clearance')
d = 5
obstacle_space = check_obstacles(d+r)
# initial_state = input_start('Start'), input_cdr('start point')
# initial_state = (initial_state[0][0], initial_state[0][1], initial_state[1])
initial_state = (11, 11, 30)
# node_state_g = input_start('Goal'), input_cdr('goal point')
# node_state_g = (node_state_g[0][0], node_state_g[0][1], node_state_g[1])
node_state_g = (20, 20, 0)
cost = 0
closed_list = OrderedSet()
cg = np.sqrt(
    (node_state_g[0]-initial_state[0])**2 + (node_state_g[1]-initial_state[1])**2)
total_cost = cg + cost
queue_nodes = heapdict.heapdict()
path_dict = {}
visited_nodes = OrderedSet()
queue_nodes[(initial_state)] = total_cost, cg, cost
A_star()