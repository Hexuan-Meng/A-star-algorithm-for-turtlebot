import numpy as np
import time
import math
import pygame
import vidmaker
from sortedcollections import OrderedSet
import heapdict

'''
Github repository - https://github.com/sandipsharan/A_star_algorithm
'''

'''
Video Link - https://drive.google.com/file/d/1vIZtnI60usW49peaX9DfhXH0A0Eim3Nj/view?usp=sharing
'''

start_time = time.time()

def clearance(d, a):
    Hm1 = Hm3 = (-15/26)
    Hm2 = Hm4 = (15/26)
    Hc1, Hc2, Hc3, Hc4, Hc5, Hc6 = (
        4850/13), (350/13), (2900/13), -(1600/13), 235, 365
    Tm1, Tm2 = -2, 2
    Tc1, Tc2, Tc3 = 1145, -895, 460
    phc1 = parallel_line_finder(Hm1, Hc1, d, 'Greater')
    phc2 = parallel_line_finder(Hm2, Hc2, d, 'Greater')
    phc3 = parallel_line_finder(Hm3, Hc3, d, 'Lesser')
    phc4 = parallel_line_finder(Hm4, Hc4, d, 'Lesser')
    ptc1 = parallel_line_finder(Tm1, Tc1, d, 'Greater')
    ptc2 = parallel_line_finder(Tm2, Tc2, d, 'Lesser')
    if a == 1:
        return phc1, phc2, phc3, phc4, ptc1, ptc2
    hx1 = line_intersection(Hm1, Hm2, phc1, phc2, 1, 1)
    hx2 = line_intersection(Hm1, -1, phc1, Hc6+d, 1, 0)
    hx3 = line_intersection(Hm4, -1, phc4, Hc6+d, 1, 0)
    hx4 = line_intersection(Hm3, Hm4, phc3, phc4, 1, 1)
    hx5 = line_intersection(Hm3, -1, phc3, Hc5-d, 1, 0)
    hx6 = line_intersection(Hm2, -1, phc2, Hc5-d, 1, 0)
    Tx1 = line_intersection(Tm1, Tm2, ptc1, ptc2, 1, 1)
    Tx2 = line_intersection(Tm1, 0, ptc1, 225+d, 1, 1)
    Tx3 = [Tc3 - d, 225+d]
    Tx4 = [Tc3 - d, 25-d]
    Tx5 = line_intersection(Tm2, 0, ptc2, 25-d, 1, 1)
    return hx1, hx2, hx3, hx4, hx5, hx6, Tx1, Tx2, Tx3, Tx4, Tx5


def line_intersection(m1, m2, c1, c2, a, b):
    A = np.array([[-m1, a], [-m2, b]])
    B = np.array([c1, c2])
    X = np.linalg.solve(A, B)
    return X


def parallel_line_finder(m, c, d, str):
    c21 = c + d*np.sqrt(m**2 + 1)
    c22 = c - d*np.sqrt(m**2 + 1)
    if str == 'Greater':
        if c21 > c:
            return c21
        else:
            return c22
    else:
        if c21 < c:
            return c21
        else:
            return c22


def coords_pygame(coords, height):
    return (coords[0], height - coords[1])


def rect_pygame(coords, height, obj_height):
    return (coords[0], height - coords[1] - obj_height)


def create_map(d, hx1, hx2, hx3, hx4, hx5, hx6, Tx1, Tx2, Tx3, Tx4, Tx5, explored, optimal_path, path):
    pygame.init()
    size = [600, 250]
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("Visualization")
    video = vidmaker.Video("anime.mp4", late_export=True)
    clock = pygame.time.Clock()
    running = True
    x1, y1 = rect_pygame([100-d, 0], 250, 100+d)
    x2, y2 = rect_pygame([100, 0], 250, 100)
    x3, y3 = rect_pygame([100-d, 150-d], 250, 100+d)
    x4, y4 = rect_pygame([100, 150], 250, 100)

    ta2, tb2 = coords_pygame(Tx1, 250)
    tc2, td2 = coords_pygame(Tx2, 250)
    te2, tf2 = coords_pygame(Tx3, 250)
    tg2, th2 = coords_pygame(Tx4, 250)
    ti2, tj2 = coords_pygame(Tx5, 250)

    ta1, tb1 = coords_pygame([460, 25], 250)
    tc1, td1 = coords_pygame([460, 225], 250)
    te1, tf1 = coords_pygame([510, 125], 250)

    ha, hb = coords_pygame(hx1, 250)
    hc, hd = coords_pygame(hx2, 250)
    he, hf = coords_pygame(hx3, 250)
    hg, hh = coords_pygame(hx4, 250)
    hi, hj = coords_pygame(hx5, 250)
    hk, hl = coords_pygame(hx6, 250)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pygame.draw.rect(screen, "teal", [x1, y1, 50+2*d, 100+d], 0)
        pygame.draw.rect(screen, "skyblue", [x2, y2, 50, 100], 0)
        pygame.draw.rect(screen, "teal", [x3, y3, 50+2*d, 100+d], 0)
        pygame.draw.rect(screen, "skyblue", [x4, y4, 50, 100], 0)
        pygame.draw.rect(screen, "teal", [0, 0, d, 250], 0)
        pygame.draw.rect(screen, "teal", [0, 0, 600, d], 0)
        pygame.draw.rect(screen, "teal", [0, 250-d, 600, d], 0)
        pygame.draw.rect(screen, "teal", [600-d, 0, d, 250], 0)
        pygame.draw.polygon(screen, "teal", ([ta2, tb2], [tc2, td2], 
                                             [te2, tf2], [tg2, th2], [ti2, tj2]), 0)
        pygame.draw.polygon(
            screen, "skyblue", [[ta1, tb1], [tc1, td1], [te1, tf1]], 0)
        pygame.draw.polygon(screen, "teal", [[ha, hb], [hc, hd], 
                                             [he, hf], [hg, hh], [hi, hj], [hk, hl]], 0)
        pygame.draw.polygon(screen, "skyblue", ((
            235, 87.5), (300, 50), (365, 87.5), (365, 162.5), (300, 200), (235, 162.5)))
        pygame.draw.circle(screen, (0, 255, 255),
                           coords_pygame(initial_state, 250), 2)
        pygame.draw.circle(screen, (0, 255, 255),
                           coords_pygame(node_state_g, 250), 1.5)
        for l in range(len(explored)):
            pygame.draw.lines(screen, "white", False, path[explored[l]][1], width=1)
            video.update(pygame.surfarray.pixels3d(
                screen).swapaxes(0, 1), inverted=False)
            pygame.display.flip()
            clock.tick(3500)
        for i in optimal_path:
            pygame.draw.circle(screen, "black", coords_pygame(i, 250), r)
            video.update(pygame.surfarray.pixels3d(
                screen).swapaxes(0, 1), inverted=False)
            pygame.display.flip()
            clock.tick(1)
        running = False
    pygame.display.flip()
    pygame.time.wait(3000)
    pygame.quit()
    video.export(verbose=True)


def check_obstacles(d, phc1, phc2, phc3, phc4, ptc1, ptc2):
    obstacles = OrderedSet()
    for x in np.arange(0, 601):
        for y in np.arange(0, 251):
            if (x >= (235 - d) and x <= (365+d) and (y-((15/26)*x) - phc2) <= 0
                and (y+((15/26)*x) - phc1) <= 0 and (y+((15/26)*x) - phc3) >= 0
                    and (y-((15/26)*x) - phc4) >= 0):
                obstacles.add((x, y))
            if (x >= (100-d) and y >= 0 and x <= (150+d) and y <= (100+d)):
                obstacles.add((x, y))
            if (x >= (100-d) and y >= (150-d) and x <= (150+d) and y <= 250):
                obstacles.add((x, y))
            if (x >= (600-d) or y >= (250-d) or x <= d or y <= d):
                obstacles.add((x, y))
            if (x >= (460-d) and (y+(2*x) - ptc1) <= 0 and y <= (225+d) and y >= (25-d) and (y-2*x - ptc2) >= 0):
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


def check_conditions(X_n, Y_n, X_i, Y_i, T_i, Thetan, cc, ls):
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
                path_dict[current_pos] = (X_i, Y_i, T_i), ls
                visited_nodes.add(current_pos)
                return
            else:
                return
        queue_nodes[current_pos] = final_cost, cost2_go, cc
        path_dict[current_pos] = (X_i, Y_i, T_i), ls
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
    cc = c2c + math.sqrt((0.5*R*(ul + ur)*math.cos(Thetan)*t)
                         ** 2) + (0.5*R * (ul + ur) * math.sin(Thetan)*t)**2
    Thetan = np.rad2deg(Thetan)
    if 0 <= Xn <= 600 and 0 <= Yn <= 600:
        check_conditions(Xn, Yn, pos[0], pos[1], pos[2], Thetan, cc, ls)
    return


def back_tracking(path, pre_queue):
    best_path = OrderedSet()
    best_path.add(pre_queue[0])
    parent_node = path[pre_queue[0]][0]
    best_path.add(parent_node)
    while parent_node != initial_state:
        parent_node = path[parent_node][0]
        best_path.add(parent_node)
        if pre_queue[0] == initial_state:
            parent_node = path[pre_queue[0]][0]
            best_path.add(parent_node)
            break
    final_path = sorted(best_path, reverse=False)
    print("Path Taken: ")
    for i in final_path:
        print(i)
    return final_path


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
                back_track = back_tracking(path_dict, queue_pop)
                print("Goal reached")
                end_time = time.time()
                path_time = end_time - start_time
                print('Time to calculate path:', path_time, 'seconds')
                create_map(d, hx1, hx2, hx3, hx4, hx5, hx6,
                           Tx1, Tx2, Tx3, Tx4, Tx5, visited_nodes, back_track, path_dict)
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
phc1, phc2, phc3, phc4, ptc1, ptc2 = clearance((d+r), 1)
obstacle_space = check_obstacles((d+r), phc1, phc2, phc3, phc4, ptc1, ptc2)
hx1, hx2, hx3, hx4, hx5, hx6, Tx1, Tx2, Tx3, Tx4, Tx5 = clearance(d, 0)
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