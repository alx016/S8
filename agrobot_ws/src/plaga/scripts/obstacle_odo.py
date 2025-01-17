#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from scipy.ndimage import convolve
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import logging
from heapq import heappop, heappush

# Add the RRT* algorithm
# def rrt_star(start, goal, map_array, iterations=3500, delta_q= 1.8, goal_tolerance= 1.5, bias_factor=0.3):
#     print("ENTRA A RRT_STAR")
#     path = [start]
#     parents = {start: None}

#     for _ in range(iterations):
#         # Randomly sample a point, biased towards less occupied areas
#         if np.random.rand() < bias_factor:
#             q_rand = sample_point_biased(map_array)
#         else:
#             q_rand = (np.random.rand() * map_array.shape[1], np.random.rand() * map_array.shape[0])

#         q_near = nearest_neighbor(path, q_rand)

#         q_new = new_point(q_near, q_rand, delta_q)

#         if is_collision_free(q_new, map_array):
#             path.append(q_new)
#             parents[q_new] = q_near

#         # Check if the goal is reached with tolerance
#         distance_to_goal = euclidean_distance(path[-1], goal)
#         if distance_to_goal < goal_tolerance:
#             print("Goal Reached:", path[-1])
#             break

#     # Connect the last point in the path to the goal
#     last_point = path[-1]
#     goal_reached = new_point(last_point, goal, delta_q)

#     if is_collision_free(goal_reached, map_array):
#         path.append(goal_reached)
#         parents[goal_reached] = last_point

#     return path, parents
def smooth_path(path, map_array):
    smoothed_path = [path[0]]  # Comienza con el punto de inicio

    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i:
            if is_collision_free_segment(smoothed_path[-1], path[j], map_array):
                smoothed_path.append(path[j])
                i = j
                break
            j -= 1
        i += 1

    return smoothed_path

def is_collision_free_segment(p1, p2, map_array):
    # Implementa un algoritmo de línea recta (como Bresenham) para verificar colisiones
    x1, y1 = int(p1[0]), int(p1[1])
    x2, y2 = int(p2[0]), int(p2[1])
    for x, y in bresenham(x1, y1, x2, y2):
        if map_array[y, x] == 1:  # Si hay un obstáculo en el camino
            return False
    return True

def bresenham(x1, y1, x2, y2):
    # Implementación del algoritmo de Bresenham para trazar una línea entre dos puntos
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return points

def rrt_star(start, goal, map_array, iterations=3500, delta_q=3.8, goal_tolerance=1.5, bias_factor=0.3):
    print("ENTRA A RRT_STAR")
    path = [start]
    parents = {start: None}

    for _ in range(iterations):
        # Randomly sample a point, biased towards less occupied areas
        if np.random.rand() < bias_factor:
            q_rand = sample_point_biased(map_array)
        else:
            q_rand = (np.random.rand() * map_array.shape[1], np.random.rand() * map_array.shape[0])

        q_near = nearest_neighbor(path, q_rand)

        q_new = new_point(q_near, q_rand, delta_q)

        if is_collision_free(q_new, map_array):
            path.append(q_new)
            parents[q_new] = q_near

        # Check if the goal is reached with tolerance
        distance_to_goal = euclidean_distance(path[-1], goal)
        if distance_to_goal < goal_tolerance:
            print("Goal Reached:", path[-1])
            break

    # Connect the last point in the path to the goal
    last_point = path[-1]
    goal_reached = new_point(last_point, goal, delta_q)

    if is_collision_free(goal_reached, map_array):
        path.append(goal_reached)
        parents[goal_reached] = last_point

    # Smooth the path
    smoothed_path = smooth_path(path, map_array)

    return smoothed_path, parents

def sample_point_biased(map_array):
    # Create a probability distribution based on occupancy values
    prob_distribution = (map_array.max() - map_array) / map_array.max()

    # Normalize the distribution
    prob_distribution /= prob_distribution.sum()

    # Sample a point based on the distribution
    indices = np.arange(prob_distribution.size)
    sampled_index = np.random.choice(indices, p=prob_distribution.flatten())
    sampled_row, sampled_col = np.unravel_index(sampled_index, prob_distribution.shape)

    return sampled_col, sampled_row


def backtrack_to_start(parents, start, goal):
    path = [goal]
    current = goal
    while current != start:
        current = parents[current]
        path.append(current)
    return path[::-1]

class MapSubscriber:
    def __init__(self):

        self.path = None
        self.map_data = None
        self.inverted_path = False
        self.path_generated = False     #Key so that when a path is generated it stays the same
        self.desired_thickness = 5      #Map obstacle thickness
        #self.path_pub_key = False

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #self.plan_planning = rospy.Subscriber('/plan', Bool, self.activacion)

        self.path_pub = rospy.Publisher('/rrt_path', Path, queue_size=10)
        #self.seguidor = rospy.Publisher('/seguidor_key', Bool, queue_size=2)

        self.start = (0, 0)
        self.start = (self.start[0] + 61, self.start[1] + 61) #60   # Añadir 20 a ambas coordenadas de self.start
        self.goal = (0, 0)

        #Position adjusters
        self.val_x = 0
        self.val_y = 0

        #LOG FILE
        # Create two separate loggers
        self.path_log = logging.getLogger('logger1')
        self.odom_log = logging.getLogger('logger2')

        # Set the logging level for both loggers
        self.path_log.setLevel(logging.INFO)
        self.odom_log.setLevel(logging.INFO)

        # Create handlers for both loggers
        self.path_log_handler = logging.FileHandler('path_log.log')
        self.odom_log_handler = logging.FileHandler('odom_log.log')

        # Create formatters and add them to the handlers
        formatter = logging.Formatter('%(asctime)s - %(message)s')
        self.path_log_handler.setFormatter(formatter)
        self.odom_log_handler.setFormatter(formatter)

        # Add the handlers to the loggers
        self.path_log.addHandler(self.path_log_handler)
        self.odom_log.addHandler(self.odom_log_handler)

    def map_callback(self, msg):
        self.map_data = msg
    
    def activacion(self, msg):
        self.path_pub_key= msg.data
    
    def odom_callback(self, data):
        # self.odom_log.info(f"odom_x_pos: {data.pose.pose.position.x}")
        # self.odom_log.info(f"odom_y_pos: {data.pose.pose.position.y}")

        self.goal = (data.pose.pose.position.x, data.pose.pose.position.y)
        
        self.val_x = 75/(self.goal[0] + 1)
        self.val_y = 73/(self.goal[1] + 1) #75
        self.goal = ((self.goal[0] + 1) * self.val_x, (self.goal[1] + 1) * self.val_y)
        # self.goal = ((self.goal[0] + 1), (self.goal[1] + 1))

    # def thicken_obstacles(self, map_array, thickness):
    #     # Define a kernel with the desired thickness
    #     kernel = np.ones((2 * thickness + 1, 2 * thickness + 1), dtype=np.uint8)

    #     # Convolve the map_array with the kernel to thicken the obstacles
    #     thickened_map = convolve(map_array, kernel, mode='constant', cval=0)

    #     # Ensure values are binary (obstacle or free space)
    #     thickened_map = (thickened_map > 0).astype(np.uint8)

    #     return thickened_map


    def thicken_obstacles(self, map_array, thickness):
        # Define a circular kernel with the desired thickness
        kernel_size = 2 * thickness + 1
        kernel = np.zeros((kernel_size, kernel_size), dtype=np.uint8)
        cv2.circle(kernel, (thickness, thickness), thickness, 1, -1)

        # Convolve the map_array with the kernel to thicken the obstacles
        thickened_map = convolve(map_array, kernel, mode='reflect')

        # Ensure values are binary (obstacle or free space)
        thickened_map = (thickened_map > 0).astype(np.uint8)

        return thickened_map

    def generate_rrt_star_path(self):
         
        if self.map_data is not None:
            if not self.path_generated:
                goal_reached= False
                print("en el segundo if")
                while (not goal_reached):
                    print("entrando al while")
                    # Convert the 1D occupancy grid data to a 2D numpy array
                    map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
                    self.path_log.info(map_array)
                    thickened_map = self.thicken_obstacles(map_array, self.desired_thickness)
                    map_array = thickened_map
                    # Define start and goal nodes (you can set your own coordinates)
                    print(self.goal)
                    print(self.start)

                    # Generate RRT* path
                    path, parents = rrt_star(self.start, self.goal, map_array)

                    # Print parents dictionary for debugging Puzzlebot72
                    #print("Parents Dictionary:", parents)
                    distance_goal = euclidean_distance(path[-1], self.goal)
                    print(distance_goal)
                    if (distance_goal <1.5):
                        print("consiguió el goal")
                        goal_reached = True

                # Backtrack to get the shortest path from goal to start
                shortest_path = backtrack_to_start(parents, self.start, path[-1])

                self.path = shortest_path

                # Visualize the path and the map
                self.plot_rrt_star(map_array, path, self.start, self.goal, shortest_path)
                print("Pase la funcion de rrt_star")

                # Set the flag to False to indicate that the path has been generated
                self.path_generated = True  

    def publish_path(self):
        # print("Entra a funcion publish")
        if self.path is not None:  # Check if self.path is not None
            if not self.inverted_path:
                self.path = self.path[::-1]
                self.inverted_path = True
            # print("Entra al if")
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = '/map'
            # print(self.path)

            for point in self.path:
                # print("entra al for")
                # self.path_log.info(f"path_x_pos: {(point[0] / self.val_x) - 1.0}")
                # self.path_log.info(f"path_y_pos: {(point[1] / self.val_y) - 1.0}")
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = '/map'
                # pose.pose.position.x = (point[0] / self.val_x) - 1.0
                # pose.pose.position.y = (point[1] / self.val_y) - 1.0
                pose.pose.position.x = (point[0] / 28 - 2.1) * 1.32
                pose.pose.position.y = (point[1] / 30 - 2.0) * 1.50
                # print("x:", pose.pose.position.x)
                # print("y:", pose.pose.position.y)
                # print("Esta es la x " + str(pose.pose.position.x))
                # print("Esta es la y " + str(pose.pose.position.y))
                pose.pose.position.z = 0
                pose.pose.orientation.w = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            #self.seguidor.publish(True)


    def plot_rrt_star(self, map_array, path, start, goal, shortest_path):
        print("Entre al plot_rrt")
        plt.imshow(map_array, cmap='gray_r', origin='lower')
        plt.scatter(start[0], start[1], color='red', marker='x')
        plt.scatter(goal[0], goal[1], color='green', marker='x')

        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.scatter(path_x, path_y, color='blue', linewidth=2, s=2)  # Adjusted marker size

        # Plot the shortest path in red
        shortest_path_x = [point[0] for point in shortest_path]
        shortest_path_y = [point[1] for point in shortest_path]
        print(shortest_path_x)
        plt.plot(shortest_path_x, shortest_path_y, color='red', linewidth=2)

        plt.title('RRT* Path Planning')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.savefig('occupancy_grid_map_rrt_star2.png')
        rospy.loginfo('Image saved as occupancy_grid_map_rrt_star.png')

def new_point(q_near, q_rand, delta_q):
    distance = euclidean_distance(q_near, q_rand)
    if distance < delta_q:
        return q_rand
    else:
        theta = np.arctan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0])
        new_x = q_near[0] + delta_q * np.cos(theta)
        new_y = q_near[1] + delta_q * np.sin(theta)
        return (new_x, new_y)

def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def is_collision_free(point, map_array):
    x, y = int(point[0]), int(point[1])
    return 0 <= x < map_array.shape[1] and 0 <= y < map_array.shape[0] and map_array[y, x] < 1

def nearest_neighbor(tree, point):
    distances = [euclidean_distance(node, point) for node in tree]
    return tree[np.argmin(distances)]

if __name__ == '__main__':

    rospy.init_node('map_subscriber', anonymous=True)
    map_subscriber = MapSubscriber()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        map_subscriber.generate_rrt_star_path()
        # print("Llama a funcion publish")
        map_subscriber.publish_path()
        rate.sleep()
        #print("Publishing")