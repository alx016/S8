#!/usr/bin/env python3

import numpy as np
from scipy.ndimage import convolve
import matplotlib.pyplot as plt
from heapq import heappop, heappush


def rrt_star(start, goal, map_array, goal_tolerance, iterations=1000, delta_q=0.1, bias_factor=0.01):
    print("Generating Tree")
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

    return path, parents

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

def load_matrix_from_txt(filename):
    """
    Loads a matrix from a text file.

    Parameters:
    - filename: The name of the file to load the matrix from.

    Returns:
    - The matrix as a 2D numpy array.
    """
    return np.loadtxt(filename, dtype=int)

class MapSubscriber:
    def __init__(self):
        self.path = None
        self.map_data = binary_map
        self.path_generated = False
        self.start = (2, 2)
        # self.start = (self.start[0] + 25, self.start[1] + 60) # Update the starting coordinates
        self.goal = (2, 8)
        self.inverted_path = False
        self.desired_thickness = 4
        self.goal_tolerance = 1
        self.pathPlanning_activationKey = False

    def thicken_obstacles(self, map_array, thickness):
        # Define a kernel with the desired thickness
        kernel = np.ones((2 * thickness + 1, 2 * thickness + 1), dtype=np.uint8)

        # Convolve the map_array with the kernel to thicken the obstacles
        thickened_map = convolve(map_array, kernel, mode='constant', cval=0)

        # Ensure values are binary (obstacle or free space)
        thickened_map = (thickened_map > 0).astype(np.uint8)

        return thickened_map

    def generate_rrt_star_path(self):
        print("Starting Path Planning")
        if self.map_data is not None:
            if not self.path_generated:
                goal_reached= False
                print("Processing")
                while (not goal_reached):
                    # Convert the 1D occupancy grid data to a 2D numpy array
                    map_array = self.map_data #np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
                    # thickened_map = map_array #self.thicken_obstacles(map_array, self.desired_thickness)
                    # map_array = thickened_map

                    # Generate RRT* path
                    path, parents = rrt_star(self.start, self.goal, map_array, self.goal_tolerance)

                    distance_goal = euclidean_distance(path[-1], self.goal)
                    # print(distance_goal)
                    if (distance_goal < self.goal_tolerance):
                        goal_reached = True

                # Backtrack to get the shortest path from goal to start
                # print("Aqui")
                shortest_path = backtrack_to_start(parents, self.start, path[-1])

                self.path = shortest_path
                # Visualize the path and the map
                self.plot_rrt_star(map_array, path, self.start, self.goal, shortest_path)

                # Set the flag to False to indicate that the path has been generated
                self.path_generated = True


    def plot_rrt_star(self, map_array, path, start, goal, shortest_path):
        print("Plotting")
        plt.imshow(map_array, cmap='gray_r', origin='lower')
        plt.scatter(start[0], start[1], color='red', marker='x')
        plt.scatter(goal[0], goal[1], color='green', marker='x')

        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.scatter(path_x, path_y, color='blue', linewidth=1, s=2)  # Adjusted marker size

        # Plot the shortest path in red
        shortest_path_x = [point[0] for point in shortest_path]
        shortest_path_y = [point[1] for point in shortest_path]
        plt.plot(shortest_path_x, shortest_path_y, color='red', linewidth=2)

        plt.title('RRT* Path Planning')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.savefig('/home/al3x/S8/map/occupancy_grid_map_rrt_star.png')
        print('Image saved as occupancy_grid_map_rrt_star.png')

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
    return (0 < x < map_array.shape[0]-2) and (0 < y < map_array.shape[1]-2) and map_array[y, x] < 1

def nearest_neighbor(tree, point):
    distances = [euclidean_distance(node, point) for node in tree]
    return tree[np.argmin(distances)]

if __name__ == '__main__':

    filename = "matrix.txt"
    binary_map = load_matrix_from_txt(filename)
    map_subscriber = MapSubscriber()
    map_subscriber.generate_rrt_star_path()
