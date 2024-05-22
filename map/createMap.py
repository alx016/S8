import random
import numpy as np
import matplotlib.pyplot as plt

def generate_random_map(size, obstacle_ratio=0.1):
    """
    Generates a random square map for path planning.

    Parameters:
    - size: The size of the map (size x size).
    - obstacle_ratio: The ratio of cells that will be obstacles (default is 0.2).

    Returns:
    - A 2D list representing the map, where 0 is a passable cell and 1 is an obstacle.
    """
    # Create an empty map
    map = [[0 for _ in range(size)] for _ in range(size)]
    
    # Randomly place obstacles
    for i in range(size):
        for j in range(size):
            if (i == 0 or i ==size - 1):
                map[i][j] = 1
            if (j == 0 or j == size - 1):
                map[i][j] = 1
            if random.random() < obstacle_ratio:
                map[i][j] = 1
    
    return map

def display_map(map):
    """
    Displays the map using matplotlib.

    Parameters:
    - map: The 2D list representing the map.
    """
    plt.imshow(map, cmap='gray_r')
    plt.title('Randomly Generated Map')
    plt.savefig("map_2.png")

def save_matrix_to_txt(matrix, filename):
    """
    Saves a 2D list (matrix) to a text file.

    Parameters:
    - matrix: The 2D list to save.
    - filename: The name of the file to save the matrix.
    """
    # Convert the 2D list to a numpy array
    np_matrix = np.array(matrix)
    
    # Save the numpy array to a text file
    np.savetxt(filename, np_matrix, fmt='%d')

# Parameters
filename = 'matrix2.txt'
map_size = 10  # Size of the map (10x10)
obstacle_ratio = 0.15  # 30% of the cells will be obstacles

# Generate and display the map
random_map = generate_random_map(map_size, obstacle_ratio)
display_map(random_map)
save_matrix_to_txt(random_map, filename)
