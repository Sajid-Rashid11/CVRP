import numpy as np
import random

MAX_LIMIT = 50.0
MIN_LIMIT = 0.10

# ------------------------------------------------------------------
# Generates distance matrix randomly in range MIN_LIMIT - MAX_LIMIT
# ------------------------------------------------------------------
def create_random_distance_matrix(coordinates):
    no_of_nodes = len(coordinates)
    distance_matrix = np.zeros((no_of_nodes, no_of_nodes))

    for i in range(no_of_nodes):
        for j in range(no_of_nodes):
            if i != j:
                rand_dist = random.uniform(MIN_LIMIT, MAX_LIMIT)
                # we are using symmetric distance
                distance_matrix[i][j] = rand_dist
                distance_matrix[j][i] = rand_dist
    return distance_matrix

# --------------------------------------------------------------
# Calculates distance of a route (depot -> route nodes -> depot)
# --------------------------------------------------------------
def route_distance(route, depot_index, dist_matrix):
    total = 0.0
    prev = depot_index
    for node in route:
        total += dist_matrix[prev][node]
        prev = node
    total += dist_matrix[prev][depot_index]
    return total
