import numpy as np
import random

MAX_LIMIT = 50.0
MIN_LIMIT = 10.0

# ------------------------------------------------------------------
# Generates distance matrix randomly in range MIN_LIMIT - MAX_LIMIT
# ------------------------------------------------------------------
def create_random_distance_matrix(coordinates):
    no_of_nodes = len(coordinates)
    distance_matrix = np.zeros((no_of_nodes, no_of_nodes))

    for i in range(no_of_nodes):
        for j in range(i, no_of_nodes): # as we are using symmetric distance we can only run the loop by dividing the matrix diagonally
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

def print_results(no_of_vehicles, total_dist, total_time, routes, weights, dist_matrix, capacity):
    print("\nOutputs:")
    print(f"Number of vehicles used: {no_of_vehicles}")
    print(f"Total distance: {total_dist}")
    print(f"Time taken to process = {total_time} milliseconds")
    print("Routes:")
    for r_idx, route in enumerate(routes, start=1):
        # Compute load of this route (ignore depots)
        route_load = sum(weights[i] for i in route)

        # adding the depot in the beginning if needed
        if route[0] != 0:
            route = [0] + route

        # adding the depot at last if needed
        if route[len(route) - 1] != 0:
            route = route + [0]

        # Compute distance of this route
        route_dist = 0.0
        for a, b in zip(route, route[1:]):
            route_dist += dist_matrix[a][b]

        print(f"  Route {r_idx}: {' -> '.join(map(str, route))}")
        print(f"    Load: {route_load} / {capacity}")
        print(f"    Distance: {route_dist}")
