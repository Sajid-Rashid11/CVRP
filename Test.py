import numpy as np
import math
from Graph import Graph
from Greedy import GreedySolver

def main():
    print("CVRP Solver using Greedy Algorithm")
    print("Parsing small, medium, and large test files...")
    test_files = ['small_graph.txt', 'med_graph.txt', 'large_graph.txt']
    for file in test_files:
        print(f"\nProcessing file: {file}")
        num_nodes, vehicle_capacity, coordinates, demands = parse_file(file)
        distance_matrix = create_distance_matrix(coordinates)
        graph = Graph(num_nodes, distance_matrix, demands, coordinates)
        solver = GreedySolver(graph, vehicle_capacity)
        solver.solve()
        print(f"Total Distance: {solver.total_distance}")
        print(f"Routes: {solver.routes}")
        print(f"Number of Cars used: {len(solver.routes)}")
    

def parse_file(file_path):
    num_nodes = None
    vehicle_capacity = None
    coordinates = []
    demands = []

    current_section = None

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue

            if line.startswith('NUM_NODES:'):
                num_nodes = int(line.split(':')[1].strip())
            elif line.startswith('VEHICLE_CAPACITY:'):
                vehicle_capacity = int(line.split(':')[1].strip())

            # Section headers
            elif line == 'COORDINATES:':
                current_section = 'COORDINATES'
            elif line == 'DEMANDS:':
                current_section = 'DEMANDS'

            elif current_section == 'COORDINATES':
                parts = line.split(',')
                if len(parts) == 2:
                    x = float(parts[0].strip())
                    y = float(parts[1].strip())
                    coordinates.append((x, y))
            elif current_section == 'DEMANDS':
                demand = int(line.strip())
                demands.append(demand)
            
        return num_nodes, vehicle_capacity, coordinates, demands
        
def create_distance_matrix(coordinates):
    num_nodes = len(coordinates)
    distance_matrix = np.zeros((num_nodes, num_nodes))

    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                dist = math.sqrt((coordinates[i][0] - coordinates[j][0]) ** 2 + 
                                 (coordinates[i][1] - coordinates[j][1]) ** 2)
                distance_matrix[i][j] = dist
            else:
                distance_matrix[i][j] = 0.0
    return distance_matrix

if __name__ == "__main__":
    main()