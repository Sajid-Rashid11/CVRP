import numpy as np
import math

class Graph:
    def __init__(self, num_nodes, distance_matrix=None, demands=None, coordinates=None):
        self.num_nodes = num_nodes
        self.distance_matrix = distance_matrix
        self.demands = demands
        self.coordinates = coordinates  #store (x,y) coordinates of each node
        self.depot = 0

    def get_angle_from_depot(self, node_index):
        """
        Calculate polar angle from depot to the given node.
        Returns angle in radians
        """
        if node_index == self.depot:
            return 0.0
        
        depot_x, depot_y = self.coordinates[self.depot]
        node_x, node_y = self.coordinates[node_index]
        delta_x = node_x - depot_x
        delta_y = node_y - depot_y
        angle = math.atan2(delta_y, delta_x)

        #normalize angle to [0, 2*pi]
        if angle < 0:
            angle += 2 * math.pi

        return angle
    
    