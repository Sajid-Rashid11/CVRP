import math


class GreedySolver:
    def __init__(self, graph, vehicle_capacity):
        self.graph = graph
        self.vehicle_capacity = vehicle_capacity
        self.routes = []
        self.total_distance = 0.0

    def solve(self):
        unvisited_nodes = set(range(1,self.graph.num_nodes-1))  #exclude depot (node 0)
        
        while unvisited_nodes:
            #start new route from the depot (node 0)
            curr_route = [0]
            curr_location = 0
            remaining_capacity = self.vehicle_capacity

            #choose nearest neighbor with best distance to capacity ratio
            while unvisited_nodes:
                best_node = self.get_best_node(curr_location, unvisited_nodes, remaining_capacity)
                if best_node is None:
                    break  # No more nodes can be added to this route
                
                curr_route.append(best_node)
                unvisited_nodes.remove(best_node)
                remaining_capacity -= self.graph.demands[best_node]
                self.total_distance += self.graph.distance_matrix[curr_location][best_node]
                curr_location = best_node
            #return to depot and add distance
            self.total_distance += self.graph.distance_matrix[curr_location][0]
            curr_route.append(0) 
            self.routes.append(curr_route)
        
    #gets closest node that fits in remaining capacity
    def get_best_node(self, curr_node, unvisited_nodes, remaining_capacity):
        best_node = None
        smallest_distance = math.inf
        for node in unvisited_nodes:
            if self.graph.demands[node] <= remaining_capacity:
                if self.graph.distance_matrix[curr_node][node] < smallest_distance:
                    smallest_distance = self.graph.distance_matrix[curr_node][node]
                    best_node = node
        return best_node