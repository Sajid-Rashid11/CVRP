from Test import parse_file
from Helper import create_random_distance_matrix, print_results
from math import inf
from copy import deepcopy
import time

def bounded_branch_and_prune(
    dist,
    demands,
    capacity,
    max_nodes=None,         # optional: limit on search nodes
    time_limit_sec=None,    # optional: wall-clock time limit
):
    """
    Branch-and-prune CVRP solver with:
      - capacity constraint (no route exceeds capacity),
      - unlimited vehicles but objective favors fewer vehicles,
      - greedy initial solution for a good upper bound,
      - optional node and time limits.

    Parameters
    ----------
    dist : list[list[float]]
        Distance matrix, dist[i][j] is distance from node i to node j.
        Node 0 is depot, nodes 1..n are customers.
    demands : list[float]
        Demands per node. demands[0] must be 0.
    capacity : float
        Vehicle capacity.
    max_nodes : int or None
        If set, stop search after visiting this many nodes (return best so far).
    time_limit_sec : float or None
        If set, stop search after this many seconds (return best so far).

    Returns
    -------
    best_routes : list[list[int]]
        Each route: [0, ..., 0] including depot at start and end.
    best_distance : float
    best_vehicles : int
    stopped_early : bool
        True if search was cut by max_nodes/time_limit.
    """

    start_time = time.time()
    n = len(dist) - 1
    customers = list(range(1, n + 1))

    # --- Basic checks ---
    if len(demands) != len(dist):
        raise ValueError("demands must have the same length as dist dimension.")
    if demands[0] != 0:
        raise ValueError("demands[0] (depot) must be 0.")
    for i in customers:
        if demands[i] > capacity:
            raise ValueError(
                f"Customer {i} demand ({demands[i]}) exceeds vehicle capacity ({capacity})."
            )

    # --- Objective: minimize vehicles then distance ---
    max_dist = max(dist[i][j] for i in range(len(dist)) for j in range(len(dist)))
    max_possible_distance = 2 * max_dist * n
    BIG_M = max_possible_distance + 1

    # --- Greedy initial solution (for an initial upper bound) ---
    def greedy_solution():
        unvisited = set(customers)
        routes = []
        loads = []

        while unvisited:
            route = [0]
            load = 0
            last = 0

            while True:
                feasible = [i for i in unvisited if load + demands[i] <= capacity]
                if not feasible:
                    break
                # nearest neighbor
                next_cust = min(feasible, key=lambda j: dist[last][j])
                route.append(next_cust)
                load += demands[next_cust]
                last = next_cust
                unvisited.remove(next_cust)

            route.append(0)
            routes.append(route)
            loads.append(load)

        total_dist = 0.0
        for r in routes:
            for a, b in zip(r, r[1:]):
                total_dist += dist[a][b]

        used_veh = len(routes)
        return routes, loads, total_dist, used_veh

    g_routes, g_loads, g_dist, g_veh = greedy_solution()

    best_routes = deepcopy(g_routes)
    best_distance = g_dist
    best_vehicles = g_veh
    best_obj = BIG_M * g_veh + g_dist

    # --- Lower bound & feasibility helpers ---
    def lower_bound_distance(unserved, last_node, current_distance):
        # Same LB as before: close current route + depot-to-customer-to-depot
        lb = current_distance
        if last_node != 0:
            lb += dist[last_node][0]
        for i in unserved:
            lb += 2 * dist[0][i]
        return lb

    def feasible_next_customers(unserved, current_load):
        return [i for i in unserved if current_load + demands[i] <= capacity]

    # --- DFS with pruning, node/time limits ---
    nodes_visited = 0
    stopped_early = False

    def search(unserved, routes, loads, current_distance, used_vehicles):
        nonlocal best_obj, best_routes, best_distance, best_vehicles
        nonlocal nodes_visited, stopped_early

        # Check limits
        if stopped_early:
            return
        if max_nodes is not None and nodes_visited >= max_nodes:
            stopped_early = True
            return
        if time_limit_sec is not None and (time.time() - start_time) >= time_limit_sec:
            stopped_early = True
            return

        nodes_visited += 1
        current_route = routes[-1]
        last_node = current_route[-1]

        # A. All customers served
        if not unserved:
            final_distance = current_distance
            if last_node != 0:
                final_distance += dist[last_node][0]
                current_route.append(0)

            obj = BIG_M * used_vehicles + final_distance
            if obj < best_obj:
                best_obj = obj
                best_routes = deepcopy(routes)
                best_distance = final_distance
                best_vehicles = used_vehicles

            # backtrack depot closure
            if current_route[-1] == 0 and len(current_route) > 1:
                current_route.pop()
            return

        # B. Prune by lower bound on objective
        lb_dist = lower_bound_distance(unserved, last_node, current_distance)
        lb_obj = BIG_M * used_vehicles + lb_dist
        if lb_obj >= best_obj:
            return

        # C. Branch 1: add next feasible customer
        feasibles = feasible_next_customers(unserved, loads[-1])
        feasibles.sort(key=lambda i: dist[last_node][i])  # nearest first

        for i in feasibles:
            new_distance = current_distance + dist[last_node][i]
            new_obj = BIG_M * used_vehicles + new_distance
            if new_obj >= best_obj:
                continue

            current_route.append(i)
            loads[-1] += demands[i]
            unserved.remove(i)

            search(unserved, routes, loads, new_distance, used_vehicles)

            unserved.add(i)
            loads[-1] -= demands[i]
            current_route.pop()

        # D. Branch 2: close current route and start new vehicle
        if last_node != 0:
            close_distance = current_distance + dist[last_node][0]

            # quick bound for new route
            lb_after_close_dist = lower_bound_distance(unserved, 0, close_distance)
            lb_after_close_obj = BIG_M * (used_vehicles + 1) + lb_after_close_dist
            if lb_after_close_obj >= best_obj:
                return

            current_route.append(0)
            routes.append([0])
            loads.append(0)

            search(unserved, routes, loads, close_distance, used_vehicles + 1)

            loads.pop()
            routes.pop()
            current_route.pop()

    # Initial DFS call
    initial_routes = [[0]]
    initial_loads = [0]
    search(set(customers), initial_routes, initial_loads, 0.0, 1)

    return best_routes, best_distance, best_vehicles, stopped_early

def branch_and_prune(dist, demands, capacity):
    """
    Solve CVRP using branch-and-prune (depth-first search + bounding).

    Parameters
    ----------
    dist : list[list[float]]
        Distance matrix, dist[i][j] is distance from node i to node j.
        Node 0 is the depot, nodes 1..n are customers.
    demands : list[float]
        Demands per node. demands[0] must be 0 (depot),
        demands[i] is total weight of orders of customer i.
    capacity : float
        Vehicle capacity (same for all vehicles).

    Returns
    -------
    best_routes : list[list[int]]
        List of routes, each route is a list of node indices, starting
        and ending at 0 (depot).
    best_distance : float
        Total distance of the solution.
    best_vehicles : int
        Number of vehicles used.
    """

    n = len(dist) - 1  # number of customers (nodes 1..n)
    customers = list(range(1, n + 1))

    # --- Basic sanity checks ---
    if len(demands) != len(dist):
        raise ValueError("demands must have the same length as dist dimension.")
    if demands[0] != 0:
        raise ValueError("demands[0] (depot) must be 0.")
    for i in customers:
        if demands[i] > capacity:
            raise ValueError(
                f"Customer {i} demand ({demands[i]}) exceeds vehicle capacity ({capacity})."
            )

    # Precompute a large M for lexicographic objective: minimize vehicles, then distance
    max_dist = max(dist[i][j] for i in range(len(dist)) for j in range(len(dist)))
    max_possible_distance = 2 * max_dist * n
    BIG_M = max_possible_distance + 1  # ensure +1 so 1 extra vehicle always worse

    best_obj = inf
    best_routes = None
    best_distance = inf
    best_vehicles = None

    def lower_bound_distance(unserved, last_node, current_distance):
        lb = current_distance
        if last_node != 0:
            lb += dist[last_node][0]
        for i in unserved:
            lb += 2 * dist[0][i]
        return lb

    def feasible_next_customers(unserved, current_load):
        return [i for i in unserved if current_load + demands[i] <= capacity]

    def search(unserved, routes, loads, current_distance, used_vehicles):
        nonlocal best_obj, best_routes, best_distance, best_vehicles

        current_route = routes[-1]
        last_node = current_route[-1]

        # A. All customers served -> close last route, update best solution
        if not unserved:
            final_distance = current_distance
            if last_node != 0:
                final_distance += dist[last_node][0]
                current_route.append(0)

            obj = BIG_M * used_vehicles + final_distance
            if obj < best_obj:
                best_obj = obj
                best_routes = deepcopy(routes)
                best_distance = final_distance
                best_vehicles = used_vehicles

            # Backtrack depot closure if we added it
            if current_route[-1] == 0 and len(current_route) > 1:
                current_route.pop()
            return

        # B. Prune by lower bound
        lb_dist = lower_bound_distance(unserved, last_node, current_distance)
        lb_obj = BIG_M * used_vehicles + lb_dist
        if lb_obj >= best_obj:
            return

        # C. Branch 1: add a feasible next customer
        feasibles = feasible_next_customers(unserved, loads[-1])
        feasibles.sort(key=lambda i: dist[last_node][i])

        for i in feasibles:
            new_distance = current_distance + dist[last_node][i]
            new_obj = BIG_M * used_vehicles + new_distance

            if new_obj >= best_obj:
                continue

            current_route.append(i)
            loads[-1] += demands[i]
            unserved.remove(i)

            search(unserved, routes, loads, new_distance, used_vehicles)

            unserved.add(i)
            loads[-1] -= demands[i]
            current_route.pop()

        # D. Branch 2: close current route and start a new one (unlimited vehicles)
        if last_node != 0:
            close_distance = current_distance + dist[last_node][0]

            lb_after_close_dist = lower_bound_distance(unserved, 0, close_distance)
            lb_after_close_obj = BIG_M * (used_vehicles + 1) + lb_after_close_dist
            if lb_after_close_obj >= best_obj:
                return

            current_route.append(0)
            routes.append([0])
            loads.append(0)

            search(unserved, routes, loads, close_distance, used_vehicles + 1)

            loads.pop()
            routes.pop()
            current_route.pop()

    # Initial state
    initial_routes = [[0]]
    initial_loads = [0]
    search(set(customers), initial_routes, initial_loads, 0.0, 1)

    return best_routes, best_distance, best_vehicles

def main():
    print("Solving CVRP using Branch and Prune Approach")
    test_files = ["small_graph.txt", "med_graph.txt", "large_graph.txt"]
    # test_files = ["small_graph.txt"]
    # test_files = ["med_graph.txt"]
    # test_files = ["large_graph.txt"]
    for file in test_files:
        print(f"\nSolving for file: {file}")
        no_of_customers, capacity, coordinates, demands = parse_file(file)
        print("\nInputs:")
        print(f"No of customers = {no_of_customers - 1}")
        print(f"Vehicle capacity = {capacity}")
        print(f"Coordinates = {coordinates}")
        print(f"Demands = {demands}")
        dist_matrix = create_random_distance_matrix(coordinates)
        start_time = time.perf_counter()
        routes, total_dist, num_vehicle, stopped = bounded_branch_and_prune(
            dist_matrix,
            demands,
            capacity,
            max_nodes=1000,  # example node limit
            time_limit_sec=30.0  # example time limit
        )
        # routes, total_dist, num_vehicle = branch_and_prune(dist_matrix, demands, capacity)
        end_time = time.perf_counter()
        elapsed_time = (end_time - start_time) * 1000.0
        print(f"\nStopped early: {stopped}")
        print_results(num_vehicle, total_dist, elapsed_time, routes, demands, dist_matrix, capacity)

if __name__ == "__main__":
    main()