import math

# -------------------------------------------------------------
# Helper: distance of a route (depot -> route nodes -> depot)
# -------------------------------------------------------------
def route_distance(route, depot_index, dist_matrix):
    total = 0.0
    prev = depot_index
    for node in route:
        total += dist_matrix[prev][node]
        prev = node
    total += dist_matrix[prev][depot_index]
    return total

def heuristic_sweep(
    no_of_customers,
    weights,
    coords,
    vehicle_capacity,
    dist_matrix,
    depot_index=0
    # average_speed=1.0
):
    """
    Sweep heuristic for CVRP with unlimited number of vehicles
    Each vehicle has the same capacity
    Customer orders are given as total weight per customer

    Parameters
    ----------
    no_of_customers : int
        Number of customers (nodes 1..no_of_customers, depot is node 0)

    weights : list[float]
        weights[i] = total weight of all orders for customer i
        weights[0] must be 0 (depot)

    coords : list[(float, float)]
        coords[i] = (x, y) of node i, including depot at index 0
        Used for computing polar angles for sweep

    vehicle_capacity : float
        Capacity (max weight) each vehicle can carry.

    dist_matrix : list[list[float]]
        dist_matrix[i][j] = distance from node i to node j,
        for i, j in 0..n_customers

    depot_index : int, optional
        Index of the depot (default 0)

    average_speed : float, optional
        Average travel speed (distance units per time unit)
        If distances in km and speed in km/h → time in hours

    Returns
    -------
    result : dict
        {
            "routes": [[customer_ids...], ...],    # one route per vehicle
            "route_loads": [load_per_route, ...],  # total weight on each route
            "route_distances": [dist_per_route, ...],
            "total_distance": float,
            "total_travel_time": float,
            "num_vehicles": int
        }

    Notes
    -----
    - There is no limit on number of vehicles: we can create as many routes as needed, but we respect capacity limits
    - The number of vehicles is minimized by greedily filling each route in angular order until capacity is reached
    """

    # -----------------------------
    # Basic sanity checks
    # -----------------------------
    if len(weights) <= no_of_customers or len(coords) <= no_of_customers:
        raise ValueError("weights and coords must include depot[0] + customers[1..n_customers].")

    if len(dist_matrix) <= no_of_customers or any(len(row) <= no_of_customers for row in dist_matrix):
        raise ValueError("dist_matrix must be (n_customers+1) x (n_customers+1).")

    # Each customer's weight must fit on at least one vehicle
    for i in range(1, no_of_customers + 1):
        if weights[i] > vehicle_capacity:
            raise ValueError(
                f"Customer {i} has weight {weights[i]} which exceeds vehicle capacity {vehicle_capacity}."
            )

    x0, y0 = coords[depot_index]

    # -------------------------------------------------------------
    # Compute polar angle and radius for each customer
    # -------------------------------------------------------------
    polar_angles = []
    for i in range(1, no_of_customers + 1):
        x, y = coords[i]
        dx = x - x0
        dy = y - y0
        angle = math.atan2(dy, dx)
        radius = math.hypot(dx, dy)
        w = weights[i]

        polar_angles.append((i, angle, radius, w))

    # -------------------------------------------------------------
    # Sort by angle then radius (the sweep order)
    # -------------------------------------------------------------
    polar_angles.sort(key=lambda t: (t[1], t[2]))

    # -------------------------------------------------------------
    # 3. Create capacity-limited routes in angle order
    #    → this implicitly determines the number of vehicles used
    # -------------------------------------------------------------
    routes = []
    current_route = []
    current_load = 0.0

    for customer_id, angle, radius, w in polar_angles:
        if current_route and current_load + w > vehicle_capacity:
            # start a new vehicle/route
            routes.append(current_route)
            current_route = [customer_id]
            current_load = w
        else:
            current_route.append(customer_id)
            current_load += w

    if current_route:
        routes.append(current_route)

    route_loads = [sum(weights[i] for i in route) for route in routes]
    route_distances = []
    for route in routes:
        route_distances.append(route_distance(route, depot_index, dist_matrix))
    total_distance = sum(route_distances)
    num_vehicles = len(routes)

    return {
        "routes": routes,
        "route_loads": route_loads,
        "route_distances": route_distances,
        "total_distance": total_distance,
        # "total_travel_time": total_travel_time,
        "num_vehicles": num_vehicles,
    }

def main():
    print("Solving CVRP using Heuristic Sweep Approach")
    # depot + 4 customers
    coords = [
        (0.0, 0.0),  # depot (0)
        (1.0, 0.0),  # customer 1
        (0.0, 1.0),  # customer 2
        (-1.0, 0.0),  # customer 3
        (0.0, -1.0)  # customer 4
    ]

    # total weight of orders per customer (depot weight = 0)
    weights = [0, 3, 4, 5, 2]
    n_customers = 4
    vehicle_capacity = 6

    # distance matrix (Euclidean)
    N = n_customers + 1
    dist_matrix = [[0.0] * N for _ in range(N)]
    for i in range(N):
        for j in range(N):
            xi, yi = coords[i]
            xj, yj = coords[j]
            dist_matrix[i][j] = math.hypot(xi - xj, yi - yj)

    result = heuristic_sweep(
        no_of_customers=n_customers,
        weights=weights,
        coords=coords,
        vehicle_capacity=vehicle_capacity,
        dist_matrix=dist_matrix,
        depot_index=0,
        # average_speed=40.0  # e.g., 40 distance units per hour
    )

    print("Number of vehicles used:", result["num_vehicles"])
    print("Routes:", result["routes"])
    print("Route loads:", result["route_loads"])
    print("Route distances:", result["route_distances"])
    print("Total distance:", result["total_distance"])
    # print("Total travel time:", result["total_travel_time"])

if __name__ == "__main__":
    main()