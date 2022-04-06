"""Implements a solver for the PDPTW by using the routing solver of Google OR Tools"""

import math

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from constants import *
from instance import Instance


class Solver:

    def __init__(self, instance):
        self._instance = instance

    def _print_solution(self, manager, routing, solution):
        """Prints the solution on console"""
        num_drivers = len(self._instance.driver_df())
        total_distance = 0
        max_route_distance = 0
        total_travel_time = 0
        max_route_travel_time = 0
        max_pickup_distance_in_meters = 0
        late_deliveries = []
        time_dimension = routing.GetDimensionOrDie('Time')
        distance_dimension = routing.GetDimensionOrDie('Distance')
        for vehicle_id in range(num_drivers):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            route_travel_time = 0
            route_load = 0
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                node_index = manager.IndexToNode(index)
                route_load += self._instance.demands()[node_index]
                plan_output += ' {0} Load({1}) Time({2},{3}) -> '.format(
                    node_index, route_load, solution.Min(time_var),
                    solution.Max(time_var))
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += distance_dimension.GetTransitValue(previous_index, index, vehicle_id)
                route_travel_time += time_dimension.GetTransitValue(previous_index, index, vehicle_id)
                delivery_node = node_index in self._instance.delivery_location_node_indices()
                if delivery_node:
                    target_delivery_time = self._instance.time_windows()[node_index][1]
                    earliest_delivery_time = solution.Min(time_var)
                    if  earliest_delivery_time > target_delivery_time:
                        late_deliveries.append((node_index, earliest_delivery_time - target_delivery_time))
                else:
                    max_pickup_distance_in_meters = max(
                        max_pickup_distance_in_meters, 
                        distance_dimension.GetTransitValue(previous_index, index, vehicle_id))
            # handle the last node visited
            time_var = time_dimension.CumulVar(index)
            node_index = manager.IndexToNode(index)
            delivery_node = node_index in self._instance.delivery_location_node_indices()
            if delivery_node:
                target_delivery_time = self._instance.time_windows()[node_index][1]
                earliest_delivery_time = solution.Min(time_var)
                if  earliest_delivery_time > target_delivery_time:
                    late_deliveries.append((node_index, earliest_delivery_time - target_delivery_time))
            plan_output += ' {0} Load({1}) Time({2},{3}) \n'.format(
                manager.IndexToNode(index), route_load, solution.Min(time_var),
                solution.Max(time_var))
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            plan_output += 'Load of the route: {}\n'.format(route_load)
            plan_output += 'Time of the route: {}sec\n'.format(solution.Min(time_var))
            print(plan_output)
            total_distance += route_distance
            max_route_distance = max(max_route_distance, route_distance)
            total_travel_time += route_travel_time
            max_route_travel_time = max(max_route_travel_time, route_travel_time)
        print('Total distance of all routes: {}m'.format(total_distance))
        print('Max route distance: {}m'.format(max_route_distance))
        print('Max pickup distance: {}m'.format(max_pickup_distance_in_meters))
        print('Total travel time of all routes: {}sec'.format(int(total_travel_time)))
        print('Max route travel time: {}sec'.format(int(max_route_travel_time)))
        print('Number of late deliveries: {}'.format(len(late_deliveries)))
        delayed_nodes = [late_delivery[0] for late_delivery in late_deliveries]
        print('Delayed nodes: {}'.format(delayed_nodes))
        delays = [late_delivery[1] for late_delivery in late_deliveries]
        print('Delays: {}'.format(delays))
        total_delay = sum(delays)
        print('Total delay: {}sec'.format(total_delay))

    def run(self):
        """Creates the routing problem and solves it"""
        big_m = 1_000_000
        num_nodes = len(self._instance.distance_matrix())
        num_drivers = len(self._instance.driver_df())
        print(f"{num_nodes} nodes with {num_drivers} drivers")
        depot_node_index = 0

        # Create the routing index manager
        manager = pywrapcp.RoutingIndexManager(num_nodes, num_drivers, depot_node_index)
        # Create Routing Model
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes"""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return self._instance.distance_matrix()[from_node][to_node]

        distance_callback_index = routing.RegisterTransitCallback(distance_callback)

        dimension_name = 'Distance'
        routing.AddDimension(
            distance_callback_index,
            0,  # no slack
            big_m,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name)
        distance_dimension = routing.GetDimensionOrDie(dimension_name)

        def demand_callback(from_index):
            """Returns the demand of the node"""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = manager.IndexToNode(from_index)
            return self._instance.demands()[from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            self._instance.capacities(),  # vehicle maximum capacities
            True,  # start cumul to zero
            'Capacity')

        for request in self._instance.pickup_and_deliveries():
            pickup_index = manager.NodeToIndex(request[0])
            delivery_index = manager.NodeToIndex(request[1])
            # an order must be served by the same driver
            # driver start and end location visits are also modeled as dummy pickup and delivery
            routing.AddPickupAndDelivery(pickup_index, delivery_index)
            routing.solver().Add(
                routing.VehicleVar(pickup_index) == routing.VehicleVar(
                    delivery_index))
            # pickup before delivery
            routing.solver().Add(
                distance_dimension.CumulVar(pickup_index) <=
                distance_dimension.CumulVar(delivery_index))
            # max pickup distance hard constraint
            distance_dimension.TransitVar(pickup_index).SetMax(MAX_PICKUP_DISTANCE_IN_METERS)
            
            
        time_callback_indices = []
        for driver_index in range(self._instance.num_drivers()):

            def time_callback(from_index, to_index, driver_idx=driver_index):
                """Returns the travel time between the two nodes."""
                # Convert from routing variable Index to time matrix NodeIndex.
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                distance_in_m = self._instance.distance_matrix()[from_node][to_node]
                distance_in_km = distance_in_m / 1000
                vehicle_type = self._instance.driver_vehicle_type(driver_idx)
                speed = VEHICLE_TYPE_2_DRIVER_SPEED[vehicle_type]
                time_in_h = distance_in_km / speed
                time_in_s = time_in_h * 3600
                return int(time_in_s)

            time_callback_index = routing.RegisterTransitCallback(time_callback)
            time_callback_indices.append(time_callback_index)

        time = 'Time'
        routing.AddDimensionWithVehicleTransits(
            time_callback_indices,
            big_m,  # allow waiting time
            big_m,  # maximum time per vehicle
            False,  # Don't force start cumul to zero.
            time)
        time_dimension = routing.GetDimensionOrDie(time)

        # Add time window constraints for each location except depot.
        for location_node_index, time_window in enumerate(self._instance.time_windows()):
            if location_node_index == 0:
                continue
            index = manager.NodeToIndex(location_node_index)
            lb = time_window[0]
            ub = time_window[1]
            if location_node_index in self._instance.delivery_location_node_indices():
                time_dimension.CumulVar(index).SetMin(lb)
                time_dimension.SetCumulVarSoftUpperBound(index, ub, LATE_DELIVERY_PER_SEC_PENALTY)
            else:
                time_dimension.CumulVar(index).SetRange(lb, ub)

        # Define cost of each arc
        def arc_cost_callback(from_index, to_index):
            """Returns the arc cost"""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            distance = self._instance.distance_matrix()[from_node][to_node]
            long_distance_penalty = ABOVE_MAX_DISTANCE_PER_METER_PENALTY * max(0, distance - SOFT_MAX_DISTANCE_IN_METERS)
            return distance + long_distance_penalty

        arc_cost_callback_index = routing.RegisterTransitCallback(arc_cost_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(arc_cost_callback_index)

        for i in range(num_drivers):
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(i)))
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.End(i)))

        # Setting search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC)
        search_parameters.time_limit.seconds = 30
        search_parameters.log_search = True

        # Solve the problem
        solution = routing.SolveWithParameters(search_parameters)
        print("Solver status: ", routing.status())

        # Print solution on console
        if solution:
            self._print_solution(manager, routing, solution)
        else:
            print('No solution found !')


def main():
    instance = Instance()
    solver = Solver(instance)
    solver.run()


if __name__ == "__main__":
    main()
