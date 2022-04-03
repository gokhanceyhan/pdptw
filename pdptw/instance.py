
import os

from haversine import haversine, Unit
import numpy as np
import pandas as pd

from constants import *


class Instance:

    def __init__(self):
        self._capacities = None
        self._demands = None
        self._pickup_location_node_indices = None
        self._delivery_location_node_indices = None
        self._depot_location_node_index = 0
        self._distance_matrix = None
        self._driver_df = None
        self._driver_id_2_start_location_node_index = {}
        self._driver_id_2_end_location_node_index = {}
        self._order_df = None
        self._order_id_2_pickup_location_node_index = {}
        self._order_id_2_delivery_location_node_index = {}
        self._pickup_and_deliveries = None
        self._time_windows = None

        self._read_drivers()
        self._read_orders()
        self._create_capacitites()
        self._create_pickup_nodes()
        self._create_delivery_nodes()
        self._create_demands()
        self._create_distance_matrix()
        self._create_pickup_and_deliveries()
        self._create_time_windows()

    def _create_capacitites(self):
        self._capacities = [
            VEHICLE_TYPE_2_DRIVER_CAPACITY[driver['vehicle']] for _, driver in self._driver_df.iterrows()]

    def _create_pickup_nodes(self):
        num_drivers = len(self._driver_df)
        num_orders = len(self._order_df)
        self._pickup_location_node_indices = [
            i + 2 * num_drivers + 1 for i in range(num_orders)]

    def _create_delivery_nodes(self):
        num_drivers = len(self._driver_df)
        num_orders = len(self._order_df)
        self._delivery_location_node_indices = [
            i + num_orders + 2 * num_drivers + 1 for i in range(num_orders)]

    def _create_demands(self):
        num_drivers = len(self._driver_df)
        num_orders = len(self._order_df)
        # zero demand for the depot
        demands = [0]
        # zero demand for the driver start / end locations
        demands.extend([0] * num_drivers * 2)
        # positive demand for order pickup num_locations
        pickup_demands = list(self._order_df["no_of_items"])
        demands.extend(pickup_demands)
        delivery_demands = list(-1 * self._order_df["no_of_items"])
        demands.extend(delivery_demands)
        self._demands = demands

    def _create_distance_matrix(self):
        num_drivers = len(self._driver_df)
        num_orders = len(self._order_df)
        big_m = 1_000_000
        distances = []
        # add the distances from the central depot
        location_node_index = 0
        from_depot_to_depot = [big_m]
        from_depot_to_driver_start_locations = [0] * num_drivers
        from_depot_to_driver_end_locations = [big_m] * num_drivers
        from_depot_to_order_locations = [big_m] * num_orders * 2
        from_depot = from_depot_to_depot + from_depot_to_driver_start_locations + \
            from_depot_to_driver_end_locations + from_depot_to_order_locations
        distances.append(from_depot)
        # add distances from the driver locations
        location_node_index += 1
        for driver_id, driver in self._driver_df.iterrows():
            # for driver start location
            self._driver_id_2_start_location_node_index[driver_id] = location_node_index
            start_location = (driver["start_location_lat"], driver["start_location_long"])
            # to depot
            distances_ = [big_m]
            # to driver start locations
            distances_.extend([big_m] * num_drivers)
            # to driver end locations
            distances_.extend([0] * num_drivers)
            # to order pickup locations
            for order_id, order in self._order_df.iterrows():
                pickup_location = (order["restaurant_lat"], order["restaurant_long"])
                distance = haversine(start_location, pickup_location, unit=Unit.METERS)
                distances_.append(int(distance))
            # to order end locations
            distances_.extend([big_m] * num_orders)
            distances.append(distances_)
            # increment the location node index
            location_node_index += 1
        for driver_id, driver in self._driver_df.iterrows():
            # for driver end location
            self._driver_id_2_end_location_node_index[driver_id] = location_node_index
            # to depot
            distances_ = [0]
            # to all driver and order locations
            distances_.extend([big_m] * (num_drivers + num_orders) * 2)
            distances.append(distances_)
            # increment the location node index
            location_node_index += 1

        # add distances from the order locations
        for order_id, order in self._order_df.iterrows():
            # for pickup location
            self._order_id_2_pickup_location_node_index[order_id] = location_node_index
            pickup_location = (order["restaurant_lat"], order["restaurant_long"])
            # to depot
            distances_ = [big_m]
            # to driver start locations
            distances_.extend([big_m] * num_drivers * 2)
            # to order locations
            for order_id_, order_ in self._order_df.iterrows():
                pickup_location_ = (order_["restaurant_lat"], order_["restaurant_long"])
                distance = haversine(pickup_location, pickup_location_, unit=Unit.METERS)
                distances_.append(int(distance))
            for order_id_, order_ in self._order_df.iterrows():
                delivery_location_ = (order_["customer_lat"], order_["customer_long"])
                distance = haversine(pickup_location, delivery_location_, unit=Unit.METERS)
                distances_.append(int(distance))
            distances.append(distances_)
            # increment the location node index
            location_node_index += 1
        for order_id, order in self._order_df.iterrows():
            # for delivery location
            self._order_id_2_delivery_location_node_index[order_id] = location_node_index
            delivery_location = (order["customer_lat"], order["customer_long"])
            # to depot
            distances_ = [big_m]
            # to driver start locations
            distances_.extend([big_m] * num_drivers)
            # to driver end locations
            distances_.extend([0] * num_drivers)
            # to order locations
            for order_id_, order_ in self._order_df.iterrows():
                pickup_location_ = (order_["restaurant_lat"], order_["restaurant_long"])
                distance = haversine(delivery_location, pickup_location_, unit=Unit.METERS)
                distances_.append(int(distance))
            for order_id_, order_ in self._order_df.iterrows():
                delivery_location_ = (order_["customer_lat"], order_["customer_long"])
                distance = haversine(delivery_location, delivery_location_, unit=Unit.METERS)
                distances_.append(int(distance))
            distances.append(distances_)
            # increment the location node index
            location_node_index += 1
        self._distance_matrix = np.asarray(distances)

    def _create_pickup_and_deliveries(self):
        pickup_and_deliveries = []
        # for driver start and end locations
        for driver_id in self._driver_df.index.values:
            start = self._driver_id_2_start_location_node_index[driver_id]
            end = self._driver_id_2_end_location_node_index[driver_id]
            pickup_and_deliveries_ = [start, end]
            pickup_and_deliveries.append(pickup_and_deliveries_)
        # for regular order pickup and delivery locations
        for order_id in self._order_df.index.values:
            pickup_and_delivery = []
            pickup = self._order_id_2_pickup_location_node_index[order_id]
            delivery = self._order_id_2_delivery_location_node_index[order_id]
            pickup_and_delivery.append(pickup)
            pickup_and_delivery.append(delivery)
            pickup_and_deliveries.append(pickup_and_delivery)
        self._pickup_and_deliveries = pickup_and_deliveries

    def _create_time_windows(self):
        time_windows = []
        start_time = 0
        end_time = 10_000_000
        # for the depot
        depot_time_window = (start_time, end_time)
        time_windows.append(depot_time_window)
        # for the driver start locations
        for shift_start_time in list(self._driver_df["shift_start_sec"]):
            time_windows.append((shift_start_time, end_time))
        # for the driver end locations
        for shift_end_time in list(self._driver_df["shift_end_sec"]):
            time_windows.append((start_time, shift_end_time))
        # for order pickup locations
        for pickup_time in list(self._order_df["prep_duration_sec"]):
            time_windows.append((pickup_time, end_time))
        # for order delivery locations
        for delivery_time in list(self._order_df["preferred_otd_sec"]):
            time_windows.append((start_time, delivery_time))
        self._time_windows = time_windows

    def _read_drivers(self):
        path = os.path.join(WORKING_DIR, DRIVER_FILE_NAME)
        self._driver_df = pd.read_csv(path).set_index("driver_id")

    def _read_orders(self):
        path = os.path.join(WORKING_DIR, ORDER_FILE_NAME)
        self._order_df = pd.read_csv(path).set_index("order_id")

    def capacities(self):
        return self._capacities

    def pickup_location_node_indices(self):
        return self._pickup_location_node_indices

    def delivery_location_node_indices(self):
        return self._delivery_location_node_indices

    def depot_location_node_index(self):
        return self._depot_location_node_index

    def demands(self):
        return self._demands

    def distance_matrix(self):
        return self._distance_matrix

    def driver_df(self):
        return self._driver_df

    def driver_id_2_start_location_node_index(self):
        return self._driver_id_2_start_location_node_index

    def driver_id_2_end_location_node_index(self):
        return self._driver_id_2_end_location_node_index

    def order_df(self):
        return self._order_df

    def pickup_and_deliveries(self):
        return self._pickup_and_deliveries

    def print(self):
        print(self._driver_df())
        print(self._order_df())
        print(self._delivery_location_node_indices())
        print(self._distance_matrix())
        print(self._capacities())
        print(self._demands())
        print(self._pickup_and_deliveries())
        print(self._time_windows())

    def time_windows(self):
        return self._time_windows

    def num_drivers(self):
        return len(self._driver_df)

    def driver_vehicle_type(self, driver_idx):
        return self._driver_df.loc[driver_idx, 'vehicle']
