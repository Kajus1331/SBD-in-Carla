import math
import carla

vehicle_data = {}

def applySettings(dummy_vehicle, traffic_manager):
    # Set parameters of TM vehicle control, we don't want lane changes for this vehicle
    #traffic_manager.set_global_distance_to_leading_vehicle(20)
    #traffic_manager.distance_to_leading_vehicle(dummy_vehicle, 5)
    #9traffic_manager.update_vehicle_lights(dummy_vehicle, True)
    traffic_manager.random_left_lanechange_percentage(dummy_vehicle, 0)
    traffic_manager.random_right_lanechange_percentage(dummy_vehicle, 0)
    traffic_manager.auto_lane_change(dummy_vehicle, False)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.ignore_vehicles_percentage(dummy_vehicle, 100)