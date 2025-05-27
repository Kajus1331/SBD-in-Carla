from carlaUtil import *
from vehicleSettings import *
import carla
import time
import math
from typing import List
from collections import deque
import pygame
import threading

import sys
# path_to_carla = 'C:/Users/Kajus/Documents/Carla/CARLA_0.9.14/WindowsNoEditor/PythonAPI/carla'
sys.path.insert(0, path_to_carla) # i.e 'C:<>/<>/PythonAPI/carla'
from agents.navigation.controller import VehiclePIDController

# ----------------------------
# Queue Management
# ----------------------------
class QueueManager:
    
    def __init__(self):
        self.def_queue = deque()
        self.queues = {
            1: deque(),
            2: deque(),
            3: deque(),
            4: deque(),
            5: deque(),
            6: deque(),
            7: deque(),
            8: deque(),
            9: deque(),
            10: deque(),
        }
        self.queueMeta = {
            1: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            2: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            3: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            4: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            5: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            6: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            7: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            8: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            9: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
            10: {"first_veh": False, "last_veh": False, "veh_added_flag": False,
                "veh_popped_flag": False, "last_print_time": None, "slot_count": 0,
                "print": False},
        }
        self.queueDict = {
            # lane 1
            339: 1, 31185: 1, 31092: 1, 338: 1, 31601: 1, # Segment 1
            337: 2, 3761: 2, 3862: 2, 336: 2, # Segment 2 etc..
            335: 3, 343: 3, 3266: 3, 342: 3, 350: 3,
            31174: 4, 349: 4, 3902: 4, 348: 4, 3775: 4, 347: 4, 31073: 4, 346: 4,
            3144: 5, 345: 5, 36: 5,

            # lane 2
            439: 6, 41185: 6, 41092: 6, 438: 6, 41601: 6,
            437: 7, 4761: 7, 4862: 7, 436: 7,
            435: 8, 443: 8, 4266: 8, 442: 8, 450: 8,
            41174: 9, 449: 9, 4902: 9, 448: 9, 4775: 9, 447: 9, 41073: 9, 446: 9,
            4144: 10, 445: 10, 46: 10,
        }
        self.global_slot_count = 0

        self.queuePairMapping = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5,
                                 6: 1, 7: 2, 8: 3, 9: 4, 10: 5}
        
        self.sharedSlotCounters = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

    def print_queues(self):
        for qnum in sorted(self.queues.keys()):
            print(f"Queue {qnum}: {list(self.queues[qnum])}")

    # remove a vehicle from its previous queue
    def remove_vehicle_from_previous_queue(self, vehicle, old_road_id):
        vehicle_id = vehicle.id
        qnum = self.queueDict.get(old_road_id, None)
        q = self.queues[qnum] if qnum is not None else self.def_queue
        if q and len(q) > 0 and q[0] == vehicle_id:
            q.popleft()
            if qnum is not None:
                meta = self.queueMeta[qnum]
                meta["veh_popped_flag"] = True
                meta["last_veh"] = True
    
    # update the queue with a new vehicle
    def update_queue(self, vehicle, road_id):
        vehicle_id = vehicle.id
        qnum = self.queueDict.get(road_id, None)
        q = self.queues[qnum] if qnum is not None else self.def_queue
        if vehicle_id in q:
            return
        q.append(vehicle_id)
        if qnum is not None:
            meta = self.queueMeta[qnum]
            meta["veh_added_flag"] = True
            if not meta["first_veh"]:
                meta["first_veh"] = True

    # add a new empty slot to the queue if no new vehicle is added
    def handle_queue_timer(self, qnum, current_time, interval):
        meta = self.queueMeta[qnum]
        if meta["first_veh"]:
            if meta["last_print_time"] is None:
                meta["last_print_time"] = current_time
            if current_time - meta["last_print_time"] >= interval:
                pair_id = self.queuePairMapping[qnum]
                self.sharedSlotCounters[pair_id] += 1
                new_slot_id = self.sharedSlotCounters[pair_id]
                if not meta["veh_added_flag"]:
                    self.queues[qnum].append(("empty", new_slot_id, False))
                    if meta.get("print", False):
                        self.print_queues()
                    meta["last_print_time"] = current_time
                else:
                    meta["veh_added_flag"] = False
                    meta["last_print_time"] = current_time
                if meta["last_veh"]:
                    if not meta["veh_popped_flag"] and self.queues[qnum]:
                        self.queues[qnum].popleft()
                    else:
                        meta["veh_popped_flag"] = False

# ----------------------------
# Vehicle Management
# ----------------------------
class VehicleManager:
    def __init__(self, queue_manager: QueueManager):
        self.vehicles: List[carla.Vehicle] = []
        self.vehicleDict = {}
        self.queue_manager = queue_manager  
        self.lane_change_lock = {}
        self.changing_lane_queue = []

    @staticmethod
    def get_speed_kmh(vehicle):
        velocity = vehicle.get_velocity()
        speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return speed_mps * 3.6

    def update_vehicle_dict(self, world, vehicle, first_car, simulation_start_time):
        current_time = time.time()
        if vehicle in self.changing_lane_queue:
            return

        # get waypoint information.
        vehicle_location = vehicle.get_location()
        vehicle_waypoint = world.get_map().get_waypoint(vehicle_location, project_to_road=True)
        # compute a composite id based on lane_id and road_id
        computed_composite_id = int(str(vehicle_waypoint.lane_id) + str(vehicle_waypoint.road_id))

        old_composite_id = self.vehicleDict.get(vehicle, computed_composite_id)

        # if the computed composite id does not match what we have stored
        # update our record and possibly the queues
        if computed_composite_id != old_composite_id:
            qnum_old = self.queue_manager.queueDict.get(old_composite_id, None)
            qnum_new = self.queue_manager.queueDict.get(computed_composite_id, None)
            if qnum_new != qnum_old:
                self.queue_manager.remove_vehicle_from_previous_queue(vehicle, old_composite_id)
                self.vehicleDict[vehicle] = computed_composite_id
                self.queue_manager.update_queue(vehicle, computed_composite_id)
                if vehicle == first_car and simulation_start_time is not None:
                    elapsed_time = current_time - simulation_start_time
            else:
                self.vehicleDict[vehicle] = computed_composite_id
        else:
            self.vehicleDict[vehicle] = computed_composite_id

    def mark_transition_state_up(self, vehicle):
        # mark the empty slot immediately in front of the vehicle as 'in transition'
        road_id = self.vehicleDict.get(vehicle)
        if road_id is None:
            return
        qnum = self.queue_manager.queueDict.get(road_id, None)
        if qnum is None:
            return
        q = self.queue_manager.queues[qnum]
        try:
            idx = q.index(vehicle.id)
        except ValueError:
            return
        # check that there is an empty slot immediately ahead
        if idx > 0 and isinstance(q[idx - 1], tuple) and q[idx - 1][0] == "empty":
            empty_slot = q[idx - 1]
            if len(empty_slot) < 3 or not empty_slot[2]:
                q[idx - 1] = (empty_slot[0], empty_slot[1], True)
                print(f"Slot {empty_slot[1]} marked as transition for Vehicle {vehicle.id} (moving up).")
        else:
            print(f"No empty slot ahead for Vehicle {vehicle.id} to mark as transition.")

    def mark_transition_state_down(self, vehicle):
        # mark the empty slot immediately behind the vehicle as 'in transition'
        road_id = self.vehicleDict.get(vehicle)
        if road_id is None:
            return
        qnum = self.queue_manager.queueDict.get(road_id, None)
        if qnum is None:
            return
        q = self.queue_manager.queues[qnum]
        try:
            idx = q.index(vehicle.id)
        except ValueError:
            return
        # check that there is an empty slot immediately behind
        if idx < len(q) - 1 and isinstance(q[idx + 1], tuple) and q[idx + 1][0] == "empty":
            empty_slot = q[idx + 1]
            if len(empty_slot) < 3 or not empty_slot[2]:
                q[idx + 1] = (empty_slot[0], empty_slot[1], True)
                print(f"Slot {empty_slot[1]} marked as transition for Vehicle {vehicle.id} (moving down).")
        else:
            print(f"No empty slot behind for Vehicle {vehicle.id} to mark as transition.")

    def mark_transition_state_lane_change(self, vehicle, target_composite_id):
        # mark the target slot in the target queue as 'in transition'
        current_queue_num = self.queue_manager.queueDict.get(self.vehicleDict.get(vehicle), None)
        target_queue_num = self.queue_manager.queueDict.get(target_composite_id, None)
        if current_queue_num is None or target_queue_num is None:
            return False
        target_queue = self.queue_manager.queues[target_queue_num]
        try:
            current_queue = self.queue_manager.queues[current_queue_num]
            idx = current_queue.index(vehicle.id) - 1
            print("idx")
            print(idx)
        except ValueError:
            return False
        if len(target_queue) > idx and isinstance(target_queue[idx], tuple) and target_queue[idx][0] == "empty":
            empty_slot = target_queue[idx]
            target_queue[idx] = (empty_slot[0], empty_slot[1], True)
            print(f"Target slot {empty_slot[1]} marked as transition for lane change of Vehicle {vehicle.id}.")
            return True
        else:
            print("Target slot not empty; cannot mark transition for lane change.")
            return False

    def update_vehicle_queue_position(self, vehicle):
        # swap vehicle with an 'empty' slot ahead if available
        road_id = self.vehicleDict.get(vehicle)
        if road_id is None:
            return
        qnum = self.queue_manager.queueDict.get(road_id, None)
        if qnum is None:
            return
        q = self.queue_manager.queues[qnum]
        try:
            idx = q.index(vehicle.id)
        except ValueError:
            return

        if idx > 0 and isinstance(q[idx - 1], tuple) and q[idx - 1][0] == "empty" and len(q[idx - 1]) > 2 and q[idx - 1][2]:
            empty_slot = q[idx - 1]
            q[idx - 1] = vehicle.id
            q[idx] = (empty_slot[0], empty_slot[1], False)
            print(f"Vehicle {vehicle.id} moved up into slot {empty_slot[1]}, transition state cleared.")
        else:
            print(f"No transition-marked empty slot available for Vehicle {vehicle.id} moving up.")


    def update_vehicle_queue_position_down(self, vehicle):
        # swap vehicle with an 'empty' slot behind if available
        road_id = self.vehicleDict.get(vehicle)
        if road_id is None:
            return
        qnum = self.queue_manager.queueDict.get(road_id, None)
        if qnum is None:
            return
        q = self.queue_manager.queues[qnum]
        try:
            idx = q.index(vehicle.id)
        except ValueError:
            return

        if idx < len(q) - 1 and isinstance(q[idx + 1], tuple) and q[idx + 1][0] == "empty" and len(q[idx + 1]) > 2 and q[idx + 1][2]:
            empty_slot = q[idx + 1]
            q[idx + 1] = vehicle.id
            q[idx] = (empty_slot[0], empty_slot[1], False)
            print(f"Vehicle {vehicle.id} moved down into slot {empty_slot[1]}, transition state cleared.")
        else:
            print(f"No transition-marked empty slot available for Vehicle {vehicle.id} moving down.")
    
    def update_lane_change_queue(self, vehicle, old_composite_id, target_composite_id):
        # update the queue for a vehicle that is changing lanes
        current_queue_num = self.queue_manager.queueDict.get(old_composite_id, None)
        target_queue_num = self.queue_manager.queueDict.get(target_composite_id, None)
        if current_queue_num is None or target_queue_num is None:
            print("Queue not found for update")
            return
        current_queue = self.queue_manager.queues[current_queue_num]
        target_queue = self.queue_manager.queues[target_queue_num]
        try:
            idx = current_queue.index(vehicle.id) - 1
        except ValueError:
            print("Vehicle not in current queue")
            return

        if len(target_queue) > idx and isinstance(target_queue[idx], tuple) and target_queue[idx][0] == "empty" and len(target_queue[idx]) > 2 and target_queue[idx][2]:
            empty_slot = target_queue[idx]
            meta = self.queue_manager.queueMeta[current_queue_num]
            current_queue[idx + 1] = ("empty", meta["slot_count"], False)
            target_queue[idx] = vehicle.id
            self.vehicleDict[vehicle] = target_composite_id
            self.lane_change_lock[vehicle] = time.time()
            print(f"Vehicle {vehicle.id} updated from queue {current_queue_num} to {target_queue_num} at slot {idx}; transition cleared.")
        else:
            print("Target slot in new lane is not marked as transition; cannot update queue.")

    # get the adjusted index for a vehicle in the target queue.
    def get_adjusted_index(self, current_queue, target_queue, vehicle_id):
        try:
            idx = current_queue.index(vehicle_id)
        except ValueError:
            return None  
        if len(current_queue) > len(target_queue):
            diff = len(current_queue) - len(target_queue)
            adjusted_idx = idx - diff
            if adjusted_idx < 0:
                adjusted_idx = 0
        else:
            adjusted_idx = idx
        return adjusted_idx
        
    # check if the target slot is empty.
    def is_target_slot_empty(self, vehicle, current_queue, target_queue):
        adjusted_idx = self.get_adjusted_index(current_queue, target_queue, vehicle.id)
        if adjusted_idx is None:
            return False
        # If the target queue is shorter than the adjusted index, assume the slot is free.
        if len(target_queue) <= adjusted_idx:
            return True
        slot = target_queue[adjusted_idx-1]
        return isinstance(slot, tuple) and slot[0] == "empty"

    # delete a vehicle if it is at the target coordinates (end of motorway)
    def delete_vehicle_if_at_target_coords(self, vehicle, tolerance=1.0):
        target_coords = [
            (-493.2, 177.7, 0.3),
            (-489.7, 179.3, 0.3),
            (-486.2, 177.7, 0.3)
        ]
        loc = vehicle.get_location()
        for (tx, ty, tz) in target_coords:
            if (abs(loc.x - tx) < tolerance and 
                abs(loc.y - ty) < tolerance and 
                abs(loc.z - tz) < tolerance):
                old_road_id = self.vehicleDict.get(vehicle)
                self.queue_manager.remove_vehicle_from_previous_queue(vehicle, old_road_id)
                vehicle.destroy()
                if vehicle in self.vehicles:
                    self.vehicles.remove(vehicle)
                if vehicle in self.vehicleDict:
                    del self.vehicleDict[vehicle]
                print(f"Deleted vehicle {vehicle.id} at location ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
                return True
        return False
    
# ----------------------------
# Draw Queues Graphically
# ----------------------------
def draw_queues(screen, queue_manager, selected_vehicle_id=None):
    screen.fill((30, 30, 30))
    
    clickable_boxes = []
    
    screen_width, screen_height = screen.get_size()
    margin_left = 150       
    margin_top = 50
    right_margin = 50
    cell_width = 50
    cell_height = 30
    gap = 10              
    queue_gap = 10

    max_cols = (screen_width - margin_left - right_margin) // (cell_width + gap)
    
    current_y = margin_top
    font = pygame.font.SysFont("Arial", 20)
    
    for qnum in sorted(queue_manager.queues.keys()):
        label = font.render(f"Road {qnum}", True, (255, 255, 255))
        screen.blit(label, (10, current_y + (cell_height // 2 - 10)))
        
        # reverse the queue order, top of the queue is on the right
        queue_items = list(queue_manager.queues[qnum])
        reversed_items = list(reversed(queue_items))
        
        num_items = len(reversed_items)
        rows_needed = (num_items + max_cols - 1) // max_cols if num_items > 0 else 1
        
        for j, item in enumerate(reversed_items):
            row_index = j // max_cols
            col_index = j % max_cols
            cell_x = margin_left + col_index * (cell_width + gap)
            cell_y = current_y + row_index * (cell_height + gap)
            rect = pygame.Rect(cell_x, cell_y, cell_width, cell_height)
            
            if isinstance(item, tuple) and item[0] == "empty":
                if len(item) > 2 and item[2]:
                    color = (255, 165, 0)  # orange for transition
                else:
                    color = (128, 128, 128)  # gray for normal empty slot
                item_text = str(item[1])
            else:
                # item here is assumed to be the vehicle id
                if selected_vehicle_id is not None and item == selected_vehicle_id:
                    color = (0, 255, 0)  # green for selected vehicle
                else:
                    color = (255, 0, 0)  # red for regular vehicles
                item_text = str(item)
                clickable_boxes.append((rect, item))
            
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, (255, 255, 255), rect, 2)
            text_surface = font.render(item_text, True, (255, 255, 255))
            text_rect = text_surface.get_rect(center=rect.center)
            screen.blit(text_surface, text_rect)
        
        current_y += rows_needed * (cell_height + gap) + queue_gap

    pygame.display.flip()
    return clickable_boxes

# calculate the distance between two vehicles in the same lane
def lane_distance(vehicle_front, vehicle_back, world, sample_interval=1.0, tolerance=2.0):
    carla_map = world.get_map()
    
    wp_front = carla_map.get_waypoint(vehicle_front.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    wp_back = carla_map.get_waypoint(vehicle_back.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if wp_front.lane_id != wp_back.lane_id:
        raise ValueError("Vehicles are not in the same lane or road.")
    
    distance = 0.0
    current_wp = wp_back
    
    # walk through the lane until we're within a tolerance of the front vehicle
    while True:
        if current_wp.transform.location.distance(vehicle_front.get_location()) < tolerance:
            distance += current_wp.transform.location.distance(vehicle_front.get_location())
            break
        
        next_wps = current_wp.next(sample_interval)
        if not next_wps:
            break
        
        next_wp = next_wps[0]
        segment_distance = current_wp.transform.location.distance(next_wp.transform.location)
        distance += segment_distance
        current_wp = next_wp
        
    return distance

# calculate the distance between the centers of two vehicles
def center_to_center_distance(vehicle1, vehicle2):
    loc1 = vehicle1.get_transform().location
    loc2 = vehicle2.get_transform().location
    dx = loc2.x - loc1.x
    dy = loc2.y - loc1.y
    dz = loc2.z - loc1.z
    return math.sqrt(dx**2 + dy**2 + dz**2)

# draw empty slots in the simulation based on the current vehicle's waypoint
def draw_slots_simulation(world, queue_manager, vehicle_manager, slot_spacing=10.0):
    carla_map = world.get_map()
    for qnum, q in queue_manager.queues.items():
        ref_vehicle = None
        for item in q:
            if not (isinstance(item, tuple) and item[0] == "empty"):
                for veh in vehicle_manager.vehicles:
                    if veh.id == item:
                        ref_vehicle = veh
                        break
            if ref_vehicle is not None:
                break

        if ref_vehicle is None:
            continue

        wp = carla_map.get_waypoint(ref_vehicle.get_location(), project_to_road=True)
        
        empty_slot_count = 0
        for element in q:
            if isinstance(element, tuple) and element[0] == "empty":
                offset_distance = slot_spacing * (empty_slot_count + 1)
                prev_wps = wp.previous(offset_distance)
                if prev_wps:
                    slot_wp = prev_wps[0]
                    # draw a small box to represent the slot
                    bbox = carla.BoundingBox(slot_wp.transform.location, carla.Vector3D(0.5, 0.5, 0.5))
                    world.debug.draw_box(
                        box=bbox,
                        rotation=slot_wp.transform.rotation,
                        thickness=0.5,
                        life_time=0.01,
                        persistent_lines=False
                    )
                empty_slot_count += 1

# ----------------------------
# Main Simulation Loop
# ----------------------------
def main():
    acceleration_in_progress = False
    deceleration_in_progress = False
    lane_change_in_progress = False

    # set up CARLA
    client, world, blueprint_library, spawn_points, traffic_manager, tm_port = setupCarla()
    traffic_manager.set_global_distance_to_leading_vehicle(5)
    green_traffic_lights(world)
    target_speed = 62  # ~60 km/h

    TOTAL_CARS_TO_SPAWN = 5
    TOTAL_CARS_TO_SPAWN_LANE2 = 5
    SPAWN_INTERVAL_SECONDS = 2.4

    queue_manager = QueueManager()
    vehicle_manager = VehicleManager(queue_manager)

    first_car = None 
    simulation_start_time = time.time()
    slot_timer_interval = 0.58

    pygame.init()
    screen = pygame.display.set_mode((1600, 800))
    pygame.display.set_caption("Vehicle Queue Display")

    spawned_count = 0
    spawned_count_lane2 = 0
    last_spawn_time = time.time()
    last_spawn_time_lane2 = time.time()
    second_lane_spawn = time.time()

    last_print_time_distance = time.time()

    selected_vehicle = None 
    
    # accelerate then decelerate maneuver using traffic manager
    def accelerate_then_decelerate2(vehicle):
        nonlocal acceleration_in_progress

        # Mark the target slot as in transition before the maneuver begins.
        vehicle_manager.mark_transition_state_up(vehicle)

        traffic_manager.set_desired_speed(vehicle, 82)
        time.sleep(2.84)
        print(VehicleManager.get_speed_kmh(vehicle))
        traffic_manager.set_desired_speed(vehicle, target_speed)
        vehicle_manager.update_vehicle_queue_position(vehicle)
        acceleration_in_progress = False
        time.sleep(2)
        # post_maneuver_spacing_adjustment(vehicle,)
    
    # accelerate then decelerate maneuver using PID controller
    def accelerate_then_decelerate_pid(vehicle, world, vehicle_manager, target_speed=60, overshoot_speed=80, duration=2.84,
                                       max_throttle=1.0, max_brake=0.0, max_steering=0.8, dt=0.05):
        """
        Move vehicle up a slot using VehiclePIDController instead of traffic manager.       
        Args:
            vehicle: carla.Vehicle object.
            world: carla.World.
            vehicle_manager: your custom vehicle manager module.
            target_speed: speed to return to after acceleration (km/h).
            overshoot_speed: speed used to move up a slot (km/h).
            duration: time to hold the higher speed (seconds).
            max_throttle: maximum throttle (0 to 1).
            max_brake: maximum brake (0 to 1).
            max_steering: maximum steering value (0 to 1).
            dt: timestep for PID calculations.
        """     
        vehicle.set_autopilot(False)        
        pid_controller = VehiclePIDController(
            vehicle,
            args_lateral={'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': dt},
            args_longitudinal={'K_P': 0.8, 'K_I': 0.05, 'K_D': 0.0, 'dt': dt},
            max_throttle=max_throttle,
            max_brake=max_brake,
            max_steering=max_steering
        )       
        vehicle_manager.mark_transition_state_up(vehicle)       
        print("[INFO] Accelerating to move up a slot...")       

        start_time = time.time()
        while time.time() - start_time < duration:
            transform = vehicle.get_transform()
            forward_vector = transform.get_forward_vector()
            destination_location = transform.location + forward_vector * 10
            destination = world.get_map().get_waypoint(destination_location, project_to_road=True)      
            control = pid_controller.run_step(overshoot_speed, destination)
            vehicle.apply_control(control)      
            world.tick()
            time.sleep(dt)      
        print("[INFO] Returning to cruising speed...")     

        cruise_time = 0.6  # seconds
        start_time = time.time()
        while time.time() - start_time < cruise_time:
            transform = vehicle.get_transform()
            forward_vector = transform.get_forward_vector()
            destination_location = transform.location + forward_vector * 10
            destination = world.get_map().get_waypoint(destination_location, project_to_road=True)      
            control = pid_controller.run_step(target_speed+2, destination)
            vehicle.apply_control(control)      
            world.tick()
            time.sleep(dt)      
        vehicle_manager.update_vehicle_queue_position(vehicle)
        print("[INFO] Slot maneuver complete.")     
        vehicle.set_autopilot(True, tm_port)    
        # post_maneuver_spacing_adjustment(vehicle)

    # decelerate then accelerate maneuver using traffic manager
    def decelerate_then_accelerate(vehicle):
        nonlocal deceleration_in_progress

        vehicle_manager.mark_transition_state_down(vehicle)

        traffic_manager.set_desired_speed(vehicle, 42)
        while VehicleManager.get_speed_kmh(vehicle) > 41:
            time.sleep(0.05)
        time.sleep(0.7)
        traffic_manager.set_desired_speed(vehicle, target_speed)
        vehicle_manager.update_vehicle_queue_position_down(vehicle)
        deceleration_in_progress = False
        time.sleep(2)
        # post_maneuver_spacing_adjustment(vehicle,)
    
    # change lane maneuver using pid controller
    def change_lane(vehicle, world, direction):
        nonlocal lane_change_in_progress
        if lane_change_in_progress:
            print("Lane change already in progress.")
            return

        vehicle.set_autopilot(False)

        dt = 1.0 / 20.0

        controller = VehiclePIDController(vehicle,
                                          args_lateral={'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': dt},
                                          args_longitudinal = {'K_P': 0.8, 'K_I': 0.05, 'K_D': 0, 'dt': dt},
                                          max_throttle=1, #0.75 def
                                          max_brake=0, # 0.3 def
                                          max_steering=0.8)
        carla_map = world.get_map()
        current_wp = carla_map.get_waypoint(vehicle.get_location())

        if direction:
            adjacent_wp = current_wp.get_right_lane()
        else:
            adjacent_wp = current_wp.get_left_lane()

        if adjacent_wp is None:
            print(f"No adjacent lane available for a {direction} lane change.")
            return

        offset_distance = 50.0  # meters ahead in the adjacent lane OFFSET
        initial_waypoints = adjacent_wp.next(offset_distance)
        if not initial_waypoints:
            print(f"Could not get an offset waypoint at {offset_distance} meters.")
            return
        offset_wp = initial_waypoints[0]

        # list of waypoints
        separation = 1.0  # distance between waypoints along the lane
        waypoints = offset_wp.next_until_lane_end(separation)
        if not waypoints:
            print("No waypoints found in the adjacent lane after the offset.")
            return

        print(f"Found {len(waypoints)} waypoints for lane change maneuver to the {direction}.")

        target_lane_id = waypoints[0].lane_id
        target_road_id = waypoints[0].road_id
        target_composite_id = int(str(target_lane_id) + str(target_road_id))
        target_queue_num = queue_manager.queueDict.get(target_composite_id, None)
        if target_queue_num is None:
            print("Target queue not found.")
            return
        target_queue = queue_manager.queues[target_queue_num]    

        current_composite_id = vehicle_manager.vehicleDict.get(vehicle, None)
        if current_composite_id is None:
            print("Vehicle has no current queue id.")
            return
        current_queue_num = queue_manager.queueDict.get(current_composite_id, None)
        if current_queue_num is None:
            print("Current queue not found.")
            return
        current_queue = queue_manager.queues[current_queue_num]
        if not vehicle_manager.is_target_slot_empty(vehicle, current_queue, target_queue):
            print("Cannot change lanes - target lane position is not empty")
            return
        
        old_composite_id = current_composite_id
        vehicle_manager.changing_lane_queue.append(vehicle)
        # for wp in waypoints:
        #     world.debug.draw_string(
        #     wp.transform.location, 'O',
        #     draw_shadow=False,
        #     color=carla.Color(r=255, g=0, b=0),
        #     life_time=5.0,  # long lifetime so they're visible throughout
        #     persistent_lines=True
        #     )
        print(target_composite_id)
        if vehicle_manager.mark_transition_state_lane_change(vehicle, target_composite_id):
            start_time = time.time()
            for wp in waypoints or time.time() - start_time < 2:
                # draw waypoint
                world.debug.draw_string(wp.transform.location, 'O',
                                          draw_shadow=False,
                                          color=carla.Color(r=255, g=0, b=0),
                                          life_time=1.0,
                                          persistent_lines=True)
                reached = False
                lane_change_in_progress = True
                while not reached:
                    control_signal = controller.run_step(60*1.15, wp)
                    vehicle.apply_control(control_signal)
                    world.tick()
                    vloc = vehicle.get_location()
                    tloc = wp.transform.location
                    distance = math.sqrt((vloc.x - tloc.x)**2 + (vloc.y - tloc.y)**2)
                    if distance < 3.5:
                        reached = True
                        print(VehicleManager.get_speed_kmh(vehicle))
                break
            print("Lane change maneuver complete using waypoint method.")
            vehicle.set_autopilot(True)
            lane_change_in_progress = False
            vehicle_manager.update_lane_change_queue(vehicle, old_composite_id, target_composite_id)
            vehicle_manager.changing_lane_queue.remove(vehicle)
            time.sleep(2)
            # post_maneuver_spacing_adjustment(vehicle,)

    
    # post-maneuver spacing adjustment function that ensures the vehicle maintains a proper distance from the vehicle ahead after a maneuver
    def post_maneuver_spacing_adjustment(vehicle, world=world, vehicle_manager=vehicle_manager, queue_manager=queue_manager, traffic_manager=traffic_manager, sample_interval=1.0, distance_tolerance=1):
        maneuver_vehicle = vehicle

        current_comp_id = vehicle_manager.vehicleDict.get(vehicle)
        if current_comp_id is None:
            print("Current vehicle composite ID not found.")
            return
        
        current_qnum = queue_manager.queueDict.get(current_comp_id, None)
        if current_qnum is None:
            print("Queue number for current vehicle not found.")
            return

        current_queue = queue_manager.queues.get(current_qnum, None)
        if current_queue is None:
            print("Current queue not found.")
            return

        try:
            idx = current_queue.index(vehicle.id)
        except ValueError:
            print("Vehicle not found in its queue.")
            return

        def find_vehicle_in_queue_reverse(q):
            for item in reversed(q):
                if not (isinstance(item, tuple) and item[0] == "empty"):
                    return item
            return None

        vehicle_ahead = None
        for i in range(idx - 1, -1, -1):
            candidate = current_queue[i]
            if not (isinstance(candidate, tuple) and candidate[0] == "empty"):
                for v in vehicle_manager.vehicles:
                    if v.id == candidate:
                        vehicle_ahead = v
                        break
            if vehicle_ahead is not None:
                break

        if vehicle_ahead is None:
            sorted_qnums = sorted(queue_manager.queues.keys())
            try:
                current_q_index = sorted_qnums.index(current_qnum)
            except ValueError:
                print("Current queue number not in sorted queue keys.")
                return
            for next_qnum in sorted_qnums[current_q_index + 1:]:
                next_queue = queue_manager.queues.get(next_qnum, [])
                candidate = find_vehicle_in_queue_reverse(next_queue)
                if candidate is not None:
                    for v in vehicle_manager.vehicles:
                        if v.id == candidate:
                            vehicle_ahead = v
                            break
                if vehicle_ahead is not None:
                    break
                
        if vehicle_ahead is None:
            print("No vehicle ahead found.")
            return

        try:
            current_distance = lane_distance(vehicle_ahead, vehicle, world, sample_interval=sample_interval, tolerance=2.0)
        except ValueError as e:
            print(f"Error computing lane distance: {e}")
            return

        target_distance = round(current_distance / 10) * 10

        print(current_distance)

        if abs(current_distance - target_distance) < distance_tolerance:
            print(f"Distance {current_distance:.2f} m is within tolerance of target {target_distance} m. No adjustment needed.")
            return

        def adjust_speed():
            print(f"Starting spacing adjustment for vehicle {vehicle.id}: current_distance={current_distance:.2f} m, target={target_distance} m")
            while True:
                try:
                    dist = lane_distance(vehicle_ahead, vehicle, world, sample_interval=sample_interval, tolerance=0.5)
                except ValueError as e:
                    print(f"Error computing lane distance during adjustment: {e}")
                    break

                diff = target_distance - dist
                #print(target_distance)
                if abs(diff) < distance_tolerance:
                    print(f"Vehicle {vehicle.id} reached target spacing of ~{target_distance} m (current: {dist:.2f} m)")
                    break

                current_speed = VehicleManager.get_speed_kmh(vehicle)
                delta_speed = 5.0
                new_speed = current_speed

                if diff < 0:
                    # distance is too short so we accelerate
                    new_speed = target_speed + delta_speed
                    print(f"Accelerating vehicle {vehicle.id} from {current_speed:.2f} km/h to {new_speed:.2f} km/h (diff: {diff:.2f} m)")
                else:
                    # distance is too long so we decelerate
                    new_speed = target_speed - delta_speed
                    print(f"Decelerating vehicle {vehicle.id} from {current_speed:.2f} km/h to {new_speed:.2f} km/h (diff: {diff:.2f} m)")

                traffic_manager.set_desired_speed(vehicle, new_speed)

            traffic_manager.set_desired_speed(vehicle, target_speed)
            print(f"Vehicle {vehicle.id} speed reset to default: {target_speed} km/h")

        adjust_thread = threading.Thread(target=adjust_speed)
        adjust_thread.start()

    # spawn vehicles in lane 1 
    def spawn_lane1():
        nonlocal spawned_count, last_spawn_time, first_car, selected_vehicle, second_lane_spawn
        while spawned_count < TOTAL_CARS_TO_SPAWN:
            current_time = time.time()
            if current_time - last_spawn_time >= SPAWN_INTERVAL_SECONDS:
                vehicle = spawnDummy(blueprint_library, spawn_points, world, 173)
                vehicle.set_autopilot(True, tm_port)
                applySettings(vehicle, traffic_manager)
                traffic_manager.set_desired_speed(vehicle, target_speed)
                traffic_manager.global_percentage_speed_difference(0)
                vehicle_manager.vehicles.append(vehicle)
                vehicle_manager.vehicleDict[vehicle] = 340  # Initial road ID placeholder.
                if first_car is None:
                    first_car = vehicle
                    #second_lane_spawn = current_time
                if spawned_count == 1:
                    selected_vehicle = vehicle

                spawned_count += 1
                last_spawn_time = current_time
            time.sleep(0.001)  # short sleep to yield control

    # spawn vehicles in lane 2
    def spawn_lane2():
        nonlocal spawned_count_lane2, last_spawn_time_lane2, second_lane_spawn
        while spawned_count_lane2 < TOTAL_CARS_TO_SPAWN_LANE2:
            current_time = time.time()
            if current_time - second_lane_spawn > 2.7:
                if current_time - last_spawn_time_lane2 >= SPAWN_INTERVAL_SECONDS:
                    vehicle = spawnDummy(blueprint_library, spawn_points, world, 172)
                    vehicle.set_autopilot(True, tm_port)
                    applySettings(vehicle, traffic_manager)
                    traffic_manager.set_desired_speed(vehicle, target_speed)
                    traffic_manager.global_percentage_speed_difference(0)
                    vehicle_manager.vehicles.append(vehicle)
                    vehicle_manager.vehicleDict[vehicle] = 340  # Initial road ID placeholder.
    
                    spawned_count_lane2 += 1
                    last_spawn_time_lane2 = current_time
            time.sleep(0.001)

    lane1_thread = threading.Thread(target=spawn_lane1)
    lane2_thread = threading.Thread(target=spawn_lane2)
    lane1_thread.start()
    lane2_thread.start()
    # lane2_spawn_begun = False
    start_time = time.time()
    last_print_time = 0
    try:
        while True:
            current_time = time.time()
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        if len(vehicle_manager.vehicles) > 1:
                            if not acceleration_in_progress:
                                acceleration_in_progress = True
                                threading.Thread(
                                    target=accelerate_then_decelerate_pid,
                                    args=(selected_vehicle, world, vehicle_manager),
                                    kwargs={
                                        'target_speed': 60,
                                        'overshoot_speed': 80,
                                        'duration': 2.1,
                                        'max_throttle': 0.81,
                                        'max_brake': 0.6,
                                        'max_steering': 0.8
                                    }
                                ).start()
                                print("Accelerating second vehicle to 80 km/h then decelerating.")
                            else:
                                print("Acceleration routine already in progress. Please wait.")
                        else:
                            print("Not enough vehicles spawned to accelerate the second vehicle.")
                    elif event.key == pygame.K_DOWN:
                        if len(vehicle_manager.vehicles) > 1:
                            if not deceleration_in_progress:
                                deceleration_in_progress = True
                                #threading.Thread(target=decelerate_then_accelerate, args=(vehicle_manager.vehicles[3],)).start()
                                threading.Thread(target=decelerate_then_accelerate, args=(selected_vehicle,)).start()
                                print("Decelerating second vehicle to 42 km/h then accelerating.")
                            else:
                                print("Deceleration routine already in progress. Please wait.")
                        else:
                            print("Not enough vehicles spawned to decelerate the second vehicle.")
                    elif event.key == pygame.K_LEFT:
                        # initiate lane change to the left.
                        if selected_vehicle is not None:
                            threading.Thread(target=change_lane, args=(selected_vehicle, world, False)).start()
                        else:
                            print("No vehicle selected for lane change.")
                    elif event.key == pygame.K_RIGHT:
                        # initiate lane change to the right.
                        if selected_vehicle is not None:
                            threading.Thread(target=change_lane, args=(selected_vehicle, world, True)).start()
                        else:
                            print("No vehicle selected for lane change.")
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    click_pos = event.pos
                    for rect, vehicle_id in clickable_boxes:
                        if rect.collidepoint(click_pos):
                            for vehicle in vehicle_manager.vehicles:
                                if vehicle.id == vehicle_id:
                                    selected_vehicle = vehicle
                                    print(f"Selected vehicle {vehicle_id}")
                                    break

            for vehicle in list(vehicle_manager.vehicles):
                if VehicleManager.get_speed_kmh(vehicle) > 20:
                    vehicle_manager.update_vehicle_dict(world, vehicle, first_car, simulation_start_time)
                    vehicle_manager.delete_vehicle_if_at_target_coords(vehicle)

            for qnum in queue_manager.queues.keys():
                queue_manager.handle_queue_timer(qnum, current_time, slot_timer_interval)

            selected_vehicle_id = getattr(selected_vehicle, "id", None)
            clickable_boxes = draw_queues(screen, queue_manager, selected_vehicle_id)


            if len(vehicle_manager.vehicles) > 1 and time.time() - last_print_time_distance > 2:
                print(VehicleManager.get_speed_kmh(vehicle_manager.vehicles[1]))
                last_print_time_distance = time.time()

            draw_slots_simulation(world, queue_manager, vehicle_manager)
            
            world.tick()

    except KeyboardInterrupt:
        print("Simulation interrupted by user")
    finally:
        lane1_thread.join()
        lane2_thread.join()
        cleanup(world)
        pygame.quit()

if __name__ == '__main__':
    main()
