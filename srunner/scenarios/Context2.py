#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Object crash without prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encountering a cyclist ahead.
"""

from __future__ import print_function

from enum import Enum

import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      BasicAgentBehavior,
                                                                      HandBrakeVehicle,
                                                                      WaypointFollower,
                                                                      KeepVelocity,
                                                                      Idle,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationAlongRoute,
                                                                               InTimeToArrivalToVehicle,
                                                                               DriveDistance)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp

class Context2(BasicScenario):
    """
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 240
        # other vehicle parameters
        self._other_actor_target_velocity = 4.5
        self._other_actors_max_brake = 1.0
        self._time_to_reach = 20
        self._num_lane_changes = 1
        self.transforms = []
        self.other_actor = []
        self.other_actors_plan = []
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location

        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(Context2, self).__init__("Context2",
                                          ego_vehicles,
                                          config,
                                          world,
                                          debug_mode,
                                          criteria_enable=criteria_enable)

    def _spawn_blocker(self, location, shift = 0.9):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of the jaywalker
        :return:
        """
        # static object transform
        shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = location.x
        y_cycle = location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)


        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().get_waypoint(location)

        transform = carla.Transform(carla.Location(x_static, y_static,
                                                         spawn_point_wp.transform.location.z),
                                          carla.Rotation(yaw=270))

        static = CarlaDataProvider.request_new_actor('static.prop.vendingmachine', transform)
        static.set_simulate_physics(enabled=False)


        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        for other_actor in config.other_actors:
            location = carla.Location(other_actor.transform.location.x,
                           other_actor.transform.location.y,
                           other_actor.transform.location.z)
            transform = carla.Transform(location, other_actor.transform.rotation)

            vehicle = CarlaDataProvider.request_new_actor(other_actor.model, transform)
            vehicle.set_simulate_physics(enabled=True)

            self.other_actors.append(vehicle)


            plan = [
                location,  
                carla.Location(location.x, location.y - 21, location.z), # crossing the street
                carla.Location(location.x - 30, location.y - 21, 0), #  going to the house (middle point to make the path smooth)
                carla.Location(location.x - 37, location.y - 20.5, 0), # going the the house (endpoint)
                carla.Location(location.x - 37, location.y - 25, 0), # going to the garage (midpoint)
                carla.Location(location.x - 40, location.y - 29, 0) # going to the garage (endpoint)
            ]

            for p in plan: print(p.x, p.y, p.z)

            self.transforms.append(transform)

            self.other_actors_plan.append(plan)

            blocker = self._spawn_blocker(location)


    def _create_behavior_branch(self, node, other_actor, transform, plan = []):
        dist_to_trigger = 50 + self._num_lane_changes

        scenario_sequence = py_trees.composites.Sequence()

        # nodes
        if self._ego_route is not None:
            start_condition = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                    self._ego_route,
                                                                    self.transform.location,
                                                                    dist_to_trigger)
        else:
            start_condition = InTimeToArrivalToVehicle(self.ego_vehicles[0],
                                                       other_actor,
                                                       self._time_to_reach)


        # creating behaviour nodes
        actor_accelerate = AccelerateToVelocity(other_actor, 1.0, self._other_actor_target_velocity, name="{} accelarting".format(other_actor.id))
        actor_crossed_lane = DriveDistance(other_actor, 8, name="{} drive distance for direction changing".format(other_actor.id))
        

        # adding behaviout nodes to a behaviour sequence
        scenario_sequence.add_child(ActorTransformSetter(other_actor, transform, name='TransformSetterTS3walker'))
        scenario_sequence.add_child(HandBrakeVehicle(other_actor, True))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(HandBrakeVehicle(other_actor, False))

        if len(plan) > 0:
            actor_route = WaypointFollower(other_actor, 3, plan, avoid_collision = False)
            scenario_sequence.add_child(actor_route)
        else:
            scenario_sequence.add_child(actor_accelerate)
            scenario_sequence.add_child(actor_crossed_lane)
            scenario_sequence.add_child(HandBrakeVehicle(other_actor, True))            

        scenario_sequence.add_child(StopVehicle(other_actor, self._other_actors_max_brake, name="{} stop".format(other_actor.id)))
        scenario_sequence.add_child(Idle())

        return scenario_sequence

    def _create_behavior(self):
        """
        """ 

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="ObjectCrossing")

        # building tree
        ego_pass_machine = DriveDistance(self.ego_vehicles[0], 10, name="ego vehicle passed props")
        end_condition = DriveDistance(self.ego_vehicles[0], self._ego_vehicle_distance_driven, name="End condition ego drive distance")

        for i in range(len(self.other_actors)):
            other_actor = self.other_actors[i]
            transform = self.transforms[i]
            other_actor_plan = self.other_actors_plan[i]
            branch = self._create_behavior_branch(root, other_actor, transform, other_actor_plan)
            root.add_child(branch)


        stop_after_scenario = False
        if stop_after_scenario:
            root.add_child(ego_pass_machine)
            root.add_child(end_condition)
        else:
            root.add_child(Idle())

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()