from __future__ import print_function

import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationAlongRoute,
                                                                               InTimeToArrivalToVehicle,
                                                                               DriveDistance)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp


class Context0(BasicScenario):
    """
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=1200):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 30
        # other vehicle parameters
        self._other_actors_target_velocity = 3
        self._other_actors_max_brake = 1.0
        self._time_to_reach = 10
        self._num_lane_changes = 1
        self.transforms = []
        self.other_actor = []
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()
        super(Context0, self).__init__("Context0", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        for other_actor in config.other_actors:
            transform = carla.Transform(
            carla.Location(other_actor.transform.location.x,
                           other_actor.transform.location.y,
                           other_actor.transform.location.z),
                           other_actor.transform.rotation)

            vehicle = CarlaDataProvider.request_new_actor('walker.*', transform)
            vehicle.set_simulate_physics(enabled=True)
            self.other_actors.append(vehicle)
            self.transforms.append(transform)


    def _create_behavior_branch(self, node, other_actor, transform):
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)
        dist_to_trigger = 50 + self._num_lane_changes
        scenario_sequence = py_trees.composites.Sequence()

        # nodes
        start_condition = InTimeToArrivalToVehicle(self.ego_vehicles[0], other_actor, self._time_to_reach)

        # creating behaviour nodes
        actor_start_cross_lane = AccelerateToVelocity(other_actor, 1.0, self._other_actors_target_velocity, name="{} accelarting".format(other_actor.id))
        actor_cross_lane = DriveDistance(other_actor, lane_width * 3, name="{} drive distance for lane crossing".format(other_actor.id))
        actor_stop_crossed_lane = StopVehicle(other_actor, self._other_actors_max_brake, name="{} stop".format(other_actor.id))
        #actor_remove = ActorDestroy(other_actor, name="Destroying Actor: {}".format(other_actor.id))

        # adding behaviout nodes to a behaviour sequence
        scenario_sequence.add_child(ActorTransformSetter(other_actor, transform, name='TransformSetterTS3walker'))
        scenario_sequence.add_child(HandBrakeVehicle(other_actor, True))
        scenario_sequence.add_child(start_condition)
        scenario_sequence.add_child(HandBrakeVehicle(other_actor, False))
        scenario_sequence.add_child(actor_start_cross_lane)
        scenario_sequence.add_child(actor_cross_lane)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        #scenario_sequence.add_child(actor_remove)

        return scenario_sequence

    def _create_behavior(self):
        """
        """ 
        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="Context0")

        for other_actor, transform in zip(self.other_actors, self.transforms):
            branch = self._create_behavior_branch(root, other_actor, transform)
            root.add_child(branch)

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