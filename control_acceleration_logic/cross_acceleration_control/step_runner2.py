from __future__ import absolute_import
from __future__ import print_function

import os
import sys
from random import *
import numpy as np

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

# parameters used by the model which control the acceleration of the vehicles:
delta_s = 2
distance_min = 28
maximum_acceleration = 8
comfortable_decelaration = 2
step_duration = 1


def start_simulation(display):
    """start a simulation.

    this function is used to start a simulation

    Parameters
    ----------
    display : bool
        boolean which indicate if the graphical interface should be used.

    Returns
    -------

    """

    if display:
        sumo = 'sumo-gui'
    else:
        sumo = 'sumo'
    sumo_binary = checkBinary(sumo)
    traci.start([sumo_binary, "-c", "Net/NetworkNoTraficLight.sumocfg",
                 "--tripinfo-output", "tripinfo.xml", "--random", "--collision.check-junctions"])


def reset(display):
    """close and open a new simulation.

    this function reset the simulation.

    Parameters
    ----------
    display : bool
        boolean which indicate if the graphical interface should be used.

    Returns
    -------

    """

    traci.close()
    if display:
        sumo = 'sumo-gui'
    else:
        sumo = 'sumo'
    sumo_binary = checkBinary(sumo)
    traci.start([sumo_binary, "-c", "Net/NetworkNoTraficLight.sumocfg",
                 "--tripinfo-output", "tripinfo.xml", "--random", "--collision.check-junctions"])
    # runningVehicleID = traci.vehicle.getIDList()
    # countRunningVehicle = traci.vehicle.getIDCount()
    # return imageConstruct(runningVehicleID, runningVehicleID, 40)
    n = 40
    return np.zeros((n, n, 3), dtype=int)

def refresh_list_vehicle(list_vehicle):
    for vehicle in traci.simulation.getDepartedIDList():
        list_vehicle.append(vehicle)
        traci.vehicle.setSpeedMode(vehicle, 32)

    for vehic in list_vehicle:
        if traci.vehicle.getRouteIndex(vehic) >= 1:
            list_vehicle.remove(vehic)


def determine_speed(leader, follower):
    """Determine the new acceleration and speed for a follower vehicle.

    this function is used to determine the acceleration and the new speed of a vehicle following an other vehicle
    in order to avoid collision.

    Parameters
    ----------
    leader : string
        string containing the name of the leader vehicle.

    follower : string
        string containing the name of the vehicle follower.

    Returns
    -------
    speed : float
        float indicating the new speed of the follower vehicle.
    """
    va_1 = traci.vehicle.getSpeed(leader)
    va = traci.vehicle.getSpeed(follower)

    v0 = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(follower))  # speed in free circulation
    if traci.vehicle.getRoute(follower)[0] == traci.vehicle.getRoute(leader)[0]:
        s0 = 8  # minimum security distance
    else:
        s0 = distance_min * (traci.vehicle.getDistance(follower) / 130) + 5
    t = delta_s  # minimum time to the front vehicle
    a = maximum_acceleration
    b = comfortable_decelaration

    s_etoile = s0 + va * t + (va*(va - va_1))/(2 * np.sqrt(a * b))

    # TODO trouver une meilleur façon de calculer la distance
    # s_a = traci.vehicle.getLanePosition(leader) - traci.vehicle.getLanePosition(follower)
    s_a = traci.vehicle.getDistance(leader) - traci.vehicle.getDistance(follower)

    acceleration = a * (1 - np.power(va/v0, 4) - np.power(s_etoile/s_a, 2))

    speed = va + acceleration
    if speed < 0:
        speed = 0

    return speed



def sets_control_strategy(list_vehicle):

    for i in range(0, len(list_vehicle)-1):
        traci.vehicle.slowDown(list_vehicle[i+1], determine_speed(list_vehicle[i], list_vehicle[i+1]), step_duration)



def step_few(simulation_time, list_vehicle):
    """Control the step during the simulation.

    this function is used to run a single time step of a certain duration.

    Parameters
    ----------
    simulation_time : int
        int indicating the wished simulation time.
    leader : list
        list which contains 4 leader name.
    list_leader : list
        list which contains all the leader list.

    Returns
    -------
    done : bool
        boolean indicating if the function should be stopped
    traci.simulation.getCollidingVehiclesNumber(): int
        int containing the number of collision during the current step
    reward : int
        int containing the reward of the current step
    """
    done = 1
    refresh_list_vehicle(list_vehicle)
    for i in range(0, min(4, len(list_vehicle))):
        traci.vehicle.setColor(list_vehicle[i], (min(50*(i*5), 255), 255/max(i*1.5, 1), 0))
    sets_control_strategy(list_vehicle)
    traci.simulationStep()
    reward = traci.simulation.getArrivedNumber()

    if traci.simulation.getTime() > simulation_time:
        done = 0

    return done, traci.simulation.getCollidingVehiclesNumber(), reward


def run_simulation(display, simulation_time):
    """This function is used to run the simulation.

    this function is used to run steps until the simulation time is reached.

    Parameters
    ----------
    display : bool
        bool indicating if we want a display of the simulation.
    simulation_time : int
        int indicating the wished simulation time.

    Returns
    -------
    """
    done = 1
    reward = 0
    list_vehicle = []
    temp_collisions = 0
    collisions = 0
    start_simulation(display)

    while done:
        # set_vehicle_speed()
        done, temp_collisions, temp_reward = step_few(simulation_time, list_vehicle)
        collisions += temp_collisions
        reward += temp_reward
    print("le nombre de collisions durant cette simulation est : ", collisions)
    print("la reward de cette simulation est : ", reward)
