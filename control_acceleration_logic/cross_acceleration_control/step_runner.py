from __future__ import absolute_import
from __future__ import print_function

import project

import os
import sys
from random import *
import numpy as np

import sumo_utils # noqa

# parameters used by the model which control the acceleration of the vehicles:
delta_s = 2
distance_min = 2
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
    traci.start([sumo_binary, "-c", project.resources_dir + "Net/NetworkNoTraficLight.sumocfg",
                 "--tripinfo-output", project.resources_dir + "tripinfo.xml", "--random", "--collision.check-junctions"])


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
    traci.start([sumo_binary, "-c", project.resources_dir + "Net/NetworkNoTraficLight.sumocfg",
                 "--tripinfo-output", project.resources_dir + "tripinfo.xml", "--random", "--collision.check-junctions"])
    # runningVehicleID = traci.vehicle.getIDList()
    # countRunningVehicle = traci.vehicle.getIDCount()
    # return imageConstruct(runningVehicleID, runningVehicleID, 40)
    n = 40
    return np.zeros((n, n, 3), dtype=int)


def set_speed_mode():
    """set the vehicle comportment.

    This function is used to change the comportment of the new loaded vehicle in the simulation.
    for example disable the collision security.

    Parameters
    ----------


    Returns
    -------

    """
    number_vehicle = traci.simulation.getLoadedNumber()
    id_vehicle = traci.simulation.getLoadedIDList()
    for i in range(0, number_vehicle):
        traci.vehicle.setSpeedMode(id_vehicle[i], 32)


def leader_arriver(leaders_tab):

    """Indicate if the vehicle have leave the intersection.

    this function is used to determine when the vehicle authorized to pass the intersection have leave the intersection.
    When the vehicle is arrived he is erased from the lists containing the leaders.

    Parameters
    ----------
    leaders_tab : list
        list containing the name of the 4 leaders.

    Returns
    -------
    bool : bool
        boolean indicating if the vehicles have finished to cross the intersection
    """

    leader_bool = True
    for i in range(0, len(leaders_tab)):
        try:
            value = traci.vehicle.getRouteIndex(leaders_tab[i])
        except traci.TraCIException:
            value = 1
        if value == 0:
            leader_bool = False
        if value == 1:
            leaders_tab[i] = ""

    return leader_bool


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
    s0 = distance_min  # minimum security distance
    t = delta_s  # minimum time to the front vehicle
    a = maximum_acceleration
    b = comfortable_decelaration

    s_etoile = s0 + va * t + (va*(va - va_1))/(2 * np.sqrt(a * b))

    # TODO trouver une meilleur façon de calculer la distance
    s_a = traci.vehicle.getLanePosition(leader) - traci.vehicle.getLanePosition(follower)

    acceleration = a * (1 - np.power(va/v0, 4) - np.power(s_etoile/s_a, 2))

    speed = va + acceleration
    if speed < 0:
        speed = 2
    return speed


def bubble_sort(leader, distance_tab, speed_tab, time_to_goal):
    """Sort list by the distance.

    this function is used to sort a list of leader and other related list
    by the distance.

    Parameters
    ----------
    leader : list
        list containing the name of the leaders vehicles.

    distance_tab : list
        list containing the distance of the leaders vehicles.

    speed_tab : list
        list containing the speed of the leaders vehicles.

    time_to_goal : list
        list containing the time to goal of the leaders vehicles.

    Returns
    -------
    leader : list
        sorted list containing the name of the leaders vehicles.

    distance_tab : list
        sorted list containing the distance of the leaders vehicles.

    speed_tab : list
        sorted list containing the speed of the leaders vehicles.

    time_to_goal : list
        sorted list containing the time to goal of the leaders vehicles.
    """

    n = len(distance_tab)
    # Traverser tous les éléments du tableau
    for i in range(n):
        for j in range(0, n-i-1):
            # échanger si l'élément trouvé est plus grand que le suivant
            if distance_tab[j] > distance_tab[j+1]:
                distance_tab[j], distance_tab[j+1] = distance_tab[j+1], distance_tab[j]
                speed_tab[j], speed_tab[j + 1] = speed_tab[j + 1], speed_tab[j]
                leader[j], leader[j + 1] = leader[j + 1], leader[j]
                time_to_goal[j], time_to_goal[j + 1] = time_to_goal[j + 1], time_to_goal[j]
    return leader, distance_tab, speed_tab, time_to_goal


def sets_control_strategy(leader, list_leader, index):
    """apply the acceleration control model on the vehicles.

    this function is used to determine what is the orders of the leaders in the intersection which allow us to determine
    which vehicle is following which vehicle
    Then the correct acceleration is applied on the vehicles to avoid collision

    Parameters
    ----------
    leader : list
        list containing the name of 4 leader on which we will apply the correct acceleration.

    list_leader : list
        list containing all the leaders list in their order of incoming.

    index : int
        int indicating the index of the leaders we are currently processing in the list_leader.

    Returns
    -------

    """
    speed_tab = [0] * len(leader)
    distance_tab = [0] * len(leader)
    time_to_goal = [0] * len(leader)

    for i in range(0, len(leader)):
        if leader[i] != "" and traci.vehicle.getLaneID(leader[i]) != "":
            speed_tab[i] = traci.vehicle.getSpeed(leader[i])

            # TODO trouver une meilleure façon de determiner la distance
            distance_tab[i] = traci.lane.getLength(traci.vehicle.getLaneID(leader[i])) \
                - traci.vehicle.getLanePosition(leader[i])

            time_to_goal[i] = distance_tab[i]/(speed_tab[i] + 0.01)
        elif leader[i] != "":
            speed_tab[i] = traci.vehicle.getSpeed(leader[i])
            distance_tab[i] = 0
            time_to_goal[i] = 1
        else:
            speed_tab[i] = -10000
            distance_tab[i] = -10000
            time_to_goal[i] = -10000

    # tri à bulle :
    leader, distance_tab, speed_tab, time_to_goal = bubble_sort(leader, distance_tab, speed_tab, time_to_goal)

    for i in range(0, len(leader)-1):
        if leader[i] != "":
            if leader[i+1] != "":
                speed_tab[i+1] = distance_tab[i+1]/(time_to_goal[i] + delta_s)
                time_to_goal[i+1] = time_to_goal[i] + delta_s

    if index - 1 >= 0:
        # print(index - 1, "indexou")
        # print("list_leader index -1", list_leader[index-1][3])
        # print(list_leader[index-1][3], "list_leader actuel qui nous interesse")
        if list_leader[index - 1][3] != "" and leader[0] != "":
            traci.vehicle.slowDown(leader[0], determine_speed(list_leader[index-1][3], leader[0]), step_duration)
            # traci.vehicle.setColor(leader[0], (255, 255, 255))
            # print("vehicle blanc position :", traci.vehicle.getLanePosition(leader[0]))
            # traci.vehicle.setColor(list_leader[index-1][3], (0, 0, 255))
            # print("vehicle bleu position", traci.vehicle.getLanePosition(list_leader[index-1][3]))
            # print("distance entre les vehicle", traci.vehicle.getLanePosition(list_leader[index-1][3]) - traci.vehicle.getLanePosition(leader[0]))

    for i in range(0, len(leader)-1):
        if leader[i+1] != "" and leader[i] != "":
            traci.vehicle.slowDown(leader[i+1], determine_speed(leader[i], leader[i+1]), step_duration)

    # print("***leader***")
    # print(leader, "\n")
    # print("***speed_tab***")
    # print(speed_tab, "\n")
    # print("***distance_tab***")
    # print(distance_tab, "\n")
    # print("***time_togoal***")
    # print(time_to_goal, "\n")


def control_strategy(leader, list_leader):
    """add the vehicle arrived in the network delete the gone one and apply the strategy
    on the vehicle in the intersection.

    this function is used to add the new vehicles arrived in the list_leader then delete the one the vehicles
    that are gone and then use the function sets_control_strategy to apply the strategy on all the vehicles in the list

    Parameters
    ----------
    leader : list
        list containing the name of 4 leader on which we will apply the correct acceleration.

    list_leader : list
        list containing all the leaders list in their order of incoming.

    Returns
    -------
    leader : list
        list containing 4 latest leader.

    """

    temp = traci.simulation.getDepartedIDList()

    alea1 = randint(0, 255)
    alea2 = randint(0, 255)
    alea3 = randint(0, 255)

    for i in range(0, len(temp)):
        leader[i] = temp[i]
        traci.vehicle.setColor(leader[i], (alea1, alea2, alea3))

    if len(temp) > 0:
        list_leader.append(leader[:])

    o = 0
    while o < len(list_leader):

        if leader_arriver(list_leader[o]):
            list_leader.remove(list_leader[o])
        o += 1

    # print(list_leader, "je suis list leader entier")
    for i in range(0, len(list_leader)):
        # print(i, "iterator")
        sets_control_strategy(list_leader[i], list_leader, i)

    return leader


def step_few(simulation_time, leader, list_leader):
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
    set_speed_mode()
    leader = control_strategy(leader, list_leader)
    traci.simulationStep()
    reward = traci.simulation.getArrivedNumber()

    if traci.simulation.getTime() > simulation_time:
        done = 0

    return done, leader, traci.simulation.getCollidingVehiclesNumber(), reward


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
    leader = ["", "", "", ""]
    list_leader = []
    temp_collisions = 0
    collisions = 0
    start_simulation(display)

    while done:
        # set_vehicle_speed()
        done, leader, temp_collisions, temp_reward = step_few(simulation_time, leader, list_leader)
        collisions += temp_collisions
        reward += temp_reward
    print("le nombre de collisions durant cette simulation est : ", collisions)
    print("la reward de cette simulation est : ", reward)
