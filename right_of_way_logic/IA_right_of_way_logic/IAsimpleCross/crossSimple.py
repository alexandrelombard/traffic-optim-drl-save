from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
import matplotlib.pyplot
from Tools import statistique

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


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
    n = 3
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
        traci.vehicle.setSpeedMode(id_vehicle[i], 55)


def set_action(action, veh_id):
    """Change the actions of the given vehicles.

    this function is used to change the actions of the given vehicles.

    Parameters
    ----------
    veh_id : list
        list containing the names of the vehicles.
    action : list
        list containing the wished action for the vehicles
        0 for stop 1 for go

    Returns
    -------


    """

    for i in range(0, len(action)):
        if action[i] == 1:
            try:
                traci.vehicle.setStop(veh_id[i], traci.vehicle.getRoadID(veh_id[i]), 90, duration=0)
            except traci.TraCIException:
                print("vehicle not on the edge")
            try:
                follower = traci.vehicle.getFollower(veh_id[i])
                traci.vehicle.setStop(follower[0], traci.vehicle.getRoadID(follower[0]), 90)
            except traci.TraCIException:
                print("no follower")
        elif action[i] == 0:
            try:
                traci.vehicle.setStop(veh_id[i], traci.vehicle.getRoadID(veh_id[i]), 90)
            except traci.TraCIException:
                print("to late to brake")
            try:
                follower = traci.vehicle.getFollower(veh_id[i])
                traci.vehicle.setStop(follower[0], traci.vehicle.getRoadID(follower[0]), 90)
            except traci.TraCIException:
                print("no follower")


def leader_arriver(leaders_tab, action):
    """Indicate if the vehicle have left the intersection.

    this function is used to determine when the vehicle authorized to pass the intersection have left the intersection.

    Parameters
    ----------
    leaders_tab : list
        list containing the name of the 4 leaders.

    action : list
        list containing the wished action for the vehicles
        0 for stop 1 for go

    Returns
    -------
    leader_bool : bool
        boolean indicating if the vehicles have finished to cross the intersection
    """
    leader_bool = True
    for i in range(0, len(leaders_tab)):
        try:
            value = traci.vehicle.getRouteIndex(leaders_tab[i])
        except traci.TraCIException:
            value = 1
        if value == 0 and action[i] == 1:
            leader_bool = False

    return leader_bool


def image_construct(leader, n, display=False):
    """this build a representation of the current state of the simulation.

    this function is used to to build an N by N image of a state of the simulation.
    each colored pixel represent a vehicle.
    the intensity of the pixel represent the speed and the color of the pixel their destination

    Parameters
    ----------
    leader: list
        list containing the name of the leaders
    n : int
        size of the image.
    display : bool
        indicate if the image should be displayed

    Returns
    -------
    image : ndarray
        representation of the image

    """
    image = np.zeros((n, n, 3), dtype=int)

    if leader[0] != "":
        route_list = traci.vehicle.getRoute(leader[0])

        if route_list[len(route_list) - 1] == "2to1":
            image[1][0][0] = 255
            # print(image[pixel_pos_x][pixel_pos_y][0])
        elif route_list[len(route_list) - 1] == "2to4":
            image[1][0][1] = 255
            # print(image[pixel_pos_x][pixel_pos_y][1])
        elif route_list[len(route_list) - 1] == "2to5":
            image[1][0][2] = 255
            # print(image[pixel_pos_x][pixel_pos_y][2])
        elif route_list[len(route_list) - 1] == "2to3":
            image[1][0][0] = 255
            image[1][0][1] = 255

    if leader[1] != "":
        route_list = traci.vehicle.getRoute(leader[1])

        if route_list[len(route_list) - 1] == "2to1":
            image[2][1][0] = 255
            # print(image[pixel_pos_x][pixel_pos_y][0])
        elif route_list[len(route_list) - 1] == "2to4":
            image[2][1][1] = 255
            # print(image[pixel_pos_x][pixel_pos_y][1])
        elif route_list[len(route_list) - 1] == "2to5":
            image[2][1][2] = 255
            # print(image[pixel_pos_x][pixel_pos_y][2])
        elif route_list[len(route_list) - 1] == "2to3":
            image[2][1][0] = 255
            image[2][1][1] = 255

    if leader[2] != "":
        route_list = traci.vehicle.getRoute(leader[2])

        if route_list[len(route_list) - 1] == "2to1":
            image[1][2][0] = 255
            # print(image[pixel_pos_x][pixel_pos_y][0])
        elif route_list[len(route_list) - 1] == "2to4":
            image[1][2][1] = 255
            # print(image[pixel_pos_x][pixel_pos_y][1])
        elif route_list[len(route_list) - 1] == "2to5":
            image[1][2][2] = 255
            # print(image[pixel_pos_x][pixel_pos_y][2])
        elif route_list[len(route_list) - 1] == "2to3":
            image[1][2][0] = 255
            image[1][2][1] = 255

    if leader[3] != "":
        route_list = traci.vehicle.getRoute(leader[3])

        if route_list[len(route_list) - 1] == "2to1":
            image[0][1][0] = 255
            # print(image[pixel_pos_x][pixel_pos_y][0])
        elif route_list[len(route_list) - 1] == "2to4":
            image[0][1][1] = 255
            # print(image[pixel_pos_x][pixel_pos_y][1])
        elif route_list[len(route_list) - 1] == "2to5":
            image[0][1][2] = 255
            # print(image[pixel_pos_x][pixel_pos_y][2])
        elif route_list[len(route_list) - 1] == "2to3":
            image[0][1][0] = 255
            image[0][1][1] = 255

    # printImage(image, N)
    if display:
        matplotlib.pyplot.imshow(image)
        matplotlib.pyplot.show()

    return image


def set_loaded_vehicle(leader_tab):
    """set a stop for vehicle who are not leaders.

    This function is used to set a default stop to the vehicle who are not leaders in the simulation.

    Parameters
    ----------
    leader_tab : list
        a list of string containing the name of the leaders

    Returns
    -------

    """
    number_vehicle = traci.simulation.getDepartedNumber()
    id_vehicle = traci.simulation.getDepartedIDList()
    for i in range(0, number_vehicle):
        not_leader = True
        for j in range(0, len(leader_tab)):
            if id_vehicle[i] == leader_tab[j]:
                not_leader = False
        if not_leader:
            try:
                traci.vehicle.setStop(id_vehicle[i], traci.vehicle.getRoadID(id_vehicle[i]), 90)
                traci.vehicle.setColor(id_vehicle[i], (255, 200, 0))
            except traci.TraCIException:
                pass


def trad_action(action):
    """Pass the action from the decimal base to the binary one.

    Parameters
    ----------
    action : int
        action in decimal base

    Returns
    -------
    action_binaire : list
        containing the binary action
    """
    action_binaire = [0, 0, 0, 0]
    count = 0
    while action != 0:
        action_binaire[len(action_binaire)-1 - count] = action % 2
        action = action // 2
        count += 1

    return action_binaire


def get4leader():
    """Determine the closest vehicle to the intersection of each edge.

    This function is used to determine the 4 closest vehicle to the intersections

    Parameters
    ----------

    Returns
    -------
    leader_tab : list
        return a list with the name of the 4 vehicle
    """
    leader_tab = ["", "", "", ""]
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    for i in range(0, len(flow_edges)):
        if traci.edge.getLastStepVehicleNumber(flow_edges[i]) >= 1:
            leader_tab[i] = traci.edge.getLastStepVehicleIDs(
                flow_edges[i])[traci.edge.getLastStepVehicleNumber(flow_edges[i]) - 1]
            if leader_tab[i] != "":
                traci.vehicle.setColor(leader_tab[i], (255, 255, 255))
    return leader_tab


def step_few(action, simulation_time, reward_type, coef):
    """"Control the step during the simulation.

    this function is used to control the step of the simulation action after action.

    Parameters
    ----------
    action : list
        containing the actions 0 or 1

    simulation_time : int
        int which indicate the wished simulation time in second

    reward_type : string
        indicating the reward type used

    coef : float
        coefficient used in the reward calculation

    Returns
    -------
    average_waiting_time : float
        float containing the average waiting time for a vehicle during a step
    cumulated_waiting_time : float
        float containing the cumulated waiting time during a step
    emission_of_co2 : float
        float containing the quantity of Co2 emitted in mg during a step
    average_speed :
        float containing the average speed of a vehicle
    done : bool
        indicating if the simulation should be stopped

    """
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    count = 0
    reward = 0
    done = 0
    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0

    collision = 0
    while traci.vehicle.getIDCount() <= 0:
        set_speed_mode()
        traci.simulationStep()

    leader = get4leader()
    set_action(action, leader)

    while not leader_arriver(leader, action):

        count += 1
        set_speed_mode()
        set_loaded_vehicle(leader)
        traci.simulationStep()
        collision += traci.simulation.getCollidingVehiclesNumber()
        set_speed_mode()
        reward += statistique.reward_calculation(reward_type, coef)  # traci.simulation.getArrivedNumber()
        # reward -= collision
        average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
        cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
        average_speed += statistique.vitesseMoyenne(flow_edges)
        emission_co2 += statistique.emissionDeCo2(flow_edges)
        if traci.simulation.getTime() > simulation_time:
            done = 1

    b_action0 = True
    for i in range(0, len(action)):
        if action[i] == 1:
            b_action0 = False

    if b_action0:
        set_speed_mode()
        set_loaded_vehicle(leader)
        traci.simulationStep()
        reward += statistique.reward_calculation(reward_type, coef)

    lane_empty = False
    for i in range(0, len(leader)):
        if leader[i] == "":
            lane_empty = True
    if lane_empty:
        set_speed_mode()
        set_loaded_vehicle(leader)
        traci.simulationStep()
        reward += statistique.reward_calculation(reward_type, coef)

    if count > 0:
        average_waiting_time = average_waiting_time/count
        cumulated_waiting_time = cumulated_waiting_time/count
        average_speed = average_speed/count
        emission_co2 = emission_co2//count

    leader = get4leader()
    state_next = image_construct(leader, 3)

    return state_next, reward, done, average_waiting_time, cumulated_waiting_time, emission_co2, average_speed
