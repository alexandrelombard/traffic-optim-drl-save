import project

from __future__ import absolute_import
from __future__ import print_function

import os

import sys
import numpy as np
from Tools import simulation_parametre_tools as simu_tools, statistique

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
                 "--tripinfo-output", project.resources_dir + "tripinfo.xml", "--random"])

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
    number_vehicle = traci.simulation.getDepartedNumber()
    id_vehicle = traci.simulation.getDepartedIDList()
    for i in range(0, number_vehicle):
        traci.vehicle.setSpeedMode(id_vehicle[i], 55)


def set_action(action, veh_id):
    """Change the actions of the given vehicles.

    this function is used to change the actions of the given vehicles.

    Parameters
    ----------
    veh_id : ndarray
        list containing the names of the vehicles.
    action : ndarray
        list containing the wished action for the vehicles
        0 for stop 1 for go

    Returns
    -------


    """
    for i in range(0, len(action)):
        if action[i] == 1:
            try:
                traci.vehicle.setColor(veh_id[i], (0, 255, 0))
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
                traci.vehicle.setColor(veh_id[i], (255, 0, 0))
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
    bool : bool
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


def set_loaded_vehicle(leader_tab):
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


def strategy_first_in_first_off():
    """This function apply the first in first off strategy in the simulation.

    this function is used to determine which is the vehicle who waited the most and then determine which vehicle
    can cross the intersection with the vehicle who waited the most.

    Parameters
    ----------

    Returns
    -------
    leader : list
        list containing the leaders
    action : list
        list containing the actions taken

    """

    action = np.zeros(4)
    is_vehicle = False
    leader = ["", "", "", ""]
    waiting_time = np.zeros((2, 4))
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]

    for i in range(0, len(flow_edges)):
        if traci.edge.getLastStepVehicleNumber(flow_edges[i]) > 0:
            is_vehicle = True
            leader[i] = traci.edge.getLastStepVehicleIDs(flow_edges[i])[
                traci.edge.getLastStepVehicleNumber(flow_edges[i]) - 1]
            waiting_time[0][i] = traci.edge.getWaitingTime(flow_edges[i])
            waiting_time[1][i] = i
            if leader[i] != "":
                waiting_time[0][i] += 1

    # Bubble sort 
    n = len(waiting_time[0])
    # go through all array element
    for i in range(n):
        for j in range(0, n-i-1):
            # exchange of the current element is bigger than the next element
            if waiting_time[0][j] < waiting_time[0][j+1]:
                waiting_time[0][j], waiting_time[0][j+1] = waiting_time[0][j+1], waiting_time[0][j]
                waiting_time[1][j], waiting_time[1][j + 1] = waiting_time[1][j + 1], waiting_time[1][j]

    if is_vehicle:

        action[int(waiting_time[1][0])] = 1
        # traci.vehicle.setColor(leader[int(waiting_time[1][0])],(255,255,255))
        vehicle0 = leader[int(waiting_time[1][0])]
        vehicle1 = leader[int(waiting_time[1][1])]
        vehicle2 = leader[int(waiting_time[1][2])]
        vehicle3 = leader[int(waiting_time[1][3])]
        if route_compatibility(vehicle0, vehicle1):
            action[int(waiting_time[1][1])] = 1
            if route_compatibility(vehicle0, vehicle2) and route_compatibility(vehicle1, vehicle2):
                action[int(waiting_time[1][2])] = 1
                if route_compatibility(vehicle0, vehicle3) and route_compatibility(vehicle1, vehicle3) \
                        and route_compatibility(vehicle2, vehicle3):
                    action[int(waiting_time[1][3])] = 1
            elif route_compatibility(vehicle0, vehicle3) and route_compatibility(vehicle1, vehicle3):
                action[int(waiting_time[1][3])] = 1
        elif route_compatibility(vehicle0, vehicle2):
            action[int(waiting_time[1][2])] = 1
            if route_compatibility(vehicle0, vehicle3) and route_compatibility(vehicle2, vehicle3):
                action[int(waiting_time[1][3])] = 1
        elif route_compatibility(vehicle0, vehicle3):
            action[int(waiting_time[1][3])] = 1

        set_action(action, leader)

    return leader, action


def route_compatibility(vehicle1, vehicle2):
    """Determine the compatibility of 2 vehicle route.

    this function determine if two vehicle can cross the intersections at the same time

    Parameters
    ----------
    vehicle1 : string
        name of the first vehicle
    vehicle2 : string
        name of the second vehicle

    Returns
    -------
    bTrue : bool
         boolean indicating if the two vehicle route are compatible

    """
    b_true = False
    b_vehicle = False
    if vehicle1 != '' and vehicle2 != '':
        b_vehicle = True
    if b_vehicle and traci.vehicle.getRoute(vehicle1)[0] == "1to2":
        if traci.vehicle.getRoute(vehicle1)[1] == "2to5":
            if traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True
            # checked
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to3":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True
            # checked
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                b_true = True
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to3":
            if traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                b_true = False
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to4":
            if traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                b_true = False
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to3":
                    b_true = True

    if b_vehicle and traci.vehicle.getRoute(vehicle1)[0] == "3to2":
        if traci.vehicle.getRoute(vehicle1)[1] == "2to5":
            if traci.vehicle.getRoute(vehicle2)[0] == "1to2":
                b_true = False
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                b_true = False
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to3":
                    b_true = True
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to1":
            if traci.vehicle.getRoute(vehicle2)[0] == "1to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to3":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to5":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                b_true = False
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to3":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = False
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to4":
            if traci.vehicle.getRoute(vehicle2)[0] == "1to2":
                if traci.vehicle.getRoute(vehicle2)[1] != "2to4":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to3":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True

    if b_vehicle and traci.vehicle.getRoute(vehicle1)[0] == "4to2":
        if traci.vehicle.getRoute(vehicle1)[1] == "2to5":
            if traci.vehicle.getRoute(vehicle2)[0] == "1to2":
                b_true = False
            elif traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to3":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = True
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to1":
            if traci.vehicle.getRoute(vehicle2)[0] == "1to2":
                b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                if traci.vehicle.getRoute(vehicle2)[1] != "2to1":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "5to2":
                if traci.vehicle.getRoute(vehicle2)[1] != "2to1":
                    b_true = True
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to3":
            if traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = True

    if b_vehicle and traci.vehicle.getRoute(vehicle1)[0] == "5to2":
        if traci.vehicle.getRoute(vehicle1)[1] == "2to4":
            if traci.vehicle.getRoute(vehicle2)[0] == "1to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = False
                if traci.vehicle.getRoute(vehicle2)[1] == "2to5":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                b_true = False
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to5":
                    b_true = True
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to1":
            if traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = True
        elif traci.vehicle.getRoute(vehicle1)[1] == "2to3":
            if traci.vehicle.getRoute(vehicle2)[0] == "3to2":
                b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "4to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to5":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to1":
                    b_true = True
            elif traci.vehicle.getRoute(vehicle2)[0] == "1to2":
                if traci.vehicle.getRoute(vehicle2)[1] == "2to5":
                    b_true = True
                if traci.vehicle.getRoute(vehicle2)[1] == "2to4":
                    b_true = True
    return b_true


def step_few(simulation_time):
    """"Control the step during the simulation.

    this function is used to control the step of the simulation action after action.

    Parameters
    ----------
    simulation_time : int
        int which indicate the wished simulation time in second

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
    evacuated_vehicle : int
        indicating the number of evacuated vehicle during the step.
    """
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    count = 0
    reward = 0
    done = 1

    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0
    evacuated_vehicle = 0
    collision = 0

    while traci.vehicle.getIDCount() <= 0:
        set_speed_mode()
        traci.simulationStep()

    leader, action = strategy_first_in_first_off()

    while not leader_arriver(leader, action):

        count += 1
        set_speed_mode()
        set_loaded_vehicle(leader)
        traci.simulationStep()
        collision += traci.simulation.getCollidingVehiclesNumber()
        set_speed_mode()
        reward += traci.simulation.getArrivedNumber()
        average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
        cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
        average_speed += statistique.vitesseMoyenne(flow_edges)
        emission_co2 += statistique.emissionDeCo2(flow_edges)
        evacuated_vehicle += traci.simulation.getArrivedNumber()
        if traci.simulation.getTime() > simulation_time:
            done = 0

    b_action0 = True
    for i in range(0, len(action)):
        if action[i] == 1:
            b_action0 = False

    if b_action0:
        count += 1
        set_speed_mode()
        set_loaded_vehicle(leader)
        traci.simulationStep()
        collision += traci.simulation.getCollidingVehiclesNumber()
        set_speed_mode()
        reward += traci.simulation.getArrivedNumber()
        average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
        cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
        average_speed += statistique.vitesseMoyenne(flow_edges)
        emission_co2 += statistique.emissionDeCo2(flow_edges)
        evacuated_vehicle += traci.simulation.getArrivedNumber()

    lane_empty = False
    for i in range(0, len(leader)):
        if leader[i] == "":
            lane_empty = True
    if lane_empty:
        count += 1
        set_speed_mode()
        set_loaded_vehicle(leader)
        traci.simulationStep()
        collision += traci.simulation.getCollidingVehiclesNumber()
        set_speed_mode()
        reward += traci.simulation.getArrivedNumber()
        average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
        cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
        average_speed += statistique.vitesseMoyenne(flow_edges)
        emission_co2 += statistique.emissionDeCo2(flow_edges)
        evacuated_vehicle += traci.simulation.getArrivedNumber()

    if count > 0:
        average_waiting_time = average_waiting_time/count
        cumulated_waiting_time = cumulated_waiting_time/count
        average_speed = average_speed/count
        emission_co2 = emission_co2//count

    return collision, average_waiting_time, cumulated_waiting_time, average_speed, emission_co2, done, evacuated_vehicle


def launch_cross_first_in_first_off(display, simulation_time, reward_type, flow1, flow2, flow3, flow4):
    """Launch the simulation and make it work with the desired strategy.

    this function is used to run the function stepFew and store the statistic of the function.

    Parameters
    ----------
    display : bool
        indicate if a graphical interface should be used.
    simulation_time : int
         indicate the wished simulation time in second
    reward_type : string
            string defining which reward would be used.
    flow1 : int
        define the number of vehicle per hour on a specific edge.
    flow2 : int
        define the number of vehicle per hour on a specific edge.
    flow3 : int
        define the number of vehicle per hour on a specific edge.
    flow4 : int
        define the number of vehicle per hour on a specific edge.
    Returns
    -------
    statistic about the simulation.

    """
    simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "Net")
    start_simulation(display)
    collision_nb = 0
    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0
    evacuated_vehicle = 0
    n = 0
    done = 1
    while done:
        n += 1
        collision, average_wait, cumulated_wait, speed_average, co2_emi, done, evacuated_vehicle_temp = \
            step_few(simulation_time)

        collision_nb += collision
        average_waiting_time += average_wait
        cumulated_waiting_time += cumulated_wait
        average_speed += speed_average
        emission_co2 += co2_emi
        evacuated_vehicle += evacuated_vehicle_temp
    print(collision_nb)
    print("average waiting time :", average_waiting_time/n, "\n")
    print("average cumulated time : ", cumulated_waiting_time/n, "\n")
    print("average speed : ", average_speed/n, "\n")
    print("average emission of co2 : ", emission_co2/n, "\n")
    print("evacuated vehicle : ", evacuated_vehicle, "\n")
    traci.close()

    return collision_nb/n, average_waiting_time/n, cumulated_waiting_time/n, average_speed/n, emission_co2/n,\
        evacuated_vehicle


def launch_simulation_statistique(display, nb_episode, time_per_episode, reward_type, flow1, flow2, flow3, flow4):
    """Launch the whished number of simulation to generate statistic.

    this function is used launch the whished number of simulation to generate statistic

    Parameters
    ----------
    display : bool
        boolean which indicate if the graphical interface should be used.
    nb_episode : int
        defining the number of simulation that would be run.
    time_per_episode : int
        define the simulation time of each simulation.
    reward_type : string
        string defining which reward would be used.
    flow1 : int
        define the number of vehicle per hour on a specific edge.
    flow2 : int
        define the number of vehicle per hour on a specific edge.
    flow3 : int
        define the number of vehicle per hour on a specific edge.
    flow4 : int
        define the number of vehicle per hour on a specific edge.

    Returns
    -------
    list : list
        containing all the statistic we want.
    """

    collision = 0
    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0
    evacuated_vehicle = 0

    liste_waiting_time = []
    liste_cumulated_waiting_time = []
    liste_average_speed = []
    liste_emission_co2 = []
    liste_evacuated_vehicle = []

    for i in range(nb_episode):

        collision_temp, average_waiting_time_temp, cumulated_waiting_time_temp, average_speed_temp, emission_co2_temp, \
            evacuated_vehicle_temp = launch_cross_first_in_first_off(display, time_per_episode, reward_type, flow1,
                                                                     flow2, flow3, flow4)

        collision += collision_temp
        average_waiting_time += average_waiting_time_temp
        cumulated_waiting_time += cumulated_waiting_time_temp
        average_speed += average_speed_temp
        emission_co2 += emission_co2_temp
        evacuated_vehicle += evacuated_vehicle_temp

        liste_waiting_time.append(average_waiting_time_temp)
        liste_cumulated_waiting_time.append(cumulated_waiting_time_temp)
        liste_average_speed.append(average_speed_temp)
        liste_emission_co2.append(emission_co2_temp)
        liste_evacuated_vehicle.append(evacuated_vehicle_temp)

    print("Voici la moyenne des statisques pour ", nb_episode, "episode :")
    print("nombre de collision : ", collision/nb_episode)
    print("temp d'attente moyen : ", average_waiting_time/nb_episode)
    print("temps d'attente cumulé :", cumulated_waiting_time/nb_episode)
    print("vitesse moyenne :", average_speed/nb_episode)
    print("emission de co2 moyenne :", emission_co2/nb_episode)
    print("evacuated vehicle average : ", evacuated_vehicle/nb_episode)

    std_waiting_time = np.std(liste_waiting_time)
    std_cumulated_waiting_time = np.std(liste_cumulated_waiting_time)
    std_average_speed = np.std(liste_average_speed)
    std_emission_co2 = np.std(liste_emission_co2)
    std_evacuated_vehicle = np.std(liste_evacuated_vehicle)

    return [nb_episode, collision/nb_episode, average_waiting_time/nb_episode, std_waiting_time,
            cumulated_waiting_time/nb_episode, std_cumulated_waiting_time, average_speed/nb_episode, std_average_speed,
            emission_co2/nb_episode, std_emission_co2, evacuated_vehicle/nb_episode, std_evacuated_vehicle, flow1]



