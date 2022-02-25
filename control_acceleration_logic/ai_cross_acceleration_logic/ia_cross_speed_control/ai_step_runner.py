from __future__ import absolute_import
from __future__ import print_function

import numpy as np

import \
    control_acceleration_logic.ai_cross_acceleration_logic.ia_cross_speed_control.safety_simulation_function as safety_function
import project

import sumo_utils # noqa

# constante :
delta_s = 2
distance_min = 2
maximum_acceleration = 4
comfortable_decelaration = 8
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
                 "--tripinfo-output", project.resources_dir + "tripinfo.xml", "--random",
                 "--collision.check-junctions"])


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
                 "--tripinfo-output", project.resources_dir + "tripinfo.xml", "--random",
                 "--collision.check-junctions"])
    # runningVehicleID = traci.vehicle.getIDList()
    # countRunningVehicle = traci.vehicle.getIDCount()
    # return imageConstruct(runningVehicleID, runningVehicleID, 40)
    # n = 40
    # return np.zeros((n, n, 3), dtype=int)
    return state_observation(["", "", "", "", "", "", "", ""])


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


def set_loaded_vehicle(leader_tab):
    """set a stop for vehicle who are not leaders.

    This function is used to set a default stop to the vehicle who are not leaders in the simulation.

    Parameters
    ----------
    leader_tab : array
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


def get4leader():
    """Determine the closest vehicle to the intersection of each edge.

    This function is used to determine the 4 closest vehicle to the intersections

    Parameters
    ----------

    Returns
    -------
    leaderTab : list
        return a list with the name of the 4 vehicle
    """
    leader_tab = ["", "", "", ""]
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    for i in range(0, len(flow_edges)):
        if traci.edge.getLastStepVehicleNumber(flow_edges[i]) >= 1:
            leader_tab[i] = traci.edge.getLastStepVehicleIDs(
                flow_edges[i])[traci.edge.getLastStepVehicleNumber(flow_edges[i]) - 1]
            traci.vehicle.setColor(leader_tab[i], (255, 255, 255))
    return leader_tab


def get_road_index(vehicle, entry_array):
    """This fonctions is used to dertermine the index of the road in the entry array.

    the road of a vehicle is composed of two edge the entry array contains all the possible road with a specific index
    for each one. This index is then used to determine the compatibility of two road.

    Parameters
    ----------
    vehicle : string
        string containing the name of the vehicle.

    entry_array : ndarray
        list containing all the possible road.

    returns
    -------
    the index of the road of the vehicle in the entry array.


    """
    return entry_array.flatten().tolist().index(traci.vehicle.getRoute(vehicle)[0] + traci.vehicle.getRoute(vehicle)[1])


def state_observation(leader):
    """This fonctions is used to create a vector describing the environment state.

    the road of a vehicle is composed of two edge the entry array contains all the possible road with a specific index
    for each one. This index is then used to determine the compatibility of two road.

    Parameters
    ----------
    leader : array
        array containing the name of the 8 leaders.

    returns
    -------
    observation : array
        an array describing the state of the environment.
    """

    entry_array = np.load(project.resources_dir + 'variable_CAC/entry_array.npy')
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]

    vide = np.zeros((2,))
    vide[0] = -1000
    vide[1] = -1000

    observation = np.zeros((36,))

    if leader[0] != "":
        pos_vehicle1_edge1 = \
            np.array([traci.vehicle.getPosition(leader[0])[0], traci.vehicle.getPosition(leader[0])[1]])

        speed_vehicle1_edge1 = traci.vehicle.getSpeed(leader[0])
        road_vehicle1_edge1 = get_road_index(leader[0], entry_array)
    else:
        pos_vehicle1_edge1 = vide
        speed_vehicle1_edge1 = -1000
        road_vehicle1_edge1 = -1

    if leader[1] != "":
        pos_vehicle2_edge1 = \
            np.array([traci.vehicle.getPosition(leader[1])[0], traci.vehicle.getPosition(leader[1])[1]])

        speed_vehicle2_edge1 = traci.vehicle.getSpeed(leader[1])
        road_vehicle2_edge1 = get_road_index(leader[1], entry_array)
    else:
        pos_vehicle2_edge1 = vide
        speed_vehicle2_edge1 = -1000
        road_vehicle2_edge1 = -1

    if leader[2] != "":
        pos_vehicle1_edge2 = \
            np.array([traci.vehicle.getPosition(leader[2])[0], traci.vehicle.getPosition(leader[2])[1]])

        speed_vehicle1_edge2 = traci.vehicle.getSpeed(leader[2])
        road_vehicle1_edge2 = get_road_index(leader[2], entry_array)
    else:
        pos_vehicle1_edge2 = vide
        speed_vehicle1_edge2 = -1000
        road_vehicle1_edge2 = -1

    if leader[3] != "":
        pos_vehicle2_edge2 = \
            np.array([traci.vehicle.getPosition(leader[3])[0], traci.vehicle.getPosition(leader[3])[1]])

        speed_vehicle2_edge2 = traci.vehicle.getSpeed(leader[3])
        road_vehicle2_edge2 = get_road_index(leader[3], entry_array)
    else:
        pos_vehicle2_edge2 = vide
        speed_vehicle2_edge2 = -1000
        road_vehicle2_edge2 = -1

    if leader[4] != "":
        pos_vehicle1_edge3 = \
            np.array([traci.vehicle.getPosition(leader[4])[0], traci.vehicle.getPosition(leader[4])[1]])

        speed_vehicle1_edge3 = traci.vehicle.getSpeed(leader[4])
        road_vehicle1_edge3 = get_road_index(leader[4], entry_array)
    else:
        pos_vehicle1_edge3 = vide
        speed_vehicle1_edge3 = -1000
        road_vehicle1_edge3 = -1

    if leader[5] != "":
        pos_vehicle2_edge3 = \
            np.array([traci.vehicle.getPosition(leader[5])[0], traci.vehicle.getPosition(leader[5])[1]])

        speed_vehicle2_edge3 = traci.vehicle.getSpeed(leader[5])
        road_vehicle2_edge3 = get_road_index(leader[5], entry_array)
    else:
        pos_vehicle2_edge3 = vide
        speed_vehicle2_edge3 = -1000
        road_vehicle2_edge3 = -1

    if leader[6] != "":
        pos_vehicle1_edge4 = \
            np.array([traci.vehicle.getPosition(leader[6])[0], traci.vehicle.getPosition(leader[6])[1]])

        speed_vehicle1_edge4 = traci.vehicle.getSpeed(leader[6])
        road_vehicle1_edge4 = get_road_index(leader[6], entry_array)
    else:
        pos_vehicle1_edge4 = vide
        speed_vehicle1_edge4 = -1000
        road_vehicle1_edge4 = -1

    if leader[7] != "":
        pos_vehicle2_edge4 = \
            np.array([traci.vehicle.getPosition(leader[7])[0], traci.vehicle.getPosition(leader[7])[1]])

        speed_vehicle2_edge4 = traci.vehicle.getSpeed(leader[7])
        road_vehicle2_edge4 = get_road_index(leader[7], entry_array)
    else:
        pos_vehicle2_edge4 = vide
        speed_vehicle2_edge4 = -1000
        road_vehicle2_edge4 = -1

    observation[0] = pos_vehicle1_edge1[0]
    observation[1] = pos_vehicle1_edge1[1]

    observation[2] = pos_vehicle2_edge1[0]
    observation[3] = pos_vehicle2_edge1[1]

    observation[4] = pos_vehicle1_edge2[0]
    observation[5] = pos_vehicle1_edge2[1]

    observation[6] = pos_vehicle2_edge2[0]
    observation[7] = pos_vehicle2_edge2[1]

    observation[8] = pos_vehicle1_edge3[0]
    observation[9] = pos_vehicle1_edge3[1]

    observation[10] = pos_vehicle2_edge3[0]
    observation[11] = pos_vehicle2_edge3[1]

    observation[12] = pos_vehicle1_edge4[0]
    observation[13] = pos_vehicle1_edge4[1]

    observation[14] = pos_vehicle2_edge4[0]
    observation[15] = pos_vehicle2_edge4[1]

    observation[16] = speed_vehicle1_edge1
    observation[17] = speed_vehicle2_edge1
    observation[18] = speed_vehicle1_edge2
    observation[19] = speed_vehicle2_edge2
    observation[20] = speed_vehicle1_edge3
    observation[21] = speed_vehicle2_edge3
    observation[22] = speed_vehicle1_edge4
    observation[23] = speed_vehicle2_edge4

    observation[24] = traci.edge.getLastStepVehicleNumber(flow_edges[0])
    observation[25] = traci.edge.getLastStepVehicleNumber(flow_edges[1])
    observation[26] = traci.edge.getLastStepVehicleNumber(flow_edges[2])
    observation[27] = traci.edge.getLastStepVehicleNumber(flow_edges[3])

    observation[28] = road_vehicle1_edge1
    observation[29] = road_vehicle2_edge1
    observation[30] = road_vehicle1_edge2
    observation[31] = road_vehicle2_edge2
    observation[32] = road_vehicle1_edge3
    observation[33] = road_vehicle2_edge3
    observation[34] = road_vehicle1_edge4
    observation[35] = road_vehicle2_edge4

    return observation


def set_action(leader, action):
    """this function set the action of the leaders vehicle.

    sets the action of the vehicle.

    Parameters
    ----------
    leader : array
        containing the leaders.
    action : array
        containing the actions for each leader.
    returns
    -------

    """

    # print(action)
    for i in range(0, len(leader)):
        if leader[i] != "":
            if traci.vehicle.getNextStops(leader[i]) != ():
                traci.vehicle.setStop(leader[i], traci.vehicle.getRoadID(leader[i]), 90, duration=0)

            traci.vehicle.slowDown(leader[i], action[i], step_duration)


def detect_danger_reward(leader_tab):
    """used to return the reward of the dangerous situation.

        this function return a reward related to the dangerous situation.
    Parameters
    ----------
    leader_tab : array
        containing the leaders.

    returns
    -------
    reward : int
        containing the reward of the current step.
    """

    reward = 0
    for i in range(0, len(leader_tab)):
        for j in range(0, len(leader_tab)):
            reward += safety_function.security_detect_danger_ia_speed_control(leader_tab[i], leader_tab[j])

    return reward


def switch_leader_in_junctions_v2():
    """Used to determine the vehicles leader of the intersection.

    At each step this fonction update the list of the vehicle leader.

    Parameters
    ----------

    returns
    -------
    leader : list
        containing the leaders.

    """
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    leader = ["", "", "", "", "", "", "", ""]
    # print("leader",leader)
    # print("leader_next", leader_next)

    # all_vehicle_list = traci.vehicle.getIDList()
    #
    # for i, edge in enumerate(flow_edges):
    #     for vehicle in all_vehicle_list:
    #         if traci.vehicle.getRoute(vehicle)[0] == edge:
    #             if traci.vehicle.getRouteIndex(vehicle) < 1:
    #                 leader[i + i] = vehicle
    #                 # print(vehicle)
    #                 # print(traci.vehicle.getLeader(leader[i + i]))
    #                 if traci.vehicle.getLeader(leader[i+i]) != None and\
    #                         traci.vehicle.getRouteIndex(traci.vehicle.getLeader(leader[i+i])[0]) < 1 and\
    #                         traci.vehicle.getRoute(leader[i+i])[0] ==\
    #                         traci.vehicle.getRoute(traci.vehicle.getLeader(leader[i+i])[0])[0]:
    #
    #                     # print(traci.vehicle.getLeader(leader[i+i]))
    #                     while traci.vehicle.getLeader(leader[i+i]) != None and \
    #                             traci.vehicle.getRouteIndex(traci.vehicle.getLeader(leader[i+i])[0]) < 1 and\
    #                             traci.vehicle.getRoute(leader[i+i])[0] ==\
    #                             traci.vehicle.getRoute(traci.vehicle.getLeader(leader[i+i])[0])[0]:
    #
    #                         leader[i+i] = traci.vehicle.getLeader(leader[i+i])[0]
    #                         # print(leader[i+i])
    #                 break
    #
    # for count in range(0, 4):
    #     if leader[count+count] != "":
    #         leader[count+count+1] = traci.vehicle.getFollower(leader[count+count])[0]

    # leader_4 = get4leader()
    # for i, vehicle in enumerate(leader_4):
    #     if vehicle != "":
    #         print(traci.vehicle.getLeader(vehicle))
    #         while traci.vehicle.getLeader(vehicle) is not None \
    #                 and traci.vehicle.getRouteIndex(traci.vehicle.getLeader(vehicle)[0]) != 1 and\
    #                 traci.vehicle.getRoute(vehicle)[0] == traci.vehicle.getRoute(traci.vehicle.getLeader(vehicle)[0])[0]:
    #
    #             leader_4[i] = traci.vehicle.getLeader(vehicle)
    #
    # for i, vehic in enumerate(leader_4):
    #     if vehic != "":
    #         leader[i+i] = vehic
    #         traci.vehicle.setColor(vehic, (0, 255, 0))
    #         leader[i+i+1] = traci.vehicle.getFollower(vehic)[0]
    #         if leader[i+i+1] != "":
    #             traci.vehicle.setColor(leader[i+i+1], (0, 255, 0))

    all_vehicle_list = traci.vehicle.getIDList()
    vehicle_flow0 = []
    vehicle_flow1 = []
    vehicle_flow2 = []
    vehicle_flow3 = []

    for vehicle in all_vehicle_list:
        if traci.vehicle.getRouteIndex(vehicle) < 1 and traci.vehicle.getRoute(vehicle)[0] == flow_edges[0]:
            vehicle_flow0.append(vehicle)
            # traci.vehicle.setColor(vehicle, (0, 0, 255))
        elif traci.vehicle.getRouteIndex(vehicle) < 1 and traci.vehicle.getRoute(vehicle)[0] == flow_edges[1]:
            vehicle_flow1.append(vehicle)
            # traci.vehicle.setColor(vehicle, (0, 255, 255))
        elif traci.vehicle.getRouteIndex(vehicle) < 1 and traci.vehicle.getRoute(vehicle)[0] == flow_edges[2]:
            vehicle_flow2.append(vehicle)
            # traci.vehicle.setColor(vehicle, (255, 0, 255))
        elif traci.vehicle.getRouteIndex(vehicle) < 1 and traci.vehicle.getRoute(vehicle)[0] == flow_edges[3]:
            vehicle_flow3.append(vehicle)
            # traci.vehicle.setColor(vehicle, (50, 100, 255))
        else:
            traci.vehicle.setColor(vehicle, (255, 255, 255))
            # print(traci.vehicle.getRouteIndex(vehicle))

    vehicle_flow0.sort(key=len)
    vehicle_flow1.sort(key=len)
    vehicle_flow2.sort(key=len)
    vehicle_flow3.sort(key=len)

    leader[0] = vehicle_flow0[0] if len(vehicle_flow0) >= 1 else ""
    leader[1] = vehicle_flow0[1] if len(vehicle_flow0) >= 2 else ""
    leader[2] = vehicle_flow1[0] if len(vehicle_flow1) >= 1 else ""
    leader[3] = vehicle_flow1[1] if len(vehicle_flow1) >= 2 else ""
    leader[4] = vehicle_flow2[0] if len(vehicle_flow2) >= 1 else ""
    leader[5] = vehicle_flow2[1] if len(vehicle_flow2) >= 2 else ""
    leader[6] = vehicle_flow3[0] if len(vehicle_flow3) >= 1 else ""
    leader[7] = vehicle_flow3[1] if len(vehicle_flow3) >= 2 else ""

    for vehic in leader:
        traci.vehicle.setColor(vehic, (0, 255, 0)) if vehic != "" else ""

    # print(vehicle_flow0)

    # print(leader)
    # print("tous", all_vehicle_list)
    # print("edge", traci.edge.getLastStepVehicleIDs("1to2"))

    return leader


def step_few(simulation_time, action, image_size):
    """This function run a single time step each time it is called.

        this function run a time step of a certain duration each time it is called.

    Parameters
    ----------
    simulation_time : int
        defining the wished duration of a simulation.
    action : list
        containing the action wished for the time step (acceleration wished for each leader).
    image_size : ndarray
        unused but could be used if we want to use an image as observation.

    returns
    -------
    state_next : array
        vector observation of the environment after the time step.
    reward : int
        reward of the time step.
    done : bool
        indicating if we have reached the wished simulation time.

    """
    # printable = [round(action[0], 2), round(action[1], 2), round(action[2], 2), round(action[3], 2), round(action[4], 2), round(action[5], 2), round(action[6], 2), round(action[7], 2)]
    # print("action :", printable)
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    done = False
    set_speed_mode()
    leader = switch_leader_in_junctions_v2()
    set_loaded_vehicle(leader)
    action = acceleration_control(action, leader)
    # print(action)
    set_action(leader, action)
    # safety_function.security_idm_model(leader, step_duration)
    # safety_function.security_idm_based_new_strat2(leader, step_duration)
    safety_function.security_idm_euclidean_based_strategy2(leader, step_duration, action)
    traci.simulationStep()

    # if statistique.vitesseMoyenne(flow_edges) < 5:
    #     speed_penalty = -40
    # else:
    #     speed_penalty = 2

    reward = -(traci.simulation.getCollidingVehiclesNumber() * 1000) + statistique.vitesseMoyenne(flow_edges) + \
             (
                         traci.simulation.getArrivedNumber() * 10) + traci.simulation.getDepartedNumber() * 100  # + (detect_danger_reward(leader)/10)

    if traci.simulation.getTime() > simulation_time:
        done = True
    if traci.simulation.getCollidingVehiclesNumber() > 0:
        done = False
        # print(traci.simulation.getCollidingVehiclesIDList())

    state_next = state_observation(leader)

    return state_next, reward, done


def acceleration_control(action, leader):
    """this function is used to apply the correct acceleration.

        this function is used to determine what is the correct acceleration to apply depending on the duration of a
        time step. This function also verify that the speed given is coherent.

    Parameters
    ----------
    action : list
        list of the given action (acceleration) for each leader.
    leader : list
        list containing the leaders.

    returns
    -------
    action : list
        list containing the new speed of the vehicle with the correct acceleration applied

    """
    for i, vehicle in enumerate(leader):
        if vehicle != "":
            # print(traci.vehicle.getSpeed(vehicle))
            action[i] = traci.vehicle.getSpeed(vehicle) + (action[i] * step_duration)
            action[i] = np.clip(action[i], 0, 17)
    return action
