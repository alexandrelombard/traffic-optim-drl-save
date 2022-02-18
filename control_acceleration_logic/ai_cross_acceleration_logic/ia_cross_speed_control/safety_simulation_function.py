"""
    This file contains function related to the security of the simulation.
    This file also contains the all the needed function to generate some of our needed variable like
    the security_matrice.
"""

import project

import control_acceleration_logic.ai_cross_acceleration_logic.ia_cross_speed_control.ai_step_runner as fct
import traci
import numpy as np

# variable or constante that we need in other module to run safely the simulation.
security_matrice = np.load(project.resources_dir + 'variable_CAC/security_matrice.npy')
entry_array = np.load(project.resources_dir + 'variable_CAC/entry_array.npy')
security_distance_matrice = np.load(project.resources_dir + 'variable_CAC/security_distance_matrice.npy')


def security_set_speed_mode():
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


def security_leader_arriver(leaders_tab, action):
    """Indicate if the vehicle have leaved the intersection.

    this function is used to determine when the vehicle(s) authorized to pass the intersection
    have leaved the intersection.

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


def security_set_leaders_actions(leader_tab, action):
    """Change the actions of the leaders vehicles.

    this function is used to call the a function that change the actions of the 4 leaders and theirs followers.
    So this function is used to change the actions of the 8 closest vehicle.

    Parameters
    ----------
    leader_tab : list
        list containing the name of the 4 leaders.

    action : list
        list containing the wished action for the vehicles
        0 for stop 1 for go

    Returns
    -------

    """
    for i in range(len(leader_tab)):
        if leader_tab[i] != "":
            security_set_action(i, leader_tab[i], action)


def security_set_action(i, veh_id, action):
    """Change the actions of the vehicle.

    this function is used to call the a function that change the actions of the 4 leaders and theirs followers.
    So this function is used to change the actions of the 8 closest vehicle.

    Parameters
    ----------
    i : int
        indicating the index of the action
    veh_id : string
        string containing the name of the vehicle.

    action : list
        list containing the wished action for the vehicles
        0 for stop 1 for go

    Returns
    -------
    negative_reward : int
        unused but contains a negative reward related to the applicability of an action.
    """
    negative_reward = 0
    if action[i] == 0:
        try:

            traci.vehicle.setStop(veh_id, traci.vehicle.getRoadID(veh_id), 90)
            traci.vehicle.setColor(veh_id, (255, 0, 0))
        except traci.TraCIException:
            traci.vehicle.setColor(veh_id, (255, 255, 255))
            print("to late to brake")
            negative_reward = -6

    elif action[i] == 1:
        try:
            traci.vehicle.setColor(veh_id, (0, 255, 0))
            if traci.vehicle.getNextStops(veh_id) != ():
                traci.vehicle.setStop(veh_id, traci.vehicle.getRoadID(veh_id), 90, duration=0)
        except traci.TraCIException:
            print("the vehicle has left the edge ")
    return negative_reward


def step_few_security_matrice(action):
    """this function realise time step until the action have an effect.

        this function is used to apply an action on the environment and run until the actions is realised.
        In this case the function run until the authorized to pass vehicle leave the intersection.
        More specifically this function is used to construct the security matrice.
        Matrice that we need to run safely some simulation.

    Parameters
    ----------
    action : ndarray
        containing the wished action.

    Returns
    -------
    leader : list
        containing the leader
    reward : int
        containing the reward of the step(s)
    done : bool
        indicating if the simulation is over.
    """

    done = False
    arrived = False
    reward = 0

    while fct.get4leader() == ["", "", "", ""]:
        security_set_speed_mode()
        traci.simulationStep()

    security_set_speed_mode()
    leader = fct.get4leader()
    fct.set_loaded_vehicle(leader)
    security_set_leaders_actions(leader, action)

    while not security_leader_arriver(leader, action):
        traci.simulationStep()
        security_set_speed_mode()
        fct.set_loaded_vehicle(leader)
        arrived = True
        reward += -traci.simulation.getCollidingVehiclesNumber()

    if traci.simulation.getTime() > 1000:
        done = True
    if traci.simulation.getCollidingVehiclesNumber() > 0:
        done = False

    b_action0 = True
    for i in range(0, 4):
        if action[i] == 1:
            b_action0 = False

    if b_action0 and not arrived:
        security_set_speed_mode()
        fct.set_loaded_vehicle(leader)
        traci.simulationStep()

    lane_empty = False
    for i in range(0, len(leader)):
        if leader[i] == "":
            lane_empty = True

    if lane_empty and not arrived:
        security_set_speed_mode()
        fct.set_loaded_vehicle(leader)
        traci.simulationStep()

    return leader, reward, done


def security_matrice_construct():  # flow_edge, entry_edge1, entry_edge2, flow_edge_out, out_edge1, out_edge2):
    """this function is used to generate the security matrice.

        This function run simulation to generate the security matrice.
        The security matrice is used to determine the compatibility of two vehicle regarding their road.
        When the function as finish to run the security matrice is automatically saved in a npy file.
        You should not need to use this function as a perfectly fine security matrice is already saved in
        the variable_ files.

    Parameters
    ----------

    Returns
    -------

    """
    flow_edge = ["1to2", "5to2", "3to2", "4to2"]
    flow_edge_out = ["2to1", "2to5", "2to3", "2to4"]

    security_matrice = np.full((12, 12), True, dtype=bool)

    security_matrice_affichage = np.full((13, 13), "TrueTrue")

    tab_entry = [flow_edge[0]+flow_edge_out[1], flow_edge[0]+flow_edge_out[2], flow_edge[0]+flow_edge_out[3],
                 flow_edge[1]+flow_edge_out[0], flow_edge[1]+flow_edge_out[2], flow_edge[1]+flow_edge_out[3],
                 flow_edge[2]+flow_edge_out[0], flow_edge[2]+flow_edge_out[1], flow_edge[2]+flow_edge_out[3],
                 flow_edge[3]+flow_edge_out[0], flow_edge[3]+flow_edge_out[1], flow_edge[3]+flow_edge_out[2]]

    for i in range(0, 12):
        security_matrice_affichage[0][i + 1] = tab_entry[i]
        security_matrice_affichage[i + 1][0] = tab_entry[i]

    # index1 = tab_entry.index(entry_edge1+out_edge1)
    # index2 = tab_entry.index(entry_edge2+out_edge2)

    action = [[0, 0, 1, 1],
              [0, 1, 0, 1],
              [0, 1, 1, 0],
              [1, 0, 0, 1],
              [1, 0, 1, 0],
              [1, 1, 0, 0]]

    fct.start_simulation(False)
    for i in range(0, len(action)):

        # action = [0, 0, 0, 0]
        # action = [0, 0, 0, 1]
        # action = [0, 0, 1, 0]
        # action = [0, 0, 1, 1]
        # action = [0, 1, 0, 0]
        # action = [0, 1, 0, 1]
        # action = [0, 1, 1, 0]
        # action = [0, 1, 1, 1]
        # action = [1, 0, 0, 0]
        # action = [1, 0, 0, 1]
        # action = [1, 0, 1, 0]
        # action = [1, 0, 1, 1]
        # action = [1, 1, 0, 0]
        # action = [1, 1, 0, 1]
        # action = [1, 1, 1, 0]
        # action = [1, 1, 1, 1]
        actionnp = np.array(action[i])
        fct.reset(False)
        for steps in range(0, 800):

            leader, reward, done = step_few_security_matrice(actionnp)

            if reward < 0:
                index_leader = np.where(actionnp == 1)
                # print(index_leader[0])
                entry_edge1 = traci.vehicle.getRoute(leader[index_leader[0][0]])
                entry_edge2 = traci.vehicle.getRoute(leader[index_leader[0][1]])

                # code à tester et a changer
                road1 = entry_edge1[0]+entry_edge1[1]
                road2 = entry_edge2[0]+entry_edge2[1]

                index1 = tab_entry.index(road1)
                index2 = tab_entry.index(road2)

                security_matrice[index1][index2] = False
                security_matrice[index2][index1] = False
                security_matrice_affichage[index1 + 1][index2 + 1] = False
                security_matrice_affichage[index2 + 1][index1 + 1] = False

                # print(security_matrice)
                # print("\n\n")
                # form = "{0:13}{1:13}{2:13}{3:13}{4:13}{5:13}{6:13}{7:13}{8:13}{9:13}{10:13}{11:13}{12:13}"
                # for val in security_matrice_affichage:
                #     print(form.format(*val))
                #
                # print("\n\n")
            # if done :
            #     fct.reset(False)
    np.save('security_matrice.npy', security_matrice)
    np.save('security_matrice_affichage.npy', security_matrice_affichage)
    np.save('entry_array.npy', tab_entry)

    a = np.load('security_matrice.npy')

    print(a)

    traci.close()


def security_distance_matrice_construct():
    """this function is used to generate a security distance matrice.

        This function generate a security distance matrice that contains the minimal distance that vehicle
        should respect regarding their road to avoid collisions.
        Then the security distance matrice is saved in a npy file.

    Parameters
    ----------

    Returns
    -------

    """
    flow_edge = ["1to2", "5to2", "3to2", "4to2"]
    flow_edge_out = ["2to1", "2to5", "2to3", "2to4"]

    security_distance_matrice = np.full((12, 12), 0, dtype=float)

    security_distance_matrice_affichage = np.full((13, 13), "0.0000000")

    tab_entry = [flow_edge[0]+flow_edge_out[1], flow_edge[0]+flow_edge_out[2], flow_edge[0]+flow_edge_out[3],
                 flow_edge[1]+flow_edge_out[0], flow_edge[1]+flow_edge_out[2], flow_edge[1]+flow_edge_out[3],
                 flow_edge[2]+flow_edge_out[0], flow_edge[2]+flow_edge_out[1], flow_edge[2]+flow_edge_out[3],
                 flow_edge[3]+flow_edge_out[0], flow_edge[3]+flow_edge_out[1], flow_edge[3]+flow_edge_out[2]]

    for i in range(0, 12):
        security_distance_matrice_affichage[0][i + 1] = tab_entry[i]+"distance_min"
        security_distance_matrice_affichage[i + 1][0] = tab_entry[i]+"distance_min"

    action = [[0, 0, 1, 1],
              [0, 1, 0, 1],
              [0, 1, 1, 0],
              [1, 0, 0, 1],
              [1, 0, 1, 0],
              [1, 1, 0, 0]]

    fct.start_simulation(False)
    for i in range(0, len(action)):

        # action = [0, 0, 0, 0]
        # action = [0, 0, 0, 1]
        # action = [0, 0, 1, 0]
        # action = [0, 0, 1, 1]
        # action = [0, 1, 0, 0]
        # action = [0, 1, 0, 1]
        # action = [0, 1, 1, 0]
        # action = [0, 1, 1, 1]
        # action = [1, 0, 0, 0]
        # action = [1, 0, 0, 1]
        # action = [1, 0, 1, 0]
        # action = [1, 0, 1, 1]
        # action = [1, 1, 0, 0]
        # action = [1, 1, 0, 1]
        # action = [1, 1, 1, 0]
        # action = [1, 1, 1, 1]
        actionnp = np.array(action[i])
        fct.reset(False)
        leader, reward, done = step_few_security_matrice(actionnp)

        for steps in range(0, 800):
            index_leader = np.where(actionnp == 1)
            leader = fct.get4leader()

            if leader[index_leader[0][0]] != "" and leader[index_leader[0][1]] != "":
                if traci.vehicle.getLanePosition(leader[index_leader[0][0]]) >= \
                        traci.vehicle.getLanePosition(leader[index_leader[0][1]]):

                    vehicle_leader = leader[index_leader[0][0]]
                    vehicle_follower = leader[index_leader[0][1]]
                    distance = security_compute_follow_distance(vehicle_leader, vehicle_follower)
                else:
                    vehicle_leader = leader[index_leader[0][1]]
                    vehicle_follower = leader[index_leader[0][0]]
                    distance = security_compute_follow_distance(vehicle_leader, vehicle_follower)

            leader, reward, done = step_few_security_matrice(actionnp)

            if reward < 0:
                index_leader = np.where(actionnp == 1)
                # print(index_leader[0])
                entry_edge1 = traci.vehicle.getRoute(vehicle_leader)
                entry_edge2 = traci.vehicle.getRoute(vehicle_follower)

                # code à tester et a changer
                road1 = entry_edge1[0]+entry_edge1[1]
                road2 = entry_edge2[0]+entry_edge2[1]

                index1 = tab_entry.index(road1)
                index2 = tab_entry.index(road2)
                if distance > security_distance_matrice[index1][index2]:
                    security_distance_matrice[index1][index2] = distance
                    # security_distance_matrice[index2][index1] = distance
                    security_distance_matrice_affichage[index1 + 1][index2 + 1] = distance
                    # security_distance_matrice_affichage[index2 + 1][index1 + 1] = distance

                # print(security_matrice)
                # print("\n\n")
                # form = "{0:13}{1:13}{2:13}{3:13}{4:13}{5:13}{6:13}{7:13}{8:13}{9:13}{10:13}{11:13}{12:13}"
                # for val in security_matrice_affichage:
                #     print(form.format(*val))
                #
                # print("\n\n")
            # if done :
            #     fct.reset(False)
    np.save('security_distance_matrice.npy', security_distance_matrice)
    np.save('security_distance_matrice_affichage.npy', security_distance_matrice_affichage)
    # np.save('entry_array.npy', tab_entry)

    a = np.load('security_distance_matrice.npy')

    print(a)

    print("\n\n")
    form = "{0:13}{1:13}{2:13}{3:13}{4:13}{5:13}{6:13}{7:13}{8:13}{9:13}{10:13}{11:13}{12:13}"
    for val in security_distance_matrice_affichage:
        print(form.format(*val))

    print("\n\n")

    traci.close()


def security_compute_follow_distance(vehicle_leader, vehicle_follower):
    """This function is used to compute the distance between 2 vehicle.

        this function is used to compute the follow distance between 2 vehicle.
        follow distance : distance between the 2 vehicle if they where on the same lane.

    Parameters
    ----------
    vehicle_leader : string
        containing the name of the vehicle leader.
    vehicle_follower : string
        containing the name of the follower vehicle.

    Returns
    -------
    distance : float
        distance between the two vehicle in meters.

    """
    distance = traci.vehicle.getLanePosition(vehicle_leader) - traci.vehicle.getLanePosition(vehicle_follower)

    return distance


def security_compute_distance(vehicle1, vehicle2):
    """This function is used to compute the euclidean distance between 2 vehicle.

        this function is used to compute the euclidean distance between 2 vehicle.

    Parameters
    ----------
    vehicle1 : string
        containing the name of the vehicle leader.
    vehicle2 : string
        containing the name of the follower vehicle.

    Returns
    -------
    distance : float
        distance between the two vehicle in meters.

    """
    pos_vehicle1 = traci.vehicle.getPosition(vehicle1)
    pos_vehicle2 = traci.vehicle.getPosition(vehicle2)

    # print(pos_vehicle1)
    # print(pos_vehicle2)
    distance = np.sqrt(np.power(pos_vehicle1[0] - pos_vehicle2[0], 2) + np.power(pos_vehicle1[1] - pos_vehicle2[1], 2))
    # print(distance)

    return distance


def security_detect_danger_ia_speed_control(vehicle1, vehicle2):
    """This function is used to detect if a situation is dangerous between two vehicle.

        This function is used to detect if a situation is dangerous between two vehicle.

    Parameters
    ----------
    vehicle1 : string
        containing the name of the vehicle
    vehicle2 : string
        containing the name of the vehicle

    Returns
    -------
    reward : int
        containing the reward related to the detection of danger
    """
    reward = 0

    if vehicle1 != "" and vehicle2 != "":

        road_vehicle1 = traci.vehicle.getRoute(vehicle1)

        road_vehicle2 = traci.vehicle.getRoute(vehicle2)

        road_vehicle1 = road_vehicle1[0] + road_vehicle1[1]
        road_vehicle2 = road_vehicle2[0] + road_vehicle2[1]

        index1 = entry_array.flatten().tolist().index(road_vehicle1)
        index2 = entry_array.flatten().tolist().index(road_vehicle2)

        if not security_matrice[index1][index2]:
            if security_compute_distance(vehicle1, vehicle2) <= 5:
                reward += - 100
                traci.vehicle.setColor(vehicle1, (0, 0, 255))
                traci.vehicle.setColor(vehicle2, (0, 0, 255))
                print("danger")
    return reward


def security_determine_speed(leader, follower, distance_min, step_duration):
    """Determine the new acceleration and speed for a follower vehicle.

    this function is used to determine the acceleration and the new speed of a vehicle following an other vehicle
    in order to avoid collision.

    Parameters
    ----------
    leader : string
        string containing the name of the leader vehicle.

    follower : string
        string containing the name of the vehicle follower.

    distance_min : float
        containing the minimal distance to be respected between the 2 vehicle.

    step_duration : float
        the duration of one time step.

    Returns
    -------
    speed : float
        float indicating the new speed of the follower vehicle.
    """
    va_1 = traci.vehicle.getSpeed(leader)
    va = traci.vehicle.getSpeed(follower)

    v0 = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(follower))  # speed in free circulation
    s0 = distance_min   # minimum security distance
    t = 1  # delta_s  # minimum time to the front vehicle
    a = 2  # maximum_acceleration
    b = 4  # comfortable_decelaration

    s_etoile = s0 + va * t + (va * (va - va_1)) / (2 * np.sqrt(a * b))

    s_a = traci.vehicle.getDistance(leader) - traci.vehicle.getDistance(
        follower)  # TODO trouver une meilleur façon de calculer la distance

    acceleration = a * (1 - np.power(va / v0, 4) - np.power(s_etoile / s_a, 2))
    # acceleration = max(acceleration, -8)
    acceleration = np.clip(acceleration, -8, 2)
    speed = va + (acceleration * step_duration)
    # if speed < 0:
    #     speed = 0
    speed = np.clip(speed, 0, 17)

    return speed


def security_determine_speed_same_lane(leader, follower, distance_min, step_duration):
    """Determine the new acceleration and speed for a follower vehicle.

    this function is used to determine the acceleration and the new speed of a vehicle following an other
    vehicle on the same lane in order to avoid collision.

    Parameters
    ----------
    leader : string
        string containing the name of the leader vehicle.

    follower : string
        string containing the name of the vehicle follower.

    distance_min : float
        containing the minimal distance to be respected between the 2 vehicle.

    step_duration : float
        the duration of one time step.

    Returns
    -------
    speed : float
        float indicating the new speed of the follower vehicle.
    """
    va_1 = traci.vehicle.getSpeed(leader)
    va = traci.vehicle.getSpeed(follower)

    v0 = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(follower))  # speed in free circulation
    s0 = distance_min  # minimum security distance
    t = 1  # delta_s  # minimum time to the front vehicle
    a = 2  # maximum_acceleration
    b = 4  # comfortable_decelaration

    s_etoile = s0 + va * t + (va * (va - va_1)) / (2 * np.sqrt(a * b))

    s_a = traci.vehicle.getDistance(leader) - traci.vehicle.getDistance(
        follower)  # TODO trouver une meilleur façon de calculer la distance

    acceleration = a * (1 - np.power(va / v0, 4) - np.power(s_etoile / s_a, 2))
    # acceleration = max(acceleration, -8)
    # if acceleration > 2:
    #     acceleration = 2
    # if acceleration < -8:
    #     acceleration = -8

    acceleration = np.clip(acceleration, -8, 2)

    speed = va + (acceleration * step_duration)

    speed = np.clip(speed, 0, 17)

    return speed


def security_idm_model(leader, step_duration):

    """This function is a safe mode used to avoid the collision.

        This safe mode is based on the idm follow model to avoid collision.

    Parameters
    ----------
    leader : list
        containing the leaders.
    step_duration : float
        indicating the duration of a single time step.

    Returns
    -------

    """
    # print(leader)
    for vehicle1 in leader:
        if vehicle1 != "":
            road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
            for vehicle2 in leader:
                if vehicle2 != "" and vehicle1 != vehicle2 \
                        and traci.vehicle.getRoute(vehicle1)[0] != traci.vehicle.getRoute(vehicle2)[0]:

                    road_vehicle2 = fct.get_road_index(vehicle2, entry_array)
                    if traci.vehicle.getDistance(vehicle1) >= traci.vehicle.getDistance(vehicle2):
                        road_leader = road_vehicle1
                        road_follower = road_vehicle2
                        vehicle_leader = vehicle1
                        vehicle_follower = vehicle2
                        leader_drived_distance = traci.vehicle.getDistance(vehicle1)
                        distance_betwin_leader_follower = \
                            traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2)

                    else:
                        road_leader = road_vehicle2
                        road_follower = road_vehicle1
                        vehicle_leader = vehicle2
                        vehicle_follower = vehicle1
                        leader_drived_distance = traci.vehicle.getDistance(vehicle2)
                        distance_betwin_leader_follower = \
                            traci.vehicle.getDistance(vehicle2) - traci.vehicle.getDistance(vehicle1)

                    if not security_matrice[road_leader][road_follower]:
                        # print(security_distance_matrice)
                        if 120 > distance_betwin_leader_follower and leader_drived_distance > 00:

                            speed = security_determine_speed(vehicle_leader, vehicle_follower,
                                                             security_distance_matrice[road_leader][road_follower] + 14,
                                                             step_duration)

                            traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                            traci.vehicle.setColor(vehicle_follower, (255, 0, 0))
                    # print(traci.vehicle.getRoute(vehicle_leader)[0], traci.vehicle.getRoute(vehicle_follower)[0])
                    # if traci.vehicle.getRoute(vehicle_leader)[0] == traci.vehicle.getRoute(vehicle_follower)[0] and vehicle_leader != vehicle_follower:
                    #     # if 30 > distance_betwin_leader_follower:
                    #     # print(vehicle_leader, vehicle_follower)
                    #     speed = security_determine_speed(vehicle_leader, vehicle_follower, 15)
                    #     traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                    #     traci.vehicle.setColor(vehicle_follower, (255, 0, 255))

    for vehicle1 in leader:
        if vehicle1 != "":
            road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
            for vehicle2 in leader:
                if vehicle2 != "" and vehicle1 != vehicle2 \
                        and traci.vehicle.getRoute(vehicle1)[0] == traci.vehicle.getRoute(vehicle2)[0]:

                    road_vehicle2 = fct.get_road_index(vehicle2, entry_array)
                    if traci.vehicle.getDistance(vehicle1) >= traci.vehicle.getDistance(vehicle2):
                        road_leader = road_vehicle1
                        road_follower = road_vehicle2
                        vehicle_leader = vehicle1
                        vehicle_follower = vehicle2
                        leader_drived_distance = traci.vehicle.getDistance(vehicle1)
                        distance_betwin_leader_follower = \
                            traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2)
                    else:
                        road_leader = road_vehicle2
                        road_follower = road_vehicle1
                        vehicle_leader = vehicle2
                        vehicle_follower = vehicle1
                        leader_drived_distance = traci.vehicle.getDistance(vehicle2)
                        distance_betwin_leader_follower =\
                            traci.vehicle.getDistance(vehicle2) - traci.vehicle.getDistance(vehicle1)

                    if traci.vehicle.getRoute(vehicle_leader)[0] == traci.vehicle.getRoute(vehicle_follower)[0] \
                            and vehicle_leader != vehicle_follower:

                        # if 30 > distance_betwin_leader_follower:
                        # print(vehicle_leader, vehicle_follower)
                        speed = security_determine_speed_same_lane(vehicle_leader, vehicle_follower, 13, step_duration)
                        traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                        traci.vehicle.setColor(vehicle_follower, (255, 0, 255))


def security_idm_model2(leader, step_duration):
    """This function is a safe mode used to avoid the collision.

        This safe mode is based on the idm follow model to avoid collision.

    Parameters
    ----------
    leader : list
        containing the leaders.
    step_duration : float
        indicating the duration of a single time step.

    Returns
    -------

    """

    leader = security_sort_leader(leader)

    for i in range(0, len(leader)-1):
        vehicle1 = leader[i]
        vehicle2 = leader[i+1]
        if vehicle1 != "" and vehicle2 != "":
            road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
            road_vehicle2 = fct.get_road_index(vehicle2, entry_array)

            road_leader = road_vehicle1
            road_follower = road_vehicle2

            vehicle_leader = vehicle1
            vehicle_follower = vehicle2

            leader_drived_distance = traci.vehicle.getDistance(vehicle1)
            distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2)

            if not security_matrice[road_leader][road_follower]:
                if 120 > distance_betwin_leader_follower and leader_drived_distance > 0:
                    # print("safe_mode")
                    speed = security_determine_speed(vehicle_leader, vehicle_follower,
                                                     security_distance_matrice[road_leader][road_follower] + 12,
                                                     step_duration)

                    traci.vehicle.setColor(vehicle_follower, (255, 0, 0))
                    traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
            if traci.vehicle.getRoute(vehicle_leader)[0] == traci.vehicle.getRoute(vehicle_follower)[0] \
                    and vehicle_leader != vehicle_follower:
                # if 50 > distance_betwin_leader_follower :
                speed = security_determine_speed(vehicle_leader, vehicle_follower, 15, step_duration)
                traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                traci.vehicle.setColor(vehicle_follower, (255, 0, 0))


def security_sort_leader(leader):
    """This function is used to sort the leaders by their distance.

        This function sort the leader in a decreasing way.

    Parameters
    ----------
    leader : list
        containing the leaders

    Returns
    -------
    leader : list
        containing the sorted leaders

    """
    distance_tab = ["", "", "", "", "", "", "", ""]
    for iter, vehicle in enumerate(leader):
        if vehicle != "":
            distance_tab[iter] = traci.vehicle.getDistance(vehicle)
        else:
            distance_tab[iter] = -1000

    n = len(distance_tab)
    # Traverser tous les éléments du tableau
    for i in range(n):
        for j in range(0, n - i - 1):
            # échanger si l'élément trouvé est plus grand que le suivant
            if distance_tab[j] < distance_tab[j + 1]:
                distance_tab[j], distance_tab[j + 1] = distance_tab[j + 1], distance_tab[j]
                leader[j], leader[j + 1] = leader[j + 1], leader[j]

    return leader


def security_sort_leader_new_strat(leader):
    """This function sort the leaders according to a new strategy.

        The leader are separated in two category :
         - vehicle leader following a vehicle on an other lane
         - vehcile leader following a vehicle on the same lane

    Parameters
    ----------
    leader : list
        containing the leaders.

    Returns
    -------
    leader : list
        containing all the leaders sorted.
    leader_4 : list
        containing the first vehicle of each lane.
    """
    distance_tab = ["", "", "", ""]
    leader_4 = ["", "", "", ""]
    # we determine the leader of each edge
    for iter, vehicle in enumerate(leader):
        if iter % 2 == 0:
            # print(iter/2)
            leader_4[int(iter/2)] = vehicle

    # we get the distance drived by each leader of the edge
    for iter, vehicle in enumerate(leader_4):
        if vehicle != "":
            distance_tab[iter] = traci.vehicle.getDistance(vehicle)
        else:
            distance_tab[iter] = -1000

    # we sort the leader from the closest of the exit to the most far
    n = len(distance_tab)
    # Traverser tous les éléments du tableau
    for i in range(n):
        for j in range(0, n - i - 1):
            # échanger si l'élément trouvé est plus grand que le suivant
            if distance_tab[j] < distance_tab[j + 1]:
                distance_tab[j], distance_tab[j + 1] = distance_tab[j + 1], distance_tab[j]
                leader_4[j], leader_4[j + 1] = leader_4[j + 1], leader_4[j]

    # we get the follower of each leader of an edge
    for count, vehic in enumerate(leader_4):
        leader[count+count] = vehic
        if vehic != "":
            leader[count+count+1] = traci.vehicle.getFollower(vehic)[0]
        else:
            leader[count+count+1] = ""
    # print(leader_4)

    return leader, leader_4


def security_idm_based_new_strat(leader, step_duration):
    """This function is a safe mode used to avoid the collision.

        This safe mode is based on the idm follow model to avoid collision.

    Parameters
    ----------
    leader : list
        containing the leaders.
    step_duration : float
        indicating the duration of a single time step.

    Returns
    -------

    """

    leader, leader_4 = security_sort_leader_new_strat(leader)

    for i in range(0, len(leader_4)-1):
        vehicle1 = leader_4[i]
        vehicle2 = leader_4[i+1]
        if vehicle1 != "" and vehicle2 != "":
            road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
            road_vehicle2 = fct.get_road_index(vehicle2, entry_array)

            road_leader = road_vehicle1
            road_follower = road_vehicle2

            vehicle_leader = vehicle1
            vehicle_follower = vehicle2

            leader_drived_distance = traci.vehicle.getDistance(vehicle1)
            distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2)

            if True:  # not security_matrice[road_leader][road_follower]:
                # if 120 > distance_betwin_leader_follower and leader_drived_distance > 0:
                # print("safe_mode")
                speed = security_determine_speed(vehicle_leader, vehicle_follower,
                                                 security_distance_matrice[road_leader][road_follower] + 20,
                                                 step_duration)

                traci.vehicle.setColor(vehicle_follower, (255, 0, 0))
                traci.vehicle.slowDown(vehicle_follower, speed, step_duration)

    for iter in range(0, len(leader)-1):
        if iter % 2 == 0:
            vehicle1 = leader[iter]
            vehicle2 = leader[iter + 1]

            if vehicle1 != "" and vehicle2 != "":
                road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
                road_vehicle2 = fct.get_road_index(vehicle2, entry_array)

                road_leader = road_vehicle1
                road_follower = road_vehicle2

                vehicle_leader = vehicle1
                vehicle_follower = vehicle2

                leader_drived_distance = traci.vehicle.getDistance(vehicle1)
                distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(
                    vehicle2)

                if traci.vehicle.getRoute(vehicle_leader)[0] == traci.vehicle.getRoute(vehicle_follower)[0]\
                        and vehicle_leader != vehicle_follower:

                    # if 30 > distance_betwin_leader_follower:
                    # print(vehicle_leader, vehicle_follower)
                    speed = security_determine_speed_same_lane(vehicle_leader, vehicle_follower, 12, step_duration)
                    traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                    traci.vehicle.setColor(vehicle_follower, (255, 0, 255))


def security_get_compatibility(leader_4_and_compatibility):
    """This function is used to get the compatibility between the 4 first vehicle.

        this function is used to determine wich vehicle could pass the intersection together without collisions.

    Parameters
    ----------
    leader_4_and_compatibility : list
        containing a list with the leaders a list with the compatibly of a vehicle an all the vehicle ahead of him

    Returns
    -------
    leader_4_and_compatibility : list
        containing a list with the leaders a list with the compatibly of a vehicle an all the vehicle ahead of him

    """
    if leader_4_and_compatibility[0][0] != "":
        road_leader = fct.get_road_index(leader_4_and_compatibility[0][0], entry_array)
        for i in range(1, len(leader_4_and_compatibility[0])):
            if leader_4_and_compatibility[0][i] != "":
                if security_matrice[road_leader][fct.get_road_index(leader_4_and_compatibility[0][i], entry_array)]:
                    compatible = True
                    for w in range(i):
                        if leader_4_and_compatibility[0][i-w] != "":
                            if not security_matrice[fct.get_road_index(leader_4_and_compatibility[0][i-w], entry_array)][fct.get_road_index(leader_4_and_compatibility[0][i], entry_array)]:
                                compatible = False
                    if compatible:
                        leader_4_and_compatibility[1][i] = 1

    return leader_4_and_compatibility


def security_idm_based_new_strat2(leader, step_duration):
    """This function is a safe mode used to avoid the collision.

        This safe mode is based on the idm follow model to avoid collision.

    Parameters
    ----------
    leader : list
        containing the leaders.
    step_duration : float
        indicating the duration of a single time step.

    Returns
    -------

    """
    # leader_4 contains the leader of each edge leader contains the leader of each edge and their follower
    leader, leader_4 = security_sort_leader_new_strat(leader)
    leader_4_and_compatibility = [leader_4, [1, 0, 0, 0]]

    leader_4_and_compatible = security_get_compatibility(leader_4_and_compatibility)

    for i in range(0, len(leader_4)-1):
        vehicle1 = leader_4[i]
        vehicle2 = leader_4[i+1]
        if vehicle1 != "" and vehicle2 != "":
            road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
            road_vehicle2 = fct.get_road_index(vehicle2, entry_array)

            road_leader = road_vehicle1
            road_follower = road_vehicle2

            vehicle_leader = vehicle1
            vehicle_follower = vehicle2

            leader_drived_distance = traci.vehicle.getDistance(vehicle1)
            follower_drived_distance = traci.vehicle.getDistance(vehicle2)
            distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2)

            if leader_4_and_compatible[1][i+1] != 1 and follower_drived_distance > 50:
                speed = security_determine_speed(vehicle_leader, vehicle_follower,
                                                 security_distance_matrice[road_leader][road_follower] + 20,
                                                 step_duration)

                traci.vehicle.setColor(vehicle_follower, (255, 0, 0))
                traci.vehicle.slowDown(vehicle_follower, speed, step_duration)

    for iter in range(0, len(leader)-1):
        if iter % 2 == 0:
            vehicle1 = leader[iter]
            vehicle2 = leader[iter + 1]

            if vehicle1 != "" and vehicle2 != "":
                road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
                road_vehicle2 = fct.get_road_index(vehicle2, entry_array)

                road_leader = road_vehicle1
                road_follower = road_vehicle2

                vehicle_leader = vehicle1
                vehicle_follower = vehicle2

                leader_drived_distance = traci.vehicle.getDistance(vehicle1)
                distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(
                    vehicle2)

                if traci.vehicle.getRoute(vehicle_leader)[0] == traci.vehicle.getRoute(vehicle_follower)[0]\
                        and vehicle_leader != vehicle_follower:

                    if 20 > distance_betwin_leader_follower:
                        # print(vehicle_leader, vehicle_follower)
                        speed = security_determine_speed_same_lane(vehicle_leader, vehicle_follower, 25, step_duration)
                        traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                        traci.vehicle.setColor(vehicle_follower, (255, 0, 255))


def security_euclidean_determine_speed(leader, follower, distance_min, step_duration):
    """Determine the new acceleration and speed for a follower vehicle.

    this function is used to determine the acceleration and the new speed of a vehicle following an other vehicle
    in order to avoid collision this speed is determined by the euclidean distance.

    Parameters
    ----------
    leader : string
        string containing the name of the leader vehicle.

    follower : string
        string containing the name of the vehicle follower.

    distance_min : float
        containing the minimal distance to be respected between the 2 vehicle.

    step_duration : float
        the duration of one time step.

    Returns
    -------
    speed : float
        float indicating the new speed of the follower vehicle.
    """
    va_1 = traci.vehicle.getSpeed(leader)
    va = traci.vehicle.getSpeed(follower)

    v0 = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(follower))  # speed in free circulation
    s0 = distance_min   # minimum security distance
    t = 1  # delta_s  # minimum time to the front vehicle
    a = 2  # maximum_acceleration
    b = 4  # comfortable_decelaration

    s_etoile = s0 + va * t + (va * (va - va_1)) / (2 * np.sqrt(a * b))

    s_a = security_compute_distance(leader, follower)

    acceleration = a * (1 - np.power(va / v0, 4) - np.power(s_etoile / s_a, 2))
    # acceleration = max(acceleration, -8)
    acceleration = np.clip(acceleration, -8, 2)

    speed = va + (acceleration * step_duration)

    speed = np.clip(speed, 0, 17)

    return speed


def security_idm_euclidean_based_strategy(leader, step_duration):
    """This function is a safe mode used to avoid the collision.

        This safe mode is based on the idm follow model to avoid collision.

    Parameters
    ----------
    leader : list
        containing the leaders.
    step_duration : float
        indicating the duration of a single time step.

    Returns
    -------

    """

    # leader_4 contains the leader of each edge leader contains the leader of each edge and their follower
    leader, leader_4 = security_sort_leader_new_strat(leader)
    leader_4_and_compatibility = [leader_4, [1, 0, 0, 0]]

    leader_4_and_compatible = security_get_compatibility(leader_4_and_compatibility)

    for i in range(0, len(leader_4) - 1):
        vehicle1 = leader_4[i]
        vehicle2 = leader_4[i + 1]
        if vehicle1 != "" and vehicle2 != "":
            road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
            road_vehicle2 = fct.get_road_index(vehicle2, entry_array)

            road_leader = road_vehicle1
            road_follower = road_vehicle2

            vehicle_leader = vehicle1
            vehicle_follower = vehicle2

            leader_drived_distance = traci.vehicle.getDistance(vehicle1)
            follower_drived_distance = traci.vehicle.getDistance(vehicle2)
            distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2)

            if leader_4_and_compatible[1][i + 1] != 1 and follower_drived_distance > 50:
                speed = security_euclidean_determine_speed(vehicle_leader, vehicle_follower, 20,
                                                           step_duration)

                traci.vehicle.setColor(vehicle_follower, (255, 0, 0))
                traci.vehicle.slowDown(vehicle_follower, speed, step_duration)

    for iter in range(0, len(leader) - 1):
        if iter % 2 == 0:
            vehicle1 = leader[iter]
            vehicle2 = leader[iter + 1]

            if vehicle1 != "" and vehicle2 != "":
                road_vehicle1 = fct.get_road_index(vehicle1, entry_array)
                road_vehicle2 = fct.get_road_index(vehicle2, entry_array)

                road_leader = road_vehicle1
                road_follower = road_vehicle2

                vehicle_leader = vehicle1
                vehicle_follower = vehicle2

                leader_drived_distance = traci.vehicle.getDistance(vehicle1)
                distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(
                    vehicle2)

                if traci.vehicle.getRoute(vehicle_leader)[0] == traci.vehicle.getRoute(vehicle_follower)[0] \
                        and vehicle_leader != vehicle_follower:

                    if 20 > distance_betwin_leader_follower:
                        # print(vehicle_leader, vehicle_follower)
                        speed = security_determine_speed_same_lane(vehicle_leader, vehicle_follower, 20, step_duration)
                        traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                        traci.vehicle.setColor(vehicle_follower, (255, 0, 255))


def security_sort_leader_new_strat2(leader, action):
    """This function sort the leaders according to a new strategy.

        The leader are separated in two category :
         - vehicle leader following a vehicle on an other lane
         - vehcile leader following a vehicle on the same lane

    Parameters
    ----------
    leader : list
        containing the leaders.
    action : array
        containing the leaders actions

    Returns
    -------
    leader : list
        containing all the leaders sorted.
    leader_4 : list
        containing the first vehicle of each lane.
    action_4 : array

    action : array
    """
    distance_tab = ["", "", "", ""]
    leader_4 = ["", "", "", ""]
    action_4 = [-1, -1, -1, -1]
    # we determine the leader of each edge
    for iter, vehicle in enumerate(leader):
        if iter % 2 == 0:
            # print(iter/2)
            leader_4[int(iter/2)] = vehicle
            action_4[int(iter / 2)] = action[iter]



    return leader, leader_4, action_4, action


def security_sort_leader4(leader_4, action_4, speed_tab_4_leader):
    """This function is used to sort 4 leader.

    Parameters
    ----------
    speed_tab_4_leader : array
        containing the speed calculated by the idm
    leader_4 : array
        containing the 4 first leader.

    action_4 : array
        containing the action of the 4 first leader.

    Returns
    -------
    leader_4 : array
        sorted leader_4
    action_4 : array
        sorted action_4
    speed_tab_4_leader : array

    """

    distance_tab = ["", "", "", ""]
    # we get the distance drived by each leader of the edge
    for iter, vehicle in enumerate(leader_4):
        if vehicle != "":
            distance_tab[iter] = traci.vehicle.getDistance(vehicle)
        else:
            distance_tab[iter] = -1000

    # we sort the leader from the closest of the exit to the most far
    n = len(distance_tab)
    # Traverser tous les éléments du tableau
    for i in range(n):
        for j in range(0, n - i - 1):
            # échanger si l'élément trouvé est plus grand que le suivant
            if distance_tab[j] < distance_tab[j + 1]:
                distance_tab[j], distance_tab[j + 1] = distance_tab[j + 1], distance_tab[j]
                leader_4[j], leader_4[j + 1] = leader_4[j + 1], leader_4[j]
                action_4[j], action_4[j + 1] = action_4[j + 1], action_4[j]
                speed_tab_4_leader[j], speed_tab_4_leader[j + 1] = speed_tab_4_leader[j + 1], speed_tab_4_leader[j]

    return leader_4, action_4, speed_tab_4_leader


def security_idm_euclidean_based_strategy2(leader, step_duration, action):
    """This function is a safe mode used to avoid the collision.

        This safe mode is based on the idm follow model to avoid collision.

    Parameters
    ----------
    leader : list
        containing the leaders.
    step_duration : float
        indicating the duration of a single time step.
    action : array
        indicating the action
    Returns
    -------

    """

    # leader_4 contains the leader of each edge leader contains the leader of each edge and their follower
    leader, leader_4, action_4, action = security_sort_leader_new_strat2(leader, action)

    speed_tab_4_leader = [99, 99, 99, 99]

    for i, vehicle1 in enumerate(leader_4):
        if vehicle1 != "":
            for j, vehicle2 in enumerate(leader_4):
                if vehicle2 != "" and vehicle1 != vehicle2 \
                        and traci.vehicle.getRoute(vehicle1)[0] != traci.vehicle.getRoute(vehicle2)[0]:

                    if traci.vehicle.getDistance(vehicle1) >= traci.vehicle.getDistance(vehicle2):
                        vehicle_leader = vehicle1
                        vehicle_follower = vehicle2
                        index_follower = j
                    else:
                        vehicle_leader = vehicle2
                        vehicle_follower = vehicle1
                        index_follower = i

                    speed = security_determine_speed(vehicle_leader, vehicle_follower,
                                                     25, step_duration)
                    if speed < speed_tab_4_leader[index_follower]:
                        speed_tab_4_leader[index_follower] = speed

    leader_4, action_4, speed_tab_4_leader = security_sort_leader4(leader_4, action_4, speed_tab_4_leader)

    leader_4_and_compatibility = [leader_4, [1, 0, 0, 0]]
    leader_4_and_compatible = security_get_compatibility(leader_4_and_compatibility)

    for count in range(0, len(leader_4)-1):
        if speed_tab_4_leader[count+1] != 99 and speed_tab_4_leader[count+1] < action_4[count+1]\
                and leader_4_and_compatible[1][count+1] != 1 and traci.vehicle.getDistance(leader_4[count+1]) > 60:

            traci.vehicle.slowDown(leader_4[count+1], speed_tab_4_leader[count+1], step_duration)
            traci.vehicle.setColor(leader_4[count+1], (255, 0, 0))

    for iter_i in range(0, len(leader) - 1):
        if iter_i % 2 == 0:
            vehicle1 = leader[iter_i]
            vehicle2 = leader[iter_i + 1]

            if vehicle1 != "" and vehicle2 != "":

                vehicle_leader = vehicle1
                vehicle_follower = vehicle2

                distance_betwin_leader_follower = traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(
                    vehicle2)

                if traci.vehicle.getRoute(vehicle_leader)[0] == traci.vehicle.getRoute(vehicle_follower)[0] \
                        and vehicle_leader != vehicle_follower:

                    if 100 > distance_betwin_leader_follower:
                        # print(vehicle_leader, vehicle_follower)
                        speed = security_determine_speed_same_lane(vehicle_leader, vehicle_follower, 20, step_duration)
                        if speed < action[iter_i+1]:
                            traci.vehicle.slowDown(vehicle_follower, speed, step_duration)
                            traci.vehicle.setColor(vehicle_follower, (255, 0, 255))


def security_safe_mode_new_approach(list_leader_sort, action, step_duration):

    for i in range(0, len(list_leader_sort)-1):
        vehicle_leader = list_leader_sort[i]
        vehicle_follower = list_leader_sort[i+1]

        speed = security_determine_speed(vehicle_leader, vehicle_follower, 1, step_duration)

        traci.vehicle.slowDown(vehicle_follower, speed, step_duration)

