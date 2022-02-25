from __future__ import absolute_import
from __future__ import print_function

import project

import numpy as np
import matplotlib.pyplot
import tensorflow as tf
from tools import simulation_parameter_tools as simu_tools, statistics

from sumo_utils import * # noqa

security_matrice = np.load(project.resources_dir + 'variable_ICGV2/security_matrice.npy')
entry_array = np.load(project.resources_dir + 'variable_ICGV2/entry_array.npy')


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


def reset(display, image_size):
    """close and open a new simulation.

    this function reset the simulation.

    Parameters
    ----------
    display : bool
        boolean which indicate if the graphical interface should be used.
    image_size : int
        size of the image we want.

    Returns
    -------
    return a black image as the simulation just restarted.

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
    n = image_size
    return np.zeros((n, n, 3), dtype=int)


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
    action_binaire = [0, 0, 0, 0, 0, 0, 0, 0]
    count = 0
    while action != 0:
        action_binaire[len(action_binaire)-1 - count] = action % 2
        action = action // 2
        count += 1

    return action_binaire


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
            # traci.vehicle.setColor(leader_tab[i], (255,255,255))
    return leader_tab


def getleadandfollow():
    """Determine the closest vehicle to the intersection of each edge and their follower.

    This function is used to determine the 4 closest vehicle to the intersections and their followers

    Parameters
    ----------

    Returns
    -------
    leader_tab : list
        return a list with the name of the 8 vehicle
    """
    leader_tab = ["", "", "", "", "", "", "", ""]
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    for i in range(0, len(flow_edges)):
        if traci.edge.getLastStepVehicleNumber(flow_edges[i]) >= 1:
            leader_tab[i+i] = traci.edge.getLastStepVehicleIDs(
                flow_edges[i])[traci.edge.getLastStepVehicleNumber(flow_edges[i]) - 1]
            leader_tab[i+i+1] = traci.vehicle.getFollower(leader_tab[i+i])[0]

            # traci.vehicle.setColor(leader_tab[i], (255,255,255))
    return leader_tab


def set_leaders_actions2(leader_tab, action):
    """Change the actions of the leaders vehicles.

    this function is used to change the actions of the 4 leaders and theirs followers.
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
            set_action(i, leader_tab[i], action)


def secure_actions(leader_tab, action):
    """This function is used to delete the unsafe action taken by the IA.

        if a right of way is unsafe he will be ignored.

    Parameters
    ----------
    leader_tab : list
        containing the leaders
    action : list
        containing the actions.

    Returns
    -------
        action : list
            containing only the safe actions.
    """
    right_of_way_tab = np.where(np.array(action) == 1)
    for k in range(0, len(right_of_way_tab[0])):
        if leader_tab[right_of_way_tab[0][k]] == "":
            action[right_of_way_tab[0][k]] = 0

    right_of_way_tab = np.where(np.array(action) == 1)

    for i in range(0, len(right_of_way_tab[0])):

        road1 = traci.vehicle.getRoute(leader_tab[right_of_way_tab[0][i]])
        road1 = road1[0] + road1[1]
        for j in range(0, len(right_of_way_tab[0])):

            road2 = traci.vehicle.getRoute(leader_tab[right_of_way_tab[0][j]])
            road2 = road2[0]+road2[1]

            index1 = entry_array.flatten().tolist().index(road1)
            index2 = entry_array.flatten().tolist().index(road2)

            if not security_matrice[index1][index2]:
                if right_of_way_tab[0][i] % 2 == 0:
                    action[right_of_way_tab[0][j]] = 0

                elif right_of_way_tab[0][j] % 2 == 0:
                    action[right_of_way_tab[0][i]] = 0

                else:
                    action[right_of_way_tab[0][j]] = 0

    return action


def step_no_traffic_light_v2(action, simulation_time, image_size, reward_type, coef, security=False):
    """Control the step during the simulation.

    this function is used to take an action as a parameters and run the simulation until this actions had an effect.

    Parameters
    ----------
    action : list
        list containing the wished action for the vehicles
        0 for stop 1 for go
    simulation_time : int
        int which indicate the wished simulation time in second
    image_size : int
        size of the image we want.
    reward_type : string
        type of the reward
    coef : float
        coefficient used on the collision reward
    security : bool
        boolean that determine if secure action will be used (false by default)

    Returns
    -------
    state_next : ndarray
        ndarray of shape (N,N,3) containing a simplified image of the simulation
    reward : int
        int containing the sum of the obtained reward
    done : bool
        boolean indicating if the function should be stopped
    average_waiting_time : float
        float containing the average waiting time for a vehicle during a step
    cumulated_waiting_time : float
        float containing the cumulated waiting time during a step
    emission_of_Co2 : float
        float containing the quantity of Co2 emitted in mg during a step
    average_speed : float
        float containing the average speed of a vehicle
    evacuated_vehicle : int
        number of vehicle evacuated during the run of the function
    nb_collision : int
        number of collision during the run of the function


    """
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    reward = 0
    average_waiting_time = 0
    emission_of_co2 = 0
    cumulated_waiting_time = 0
    average_speed = 0
    evacuated_vehicle = 0
    nb_collision = 0
    done = 0
    counter = 0
    leader_tab = ["", "", "", ""]
    # action = [1, 1, 0, 1, 1, 1, 1, 1]

    while get4leader() == ["", "", "", ""]:
        set_speed_mode()
        traci.simulationStep()
        emission_of_co2, average_waiting_time, cumulated_waiting_time, average_speed, evacuated_vehicle, nb_collision, \
            reward = statistic_adder(flow_edges, reward_type, coef, emission_of_co2, average_waiting_time,
                                     cumulated_waiting_time, average_speed,
                                     evacuated_vehicle, nb_collision, reward)
    leader_tab = getleadandfollow()

    if security:
        action = secure_actions(leader_tab, action)

    set_leaders_actions2(leader_tab, action)
    while done == 0 and leader_arriver2(leader_tab, action) is not True:
        counter += 1
        set_speed_mode()
        set_loaded_vehicle(leader_tab)
        traci.simulationStep()

        # emission_of_co2 += statistique.emissionDeCo2(flow_edges)
        # average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
        # cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
        # average_speed += statistique.vitesseMoyenne(flow_edges)
        # evacuated_vehicle += traci.simulation.getArrivedNumber()
        # nb_collision += traci.simulation.getCollidingVehiclesNumber()
        # reward += statistique.reward_calculation(reward_type, coef)  # traci.simulation.getArrivedNumber()

        emission_of_co2, average_waiting_time, cumulated_waiting_time, average_speed, evacuated_vehicle, nb_collision, \
            reward = statistic_adder(flow_edges, reward_type, coef, emission_of_co2, average_waiting_time,
                                     cumulated_waiting_time, average_speed,
                                     evacuated_vehicle, nb_collision, reward)

        if traci.simulation.getCollidingVehiclesNumber() > 0:
            done = 0
            # reward += -2
        if traci.simulation.getTime() > simulation_time:
            done = 1

    b_action0 = True
    for i in range(0, 4):
        if action[i+i] == 1:
            b_action0 = False

    if b_action0:
        counter += 1
        set_speed_mode()
        set_loaded_vehicle(leader_tab)
        traci.simulationStep()
        emission_of_co2, average_waiting_time, cumulated_waiting_time, average_speed, evacuated_vehicle, nb_collision, \
            reward = statistic_adder(flow_edges, reward_type, coef, emission_of_co2, average_waiting_time,
                                     cumulated_waiting_time, average_speed,
                                     evacuated_vehicle, nb_collision, reward)
        if traci.simulation.getCollidingVehiclesNumber() > 0:
            done = 0
            # reward += -2
        if traci.simulation.getTime() > simulation_time:
            done = 1

    lane_empty = False
    for i in range(0, len(leader_tab)):
        if leader_tab[i] == "":
            lane_empty = True

    if lane_empty:
        counter += 1
        set_speed_mode()
        set_loaded_vehicle(leader_tab)
        traci.simulationStep()
        emission_of_co2, average_waiting_time, cumulated_waiting_time, average_speed, evacuated_vehicle, nb_collision, \
            reward = statistic_adder(flow_edges, reward_type, coef, emission_of_co2, average_waiting_time,
                                     cumulated_waiting_time, average_speed,
                                     evacuated_vehicle, nb_collision, reward)
        if traci.simulation.getCollidingVehiclesNumber() > 0:
            done = 0
            # reward += -2
        if traci.simulation.getTime() > simulation_time:
            done = 1

    if counter > 0:
        emission_of_co2 = emission_of_co2/counter
        average_waiting_time = average_waiting_time/counter
        cumulated_waiting_time = cumulated_waiting_time/counter
        average_speed = average_speed/counter

    state_next = image_construct(image_size)

    return state_next, reward/counter, done, average_waiting_time, cumulated_waiting_time, emission_of_co2,\
        average_speed, evacuated_vehicle, nb_collision


def statistic_adder(flow_edges, reward_type, coef, emission_of_co2, average_waiting_time, cumulated_waiting_time,
                    average_speed, evacuated_vehicle, nb_collision, reward):
    """This function is used to add our statistic.

    Parameters
    ----------
    flow_edges : list
        containing the name of our entry edges.
    reward_type : string
        the type of reward we want to use.
    coef : float
        the coefficient used to calculate the reward.
    emission_of_co2 : float
        float containing statistic about emission.
    average_waiting_time : float
        float containing statistic about waiting time.
    cumulated_waiting_time : float
        float containing statistic about waiting time.
    average_speed : float
        float containing statistic about speed.
    evacuated_vehicle : float
        float containing statistic about evacuated vehicle.
    nb_collision : float
        float containing statistic about collision.
    reward : float
        containing the reward.

    Returns
    -------
        statistic about the simulation

    """

    emission_of_co2 += statistique.emissionDeCo2(flow_edges)
    average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
    cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
    average_speed += statistique.vitesseMoyenne(flow_edges)
    evacuated_vehicle += traci.simulation.getArrivedNumber()
    nb_collision += traci.simulation.getCollidingVehiclesNumber()
    reward += statistique.reward_calculation(reward_type, coef)

    return emission_of_co2, average_waiting_time, cumulated_waiting_time, average_speed, evacuated_vehicle,\
        nb_collision, reward


def leader_arriver2(leaders_tab, action):
    """Indicate if the vehicle have leave the intersection.

    this function is used to determine when the vehicle authorized to pass the intersection have leave the intersection.

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
            if i % 2 != 0 and action[i-1] == 1:
                leader_bool = False
            elif i % 2 == 0:
                leader_bool = False

    return leader_bool


def set_action(i, veh_id, action):
    """Change the actions of the given vehicle.

    this function is used to change the actions of the given vehicle this function is used by
    the function setLeadersActions

    Parameters
    ----------
    i : int
        int indication the index of the wished action for the vehicle.
    veh_id : string
        string containing the name of the vehicle.
    action : list
        list containing the wished action for the vehicles
        0 for stop 1 for go

    Returns
    -------
    negativeReward : int
        it's a negative reward for each impossible action

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


def image_construct(n, display=False):
    """build a representation of the current state of the simulation.

    this function is used to to build an N by N image of a state of the simulation.
    each on pixel represent a vehicle.
    the intensity of the pixel represent the speed and the color of the pixel their destination

    Parameters
    ----------
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
    # initialise the image :
    running_vehicle_id = traci.vehicle.getIDList()
    count_running_vehicle = traci.vehicle.getIDCount()

    for i in range(0, count_running_vehicle):

        position = traci.vehicle.getPosition(running_vehicle_id[i])
        # intensity of the pixel in function of the speed :
        vehicle_speed = traci.vehicle.getSpeed(running_vehicle_id[i])
        lane_id = traci.lane.getIDList()
        v_max_lane = traci.lane.getMaxSpeed(lane_id[0])

        convert_pos_x = position[0] + 100
        convert_pos_y = position[1] + 100

        pixel_pos_x = round(convert_pos_x / 200 * (n - 1))
        pixel_pos_y = round(convert_pos_y / 200 * (n - 1))

        # pixel_pos_x = round(position[1]/5)+19
        # pixel_pos_y = round(position[0]/5) + 19
        # polygon = traci.simulation.getNetBoundary()
        # # hauteur = polygon[1][1]-polygon[0][0]

        route_list = traci.vehicle.getRoute(running_vehicle_id[i])

        if route_list[len(route_list)-1] == traci.vehicle.getRoadID(running_vehicle_id[i]):
            image[pixel_pos_x][pixel_pos_y][0] = 255
            image[pixel_pos_x][pixel_pos_y][1] = 255
            image[pixel_pos_x][pixel_pos_y][2] = 255
        else:
            if route_list[len(route_list)-1] == "2to1":
                image[pixel_pos_x][pixel_pos_y][0] = 40+round(215 * (vehicle_speed/18))
                # print(image[pixel_pos_x][pixel_pos_y][0])
            elif route_list[len(route_list)-1] == "2to4":
                image[pixel_pos_x][pixel_pos_y][1] = 40+round(215 * (vehicle_speed/18))
                # print(image[pixel_pos_x][pixel_pos_y][1])
            elif route_list[len(route_list)-1] == "2to5":
                image[pixel_pos_x][pixel_pos_y][2] = 40+round(215 * (vehicle_speed/18))
                # print(image[pixel_pos_x][pixel_pos_y][2])
            elif route_list[len(route_list)-1] == "2to3":
                image[pixel_pos_x][pixel_pos_y][0] = 40+round(215 * (vehicle_speed/18))
                image[pixel_pos_x][pixel_pos_y][1] = 40+round(215 * (vehicle_speed/18))

    # printImage(image, N)
    if display:
        matplotlib.pyplot.imshow(image)
        matplotlib.pyplot.show()

    return image

class Step_runner_Dqn():

    def __init__(self, model, image_size, reward_type, coef, security=True):

        self.model = model
        self.coef = coef
        self.image_size = image_size
        self.state = np.zeros((image_size, image_size, 3,))
        self.reward_type = reward_type
        self.security = security


    def run_step_simulation(self, simulation_time):

        state_tensor = tf.convert_to_tensor(self.state)
        state_tensor = tf.expand_dims(state_tensor, 0)
        action_probs = self.model(state_tensor, training=False)

        action = tf.argmax(action_probs[0]).numpy()
        action_binaire = trad_action(action)

        state_next, reward, done, average_waiting_time, cumulated_waiting_time, emission_of_co2, \
            average_speed, evacuated_vehicle, nb_collision = step_no_traffic_light_v2(action_binaire, simulation_time, self.image_size, self.reward_type, self.coef, self.security)

        self.state = state_next

        done = True if (done == 1) else False

        return done, average_waiting_time, cumulated_waiting_time, emission_of_co2, average_speed, evacuated_vehicle, nb_collision

    def start_simulation(self, display):
        start_simulation(display)

    def xml_flow_changer(self, flow1, flow2, flow3, flow4):
        simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "Net")

    def reset(self, display):
        self.state = reset(display, self.image_size)
