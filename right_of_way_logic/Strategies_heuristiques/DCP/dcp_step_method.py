import right_of_way_logic.Strategies_heuristiques.fisrtcomefirstserved.firstcomefirstserved_strict as base_fct
import traci
import numpy as np
from Tools import simulation_parametre_tools as simu_tools, statistique


def dcp_leader_arriver(list_vehicle, action):

    leader_bool = False if len(list_vehicle) < 1 else True
    removed_number = 0

    for i in range(0, len(action)):
        if action[i - removed_number] == 1 and traci.vehicle.getRouteIndex(list_vehicle[i - removed_number]) > 0:
            list_vehicle.remove(list_vehicle[i - removed_number])
            del action[i - removed_number]
            removed_number += 1
        elif action[i - removed_number] == 1:
            leader_bool = False


    return leader_bool


def dcp_set_action(action, veh_id):
    for i in range(0, len(action)):
        if action[i] == 1:
            try:
                traci.vehicle.setStop(veh_id[i], traci.vehicle.getRoadID(veh_id[i]), 90, duration=0)
                traci.vehicle.setColor(veh_id[i], (0, 255, 0))
            except traci.TraCIException:
                print("vehicle not on the edge")

        elif action[i] == 0:
            try:
                traci.vehicle.setStop(veh_id[i], traci.vehicle.getRoadID(veh_id[i]), 90)
                traci.vehicle.setColor(veh_id[i], (255, 0, 0))
            except traci.TraCIException:
                print("to late to brake")


def dcp_distance_computation(vehicle1, vehicle2):
    print(traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2))
    return traci.vehicle.getDistance(vehicle1) - traci.vehicle.getDistance(vehicle2)


def dcp_check_retro_compatibility(list_vehicle, index, action):
    compatible = True
    for i in range(0, index):
        if action[i] == 1:
            if not base_fct.route_compatibility(list_vehicle[i], list_vehicle[index]):
                compatible = False
        if action[i] == 0 and traci.vehicle.getRoute(list_vehicle[i])[0] == traci.vehicle.getRoute(list_vehicle[index])[0]:
            compatible = False

    return compatible


def dcp_refresh_list_vehicle(list_sorted_leader, action):
    for vehicle in traci.simulation.getDepartedIDList():
        list_sorted_leader.append(vehicle)
        traci.vehicle.setStop(vehicle, traci.vehicle.getRoadID(vehicle), 90)


def dcp_fcfs_strategy(list_vehicle):
    """This function apply the fcfs strategy in the simulation.

    this function is used to apply the fcfs strategy and let the vehicle leave
    the intersection in their order of incoming.

    Parameters
    ----------
    list_vehicle : list
        list containing all the vehicle in their order of incoming.

    Returns
    -------
    leader : list
        list containing the leaders
    action : list
        list containing the actions taken

    """

    action = [0] * len(list_vehicle)

    leader = list_vehicle
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]

    if len(list_vehicle) > 0:

        # traci.vehicle.setColor(leader[int(waiting_time[1][0])],(255,255,255))
        action[0] = 1
        for w in range(0, len(list_vehicle)-1):
            if dcp_distance_computation(list_vehicle[0], list_vehicle[w+1]) < 15 and dcp_check_retro_compatibility(list_vehicle, w+1, action):
                action[w+1] = 1
            else:
                action[w+1] = 0

        dcp_set_action(action, leader)

    return leader, action


def dcp_step_runner(simulation_time, list_vehicle):
    """

    Parameters
    ----------
    simulation_time : int
        indicating the wished duration of the simulation
    list_vehicle : list
        list containing all the vehicle in there order of coming

    Returns
    -------

    """
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    done = False
    count = 0

    reward = 0
    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0
    evacuated_vehicle = 0
    collision = 0

    while traci.vehicle.getIDCount() <= 0:
        base_fct.set_speed_mode()
        traci.simulationStep()
        base_fct.list_update(list_vehicle)

    leader, action = dcp_fcfs_strategy(list_vehicle)

    while not dcp_leader_arriver(list_vehicle, action):

        count += 1

        base_fct.set_speed_mode()
        traci.simulationStep()
        dcp_refresh_list_vehicle(list_vehicle, action)
        base_fct.set_speed_mode()

        collision += traci.simulation.getCollidingVehiclesNumber()
        reward += traci.simulation.getArrivedNumber()
        average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
        cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
        average_speed += statistique.vitesseMoyenne(flow_edges)
        emission_co2 += statistique.emissionDeCo2(flow_edges)
        evacuated_vehicle += traci.simulation.getArrivedNumber()

        if traci.simulation.getTime() > simulation_time:
            done = True

    if count > 0:
        average_waiting_time = average_waiting_time / count
        cumulated_waiting_time = cumulated_waiting_time / count
        average_speed = average_speed / count
        emission_co2 = emission_co2 / count

    return collision, average_waiting_time, cumulated_waiting_time,\
        average_speed, emission_co2, done, evacuated_vehicle


def launch_dcp_fcfs(display, simulation_time, reward_type, flow1, flow2, flow3, flow4):

    simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "Net")

    base_fct.start_simulation(display)

    list_vehicle = []

    collision_nb = 0
    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0
    evacuated_vehicle = 0
    n = 0

    done = False
    while not done:
        n += 1

        collision, average_wait, cumulated_wait, speed_average, co2_emi, done, evacuated_vehicle_temp = \
            dcp_step_runner(simulation_time, list_vehicle)

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

    return collision_nb/n, average_waiting_time/n, cumulated_waiting_time/n, average_speed/n,\
        emission_co2/n, evacuated_vehicle


def launch_dcp_simulation_statistique(display, nb_episode, time_per_episode, reward_type, flow1, flow2, flow3, flow4):

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

        collision_temp, average_waiting_time_temp, cumulated_waiting_time_temp, average_speed_temp, emission_co2_temp,\
            evacuated_vehicle_temp = launch_dcp_fcfs(display, time_per_episode, reward_type,
                                                     flow1, flow2, flow3, flow4)

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

    return [nb_episode, collision/nb_episode,
            average_waiting_time/nb_episode, std_waiting_time,
            cumulated_waiting_time/nb_episode, std_cumulated_waiting_time,
            average_speed/nb_episode, std_average_speed,
            emission_co2/nb_episode, std_emission_co2,
            evacuated_vehicle/nb_episode, std_evacuated_vehicle,
            flow1]

class Step_runner_dcp():

    def __init__(self, list_vehicle):
        self.list_vehicle = list_vehicle

    def run_step_simulation(self, simulation_time):

        collision, average_waiting_time, cumulated_waiting_time, \
            average_speed, emission_co2, done, evacuated_vehicle = dcp_step_runner(simulation_time, self.list_vehicle)

        self.erase_list_vehicule(done)

        return done, average_waiting_time, cumulated_waiting_time, emission_co2, average_speed, evacuated_vehicle, collision

    def erase_list_vehicule(self, done):
        if done:
            self.list_vehicle = []

    def start_simulation(self, display):
        base_fct.start_simulation(display)

    def xml_flow_changer(self, flow1, flow2, flow3, flow4):
        simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "Net")

    def reset(self, display):
        base_fct.reset(display)


