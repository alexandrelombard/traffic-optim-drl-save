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
    traci.start([sumo_binary, "-c", project.resources_dir + "traficLightNet_cycle/NetworkNoTraficLight.sumocfg",
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
    traci.start([sumo_binary, "-c", project.resources_dir + "traficLightNet_cycle/NetworkNoTraficLight.sumocfg",
                 "--tripinfo-output", project.resources_dir + "tripinfo.xml", "--random", "--collision.check-junctions"])
    # runningVehicleID = traci.vehicle.getIDList()
    # countRunningVehicle = traci.vehicle.getIDCount()
    # return imageConstruct(runningVehicleID, runningVehicleID, 40)
    n = 40
    return np.zeros((n, n, 3), dtype=int)


def step_few(simulation_time, reward_type, coef):
    """run the simulation step after step.

    this function is used to run the simulation.

    Parameters
    ----------
    simulation_time : int
        the wished simulation time in second
    reward_type : string
        type of reward wished
    coef :
        coefficient used in the reward calculation.

    Returns
    -------
    reward : int
        current reward
    done : int
        0 or 1 indicating if the simulation should be ended

    and other statistic about the simulation.

    """
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    reward = 0
    done = 1

    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0
    evacuated_vehicle = 0
    collision = 0

    traci.simulationStep()
    reward += statistique.reward_calculation(reward_type, coef)
    average_waiting_time += statistique.tempAttenteMoyen(flow_edges)
    cumulated_waiting_time += statistique.tempAttenteCumuler(flow_edges)
    average_speed += statistique.vitesseMoyenne(flow_edges)
    emission_co2 += statistique.emissionDeCo2(flow_edges)
    evacuated_vehicle += traci.simulation.getArrivedNumber()
    collision += traci.simulation.getCollidingVehiclesNumber()

    if traci.simulation.getTime() > simulation_time:
        done = 0

    return reward, done, collision, average_waiting_time, \
        cumulated_waiting_time, average_speed, emission_co2, evacuated_vehicle


def launch_optimal_trafic_light_cycle(display, simulation_time, reward_type, coef, flow1, flow2, flow3, flow4):
    """This function is used to launch one simulation.

        This function launch one episode simulation.

    Parameters
    ----------
    display : bool
        indicating if we want a display of the simulation
    simulation_time : int
        the wished duration of the simulation
    reward_type : string
        the wished reward type
    coef : float
        coefficient used for the calculation of the reward
    flow1 : int
        number of vehicle per hour on a specific edge.
    flow2 : int
        number of vehicle per hour on a specific edge.
    flow3 : int
        number of vehicle per hour on a specific edge.
    flow4 : int
        number of vehicle per hour on a specific edge.
    """
    simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "traficLightNet_cycle")
    start_simulation(display)
    reward = 0

    collision_nb = 0
    average_waiting_time = 0
    cumulated_waiting_time = 0
    average_speed = 0
    emission_co2 = 0
    evacuated_vehicle = 0

    done = 1
    n = 0
    while done:
        n += 1
        step_reward, done, collision, average_wait, cumulated_wait, speed_average, co2_emi, evacuated_vehicle_temp = \
            step_few(simulation_time, reward_type, coef)
        reward += step_reward

        collision_nb += collision
        average_waiting_time += average_wait
        cumulated_waiting_time += cumulated_wait
        average_speed += speed_average
        emission_co2 += co2_emi
        evacuated_vehicle += evacuated_vehicle_temp

    traci.close()
    print("le nombre de vehicule sortie en un episode de :", simulation_time, "est de :", reward)
    return collision_nb/n, average_waiting_time/n, cumulated_waiting_time/n, average_speed/n,\
        emission_co2/n, evacuated_vehicle


def launch_simulation_statistique(display, nb_episode, time_per_episode, reward_type, coef, flow1, flow2, flow3, flow4):
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
    coef : float
        coefficient used for the calculation of the reward
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
            evacuated_vehicle_temp = launch_optimal_trafic_light_cycle(display, time_per_episode, reward_type, coef,
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

    return [nb_episode, collision/nb_episode, average_waiting_time/nb_episode, std_waiting_time,
            cumulated_waiting_time/nb_episode, std_cumulated_waiting_time, average_speed/nb_episode,
            std_average_speed, emission_co2/nb_episode, std_emission_co2, evacuated_vehicle/nb_episode,
            std_evacuated_vehicle, flow1]

class Step_runner_tf():

    def __init__(self, reward_type, coef):
        self.reward_type = reward_type
        self.coef = coef

    def run_step_simulation(self, simulation_time):

        reward, done, collision, average_waiting_time, \
            cumulated_waiting_time, average_speed, emission_co2, evacuated_vehicle = step_few(simulation_time, self.reward_type, self.coef)

        done = True if(done == 0) else False

        return done, average_waiting_time, cumulated_waiting_time, emission_co2, average_speed, evacuated_vehicle, collision

    def start_simulation(self, display):
        start_simulation(display)

    def xml_flow_changer(self, flow1, flow2, flow3, flow4):
        simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "traficLightNet_cycle")

    def reset(self, display):
        reset(display)

