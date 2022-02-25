import project

import numpy as np

import os
import sys

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

import control_acceleration_logic.ai_cross_acceleration_logic.ia_cross_speed_control.ai_step_runner as fct_ia_step_runner
from sumolib import checkBinary

from tools import statistics


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
    # n = 40
    # return np.zeros((n, n, 3), dtype=int)
    return observation_new_approach([])

def get_flow_list():
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

    return vehicle_flow0, vehicle_flow1, vehicle_flow2, vehicle_flow3

def observation_maker(list_flow, obs_new_approach, entry_array):

    for i in range(0, 4):
        if len(list_flow) >= i+1:
            obs_new_approach.append(traci.vehicle.getPosition(list_flow[i])[0])
            obs_new_approach.append(traci.vehicle.getPosition(list_flow[i])[1])
            obs_new_approach.append(fct_ia_step_runner.get_road_index(list_flow[i], entry_array))
            obs_new_approach.append(traci.vehicle.getSpeed(list_flow[i]))
        else:
            obs_new_approach.append(-1000)
            obs_new_approach.append(-1000)
            obs_new_approach.append(-1000)
            obs_new_approach.append(-1000)


def observation_new_approach(list_sorted_leader):
    entry_array = np.load(project.resources_dir + 'variable_CAC/entry_array.npy')
    obs_new_approach = [] #np.zeros((212,))
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]

    list_flow0, list_flow1, list_flow2, list_flow3 = get_flow_list()

    # for i in range(0, 52):
    #     if len(list_sorted_leader) >= i+1:
    #         obs_new_approach.append(traci.vehicle.getPosition(list_sorted_leader[i])[0])
    #         obs_new_approach.append(traci.vehicle.getPosition(list_sorted_leader[i])[1])
    #         obs_new_approach.append(fct_ia_step_runner.get_road_index(list_sorted_leader[i], entry_array))
    #         obs_new_approach.append(traci.vehicle.getSpeed(list_sorted_leader[i]))
    #     else:
    #         obs_new_approach.append(-1000)
    #         obs_new_approach.append(-1000)
    #         obs_new_approach.append(-1000)
    #         obs_new_approach.append(-1000)


    observation_maker(list_flow0, obs_new_approach, entry_array)
    observation_maker(list_flow1, obs_new_approach, entry_array)
    observation_maker(list_flow2, obs_new_approach, entry_array)
    observation_maker(list_flow3, obs_new_approach, entry_array)


    for flow in flow_edges:
        obs_new_approach.append(traci.edge.getLastStepVehicleNumber(flow))

    # print("size obs", len(obs_new_approach))

    return np.array(obs_new_approach)


def refresh_list_vehicle(list_sorted_leader):
    for vehicle in traci.simulation.getDepartedIDList():
        list_sorted_leader.append(vehicle)

    for vehic in list_sorted_leader:
        if traci.vehicle.getRouteIndex(vehic) >= 1:
            list_sorted_leader.remove(vehic)


def flow_list_acceleration_control(action, list_flow, count):
    for i in range(0, min(len(list_flow), 4)):
        if list_flow != "":
            # print(traci.vehicle.getSpeed(vehicle))
            action[i+count] = traci.vehicle.getSpeed(list_flow[i]) + (action[i+count] * fct_ia_step_runner.step_duration)
            action[i+count] = np.clip(action[i+count], 0, 17)

    return action

def acceleration_control(action, list_flow0, list_flow1, list_flow2, list_flow3):
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
    # for i in range(0, min(len(leader), 52)):
    #     if leader[i] != "":
    #         # print(traci.vehicle.getSpeed(vehicle))
    #         action[i] = traci.vehicle.getSpeed(leader[i]) + (action[i] * fct_ia_step_runner.step_duration)
    #         action[i] = np.clip(action[i], 0, 17)
    flow_list_acceleration_control(action, list_flow0, 0)
    flow_list_acceleration_control(action, list_flow1, 4)
    flow_list_acceleration_control(action, list_flow2, 8)
    flow_list_acceleration_control(action, list_flow3, 12)

    return action


def flow_list_set_action(action, list_flow, count):
    for i in range(0, min(len(list_flow), 4)):
        if list_flow[i] != "":
            if traci.vehicle.getNextStops(list_flow[i]) != ():
                traci.vehicle.setStop(list_flow[i], traci.vehicle.getRoadID(list_flow[i]), 90, duration=0)

            traci.vehicle.slowDown(list_flow[i], action[i+count], fct_ia_step_runner.step_duration)

def set_action(action, list_flow0, list_flow1, list_flow2, list_flow3):
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
    # for i in range(0, min(len(leader), 52)):
    #     if leader[i] != "":
    #         if traci.vehicle.getNextStops(leader[i]) != ():
    #             traci.vehicle.setStop(leader[i], traci.vehicle.getRoadID(leader[i]), 90, duration=0)
    #
    #         traci.vehicle.slowDown(leader[i], action[i], fct_ia_step_runner.step_duration)
    flow_list_set_action(action, list_flow0, 0)
    flow_list_set_action(action, list_flow1, 4)
    flow_list_set_action(action, list_flow2, 8)
    flow_list_set_action(action, list_flow3, 12)

def acceleration_control_step(simulation_time, action, list_sorted_leader):
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]

    done = False
    fct_ia_step_runner.set_speed_mode()
    fct_ia_step_runner.set_loaded_vehicle(list_sorted_leader)

    list_flow0, list_flow1, list_flow2, list_flow3 = get_flow_list()

    # action[0] = 2
    action = acceleration_control(action, list_flow0, list_flow1, list_flow2, list_flow3)
    set_action(action,  list_flow0, list_flow1, list_flow2, list_flow3)

    # sfty.security_safe_mode_new_approach(list_sorted_leader, action, fct_ia_step_runner.step_duration)
    traci.simulationStep()
    refresh_list_vehicle(list_sorted_leader)

    reward = -traci.simulation.getCollidingVehiclesNumber() * 1000 + traci.simulation.getArrivedNumber() * 100 + \
             traci.simulation.getDepartedNumber() * 50 - traci.simulation.getMinExpectedNumber() + statistique.vitesseMoyenne(flow_edges) * 2 #(100*traci.simulation.getMinExpectedNumber() if traci.simulation.getMinExpectedNumber() > 10 else 3)

    if traci.simulation.getTime() > simulation_time:
        done = True

    state_next = observation_new_approach(list_sorted_leader)

    return state_next, reward, done
