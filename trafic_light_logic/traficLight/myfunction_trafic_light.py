from __future__ import absolute_import
from __future__ import print_function

import project

import numpy as np
import matplotlib.pyplot
from tools import statistics

import sumo_utils # noqa


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
    traci.start([sumo_binary, "-c", project.resources_dir + "traficLightNet/NetworkNoTraficLight.sumocfg",
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
    traci.start([sumo_binary, "-c", project.resources_dir + "traficLightNet/NetworkNoTraficLight.sumocfg",
                 "--tripinfo-output", project.resources_dir + "tripinfo.xml", "--random", "--collision.check-junctions"])
    # runningVehicleID = traci.vehicle.getIDList()
    # countRunningVehicle = traci.vehicle.getIDCount()
    # return imageConstruct(runningVehicleID, runningVehicleID, 40)
    n = 40
    return np.zeros((n, n, 3), dtype=int)


def step_few(action, simulation_time, reward_type, coef):
    """run the simulation step after step.

    this function is used to run the simulation with the given actions.

    Parameters
    ----------
    action : int
        0 for one phase 1 for the other
    simulation_time : int
        the wished simulation time in second
    reward_type : string
        type of the reward
    coef : float
        coefficient used to calculate the reward

    Returns
    -------
    state_next : NArray
        image of the current step
    reward : int
        current reward
    done : int
        0 or 1 indicating if the simulation should be ended

    """
    reward = 0
    done = 0

    change_traffic_phase(action)

    for i in range(0, 2):
        traci.simulationStep()
        reward += statistique.reward_calculation(reward_type, coef)

    running_vehicle_id = traci.vehicle.getIDList()
    count_running_vehicle = traci.vehicle.getIDCount()
    state_next = image_construct(running_vehicle_id, count_running_vehicle, 40)

    if traci.simulation.getTime() > simulation_time:
        done = 1

    return state_next, reward, done


def change_traffic_phase(phase):
    """This function is used to change the phase of the traffic light.

    this function is used to change the traffic light phase to the given one.

    Parameters
    ----------
    phase : int
        0 or 1

    Returns
    -------
    negativeReward : int
        it's a negative reward for each impossible action

    """
    if phase == 0:
        if traci.trafficlight.getPhase("2") != 1 and traci.trafficlight.getPhase("2") != 2:
            traci.trafficlight.setPhase("2", 1)
    if phase == 1:
        if traci.trafficlight.getPhase("2") != 3 and traci.trafficlight.getPhase("2") != 4:
            traci.trafficlight.setPhase("2", 3)


def image_construct(running_vehicle_id, count_running_vehicle, n, display=False):
    """build a representation of the current state of the simulation.

    this function is used to to build an N by N image of a state of the simulation.
    each on pixel represent a vehicle.
    the intensity of the pixel represent the speed and the color of the pixel their destination

    Parameters
    ----------
    running_vehicle_id : list
        containing all the vehicles name on the simulation
    count_running_vehicle : int
        number of vehicle in the simulation
    n : int
        size of the image.
    display : bool
        indicate if the image should be displayed


    Returns
    -------
    image : Ndarray
        representation of the image

    """
    image = np.zeros((n, n, 3), dtype=int)
    # initialise the image :

    for i in range(0, count_running_vehicle):

        position = traci.vehicle.getPosition(running_vehicle_id[i])
        # intensity of the pixel depending on the speed :
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
                image[pixel_pos_x][pixel_pos_y][0] = 40+round(215 * (vehicle_speed/v_max_lane))
                # print(image[pixel_pos_x][pixel_pos_y][0])
            elif route_list[len(route_list)-1] == "2to4":
                image[pixel_pos_x][pixel_pos_y][1] = 40+round(215 * (vehicle_speed/v_max_lane))
                # print(image[pixel_pos_x][pixel_pos_y][1])
            elif route_list[len(route_list)-1] == "2to5":
                image[pixel_pos_x][pixel_pos_y][2] = 40+round(215 * (vehicle_speed/v_max_lane))
                # print(image[pixel_pos_x][pixel_pos_y][2])
            elif route_list[len(route_list)-1] == "2to3":
                image[pixel_pos_x][pixel_pos_y][0] = 40+round(215 * (vehicle_speed/v_max_lane))
                image[pixel_pos_x][pixel_pos_y][1] = 40+round(215 * (vehicle_speed/v_max_lane))

    # printImage(image, N)
    if display:
        matplotlib.pyplot.imshow(image)
        matplotlib.pyplot.show()

    return image
