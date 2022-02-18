from __future__ import absolute_import
from __future__ import print_function

import os
import sys

import keras

import \
    control_acceleration_logic.ai_cross_acceleration_logic.ia_cross_speed_control.sumo_ppo_baselines as sumo_ppo_baselines
import control_acceleration_logic.ai_cross_acceleration_logic.ia_cross_speed_control.sumo_sac_baselines as sac
import control_acceleration_logic.cross_acceleration_control.step_runner as acceleration_control_idm
import control_acceleration_logic.cross_acceleration_control.step_runner2 as acceleration_control_idm2
import project
import right_of_way_logic.ai_right_of_way_logic.IAsimpleCross.sumo_dqn_simple as sumo_dqn_simple
import \
    right_of_way_logic.ai_right_of_way_logic.IAsimpleCross.sumo_dqn_with_trained_model_simple as sumo_dqn_trained_simple
import right_of_way_logic.ai_right_of_way_logic.noTraficLight.myfunction as mfct
import right_of_way_logic.ai_right_of_way_logic.noTraficLight.sumo_dqn_2 as sumo_dqn_2
import right_of_way_logic.ai_right_of_way_logic.noTraficLight.sumo_dqn_with_trained_model_2 as sumo_dqn_trained_2
import right_of_way_logic.heuristic_strategies.dcp.dcp_step_method as dcp
import right_of_way_logic.heuristic_strategies.first_in_first_out.cross_first_in_first_out as crossFirstInFirstOff
import right_of_way_logic.heuristic_strategies.fisrt_come_first_served.first_come_first_served_strict as fcfs
import trafic_light_logic.traficLight.sumo_dqn_trafic_light as sumo_dqn_light
import trafic_light_logic.traficLight.sumo_dqn_with_trained_model_trafic_light as sumo_trained_light
from tools import statistics
from trafic_light_logic.trafic_light_optimal_cycle import trafic_light_optimal_cycle as tf_cycle_opti

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

"""Class environment .

This class is used to launch a  simulation with the wished configuration.

Parameters
----------
mode : int
    mode 1, 2 or 3 
display : bool
    boolean which indicate if the simulation should be displayed.
training : bool
    boolean which indicate if we want to launch a reinforcement learning.
simulationTime : int 
    indicate the wished simulation time per episode in second.
Returns
-------


"""


class Environment:

    def __init__(self, mode, display, training, simulation_time, image_size,
                 reward_type="nb_vehicle", coef=1, flow1=300, flow2=300, flow3=300, flow4=300):

        self.mode = mode
        self.display = display
        self.training = training
        self.simulation_time = simulation_time
        self.image_size = image_size
        self.reward_type = reward_type
        self.coef = coef
        self.flow1 = flow1
        self.flow2 = flow2
        self.flow3 = flow3
        self.flow4 = flow4

    # Mode 3 = mode feu de circulation
    # Mode 2 = mode intersection sans feu IA
    # Mode 1 = mode intersection sans feu premier arriver premier premier sortie

    """launch simulation function.

    This function launch the simulation with the parameters of the class
    """

    def launch_simulation(self):

        if self.mode == 1:
            crossFirstInFirstOff.launch_cross_first_in_first_off(self.display, self.simulation_time, self.reward_type,
                                                                 self.flow1, self.flow2, self.flow3, self.flow4)

        if self.mode == 2:
            if self.training:
                sumo_dqn_2.deep_learning(self.display, self.simulation_time, self.image_size, self.reward_type,
                                         self.coef, self.flow1, self.flow2, self.flow3, self.flow4)

            if not self.training:
                sumo_dqn_trained_2.launch_model(self.display, self.simulation_time, self.image_size, self.reward_type,
                                                self.coef, self.flow1, self.flow2, self.flow3, self.flow4)
        if self.mode == 3:
            if self.training:
                sumo_dqn_light.launch_traffic_light_simulation(self.display, self.simulation_time, self.reward_type,
                                                               self.coef, self.flow1, self.flow2, self.flow3,
                                                               self.flow4)
            if not self.training:
                sumo_trained_light.launch_traffic_light_trained_simulation(self.display, self.simulation_time,
                                                                           self.reward_type, self.coef, self.flow1,
                                                                           self.flow2, self.flow3, self.flow4)

        if self.mode == 4:
            tf_cycle_opti.launch_optimal_trafic_light_cycle(self.display, self.simulation_time, self.reward_type,
                                                            self.coef, self.flow1, self.flow2, self.flow3, self.flow4)

        if self.mode == 5:
            if self.training:
                sumo_dqn_simple.deep_learning(self.display, self.simulation_time, self.reward_type, self.coef,
                                              self.flow1, self.flow2, self.flow3, self.flow4)
            if not self.training:
                sumo_dqn_trained_simple.launch_model(self.display, self.simulation_time, self.reward_type, self.coef,
                                                     self.flow1, self.flow2, self.flow3, self.flow4)

        if self.mode == 6:
            fcfs.launch_cross_first_come_first_served(self.display, self.simulation_time, self.reward_type,
                                                      self.flow1, self.flow2, self.flow3, self.flow4)

        if self.mode == 7:
            if self.training:
                sumo_ppo_baselines.deep_learning_ppo(self.display, self.simulation_time, self.image_size,
                                                     self.reward_type, self.coef, self.flow1,
                                                     self.flow2, self.flow3, self.flow4)
            if not self.training:
                sumo_ppo_baselines.launch_ppo(self.display, self.simulation_time, self.image_size, self.reward_type,
                                              self.coef, self.flow1, self.flow2, self.flow3, self.flow4)

        if self.mode == 8:
            acceleration_control_idm.run_simulation(self.display, self.simulation_time)

        if self.mode == 9:
            dcp.launch_dcp_fcfs(self.display, self.simulation_time, self.reward_type,
                                self.flow1, self.flow2, self.flow3, self.flow4)

        if self.mode == 10:
            acceleration_control_idm2.run_simulation(self.display, self.simulation_time)

        if self.mode == 11:
            if self.training:
                sac.sac_deep_learning(self.display, self.simulation_time, self.image_size)
            if not self.training:
                sac.launch_sac_model(self.display, self.simulation_time, self.image_size)

    def simulation_benchmark(self, display, nb_episode, time_per_episode, reward_type, flow1, flow2, flow3, flow4):

        list_stat_fifo = []
        list_stat_dqn_2 = []
        list_stat_tf_opti_cycle = []
        list_stat_fcfs = []
        list_stat_dcp = []

        list_stat_fifo_quart = []
        list_stat_dqn_2_quart = []
        list_stat_tf_opti_cycle_quart = []
        list_stat_fcfs_quart = []
        list_stat_dcp_quart = []

        dqn_method = mfct.Step_runner_Dqn(keras.models.load_model(project.models_dir + 'save_model'), 50,
                                          "waiting_time", 100, security=True)

        tf_method = tf_cycle_opti.Step_runner_tf(reward_type, 100)

        fcfs_method = fcfs.Step_runner_fcfs([])

        dcp_method = dcp.Step_runner_dcp([])

        for i in range(1, 8):
            # list_stat_fifo.append(crossFirstInFirstOff.launch_simulation_statistique(display, nb_episode,
            #                                                                          time_per_episode,
            #                                                                          reward_type, flow1*i, flow2*i,
            #                                                                          flow3*i, flow4*i))

            list_stat_fifo.append(
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0])
            list_stat_fifo_quart.append(
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0])

            list_1, list_dqn_seuil = statistics.launch_statistique(display, nb_episode,
                                                                   time_per_episode,
                                                                   flow1 * i, flow2 * i,
                                                                   flow3 * i, flow4 * i, dqn_method)

            list_stat_dqn_2.append(list_1)
            list_stat_dqn_2_quart.append(list_dqn_seuil)

            list_1, list_tf_seuil = statistics.launch_statistique(display, nb_episode,
                                                                  time_per_episode,
                                                                  flow1 * i, flow2 * i,
                                                                  flow3 * i, flow4 * i, tf_method)

            list_stat_tf_opti_cycle.append(list_1)
            list_stat_tf_opti_cycle_quart.append(list_tf_seuil)

            list_1, list_fcfs_seuil = statistics.launch_statistique(display, nb_episode,
                                                                    time_per_episode,
                                                                    flow1 * i, flow2 * i,
                                                                    flow3 * i, flow4 * i, fcfs_method)

            list_stat_fcfs.append(list_1)
            list_stat_fcfs_quart.append(list_fcfs_seuil)

            list_1, list_dcp_seuil = statistics.launch_statistique(display, nb_episode,
                                                                   time_per_episode,
                                                                   flow1 * i, flow2 * i,
                                                                   flow3 * i, flow4 * i, dcp_method)

            list_stat_dcp.append(list_1)
            list_stat_dcp_quart.append(list_dcp_seuil)

        # for w in range(0, len(list_stat_tf_opti_cycle)):
        #
        #     self.statistic_printer("dqn", list_stat_dqn_2, w)
        #
        #     self.statistic_printer("fifo", list_stat_fifo, w)
        #
        #     self.statistic_printer("feu", list_stat_tf_opti_cycle, w)
        #
        #     self.statistic_printer("fcfs", list_stat_fcfs, w)
        #
        #     self.statistic_printer("dcp", list_stat_dcp, w)

        # statistique.benchmark_csv_maker("benchmark", list_stat_fifo, list_stat_dqn_2, list_stat_tf_opti_cycle, list_stat_fcfs, list_stat_dcp)
        # statistique.benchmark_csv_maker("benchmark_without_q1q2", list_stat_fifo_quart, list_stat_dqn_2_quart, list_stat_tf_opti_cycle_quart, list_stat_fcfs_quart, list_stat_dcp_quart)
        #
        statistics.benchmark_csv_init("benchmark.csv")
        statistics.benchmark_csv_line_writer("benchmark.csv", "fifo", list_stat_fifo)
        statistics.benchmark_csv_line_writer("benchmark.csv", "dqn", list_stat_dqn_2)
        statistics.benchmark_csv_line_writer("benchmark.csv", "Feu", list_stat_tf_opti_cycle)
        statistics.benchmark_csv_line_writer("benchmark.csv", "fcfs", list_stat_fcfs)
        statistics.benchmark_csv_line_writer("benchmark.csv", "dcp", list_stat_dcp)

        statistics.benchmark_csv_init("benchmark_without_q1q3.csv")
        statistics.benchmark_csv_line_writer("benchmark_without_q1q3.csv", "fifo", list_stat_fifo_quart)
        statistics.benchmark_csv_line_writer("benchmark_without_q1q3.csv", "dqn", list_stat_dqn_2_quart)
        statistics.benchmark_csv_line_writer("benchmark_without_q1q3.csv", "Feu", list_stat_tf_opti_cycle_quart)
        statistics.benchmark_csv_line_writer("benchmark_without_q1q3.csv", "fcfs", list_stat_fcfs_quart)
        statistics.benchmark_csv_line_writer("benchmark_without_q1q3.csv", "dcp", list_stat_dcp_quart)

    def statistic_printer(self, method_name, list_stat, index):
        print("\n\n")
        print("Voici la moyenne des statisques pour le " + method_name, list_stat[index][0],
              "episode avec des flow de :",
              list_stat[index][12], "vehicule par heure.")

        print("nombre de collision : ", list_stat[index][1])
        print("temp d'attente moyen : ", list_stat[index][2])
        print("temps d'attente cumulé :", list_stat[index][4])
        print("vitesse moyenne :", list_stat[index][6])
        print("emission de co2 moyenne :", list_stat[index][8])
        print("evacuated vehicle average : ", list_stat[index][10])


instance = Environment(mode=1, display=True, training=True, simulation_time=1000, image_size=50,
                       reward_type="mix_vehicle_time", coef=10, flow1=500, flow2=500, flow3=500, flow4=500)
#
# instance.launch_simulation()
instance.simulation_benchmark(display=False, nb_episode=6, time_per_episode=1000, reward_type="waiting_time", flow1=100,
                              flow2=100, flow3=100, flow4=100)
