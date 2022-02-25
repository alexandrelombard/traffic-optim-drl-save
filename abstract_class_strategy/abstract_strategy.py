import os
import sys
from abc import ABC, abstractmethod

import project

import sumo_utils # noqa


class ROWStrategy(ABC):
    @property
    def display(self):
        return bool

    def start_simulation(self, display):
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

    @abstractmethod
    def reset(self):
        ...

    @abstractmethod
    def set_loaded_vehicle(self):
        ...

    def set_speed_mode(self):
        ...

    @abstractmethod
    def perception(self):
        ...

    @abstractmethod
    def set_action(self):
        ...

    @abstractmethod
    def control_strategy(self):
        ...

    @abstractmethod
    def step_few(self):
        ...

    @abstractmethod
    def run_simulation(self):
        ...

    @abstractmethod
    def leader_arriver(self):
        ...

    def close_sumo(self):
        traci.close()
