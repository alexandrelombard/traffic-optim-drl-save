import numpy as np
import gym
from gym import spaces
import right_of_way_logic.IA_right_of_way_logic.noTraficLight.myfunction as function
import traci


class SumoDqnEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, display, simulation_time, image_size, reward_type='waiting_time', coef=100, flow1=300, flow2=300,
                 flow3=300, flow4=300):

        super(SumoDqnEnv, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.image_size = image_size
        self.display = display
        self.simulation_time = simulation_time
        self.reward_type = reward_type
        self.coef = coef
        self.flow1 = flow1
        self.flow2 = flow2
        self.flow3 = flow3
        self.flow4 = flow4

        n_action = 256
        self.action_space = spaces.Discrete(n_action)
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(image_size, image_size, 3), dtype=np.uint8)

        function.start_simulation(self.display)

    def step(self, action):

        state_next, reward, done, \
            average_waiting_time, cumulated_waiting_time, \
            emission_of_co2, average_speed, evacuated_vehicle,\
            nb_collision = function.step_no_traffic_light_v2(function.trad_action(action), self.simulation_time,
                                                             self.image_size, self.reward_type, self.coef)

        observation = state_next
        info = {}
        return observation, reward, done, info

    def reset(self):
        return function.reset(self.display, self.image_size)  # reward, done, info can't be included

    def render(self, mode='human'):
        pass

    def close(self):
        traci.close()
