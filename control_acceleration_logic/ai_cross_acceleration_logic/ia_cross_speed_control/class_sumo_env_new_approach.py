import numpy as np
import gym
from gym import spaces
import control_acceleration_logic.ai_cross_acceleration_logic.ia_cross_speed_control.ai_step_runner_new_approach as function_env
import traci


class SumoAccelEnv(gym.Env):
  """
  Custom Environment that follows gym interface.
  This is a simple env where the agent must learn to go always left.
  """
  # Because of google colab, we cannot implement the GUI ('human' render mode)
  metadata = {'render.modes': ['console']}
  # Define constants for clearer code

  def __init__(self, display, simulation_time, image_size, reward_type=0, coef=0, flow1=0, flow2=0, flow3=0, flow4=0):
    super(SumoAccelEnv, self).__init__()

    # Size of the image-grid
    self.image_size = image_size
    # bool display
    self.display = display
    # time max per episode
    self.simulationTime = simulation_time
    # liste containing all the vehicle that have not passed the intersection yet
    self.list_sorted_leader = []

    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions, we have two: left and right
    n_actions = 16
    self.action_space = spaces.Box(-1, 1, (n_actions, ), dtype=np.float32)

    self.observation_space = spaces.Box(-1000, 100, shape=(68,))

    function_env.start_simulation(self.display)

  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array)
    """
    self.list_sorted_leader =[]
    return function_env.reset(self.display)

  def step(self, action):
    # Optionally we can pass additional info, we are not using that for now
    info = {}
    action = self.acceleration_control(action)
    # action = self.speed_control(action)
    # action = [17, 17, 17, 17, 17, 17, 17, 17]
    obs, reward, done = function_env.acceleration_control_step(self.simulationTime, action, self.list_sorted_leader)
    return obs, reward, done, info

  def render(self, mode='console'):
    pass

  def close(self):
    traci.close()

  def acceleration_control(self, action):
    add = [-1]*16
    action = (action * 3) + add
    return action

  def speed_control(self, action):
    add = [8.5]*16
    action = (action * 8.5) + add
    return action