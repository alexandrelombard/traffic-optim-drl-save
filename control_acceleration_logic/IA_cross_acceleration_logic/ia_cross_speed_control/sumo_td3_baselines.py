import numpy as np

from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
import control_acceleration_logic.IA_cross_acceleration_logic.ia_cross_speed_control.class_sumo_env as sumo_class_env
from stable_baselines3.common.env_checker import check_env


def deep_learning_td3():
    env = sumo_class_env.SumoAccelEnv(False, 1000, 40)
    check_env(env, warn=True)

    # The noise objects for TD3
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    policy_kwargs = dict(net_arch=[512, 256, 128])

    # model = TD3("MlpPolicy", env, policy_kwargs=policy_kwargs, action_noise=action_noise, verbose=1)
    model = TD3("MlpPolicy", env, action_noise=action_noise, verbose=1)
    model.learn(total_timesteps=350000, log_interval=3)
    model.save("td3_pendulum")
    env = model.get_env()

    del model  # remove to demonstrate saving and loading

    model = TD3.load("td3_pendulum")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        print("print run")
        obs, rewards, dones, info = env.step(action)
        env.render()


def launch_td3():

    env = sumo_class_env.SumoAccelEnv(True, 1000, 40)
    model = TD3.load("td3_pendulum")

    obs = env.reset()
    reward_ep = 0
    while True:
        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        reward_ep += rewards
        if done:
            print("reward ep :", reward_ep)
            reward_ep = 0
            obs = env.reset()
        env.render()

