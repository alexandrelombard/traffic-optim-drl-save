import control_acceleration_logic.IA_cross_acceleration_logic.ia_cross_speed_control.class_sumo_env_new_approach as sumo_class_new_approach
from stable_baselines3.common.env_checker import check_env

from stable_baselines3 import SAC

def sac_deep_learning(display, simulation_time, image_size):

    env = sumo_class_new_approach.SumoAccelEnv(display, simulation_time, image_size)
    check_env(env, warn=True)

    model = SAC("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=1500000, log_interval=1)
    model.save("sac_pendulum")




def launch_sac_model(display, simulation_time, image_size):
    env = sumo_class_new_approach.SumoAccelEnv(display, simulation_time, image_size)

    model = SAC.load("sac_pendulum")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()