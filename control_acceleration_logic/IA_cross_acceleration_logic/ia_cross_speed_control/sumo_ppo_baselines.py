from stable_baselines3 import PPO
import control_acceleration_logic.IA_cross_acceleration_logic.ia_cross_speed_control.class_sumo_env_new_approach as sumo_class_new_approach
from stable_baselines3.common.env_checker import check_env


# # Create log dir
# log_dir = "Info/"
# os.makedirs(log_dir, exist_ok=True)
#
# class PlottingCallback(BaseCallback):
#     """
#     Callback for plotting the performance in realtime.
#
#     :param verbose: (int)
#     """
#     def __init__(self, verbose=1):
#         super(PlottingCallback, self).__init__(verbose)
#         self._plot = None
#
#     def _on_step(self) -> bool:
#         # get the monitor's data
#         x, y = ts2xy(load_results(log_dir), 'timesteps')
#         if self._plot is None: # make the plot
#             plt.ion()
#             fig = plt.figure(figsize=(6,3))
#             ax = fig.add_subplot(111)
#             line, = ax.plot(x, y)
#             self._plot = (line, ax, fig)
#             plt.show()
#         else: # update and rescale the plot
#             self._plot[0].set_data(x, y)
#             self._plot[-2].relim()
#             self._plot[-2].set_xlim([self.locals["total_timesteps"] * -0.02,
#                                     self.locals["total_timesteps"] * 1.02])
#             self._plot[-2].autoscale_view(True, True, True)
#             self._plot[-1].canvas.draw()






def deep_learning_ppo(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4):



    # Parallel environments
    # env = sumo_class_env.SumoAccelEnv(display, simulation_time, 40)
    env = sumo_class_new_approach.SumoAccelEnv(display, simulation_time, image_size)
    check_env(env, warn=True)

    # env = make_vec_env(lambda: env, n_envs=1, monitor_dir=log_dir)

    policy_kwargs = dict(net_arch=[512, 256, 128])



    # model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1)
    model = PPO("MlpPolicy", env, verbose=1)
    # model = PPO.load("ppo_cartpole", env, verbose=1)

    # ploting_callback = PlottingCallback()

    model.learn(total_timesteps=100000)
    model.save("ppo_cartpole")

    del model  # remove to demonstrate saving and loading

    model = PPO.load("ppo_cartpole")

    obs = env.reset()
    env.close()
    # while True:
    #     action, _states = model.predict(obs)
    #     obs, rewards, done, info = env.step(action)
    #     env.render()


def launch_ppo(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4):

    # env = sumo_class_env.SumoAccelEnv(display, simulation_time, 40)
    env = sumo_class_new_approach.SumoAccelEnv(display, simulation_time, image_size)
    model = PPO.load("ppo_cartpole")

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