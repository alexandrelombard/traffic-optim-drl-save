import right_of_way_logic.ai_right_of_way_logic.noTraficLight.Environment_class_dqn as environment

from stable_baselines3 import DQN


def deep_learning_baselines_dqn(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4):

    env = environment.SumoDqnEnv(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4)

    # policy_kwargs = dict(net_arch=[512, 256, 128])
    model = DQN("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=100000, log_interval=4)
    model.save("dqn_cartpole")

    del model  # remove to demonstrate saving and loading

    model = DQN.load("dqn_cartpole")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()


def launch_dqn(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4):

    env = environment.SumoDqnEnv(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4)
    model = DQN.load("dqn_cartpole")

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