import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import control_acceleration_logic.IA_cross_acceleration_logic.ia_cross_speed_control.ia_step_runner as env

def ddpg_launcher(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4):

    upper_bound = 17
    lower_bound = 1
    # To store reward history of each episode
    ep_reward_list = []
    # To store average reward history of last few episodes
    avg_reward_list = []

    def policy(state):

        sampled_actions = tf.squeeze(actor_model(state))
        legal_action = np.clip(sampled_actions, lower_bound, upper_bound)

        return [np.squeeze(legal_action)]

    actor_model = tf.keras.models.load_model('./save_actor_model')
    target_actor = tf.keras.models.load_model('./save_target_actor')
    total_episodes = 5

    env.start_simulation(display)
    for ep in range(total_episodes):

        prev_state = env.reset(display)
        episodic_reward = 0

        while True:
            # Uncomment this to see the Actor in action
            # But not in a python notebook.
            # env.render()

            tf_prev_state = tf.expand_dims(tf.convert_to_tensor(prev_state), 0)
            action = policy(tf_prev_state)
            print(action)
            # Recieve state and reward from environment.
            state, reward, done = env.step_few(simulation_time, action[0], image_size)

            episodic_reward += reward

            # End this episode when `done` is True
            if done:
                break

            prev_state = state

        ep_reward_list.append(episodic_reward)

        # Mean of last 40 episodes
        avg_reward = np.mean(ep_reward_list[-40:])
        print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))
        avg_reward_list.append(avg_reward)

    # Plotting graph
    # Episodes versus Avg. Rewards
    plt.plot(avg_reward_list)
    plt.xlabel("Episode")
    plt.ylabel("Avg. Epsiodic Reward")
    plt.show()
