import csv

import keras
import keras.layers
import keras.losses

import tensorflow as tf
import numpy as np


import right_of_way_logic.IA_right_of_way_logic.noTraficLight.myfunction as my_function


# Hyper-parameters definition
from Tools import statistique
import time


def deep_learning(display, simulation_time):
    print("DEEP LEARNING")
    num_iterations: int = 20000  # @param {type:"integer"}

    initial_collect_steps: int = 100  # @param {type:"integer"}
    collect_steps_per_iteration: int = 1  # @param {type:"integer"}
    replay_buffer_max_length: int = 100000  # @param {type:"integer"}

    batch_size: int = 64  # @param {type:"integer"}
    learning_rate = 1e-4  # @param {type:"number"}
    log_interval: int = 200  # @param {type:"integer"}

    gamma = 0.999

    epsilon = 1.0
    epsilon_min = 0.01
    epsilon_max = 1.0
    epsilon_interval = (epsilon_max - epsilon_min)

    num_eval_episodes: int = 10  # @param {type:"integer"}
    eval_interval: int = 1000  # @param {type:"integer"}

    max_steps_per_episode = 1000

    # Q-Network definition
    num_actions = 8
    input_shape = (40, 40, 3, )         # TODO Adjust with appropriate shape
    action_shape = (num_actions,)

    def build_model():
        return keras.Sequential([
            keras.layers.Input(shape=input_shape),
            keras.layers.Conv2D(filters=32, kernel_size=8, strides=4, activation='relu'),
            keras.layers.Conv2D(filters=64, kernel_size=4, strides=2, activation='relu'),
            keras.layers.Conv2D(filters=64, kernel_size=1, strides=1, activation='relu'),
            keras.layers.Flatten(),
            keras.layers.Dense(64, activation='relu'),
            keras.layers.Dense(num_actions, activation='linear')
        ])

    model = build_model()
    model_target = build_model()

    optimizer = tf.keras.optimizers.Adam(learning_rate=learning_rate)
    train_step_counter = tf.Variable(0)

    # Experience replay buffers
    action_history = []
    state_history = []
    state_next_history = []
    rewards_history = []
    done_history = []
    episode_reward_history = []
    running_reward = 0
    episode_count = 0
    frame_count = 0
    max_memory_length = 100000

    # Loop
    epsilon_random_frames = 100000
    epsilon_greedy_frames = 10000000.0
    update_after_actions = 4
    update_target_network = 10000
    final_reward = 200
    loss_function = keras.losses.Huber()

    time_stamp = str(time.time())
    time_stamp = 'stat/statistique' + time_stamp+'.csv'
    with open(time_stamp, 'w', newline='') as fichiercsv:
        writer = csv.writer(fichiercsv)
        writer.writerow(['Average waiting time', 'Cumulated waiting time', 'average speed', 'emission of Co2'])
    fichiercsv.close()
    my_function.start_simulation(display)
    while True:  # Run until solved
        state = np.array(my_function.reset(display))       # TODO Create the reset function (restart sim)
        episode_reward = 0
        nb_frame_episode = 0
        episode_emission_co2 = 0
        episode_cumulated_waiting_time = 0
        episode_average_speed = 0
        episode_average_waiting_time = 0

        for time_step in range(1, max_steps_per_episode):
            # env.render(); Adding this line would show the attempts
            # of the agent in a pop up window.
            frame_count += 1
            nb_frame_episode += 1

            # Use epsilon-greedy for exploration
            if frame_count < epsilon_random_frames or epsilon > np.random.rand(1)[0]:
                # Take random action
                # action = np.random.choice(num_actions)
                action = [0, 0, 0, 0, 0, 0, 0, 0]
                for i in range(0, len(action)):
                    action[i] = np.random.randint(0, 2)

            else:
                # Predict action Q-values
                # From environment state
                state_tensor = tf.convert_to_tensor(state)
                state_tensor = tf.expand_dims(state_tensor, 0)
                action_probs = model(state_tensor, training=False)
                # Take best action
                # action = tf.argmax(action_probs[0]).numpy()
                action = np.array(action_probs)[0]
                for i in range(0, len(action)):
                    if action[i] < 0.5:

                        action[i] = 0
                    else:
                        action[i] = 1

            # Decay probability of taking random action
            epsilon -= epsilon_interval / epsilon_greedy_frames
            epsilon = max(epsilon, epsilon_min)

            # Apply the sampled action in our environment
            print(action)
            state_next, reward, done, average_waiting_time, cumulated_waiting_time, emission_of_co2, average_speed = \
                my_function.step_no_traffic_light_v2(action, simulation_time)
            state_next = np.array(state_next)

            episode_reward += reward
            episode_cumulated_waiting_time += cumulated_waiting_time
            episode_emission_co2 += emission_of_co2
            episode_average_speed += average_speed
            episode_average_waiting_time += average_waiting_time

            # Save actions and states in replay buffer
            action_history.append(action)
            state_history.append(state)
            state_next_history.append(state_next)
            done_history.append(done)
            rewards_history.append(reward)
            state = state_next

            # Update every fourth frame and once batch size is over 32
            if frame_count % update_after_actions == 0 and len(done_history) > batch_size:

                # Get indices of samples for replay buffers
                indices = np.random.choice(range(len(done_history)), size=batch_size)

                # Using list comprehension to sample from replay buffer
                state_sample = np.array([state_history[i] for i in indices])
                state_next_sample = np.array([state_next_history[i] for i in indices])
                rewards_sample = [rewards_history[i] for i in indices]
                action_sample = [action_history[i] for i in indices]
                done_sample = tf.convert_to_tensor(
                    [float(done_history[i]) for i in indices]
                )

                # Build the updated Q-values for the sampled future states
                # Use the target model for stability
                future_rewards = model_target.predict(state_next_sample)
                # Q value = reward + discount factor * expected future reward
                updated_q_values = rewards_sample + gamma * tf.reduce_max(
                    future_rewards, axis=1
                )

                # If final frame set the last value to -1
                updated_q_values = updated_q_values * (1 - done_sample) - done_sample

                # Create a mask so we only calculate loss on the updated Q-values
                # masks = tf.one_hot(action_sample, num_actions)
                masks = action_sample

                with tf.GradientTape() as tape:
                    # Train the model on the states and updated Q-values
                    q_values = model(state_sample)

                    # Apply the masks to the Q-values to get the Q-value for action taken
                    q_action = tf.reduce_sum(tf.multiply(q_values, masks), axis=1)
                    # Calculate loss between new Q-value and old Q-value
                    loss = loss_function(updated_q_values, q_action)

                # Backpropagation
                grads = tape.gradient(loss, model.trainable_variables)
                optimizer.apply_gradients(zip(grads, model.trainable_variables))

            if frame_count % update_target_network == 0:
                # update the the target network with new weights
                model_target.set_weights(model.get_weights())
                # Log details
                template = "running reward: {:.2f} at episode {}, frame count {}"
                print(template.format(running_reward, episode_count, frame_count))

            # Limit the state and reward history
            if len(rewards_history) > max_memory_length:
                del rewards_history[:1]
                del state_history[:1]
                del state_next_history[:1]
                del action_history[:1]
                del done_history[:1]

            if done:
                break
        statistique.csvFileWriter(time_stamp, episode_average_waiting_time / nb_frame_episode,
                                  episode_cumulated_waiting_time / nb_frame_episode,
                                  episode_average_speed / nb_frame_episode,
                                  episode_emission_co2 / nb_frame_episode, episode_count)
        print("running reward:", running_reward, "at episode", episode_count, "frame count", frame_count)
        # Update running reward to check condition for solving
        episode_reward_history.append(episode_reward)
        if len(episode_reward_history) > 100:
            del episode_reward_history[:1]
        running_reward = np.mean(episode_reward_history)

        episode_count += 1

        if frame_count % 1000 < 205:  # running_reward > final_reward:  # Condition to consider the task solved
            model.save("./save_model")
            model_target.save("./save_model_target")
            statistique.csvFilereadAndPrint(time_stamp)

        if running_reward > final_reward:
            print("Solved at episode {}!".format(episode_count))
            model.save("./save_model")
            model_target.save("./save_model_target")
            statistique.csvFilereadAndPrint(time_stamp)
            break
