import keras
import keras.layers
import keras.losses

import tensorflow as tf
import numpy as np

import traffic_light_logic.traffic_light.myfunction_traffic_light as my_function


def launch_traffic_light_simulation(display, simulation_time, reward_type, coef, flow1, flow2, flow3, flow4):
    # Hyper-parameters definition
    num_iterations: int = 20000  # @param {type:"integer"}

    initial_collect_steps: int = 100  # @param {type:"integer"}
    collect_steps_per_iteration: int = 1  # @param {type:"integer"}
    replay_buffer_max_length: int = 100000  # @param {type:"integer"}

    batch_size: int = 64  # @param {type:"integer"}
    learning_rate = 1e-4  # @param {type:"number"}
    log_interval: int = 200  # @param {type:"integer"}

    gamma = 0.999

    epsilon = 1.0
    epsilon_min = 0.1
    epsilon_max = 1.0
    epsilon_interval = (epsilon_max - epsilon_min)

    num_eval_episodes: int = 10  # @param {type:"integer"}
    eval_interval: int = 1000  # @param {type:"integer"}

    max_steps_per_episode = 1000

    # Q-Network definition
    num_actions = 2
    input_shape = (40, 40, 3, )         # TODO Adjust with appropriate shape
    action_shape = (num_actions, )

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
    epsilon_random_frames = 50000
    epsilon_greedy_frames = 1000000.0
    update_after_actions = 4
    update_target_network = 10000
    final_reward = 50
    loss_function = keras.losses.Huber()

    my_function.start_simulation(display)
    while True:  # Run until solved
        state = np.array(my_function.reset(display))       # TODO Create the reset function (restart sim)
        episode_reward = 0

        for timestep in range(1, max_steps_per_episode):
            # env.render(); Adding this line would show the attempts
            # of the agent in a pop up window.
            frame_count += 1

            # Use epsilon-greedy for exploration
            if frame_count < epsilon_random_frames or epsilon > np.random.rand(1)[0]:
                # Take random action
                action = np.random.choice(num_actions)
            else:
                # Predict action Q-values
                # From environment state
                state_tensor = tf.convert_to_tensor(state)
                state_tensor = tf.expand_dims(state_tensor, 0)
                action_probs = model(state_tensor, training=False)
                # Take best action
                action = tf.argmax(action_probs[0]).numpy()

            # Decay probability of taking random action
            epsilon -= epsilon_interval / epsilon_greedy_frames
            epsilon = max(epsilon, epsilon_min)

            # Apply the sampled action in our environment
            # print(f"Applying action: {action}")
            state_next, reward, done = my_function.step_few(action, simulation_time, reward_type, coef)
            state_next = np.array(state_next)

            episode_reward += reward

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
                masks = tf.one_hot(action_sample, num_actions)

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
        print("running reward:", running_reward, "at episode", episode_count, "frame count", frame_count)
        # Update running reward to check condition for solving
        episode_reward_history.append(episode_reward)
        if len(episode_reward_history) > 100:
            del episode_reward_history[:1]
        running_reward = np.mean(episode_reward_history)

        episode_count += 1

        if frame_count % 1000 < 100:  # Condition to consider the task solved
            print("Solved at episode {}!".format(episode_count))

            model.save("./trafic_Light_Model")
            model_target.save("./trafic_light_model_target")

        if frame_count > 1050000:
            print("Solved at episode {}!".format(episode_count))

            model.save("./trafic_Light_Model")
            model_target.save("./trafic_light_model_target")
