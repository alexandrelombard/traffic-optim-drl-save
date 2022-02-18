import keras
import keras.layers
import keras.losses
import numpy as np
import tensorflow as tf

import right_of_way_logic.ai_right_of_way_logic.noTraficLight.myfunction as my_function


def launch_model(display, simulation_type):
    max_steps_per_episode = 1000

    # Q-Network definition
    num_actions = 8
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

    # model = build_model()
    # model_target = build_model()

    # model.load_weights("./trained_model")
    # model_target.load_weights("./trained_model_target")

    model = keras.models.load_model('./trained_model/save_model_trained_new')
    model_target = keras.models.load_model('./trained_model/save_model_target_trained_new')

    model.summary()
    model_target.summary()

    episode_count = 0
    frame_count = 0

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

            # Predict action Q-values
            # From environment state
            state_tensor = tf.convert_to_tensor(state)
            state_tensor = tf.expand_dims(state_tensor, 0)
            action_probs = model(state_tensor, training=False)
            # Take best action
            action = np.array(action_probs)[0]
            for i in range(0, len(action)):
                if action[i] < 0.5:

                    action[i] = 0
                else:
                    action[i] = 1

            # Apply the sampled action in our environment
            print(f"Applying action: {action}")
            state_next, reward, done, average_waiting_time, cumulated_waiting_time, emission_of_co2, average_speed = \
                my_function.step_no_traffic_light_v2(action, simulation_type)
            state_next = np.array(state_next)

            episode_reward += reward
            episode_cumulated_waiting_time += cumulated_waiting_time
            episode_emission_co2 += emission_of_co2
            episode_average_speed += average_speed
            episode_average_waiting_time += average_waiting_time

            # Save state
            state = state_next

            if done:
                break
        # statistique.csvFileWriter('stat_model_trained/stat_entrainer_2M.csv',
        #                           episode_average_waiting_time / nb_frame_episode,
        #                           episode_cumulated_waiting_time / nb_frame_episode,
        #                           episode_average_speed / nb_frame_episode,
        #                           episode_emission_co2 / nb_frame_episode,
        #                           episode_count)
        print("episode reward:", episode_reward, "at episode", episode_count, "frame count", frame_count)

        episode_count += 1
