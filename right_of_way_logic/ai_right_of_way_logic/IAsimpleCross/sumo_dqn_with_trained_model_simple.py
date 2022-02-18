import keras
import keras.layers
import keras.losses
import numpy as np
import tensorflow as tf
from tools import simulation_parameter_tools as simu_tools

import right_of_way_logic.ai_right_of_way_logic.IAsimpleCross.crossSimple as env


def launch_model(display, simulation_type, reward_type, coef, flow1, flow2, flow3, flow4):
    max_steps_per_episode = 1000

    # Q-Network definition
    num_actions = 16
    input_shape = (3, 3, 3, )         # TODO Adjust with appropriate shape
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

    model = keras.models.load_model('./model_simple/save_model_simple')
    model_target = keras.models.load_model('./model_simple/save_model_target_simple')

    model.summary()
    model_target.summary()

    episode_count = 0
    frame_count = 0

    env.start_simulation(display)

    while True:  # Run until solved
        simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4)
        state = np.array(env.reset(display))       # TODO Create the reset function (restart sim)
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
            action = tf.argmax(action_probs[0]).numpy()
            action_binaire = env.trad_action(action)
            # Apply the sampled action in our environment
            print(f"Applying action: {action}")
            state_next, reward, done, average_waiting_time, cumulated_waiting_time, emission_of_co2, average_speed = \
                env.step_few(action_binaire, simulation_type, reward_type, coef)
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
