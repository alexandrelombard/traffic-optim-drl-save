import keras
import keras.layers
import keras.losses
import numpy as np
import tensorflow as tf
import traci

import right_of_way_logic.ai_right_of_way_logic.noTraficLight.myfunction as env
from tools import simulation_parameter_tools as simu_tools


def launch_model(display, simulation_type, image_size, reward_type, coef, flow1, flow2, flow3, flow4):
    max_steps_per_episode = 1000

    # Q-Network definition
    num_actions = 256
    input_shape = (image_size, image_size, 3, )         # TODO Adjust with appropriate shape
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

    model = keras.models.load_model('./save_model')
    model_target = keras.models.load_model('./save_model_target')

    model.summary()
    model_target.summary()

    episode_count = 0
    frame_count = 0

    env.start_simulation(display)

    while True:  # Run until solved
        simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "Net")
        state = np.array(env.reset(display, image_size))       # TODO Create the reset function (restart sim)
        episode_reward = 0
        nb_frame_episode = 0
        episode_emission_co2 = 0
        episode_cumulated_waiting_time = 0
        episode_average_speed = 0
        episode_average_waiting_time = 0
        episode_evacuated_vehicle = 0
        episode_nb_collision = 0


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
            # Apply the sampled action in our environmentS
            print(f"Applying action: {action}")
            state_next, reward, done, average_waiting_time, cumulated_waiting_time, emission_of_co2, average_speed, evacuated_vehicle, nb_collision = \
                env.step_no_traffic_light_v2(action_binaire, simulation_type, image_size, reward_type, coef, security=True)

            state_next = np.array(state_next)

            episode_reward += reward
            episode_cumulated_waiting_time += cumulated_waiting_time
            episode_emission_co2 += emission_of_co2
            episode_average_speed += average_speed
            episode_average_waiting_time += average_waiting_time
            episode_evacuated_vehicle += evacuated_vehicle
            episode_nb_collision += nb_collision

            # Save state
            state = state_next

            if done:
                break
        # statistique.csvFileWriter(time_stamp, episode_average_waiting_time/nb_frame_episode,
        #                           episode_cumulated_waiting_time/nb_frame_episode,
        #                           episode_average_speed/nb_frame_episode,
        #                           episode_emission_co2/nb_frame_episode, episode_evacuated_vehicle, episode_nb_collision, episode_reward, episode_count)
        print("episode reward:", episode_reward, "episode collision",episode_nb_collision, "episode evacuated vehicle",
              episode_evacuated_vehicle,"at episode", episode_count, "frame count", frame_count)

        episode_count += 1


def launch_statistic_dqn_2_model(display, nb_episode, time_per_episode, image_size, reward_type, coef, flow1, flow2, flow3, flow4):

    model = keras.models.load_model('./save_model')
    model_target = keras.models.load_model('./save_model_target')

    liste_waiting_time = []
    liste_cumulated_waiting_time = []
    liste_average_speed = []
    liste_emission_co2 = []
    liste_evacuated_vehicle = []

    episode_reward = 0
    episode_cumulated_waiting_time = 0
    episode_emission_co2 = 0
    episode_average_speed = 0
    episode_average_waiting_time = 0
    episode_evacuated_vehicle = 0
    episode_nb_collision = 0

    average_episode_reward = 0
    average_episode_cumulated_waiting_time = 0
    average_episode_emission_co2 = 0
    average_episode_average_speed = 0
    average_episode_average_waiting_time = 0
    average_episode_evacuated_vehicle = 0
    average_episode_nb_collision = 0


    step_count = 0

    simu_tools.xml_flow_changer(flow1, flow2, flow3, flow4, "Net")
    env.start_simulation(display)
    for i in range(nb_episode):
        state = env.reset(display, image_size)
        done = 0
        while not done:
            step_count += 1
            state_tensor = tf.convert_to_tensor(state)
            state_tensor = tf.expand_dims(state_tensor, 0)
            action_probs = model(state_tensor, training=False)

            # Take best action
            action = tf.argmax(action_probs[0]).numpy()
            action_binaire = env.trad_action(action)
            # Apply the sampled action in our environmentS
            print(f"Applying action: {action}")
            state_next, reward, done, average_waiting_time, cumulated_waiting_time, emission_of_co2, average_speed, evacuated_vehicle, nb_collision = \
                env.step_no_traffic_light_v2(action_binaire, time_per_episode, image_size, reward_type, coef,
                                             security=True)

            state_next = np.array(state_next)

            episode_reward += reward
            episode_cumulated_waiting_time += cumulated_waiting_time
            episode_emission_co2 += emission_of_co2
            episode_average_speed += average_speed
            episode_average_waiting_time += average_waiting_time
            episode_evacuated_vehicle += evacuated_vehicle
            episode_nb_collision += nb_collision

            # Save state
            state = state_next

        average_episode_reward += episode_reward
        average_episode_cumulated_waiting_time += episode_cumulated_waiting_time/step_count
        average_episode_emission_co2 += episode_emission_co2/step_count
        average_episode_average_speed += episode_average_speed/step_count
        average_episode_average_waiting_time += episode_average_waiting_time/step_count
        average_episode_evacuated_vehicle += episode_evacuated_vehicle
        average_episode_nb_collision += episode_nb_collision

        liste_waiting_time.append(episode_average_waiting_time/step_count)
        liste_cumulated_waiting_time.append(episode_cumulated_waiting_time/step_count)
        liste_average_speed.append(episode_average_speed/step_count)
        liste_emission_co2.append(episode_emission_co2/step_count)
        liste_evacuated_vehicle.append(episode_evacuated_vehicle)

        episode_reward = 0
        episode_cumulated_waiting_time = 0
        episode_emission_co2 = 0
        episode_average_speed = 0
        episode_average_waiting_time = 0
        episode_evacuated_vehicle = 0
        episode_nb_collision = 0

    traci.close()
    print("Voici la moyenne des statisques pour ", nb_episode,"episode :")
    print("nombre de collision : ", average_episode_nb_collision/nb_episode)
    print("temp d'attente moyen : ", average_episode_average_waiting_time/nb_episode)
    print("temps d'attente cumulé :", average_episode_cumulated_waiting_time/nb_episode)
    print("vitesse moyenne :", average_episode_average_speed/nb_episode)
    print("emission de co2 moyenne :", average_episode_emission_co2/nb_episode)
    print("evacuated vehicle average : ",  average_episode_evacuated_vehicle/nb_episode)

    std_waiting_time = np.std(liste_waiting_time)
    std_cumulated_waiting_time = np.std(liste_cumulated_waiting_time)
    std_average_speed = np.std(liste_average_speed)
    std_emission_co2 = np.std(liste_emission_co2)
    std_evacuated_vehicle = np.std(liste_evacuated_vehicle)

    return [nb_episode, average_episode_nb_collision/nb_episode, average_episode_average_waiting_time/nb_episode, std_waiting_time,
            average_episode_cumulated_waiting_time/nb_episode, std_cumulated_waiting_time, average_episode_average_speed/nb_episode, std_average_speed,
            average_episode_emission_co2/nb_episode,std_emission_co2, average_episode_evacuated_vehicle/nb_episode,std_evacuated_vehicle, flow1]





