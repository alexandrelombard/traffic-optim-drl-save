import keras
import keras.layers
import keras.losses
import numpy as np
import tensorflow as tf

import traffic_light_logic.traffic_light.myfunction_traffic_light as my_function


def launch_traffic_light_trained_simulation(display, simulation_time, reward_type, coef, flow1, flow2, flow3, flow4):

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

    # model = build_model()
    # model_target = build_model()

    # model.load_weights("./trained_model")
    # model_target.load_weights("./trained_model_target")

    model = keras.models.load_model('trafic_Light_Model')
    model_target = keras.models.load_model('trafic_light_model_target')

    model.summary()
    model_target.summary()

    episode_count = 0
    frame_count = 0

    my_function.start_simulation(display)

    while True:  # Run until solved
        state = np.array(my_function.reset(display))       # TODO Create the reset function (restart sim)
        episode_reward = 0

        for timestep in range(1, max_steps_per_episode):
            # env.render(); Adding this line would show the attempts
            # of the agent in a pop up window.
            frame_count += 1

            # Predict action Q-values
            # From environment state
            state_tensor = tf.convert_to_tensor(state)
            state_tensor = tf.expand_dims(state_tensor, 0)
            action_probs = model(state_tensor, training=False)
            # Take best action
            action = tf.argmax(action_probs[0]).numpy()

            # Apply the sampled action in our environment
            print(f"Applying action: {action}")
            state_next, reward, done = my_function.step_few(action, simulation_time, reward_type, coef)
            state_next = np.array(state_next)

            episode_reward += reward

            # Save state
            state = state_next

            if done:
                break
        print("episode reward:", episode_reward, "at episode", episode_count, "frame count", frame_count)

        episode_count += 1
