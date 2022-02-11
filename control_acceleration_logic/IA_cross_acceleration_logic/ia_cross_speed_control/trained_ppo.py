import control_acceleration_logic.IA_cross_acceleration_logic.ia_cross_speed_control.ia_step_runner as env
from keras.models import load_model
import numpy as np
import keras.backend as K


def trained_ppo(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4):

    n_actions = 4
    dummy_n = np.zeros((1, 1, n_actions))
    dummy_1 = np.zeros((1, 1, 1))

    model_actor = load_model('model_actor_94_11.8.hdf5')

    env.start_simulation(display)
    state = env.reset(display)
    done = False

    while True:
        state_input = K.expand_dims(state, 0)
        action_probs = model_actor.predict([state_input, dummy_n, dummy_1, dummy_1, dummy_1], steps=1)
        action = np.argmax(action_probs)
        next_state, reward, done = env.step_few(simulation_time, action_probs[0]*17, image_size)
        state = next_state
        if done:
            state = env.reset(display)

