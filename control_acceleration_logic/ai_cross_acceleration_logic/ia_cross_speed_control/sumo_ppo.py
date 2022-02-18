import random

import traci

import control_acceleration_logic.ai_cross_acceleration_logic.ia_cross_speed_control.ai_step_runner as env
import numpy as np
import keras.backend as K
from keras.layers import Input, Dense, Flatten
from keras.models import Model
from keras.applications.mobilenet_v2 import MobileNetV2
from tensorflow.keras.optimizers import Adam
from keras.callbacks import TensorBoard
import tensorflow as tf


def deep_learning_ppo(display, simulation_time, image_size, reward_type, coef, flow1, flow2, flow3, flow4):
    clipping_val = 0.2
    critic_discount = 0.5
    entropy_beta = 0.001
    gamma = 0.99
    lmbda = 0.95

    env.start_simulation(display)

    def get_advantages(values, masks, rewards):
        returns = []
        gae = 0
        for i in reversed(range(len(rewards))):
            delta = rewards[i] + gamma * values[i + 1] * masks[i] - values[i]
            gae = delta + gamma * lmbda * masks[i] * gae
            returns.insert(0, gae + values[i])

        adv = np.array(returns) - values[:-1]
        return returns, (adv - np.mean(adv)) / (np.std(adv) + 1e-10)

    def ppo_loss_print(oldpolicy_probs, advantages, rewards, values):
        def loss(y_true, y_pred):
            y_true = tf.Print(y_true, [y_true], 'y_true: ')
            y_pred = tf.Print(y_pred, [y_pred], 'y_pred: ')
            newpolicy_probs = y_pred
            # newpolicy_probs = y_true * y_pred
            newpolicy_probs = tf.Print(newpolicy_probs, [newpolicy_probs], 'new policy probs: ')

            ratio = K.exp(K.log(newpolicy_probs + 1e-10) - K.log(oldpolicy_probs + 1e-10))
            ratio = tf.Print(ratio, [ratio], 'ratio: ')
            p1 = ratio * advantages
            p2 = K.clip(ratio, min_value=1 - clipping_val, max_value=1 + clipping_val) * advantages
            actor_loss = -K.mean(K.minimum(p1, p2))
            actor_loss = tf.Print(actor_loss, [actor_loss], 'actor_loss: ')
            critic_loss = K.mean(K.square(rewards - values))
            critic_loss = tf.Print(critic_loss, [critic_loss], 'critic_loss: ')
            term_a = critic_discount * critic_loss
            term_a = tf.Print(term_a, [term_a], 'term_a: ')
            term_b_2 = K.log(newpolicy_probs + 1e-10)
            term_b_2 = tf.Print(term_b_2, [term_b_2], 'term_b_2: ')
            term_b = entropy_beta * K.mean(-(newpolicy_probs * term_b_2))
            term_b = tf.Print(term_b, [term_b], 'term_b: ')
            total_loss = term_a + actor_loss - term_b
            total_loss = tf.Print(total_loss, [total_loss], 'total_loss: ')
            return total_loss

        return loss

    def ppo_loss(oldpolicy_probs, advantages, rewards, values):
        def loss(y_true, y_pred):
            newpolicy_probs = y_pred
            ratio = K.exp(K.log(newpolicy_probs + 1e-10) - K.log(oldpolicy_probs + 1e-10))
            p1 = ratio * advantages
            p2 = K.clip(ratio, min_value=1 - clipping_val, max_value=1 + clipping_val) * advantages
            actor_loss = -K.mean(K.minimum(p1, p2))
            critic_loss = K.mean(K.square(rewards - values))
            total_loss = critic_discount * critic_loss + actor_loss - entropy_beta * K.mean(
                -(newpolicy_probs * K.log(newpolicy_probs + 1e-10)))
            return total_loss

        return loss

    def get_model_actor_image(input_dims, output_dims):
        state_input = Input(shape=(image_size, image_size, 3))
        oldpolicy_probs = Input(shape=(1, output_dims,))
        advantages = Input(shape=(1, 1,))
        rewards = Input(shape=(1, 1,))
        values = Input(shape=(1, 1,))

        feature_extractor = MobileNetV2(include_top=False, weights='imagenet')

        for layer in feature_extractor.layers:
            layer.trainable = False

        # Classification block
        x = Flatten(name='flatten')(feature_extractor(state_input))
        x = Dense(1024, activation='relu', name='fc1')(x)
        out_actions = Dense(n_actions, activation='sigmoid', name='predictions')(x)

        model = Model(inputs=[state_input, oldpolicy_probs, advantages, rewards, values],
                      outputs=[out_actions])
        model.compile(optimizer=Adam(lr=1e-4), loss=tf.keras.losses.Huber())
        model.summary()
        return model


    def get_model_critic_image(input_dims):
        state_input = Input(shape=(image_size, image_size, 3))

        feature_extractor = MobileNetV2(include_top=False, weights='imagenet')

        for layer in feature_extractor.layers:
            layer.trainable = False

        # Classification block
        x = Flatten(name='flatten')(feature_extractor(state_input))
        x = Dense(1024, activation='relu', name='fc1')(x)
        out_actions = Dense(1, activation='tanh')(x)

        model = Model(inputs=[state_input], outputs=[out_actions])
        model.compile(optimizer=Adam(lr=1e-4), loss=tf.keras.losses.Huber())
        model.summary()
        return model



    def test_reward():
        state = env.reset(display)
        done = False
        total_reward = 0
        print('testing...')
        limit = 0
        while not done:
            state_input = K.expand_dims(state, 0)
            action_probs = model_actor.predict([state_input, dummy_n, dummy_1, dummy_1, dummy_1], steps=1)
            action = np.argmax(action_probs)
            next_state, reward, done = env.step_few(simulation_time, action_probs[0]*17, image_size)
            state = next_state
            total_reward += reward
            limit += 1
            if limit > 200:
                break
        return total_reward

    def one_hot_encoding(probs):
        one_hot = np.zeros_like(probs)
        one_hot[:, np.argmax(probs, axis=1)] = 1
        return one_hot


    state = env.reset(display)
    state_dims = (image_size, image_size, 3)
    n_actions = 4

    dummy_n = np.zeros((1, 1, n_actions))
    dummy_1 = np.zeros((1, 1, 1))

    tensor_board = TensorBoard(log_dir='./logs')


    model_actor = get_model_actor_image(input_dims=state_dims, output_dims=n_actions)
    model_critic = get_model_critic_image(input_dims=state_dims)


    ppo_steps = 200
    target_reached = False
    best_reward = 0
    iters = 0
    max_iters = 30000

    while not target_reached and iters < max_iters:

        states = []
        actions = []
        values = []
        masks = []
        rewards = []
        actions_probs = []
        actions_onehot = []
        state_input = None

        for itr in range(ppo_steps):
            state_input = K.expand_dims(state, 0)
            action_dist = model_actor.predict([state_input, dummy_n, dummy_1, dummy_1, dummy_1], steps=1)
            q_value = model_critic.predict([state_input], steps=1)
            action = np.random.choice(n_actions)
            action_onehot = np.zeros(n_actions)
            action_onehot[action] = 1
            std_dev =0.35
            noise = [np.random.normal(0, std_dev) * (np.power(-1, random.randint(0, 1))),
                     np.random.normal(0, std_dev) * (np.power(-1, random.randint(0, 1))),
                     np.random.normal(0, std_dev) * (np.power(-1, random.randint(0, 1))),
                     np.random.normal(0, std_dev) * (np.power(-1, random.randint(0, 1)))]

            print("noise :", noise)

            action_noise = (action_dist * 17) + noise
            action_legal = np.clip(action_noise, 0, 17)
            observation, reward, done = env.step_few(simulation_time, action_legal[0], image_size)
            print(
                'itr: ' + str(itr) + ', action=' + str(action) + ', reward=' + str(reward) + ', q val=' + str(q_value))
            mask = not done

            states.append(state)
            actions.append(action)
            actions_onehot.append(action_onehot)
            values.append(q_value)
            masks.append(mask)
            rewards.append(reward)
            actions_probs.append(action_legal/17)

            state = observation
            if done:
                env.reset(display)

        q_value = model_critic.predict(state_input, steps=1)
        values.append(q_value)
        returns, advantages = get_advantages(values, masks, rewards)

        state_batch = tf.convert_to_tensor(states)
        action_batch = tf.convert_to_tensor(actions_probs)
        reward_batch = tf.convert_to_tensor(rewards)
        advantages_batch = tf.convert_to_tensor(advantages)
        values_batch = tf.convert_to_tensor(values[:-1])
        action_onehot_batch = tf.convert_to_tensor(actions_onehot)
        returns_batch = tf.convert_to_tensor(returns)

        actor_loss = model_actor.fit(
            [state_batch, action_batch, advantages_batch, reward_batch, values_batch],
            [action_onehot_batch], verbose=True, shuffle=True, epochs=8,
            callbacks=[tensor_board])
        critic_loss = model_critic.fit([state_batch], [returns_batch], shuffle=True, epochs=8,
                                       verbose=True, callbacks=[tensor_board])

        avg_reward = np.mean([test_reward() for _ in range(5)])
        print('total test reward=' + str(avg_reward))
        if avg_reward > best_reward:
            print('best reward=' + str(avg_reward))
            model_actor.save('model_actor_{}_{}.hdf5'.format(iters, avg_reward))
            model_critic.save('model_critic_{}_{}.hdf5'.format(iters, avg_reward))
            best_reward = avg_reward
        if best_reward > 40 or iters > max_iters:
            target_reached = True
        iters += 1
        env.reset(display)

    traci.close()
