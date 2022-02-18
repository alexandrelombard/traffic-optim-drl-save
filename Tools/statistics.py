import os
import random
import string
import sys
import optparse
from random import *
import time

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot
import csv

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def vehiculeEvacuer() :
    nbVehicule = traci.simulation.getArrivedNumber()
    return nbVehicule

def tempAttenteMoyen(edges) :
    tempsAttenteCumul = 0
    for i in range(0, len(edges)) :
        tempsAttenteCumul += traci.edge.getWaitingTime(edges[i])

    if traci.vehicle.getIDCount() > 0:
        tempMoyen = tempsAttenteCumul/traci.vehicle.getIDCount()
    else:
        tempMoyen = tempsAttenteCumul

    return tempMoyen

def tempAttenteCumuler(edges) :
    tempsAttenteCumul = 0
    for i in range(0, len(edges)) :
        tempsAttenteCumul += traci.edge.getWaitingTime(edges[i])

    return tempsAttenteCumul

def vitesseMoyenne(edges) :
    # vitesseMoyenCumul = 0
    # for i in range(0, len(edges)):
    #     vitesseMoyenCumul+= traci.edge.getLastStepMeanSpeed(edges[i])
    #
    # return vitesseMoyenCumul/len(edges)

    vitesseMoyenCumul = 0
    nb_vehicle = 0
    for i in range(0, len(edges)):
        vitesseMoyenCumul += traci.edge.getLastStepMeanSpeed(edges[i]) * traci.edge.getLastStepVehicleNumber(edges[i])
        nb_vehicle += traci.edge.getLastStepVehicleNumber(edges[i])

    if nb_vehicle > 0:
        vitessemoyenne = vitesseMoyenCumul/nb_vehicle
    else:
        vitessemoyenne = vitesseMoyenCumul

    return vitessemoyenne

def emissionDeCo2(edges) :
    emissionCo2 = 0
    for i in range(0, len(edges)):
        emissionCo2 += traci.edge.getCO2Emission(edges[i])

    return emissionCo2


def csvFileWriter(fichier, tempsAttenteMoyen, tempsAttenteCumul, vitesseMoyenne, emissionCo2, evacuated_vehicle, nb_collision, episode_reward, episode_count ) :

    with open(fichier, 'a', newline='', encoding='utf-8') as fichiercsv :
        writer = csv.writer(fichiercsv)
        writer.writerow([tempsAttenteMoyen,tempsAttenteCumul,vitesseMoyenne,emissionCo2, evacuated_vehicle, nb_collision,episode_reward, episode_count])
    fichiercsv.close()


def csvFilereadAndPrint(fichier) :
    tempsAttenteMoyen = []
    tempsAttenteCumul = []
    vitesseMoyenne = []
    emissionCo2 = []
    evacuated_vehicle = []
    nb_collision = []
    episode_reward = []
    episode_number = []

    with open(fichier, "r", encoding='utf-8') as obj_fichier:

        print("On affiche les listes contenant les lignes du .csv")
        compteur = 0

        while 1:  # La condition est donc toujours True
            ligne = obj_fichier.readline()  # On tente de lire la ligne
            if not (ligne):  # Si ligne n'existe pas, elle contient False
                break  # On sort de la boucle
            else:  # Sinon, c'est que ligne contient quelque chose
                ligne.replace('\n', '')  # On supprime le passage à la ligne finale de chaque ligne
                elements = ligne.split(',')  # On crée une liste contenant les valeurs de chaque ligne
                print(elements)
                if compteur == 0:
                    grandeur_abscisse = elements[0]
                    grandeur_ordonnee = elements[1]
                else:
                    tempsAttenteMoyen.append(float(elements[0]))
                    tempsAttenteCumul.append(float(elements[1]))
                    vitesseMoyenne.append(float(elements[2]))
                    emissionCo2.append(float(elements[3]))
                    evacuated_vehicle.append(float(elements[4]))
                    nb_collision.append(float(elements[5]))
                    episode_reward.append(float(elements[6]))
                    episode_number.append(float(elements[7]))

                compteur += 1

    obj_fichier.close()


    # plt.title("temps d'attente moyen")
    # plt.plot(range(0, len(tempsAttenteMoyen)),tempsAttenteMoyen  )
    # plt.show()
    #
    # plt.title("temps d'attente cumulé")
    # plt.plot(range(0,len(tempsAttenteCumul)), tempsAttenteCumul )
    # plt.show()
    #
    # plt.title("vitesse moyenne")
    # plt.plot(range(0,len(vitesseMoyenne)),vitesseMoyenne )
    # plt.show()
    #
    # plt.title("emission de co2")
    # plt.plot(range(0,len(emissionCo2)), emissionCo2 )
    # plt.show()
    #
    # plt.title("reward")
    # plt.plot(range(0,len(episode_reward)), episode_reward)
    # plt.show()


    # plt.subplot(221)
    # plt.title("temps d'attente moyen")
    # plt.plot(range(0, len(tempsAttenteMoyen)),tempsAttenteMoyen  )
    # plt.subplot(222)
    # plt.title("temps d'attente cumulé")
    # plt.plot(range(0,len(tempsAttenteCumul)), tempsAttenteCumul )
    # plt.subplot(223)
    # plt.title("vitesse moyenne")
    # plt.plot(range(0,len(vitesseMoyenne)),vitesseMoyenne )
    # plt.subplot(224)
    # plt.title("emission de co2")
    # plt.plot(range(0,len(emissionCo2)), emissionCo2 )
    # plt.show()

def benchmark_csv_init(name):
    with open(name, 'w', newline='') as fichiercsv:
        writer = csv.writer(fichiercsv)
        writer.writerow(["methode", "flux", "collision", "att moy", "ecart type", "min", "max", "Q1", "median", "Q3", "att cum", "ecart type",  "min", "max", "Q1", "median", "Q3", "v", "ecart type", "min", "max", "Q1", "median", "Q3", "co2", "ecart type",  "min", "max", "Q1", "median", "Q3", "evac", "ecart type",  "min", "max", "Q1", "median", "Q3"])
    fichiercsv.close()

def benchmark_csv_line_writer(name, methode, listdqn) :
    with open(name, 'a', newline='', encoding='utf-8') as fichiercsv:
        writer = csv.writer(fichiercsv)
        for i in range(0, len(listdqn)):
            writer.writerow(
                [methode, listdqn[i][37], listdqn[i][1], listdqn[i][2], listdqn[i][3], listdqn[i][4], listdqn[i][5],
                 listdqn[i][6], listdqn[i][7], listdqn[i][8], listdqn[i][9], listdqn[i][10], listdqn[i][11],
                 listdqn[i][12], listdqn[i][13], listdqn[i][14], listdqn[i][15], listdqn[i][16], listdqn[i][17],
                 listdqn[i][18], listdqn[i][19], listdqn[i][20], listdqn[i][21], listdqn[i][22], listdqn[i][23],
                 listdqn[i][24], listdqn[i][25], listdqn[i][26], listdqn[i][27], listdqn[i][28], listdqn[i][29],
                 listdqn[i][30], listdqn[i][31], listdqn[i][32], listdqn[i][33], listdqn[i][34], listdqn[i][35],
                 listdqn[i][36]])

    fichiercsv.close()

def benchmark_csv_maker(name, listfif, listdqn, list_tf, listfcf, listdcp):
    with open(name+".csv", 'w', newline='') as fichiercsv:
        writer = csv.writer(fichiercsv)
        writer.writerow(["methode", "flux", "collision", "att moy", "ecart type", "min", "max", "Q1", "median", "Q3", "att cum", "ecart type",  "min", "max", "Q1", "median", "Q3", "v", "ecart type", "min", "max", "Q1", "median", "Q3", "co2", "ecart type",  "min", "max", "Q1", "median", "Q3", "evac", "ecart type",  "min", "max", "Q1", "median", "Q3"])
    fichiercsv.close()

    with open(name + ".csv", 'a', newline='', encoding='utf-8') as fichiercsv :
        writer = csv.writer(fichiercsv)
        for i in range(0, len(listdqn)):
            writer.writerow(["dqn", listdqn[i][37], listdqn[i][1], listdqn[i][2], listdqn[i][3], listdqn[i][4], listdqn[i][5], listdqn[i][6], listdqn[i][7], listdqn[i][8], listdqn[i][9], listdqn[i][10], listdqn[i][11], listdqn[i][12], listdqn[i][13], listdqn[i][14], listdqn[i][15], listdqn[i][16], listdqn[i][17], listdqn[i][18], listdqn[i][19], listdqn[i][20], listdqn[i][21], listdqn[i][22], listdqn[i][23], listdqn[i][24], listdqn[i][25], listdqn[i][26], listdqn[i][27], listdqn[i][28], listdqn[i][29], listdqn[i][30], listdqn[i][31], listdqn[i][32], listdqn[i][33], listdqn[i][34], listdqn[i][35], listdqn[i][36]])
            writer.writerow(["fifo", listfif[i][37], listfif[i][1], listfif[i][2], listfif[i][3], listfif[i][4], listfif[i][5], listfif[i][6], listfif[i][7], listfif[i][8], listfif[i][9], listfif[i][10], listfif[i][11], listfif[i][12], listfif[i][13], listfif[i][14], listfif[i][15], listfif[i][16], listfif[i][17], listfif[i][18], listfif[i][19], listfif[i][20], listfif[i][21], listfif[i][22], listfif[i][23], listfif[i][24], listfif[i][25], listfif[i][26], listfif[i][27], listfif[i][28], listfif[i][29], listfif[i][30], listfif[i][31], listfif[i][32], listfif[i][33], listfif[i][34], listfif[i][35], listfif[i][36]])
            writer.writerow(["feu", list_tf[i][37], list_tf[i][1], list_tf[i][2], list_tf[i][3], list_tf[i][4], list_tf[i][5], list_tf[i][6], list_tf[i][7], list_tf[i][8], list_tf[i][9], list_tf[i][10], list_tf[i][11], list_tf[i][12], list_tf[i][13], list_tf[i][14], list_tf[i][15], list_tf[i][16], list_tf[i][17], list_tf[i][18], list_tf[i][19], list_tf[i][20], list_tf[i][21], list_tf[i][22], list_tf[i][23], list_tf[i][24], list_tf[i][25], list_tf[i][26], list_tf[i][27], list_tf[i][28], list_tf[i][29], list_tf[i][30], list_tf[i][31], list_tf[i][32], list_tf[i][33], list_tf[i][34], list_tf[i][35], list_tf[i][36]])
            writer.writerow(["fcfs", listfcf[i][37], listfcf[i][1], listfcf[i][2], listfcf[i][3], listfcf[i][4], listfcf[i][5], listfcf[i][6], listfcf[i][7], listfcf[i][8], listfcf[i][9], listfcf[i][10], listfcf[i][11], listfcf[i][12], listfcf[i][13], listfcf[i][14], listfcf[i][15], listfcf[i][16], listfcf[i][17], listfcf[i][18], listfcf[i][19], listfcf[i][20], listfcf[i][21], listfcf[i][22], listfcf[i][23], listfcf[i][24], listfcf[i][25], listfcf[i][26], listfcf[i][27], listfcf[i][28], listfcf[i][29], listfcf[i][30], listfcf[i][31], listfcf[i][32], listfcf[i][33], listfcf[i][34], listfcf[i][35], listfcf[i][36]])
            writer.writerow(["dcp", listdcp[i][37], listdcp[i][1], listdcp[i][2], listdcp[i][3], listdcp[i][4], listdcp[i][5], listdcp[i][6], listdcp[i][7], listdcp[i][8], listdcp[i][9], listdcp[i][10], listdcp[i][11], listdcp[i][12], listdcp[i][13], listdcp[i][14], listdcp[i][15], listdcp[i][16], listdcp[i][17], listdcp[i][18], listdcp[i][19], listdcp[i][20], listdcp[i][21], listdcp[i][22], listdcp[i][23], listdcp[i][24], listdcp[i][25], listdcp[i][26], listdcp[i][27], listdcp[i][28], listdcp[i][29], listdcp[i][30], listdcp[i][31], listdcp[i][32], listdcp[i][33], listdcp[i][34], listdcp[i][35], listdcp[i][36]])

    fichiercsv.close()


def reward_calculation(reward_type, coef):
    flow_edges = ["1to2", "5to2", "3to2", "4to2"]
    reward = 0
    if reward_type == "nb_vehicle":
        reward = traci.simulation.getArrivedNumber() * coef
    elif reward_type == "waiting_time":
        reward = -tempAttenteMoyen(flow_edges)
    elif reward_type =="emission_co2":
        reward = -emissionDeCo2(flow_edges)
    elif reward_type == "mix_vehicle_time":
        reward = -tempAttenteMoyen(flow_edges) * 100 + traci.simulation.getArrivedNumber() * 10
    else:
        print("please use a correct type of reward")

    return reward - (traci.simulation.getCollidingVehiclesNumber() * coef)


def launch_statistique(display, nb_episode, time_per_episode, flow1, flow2, flow3, flow4, class_methode):

    liste_waiting_time = []
    liste_cumulated_waiting_time = []
    liste_average_speed = []
    liste_emission_co2 = []
    liste_evacuated_vehicle = []

    episode_cumulated_waiting_time = 0
    episode_emission_co2 = 0
    episode_average_speed = 0
    episode_average_waiting_time = 0
    episode_evacuated_vehicle = 0
    episode_nb_collision = 0

    average_episode_cumulated_waiting_time = 0
    average_episode_emission_co2 = 0
    average_episode_average_speed = 0
    average_episode_average_waiting_time = 0
    average_episode_evacuated_vehicle = 0
    average_episode_nb_collision = 0

    step_count = 0

    class_methode.xml_flow_changer(flow1, flow2, flow3, flow4)
    class_methode.start_simulation(display)

    for i in range(nb_episode):
        class_methode.reset(display)
        done = 0
        step_count = 0
        while not done:
            step_count += 1



            done, average_waiting_time, cumulated_waiting_time, emission_co2, average_speed, evacuated_vehicle, collision = \
                class_methode.run_step_simulation(time_per_episode)



            episode_cumulated_waiting_time += cumulated_waiting_time
            episode_emission_co2 += emission_co2
            episode_average_speed += average_speed
            episode_average_waiting_time += average_waiting_time
            episode_evacuated_vehicle += evacuated_vehicle
            episode_nb_collision += collision


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

    min_waiting_time = np.min(liste_waiting_time)
    min_cumulated_waiting_time = np.min(liste_cumulated_waiting_time)
    min_average_speed = np.min(liste_average_speed)
    min_emission_co2 = np.min(liste_emission_co2)
    min_evacuated_vehicle = np.min(liste_evacuated_vehicle)

    max_waiting_time = np.max(liste_waiting_time)
    max_cumulated_waiting_time = np.max(liste_cumulated_waiting_time)
    max_average_speed = np.max(liste_average_speed)
    max_emission_co2 = np.max(liste_emission_co2)
    max_evacuated_vehicle = np.max(liste_evacuated_vehicle)

    median_waiting_time = np.median(liste_waiting_time)
    median_cumulated_waiting_time = np.median(liste_cumulated_waiting_time)
    median_average_speed = np.median(liste_average_speed)
    median_emission_co2 = np.median(liste_emission_co2)
    median_evacuated_vehicle = np.median(liste_evacuated_vehicle)

    quartile1_waiting_time = np.percentile(liste_waiting_time, 25)
    quartile1_cumulated_waiting_time = np.percentile(liste_cumulated_waiting_time, 25)
    quartile1_average_speed = np.percentile(liste_average_speed, 25)
    quartile1_emission_co2 = np.percentile(liste_emission_co2, 25)
    quartile1_evacuated_vehicle = np.percentile(liste_evacuated_vehicle, 25)

    quartile3_waiting_time = np.percentile(liste_waiting_time, 75)
    quartile3_cumulated_waiting_time = np.percentile(liste_cumulated_waiting_time, 75)
    quartile3_average_speed = np.percentile(liste_average_speed, 75)
    quartile3_emission_co2 = np.percentile(liste_emission_co2, 75)
    quartile3_evacuated_vehicle = np.percentile(liste_evacuated_vehicle, 75)

    quartileSeuil(liste_waiting_time, quartile1_waiting_time, quartile3_waiting_time)
    quartileSeuil(liste_cumulated_waiting_time, quartile1_cumulated_waiting_time, quartile3_cumulated_waiting_time)
    quartileSeuil(liste_average_speed, quartile1_average_speed, quartile3_average_speed)
    quartileSeuil(liste_emission_co2, quartile1_emission_co2, quartile3_emission_co2)
    quartileSeuil(liste_evacuated_vehicle, quartile1_evacuated_vehicle, quartile3_evacuated_vehicle)

    std_waiting_time2 = np.std(liste_waiting_time)
    std_cumulated_waiting_time2 = np.std(liste_cumulated_waiting_time)
    std_average_speed2 = np.std(liste_average_speed)
    std_emission_co22 = np.std(liste_emission_co2)
    std_evacuated_vehicle2 = np.std(liste_evacuated_vehicle)

    min_waiting_time2 = np.min(liste_waiting_time)
    min_cumulated_waiting_time2 = np.min(liste_cumulated_waiting_time)
    min_average_speed2 = np.min(liste_average_speed)
    min_emission_co22 = np.min(liste_emission_co2)
    min_evacuated_vehicle2 = np.min(liste_evacuated_vehicle)

    max_waiting_time2 = np.max(liste_waiting_time)
    max_cumulated_waiting_time2 = np.max(liste_cumulated_waiting_time)
    max_average_speed2 = np.max(liste_average_speed)
    max_emission_co22 = np.max(liste_emission_co2)
    max_evacuated_vehicle2 = np.max(liste_evacuated_vehicle)

    median_waiting_time2 = np.median(liste_waiting_time)
    median_cumulated_waiting_time2 = np.median(liste_cumulated_waiting_time)
    median_average_speed2 = np.median(liste_average_speed)
    median_emission_co22 = np.median(liste_emission_co2)
    median_evacuated_vehicle2 = np.median(liste_evacuated_vehicle)

    quartile1_waiting_time2 = np.percentile(liste_waiting_time, 25)
    quartile1_cumulated_waiting_time2 = np.percentile(liste_cumulated_waiting_time, 25)
    quartile1_average_speed2 = np.percentile(liste_average_speed, 25)
    quartile1_emission_co22 = np.percentile(liste_emission_co2, 25)
    quartile1_evacuated_vehicle2 = np.percentile(liste_evacuated_vehicle, 25)

    quartile3_waiting_time2 = np.percentile(liste_waiting_time, 75)
    quartile3_cumulated_waiting_time2 = np.percentile(liste_cumulated_waiting_time, 75)
    quartile3_average_speed2 = np.percentile(liste_average_speed, 75)
    quartile3_emission_co22 = np.percentile(liste_emission_co2, 75)
    quartile3_evacuated_vehicle2 = np.percentile(liste_evacuated_vehicle, 75)

    average_episode_cumulated_waiting_time2 = np.mean(liste_cumulated_waiting_time)
    average_episode_emission_co22 = np.mean(liste_emission_co2)
    average_episode_average_speed2 = np.mean(liste_average_speed)
    average_episode_average_waiting_time2 = np.mean(liste_waiting_time)
    average_episode_evacuated_vehicle2 = np.mean(liste_evacuated_vehicle)
    average_episode_nb_collision2 = average_episode_nb_collision









    return [nb_episode,
            average_episode_nb_collision/nb_episode,
            average_episode_average_waiting_time/nb_episode, std_waiting_time, min_waiting_time, max_waiting_time, quartile1_waiting_time, median_waiting_time, quartile3_waiting_time,
            average_episode_cumulated_waiting_time/nb_episode, std_cumulated_waiting_time, min_cumulated_waiting_time, max_cumulated_waiting_time, quartile1_cumulated_waiting_time, median_cumulated_waiting_time, quartile3_cumulated_waiting_time,
            average_episode_average_speed/nb_episode, std_average_speed, min_average_speed, max_average_speed, quartile1_average_speed, median_average_speed, quartile3_average_speed,
            average_episode_emission_co2/nb_episode, std_emission_co2, min_emission_co2, max_emission_co2, quartile1_emission_co2, median_emission_co2, quartile3_emission_co2,
            average_episode_evacuated_vehicle/nb_episode, std_evacuated_vehicle, min_evacuated_vehicle, max_evacuated_vehicle, quartile1_evacuated_vehicle, median_evacuated_vehicle, quartile3_evacuated_vehicle,
            flow1],\
           [nb_episode,
            average_episode_nb_collision2,
            average_episode_average_waiting_time2, std_waiting_time2, min_waiting_time2, max_waiting_time2, quartile1_waiting_time2, median_waiting_time2, quartile3_waiting_time2,
            average_episode_cumulated_waiting_time2, std_cumulated_waiting_time2, min_cumulated_waiting_time2, max_cumulated_waiting_time2, quartile1_cumulated_waiting_time2, median_cumulated_waiting_time2, quartile3_cumulated_waiting_time2,
            average_episode_average_speed2, std_average_speed2, min_average_speed2, max_average_speed2, quartile1_average_speed2, median_average_speed2, quartile3_average_speed2,
            average_episode_emission_co22, std_emission_co22, min_emission_co22, max_emission_co22, quartile1_emission_co22, median_emission_co22, quartile3_emission_co22,
            average_episode_evacuated_vehicle2, std_evacuated_vehicle2, min_evacuated_vehicle2, max_evacuated_vehicle2, quartile1_evacuated_vehicle2, median_evacuated_vehicle2, quartile3_evacuated_vehicle2,
            flow1]


def quartileSeuil(list_stat, quartile1, quartile3):
    for item in list_stat:
        if item < quartile1:
            list_stat.remove(item)
        if item > quartile3:
            list_stat.remove(item)


#csvFilereadAndPrint('stat/statistique1632496693.4022462.csv')