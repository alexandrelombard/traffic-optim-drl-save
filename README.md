# Package d'optimization de trafic 

L'objectif de ce package est de pouvoir facilement imlémenter, tester et comparer de nouvelle stratégie de pilotage d'intersection.
L'objectif générale de ce package est donc de trouver des solutions afin d'optimiser les flux et ainsi de gagner en efficacité.  

Ce package s'utilise avec sumo qui est un simulateur de trafic routier multi-étage open source permettant de realiser toute sorte de simulation.

Les stratégies implémentés pour le moment sont dédiées au cas de l'intersection.
Pour le cas de l'intersection il y a 2 type de stratégie à considerer dans ce package: 
  - Les stratégies se basant sur une logique de droit de passage (un vehicule ne passe l'intersection que si on lui donne l'autorisation).
  - Les stratégies se basant sur le contrôle de la vitesse et de l'accéleration du véhicule. 
  

## Pour commencer 

### Pré-requis 

Python 3.8 
- Sur votre ordinateur <br/> 

  Télécharger Eclipse sumo version 1.10 de préférence pour eviter tout problème de compatibilité. 

- Sur votre environement de travail<br/>

  installer les dépendances du projet disponible dans le fichier requirements.txt<br/>
  Commande python : pip install - r requirements.txt 
  
- Variable d'environement <br/>
  
  Il est possible que la variable d'environement SUMO_HOME soit créer automatiqument lors de l'installation de sumo.<br/>
  Si ce n'est pas le cas vous devez déclarer SUMO_HOME comme variable d'environement avec pour valeur le chemin jusqu'au repertoire de sumo (C:\...\Eclipse\Sumo).<br/>
  
### Installation 

Pour installer le package, il suffit d'utiliser git clone suivi du lien du repo dans la commande windows ou de télecharger directement le package. 


### Utilisation 

Pour utiliser le package une fois installé, il suffit d'instancier la classe Environnement avec les valeurs d'attribues souhaitées 
et utiliser la méthode launch_simulation ou la méthode simulation_statistique. 

Les attributs de cette classe sont : 

- l’attribut mode définit l’approche et la stratégie qui va être lancé
- l’attribut display définit si la simulation est affiché graphiquement
- l’attribut training indique si l’on souhaite réaliser un apprentissage par renforcement, cet apprentissage sera lancé si l’approche et la stratégie indiquée sont
compatibles
- l’attribut simulation time définit la durée d’une simulation
- l’attribut image size définit la taille de l’image utilisée si une image est utilisé
comme format de perception
- l’attribut vector size indique la taille du vecteur utilisé si un vecteur est utilisé
comme format de perception
- l’attribut reward type indique le type de reward utilisé si un apprentissage est
lancé
- l’attribut coef indique l’importance que l’on accorde aux collisions dans la reward
- les attributs flow indique le nombre de véhicules par heure passant dans une voie
de l’intersection


###Pour créer de nouvelle stratégie 

Si vous souhaitez créer une nouvelle heuristique, vous devez créer un nouveau fichier dans le module de l'approche souhaitée. 
Dans ce fichier, vous devez définir les différentes fonctions nécessaires à la mise en place de votre heuristique. 
Vous pouvez utiliser la AbstractClassStrategy ou non qui définit toutes les méthodes qu'il faut au minimum pour implémenter une strategie. Vous pouvez vous inspirer pour chaque méthode des stratégies déja implémenté.
Je déconseille fortement de toucher au code des heuristiques déjà implémenté.

Si vous souhaitez créer une nouvelle stratégie basée sur une solution de Deep Learning, vous devez ajouter un algorithme d'apprentissage. Pour fonctionner cette algorithle a besoin d'un environnement vous pouvez soit utiliser les classes d'environnement gym que nous avons créé, soit définir vos propre classe d'environnement gym. 
Pour créer ces classes d'environnement gym vous pouvez soit utiliser les fonctions que nous avons réalisées soit les créer de toutes pièces.

Pour accéder à vos nouvelles méthodes, il faut ajouter dans la classe
Environnement un mode dans la fonction launch_simulation. 

Pour pouvoir générer des statistiques de manière automatisée, il faut dans la fonction simulation_statistique de la classe Environnement
ajouter un code de ce style : 

            list_valeur, list_valeur_seuil = statistique.launch_statistique(display, nb_episode,
                                                                    time_per_episode,
                                                                    flow1 * i, flow2 * i,
                                                                    flow3 * i, flow4 * i, methode_fonction)

            list_stat_methode.append(list_1)
            list_stat_methode_quart.append(list_dcp_seuil)

Dans ce code on a les listes qui contiendront les valeurs récupèrerez par la fonction launch_statistique du module statistique. 

Les paramètres de cette fonction sont : 

- display indiquant si l'on souhaite afficher graphiquement les simulations 
lancées pour récupérer les statistiques. 
- nb_episode nombre de simulation réalisé pour récupérer les statistiques
- time_per_episode durée d'une simulation
- les flow indiquent le flux de véhicule entrant par heure dans une voie
- methode_fonction est une fonction permettant de réaliser une step de simulation de votre méthode cette fonction doit être definit comme ça :
   

      def methode_fonction(temps_de_simulation): 
          ...
          ...
          ...

          return done, average_waiting_time, cumulated_waiting_time, emission_co2, average_speed, evacuated_vehicle, collision
      
Vous pouvez la encore vous inspirer du code des autres fonctions de méthode.

Pour stocker les statistiques dans le fichier .csv il faut également ajouter la ligne suivante à la fin du code launch_statistique : 

    statistique.benchmark_csv_line_writer("benchmark.csv", "nom_methode", list_contenant_stat)

  


### Amélioration du package 

Maintenant que le package est fonctionnel et que la preuve de l'utilisation 
des méthodes de Deep Learning a été faite, il faut faire un travail de générification 
du code. Lors de la réalisation de cette première version du package, l'accent a été 
porté sur le test et la recherche de solution de Deep Learning 
par conséquent l'architecture et la structure actuel du package ne sont pas très génériques. 
Il y a donc un travail de restructuration à effectuer pour le rendre plus générique et modulaire. 
J'estime la durée de cette amélioration à environ une semaine.

À plus longs termes, deux solutions sont envisageables, le package 
peut soit être étendu pour gérer de nouveau type de cas
(voies d'insertions, carrefour giratoire, dépassement, optimisation d'itinéraires ...), 
soit on peut considérer que l'utilisation de SUMO est limitante c'est-à-dire que SUMO n'est pas 
assez spécifique aux solutions que nous souhaitons développer et
passer à la réalisation d'un outil de simulation plus spécifiques permettant de simuler tous les aspects
de la communication inter-véhiculaire.


### Fabriqué avec 

* [Pycharm](https://www.jetbrains.com/fr-fr/pycharm/) - IDE
* [Eclipse Sumo](https://www.eclipse.org/sumo/) - Simulateur de trafic routier
* [Keras](https://keras.io/) - Deep Learning API 
* [Stable-Baselines3](https://stable-baselines3.readthedocs.io/en/master/guide/install.html) - Deep Learning API 

## Versions
  premiere version du package.

## Auteurs

* **Ahmed Noubli** _alias_ [@anoubli](https://github.com/anoubli)
* **Alexandre Lombard** _alias_ [@alexandrelombard](https://github.com/alexandrelombard)

## License 

Ce projet est sous licence MIT





  



