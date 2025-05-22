# Planificateurs

Ce document traite des algorithmes de planification (planificateurs) disponibles dans Moveit 2.

## Planificateurs disponibles dans moveit 2

| Planificateur       | Avantages                                                                 | Inconvénients                                                                 |
|---------------------|---------------------------------------------------------------------------|--------------------------------------------------------------------------------|
| **OMPL**            | - Large variété de planificateurs basés sur l’échantillonnage             | - Peut produire des trajectoires “bizarres” (tortueuses, discontinues)        |
|                     | - Complétude probabiliste                                                  | - Nécessite des connaissances expertes pour le réglage                        |
|                     | - Capable de résoudre une grande variété de problèmes de planification     |                                                                                |
| **PILZ**            | - Générateur de trajectoires cartésiennes                                 | - Ne gère pas les contraintes                                                  |
|                     | - Rapide                                                                   | - Pas de vérification de collision                                             |
|                     | - Mouvements industriels (LIN, PTP & CIRC)                                | - Seulement adapté à des environnements avec peu d’obstacles                  |
| **CHOMP & STOMP**   | - Trajectoires lisses et répétables par défaut                            | - Temps de planification long                                                  |
|                     | - Planificateurs optimisants capables de gérer divers types de contraintes| - Incomplets                                                                   |
|                     |                                                                           | - Très difficiles à régler                                                     |

[Source tableau](https://moveit.ai/moveit%202/parallel%20planning/motion%20planning/2023/02/15/parallel-planning-with-MoveIt-2.html)  

Dans notre cas d'étude, nous voulons vérifier que le robot peut atteindre des poses dans l'espace en évitant toutes collisions avec l'environnement extérieur et toutes auto-collisions. Ainsi, d'après le tableau comparatif, **les planificateurs OMPL semblent les plus adaptés à notre problème**.  
De plus, le fait qu'il puisse "produire des trajectoires bizarres" n'est pas réellement un problème dans notre simulation. Nous voulons seulement vérifier que le robot peut aller en un point de l'espace (une contrainte d'orientation pourra être ajoutée dans un second temps).  

## OMPL planificateurs

### Types 

Au sein des planificateurs OMPL, il existe différents types : 
1. géométriques (*geometric*)
2. basé contrôle (*control-based*)
3. Multi-niveaux (*multilevel-based*)

**Les planificateurs géométriques** ne prennent en compte que les contraintes géométriques et cinématiques du système. Il est supposé que tout chemin faisable peut être transformé en une trajectoire dynamique faisable.  
Tous les planificateurs géométriques d'OMPL peuvent être utilisés pour planifier des trajectoires avec des contraintes géométriques (contrainte d'orientation, de position,...).  


**Les planificateurs basés control** sont utilisés si le système est soumis à des contraintes différentielles. Ces planificateurs reposent sur la propagation d'états plutôt que sur une simple interpolation pour planifier une trajectoire.  

**Les planificateurs multi-niveaux** sont utilisés lorsque l'espace d'état du système est de grande dimension. Ils permettent de diviser l'espace en sous-espaces pour faciliter l'espace d'état.  


Dans notre cas d'étude, **les planificateurs géométriques** semblent les plus adaptés. En effet, nous n'avons pas de contrainte différentiel et nous pourrons par la suite ajouter des contraintes géométriques à notre simulation.  

### Paramètres 

#### Paramètres principaux 

- **Range** : représente la longueur maximale du déplacement qui peut être ajouté à l'arbre des déplacements. Il influence grandement le temps d'exécution de l'algorithme.  
- **Goal bias** : dans le processus de sélectionner aléatoirement des états dans l'espace d'état pour tenter de l'atteindre, l'algorithme peut en fait choisir la pose ciblée avec une certaine probabilité. Cette probabilité est un nombre réel compris entre 0.0 et 1.0. Il ne doit pas être trop grand; il est généralement autour de 0.05.  
- **Border fraction** : Les planificateurs (comme KPIECE) utilisent une descrétisation d'une projection de l'espace d'état pour guider l'exploration. Cette descrétisation consiste à un ensemble de cellules. La fraction de frontière est la fraction de temps passé à concentrer l'exploration syr les cellules frontières (cellules à la "frontière" de l'exploration). Elle représente le pourcentage minimum utilisé pour sélectionner les cellules qui se trouvent à la frontière (minimum parce que si 95% des cellues se trouvent à la frontière, elles seront sélectionnées avec 95% de chances, même si la fraction frontière est fixée à 0.9 (90%)).  
- **Max. nearest neighbors** : représente le nombre maximum de plus proches voisins pour lesquels une connexion va ếtre testée quand un nouvel échantillon de configuration est ajouté.  

#### Autres paramètres 

- **longest_valid_segment_fraction** : correspond à la fraction dans l'espace d'état du robot, partant d'une pose où il n'est pas en collision, pour laquelle il est supposé que le mouvement du robot sur cette fraction sera sans collision. Par exemple, si le paramètre est fixé à 0.01, cela suppose que si la distance entre deux nodes (de la descrétisation) est inférieure à 1/100th de l'espace d'état, alors il n'est pas nécessaire de vérifier explicitement les sous-états entre les deux nodes.  
- **maximum_waypoint_distance** : réalise la même discrétisation du déplacement du robot pour les collisions que *longest_valid_segment_fraction* mais il définit une valeur absolue plutôt qu'une fraction de l'espace d'état. Par exemple, s'il vaut 0.1, si la distance entre deux neouds est inférieure à 0.1, les sous-états entre les deux noeuds ne sont pas explicitement vérifiés.  
- **projection_evaluator** : peut prendre une liste de liaisons ou de liens pour obtenir une approximation de la couverture d'une configuration dans l'espace.  
- **enforce_joint_model_state_space** : permet de forcer l'utilisation de l'espace articulaire pour la résolution de la planification. Initialement, l'espace cartésien est utilisé.  

#### Conditions de fin
De base, les planficateurs OMPL s'arrêtent lorsqu'un délai est dépassé. Néanmoins, il est possible de spécifier ou d'ajouter des conditions de fin par planificateur dans le fichier `ompl_plannig.yaml` via le paramètre : **termination_condition**. Les valeurs possibles sont :  
- **Iteration[num]** : s'arrête après *num* itérations. *num* étant un entier positif.  
- **CostConvergence[solutionsWindows, epsilon]** : s'arrête après que la fonction de coût ait convergée. Le paramètre *solutionsWindows* spécifie le nombre minimum de solutions à utiliser pour décider si le planification a convergé ou non. Le paramètre *epsilon* est le treshold pour considérer la convergence. Il doit être un nombre positif proche de 0.  
- **ExactSolution** : s'arrête dès qu'une solution exacte est trouvée où si le délai est dépassé.  

Dans tous les cas, le planificateur s'arrêtera quand la condition de fin spécifiée par l'utilisateur a lieu ou si le *allowed_plannig_time* est dépassé.  

## Planificateurs de trajectoires basés sur les échantillons

OMPL se concentre principalement sur les planificateurs de trajectoires basés sur les échantillons (*sampling-based motion planners*). Ce sont des algorithmes qui fonctionnent sans construire de représentation complète de l'espace d'état, mais en échantillonnant aléatoirement des états pour explorer l'espace de configuration.  
On distingue deux méthodes principales : *Probalistic roadmap* et *Tree-based planners*.  

### Probalistic roadmap (PRM)

Le principe général de cette méthode est divisé en deux étapes : la construction de la roadmap puis la phase de requête.  
La roadmap est construite par la génération d'échantillons aléatoires au sein de l'espace libre (sans collision). Lorsque le nombre d'échantillons est atteint, ils sont connectés entre eux à l'aide d'un planificateur local formant ainsi un graphe (roadmap) représentant les connectivités de l'espace libre.  
Pour une requête de planification, les configurations de départ et d'arrivée sont connectées à la roadmap puis un chemin liant les deux configurations est effectué sur le graphe. 
Si un chemin existe, la probabilité que l'algorithme le trouve tens vers 1 au fur et à mesure que le nombre d'échantillons augmente.  
Cet algorithme est également intéressant pour les espaces de configuration de haute dimension.  
Néanmoins, ce n'est pas adapté à la planification différentielle (si la position des objets dans l'environnement évolue dans le temps par exemple).  
Pour plus détails, se reférer à la source **OMPL primer**.  

### Tree-based planner 

Les planificateurs basés sur les arbres construisent une structure arborescente à partir de la configuration initiale du robot. L'arbre s'étend progressivement en explorant l'espace de configuration libre, en ajoutant des noeuds (configurations) et des arêtes (transitions valides) jusqu'à atteindre l'objectif ou couvrir l'espace.  
Pour plus détails, se reférer à la source **OMPL primer**.   

Exemple : RRT (*Rapidly-exploring Random Tree*)  
Cet algorithme génère des échantillons aléatoires dans l'espace de configuration puis pour chaque échantillon, il identifie quel noeud existant est le plus proche dans l'arbre. Il tente ensuite de connecter ce noeud au nouvel échantillon en respectant les contraintes et s'il y arrive, le nouvel échantillon est ajouté à l'arbre.

### Conclusion

| Caractéristique     | PRM                                | Planificateurs basés sur les arbres         |
| ------------------- | ---------------------------------- | ------------------------------------------- |
| **Type de requête** | Multi-requêtes                     | Requête unique                              |
| **Structure**       | Graphe (roadmap)                   | Arbre                                       |
| **Construction**    | Phase de prétraitement             | Construction en ligne (online)              |
| **Optimalité**      | PRM\* est optimal asymptotiquement | RRT\*, FMT\* sont optimaux asymptotiquement |
| **Adapté pour**     | Environnements statiques           | Environnements dynamiques ou inconnus       |


## Sources 
- [OMPL primer](https://ompl.kavrakilab.org/OMPL_Primer.pdf)  
- [OMPL planner](https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html#projection-evaluator)
- [Moveit parallel planning](https://moveit.ai/moveit%202/parallel%20planning/motion%20planning/2023/02/15/parallel-planning-with-MoveIt-2.html)