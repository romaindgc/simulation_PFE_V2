# simulation_PFE_V2

Ce répertoire contient la seconde version du simulateur réalisé dans le cadre du PFE intitulé : *Robotisation de la texturation de surface par laser à impulsions ultrabrèves*.

## Utiliser la simulation

### Build 
Il faut build les packages du package `icube_ur_cell`, à savoir les packages :
- `icube_ur_cell_description` 
- `icube_ur_cell_moveit_config`
- `icube_ur_cell_control`
- `vs_ur_ws`

Pour build individuellement ces packages, utiliser la commande suivante : 
```bash
colcon build --mixin --packages-select <package_name>
```

### Lancer la simulation
Pour lancer la simulation, exécuter la commande suivante : 
```bash
ros2 launch vs_ur_ws ws_sim.launch.py
```

### Modifier les paramètres

Les paramètres suivants peuvent être modifiés dans la simulation :  
1. Longueur de la plaque (m) : `plate_length`
2. Largeur de la plaque (m) : `plate_width`
3. Distance entre la base du robot et le centre de la box selon x dans le repère *world* (m) : `robotBase_to_box_x`
4. Distance entre la base du robot et le centre de la box selon y dans le repère *world* (m) : `robotBase_to_box_y`
5. Distance entre le sol et le dessous de la box selon z dans le repère *world* (m) : `box_to_ground`
6. La distance de travail sous le scanner (m) : `working_distance_under_lens`
7. Le temps accordé au planificateur pour résoundre la planfication (s) : `planning_time`
8. Le nombre maximum d'itérations autorisées pour réessayer la planification pour un point cible : `max_planning_iterations`

>
> NB : la box étant une boîte enveloppant la tête galvanométrique, la lentille télécentrique et la bague d'éclairage.
>

#### Dimensions de la plaque

Si vous souhaitez modifer les dimensions de la plaque, plusieurs étapes sont nécessaires :

1. Modifier les dimensions dans l'URDF du robot 
2. Modifier les paramètres `plate_length` et `plate_width` dans `vs_ur_ws.cpp` dans la package `vs_ur_ws`

#### Position relative entre la base du robot et le scanner

Pour modifier la position relative de la box par rapport à la base du robot, il suffit de modifier les paramètres `robotBase_to_box_x`, `robotBase_to_box_y` et `box_to_ground`.

