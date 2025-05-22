# README Romain simulation

## Table of Contents
 1. [Description du package vs_ros2](#Description-du-package-vs_ros2)
 2. [Utils](#Utils)
 3. [Déplacement avec MoveIt dans RViz](#Déplacement-avec-MoveIt-dans-RViz)
 4. [Subheading 3](#sub-heading-3)

## Description du package vs_ros2

## Utils

### Installer les dépendances avec Rosdep
1. Bien mettre les dépendances nécessaires dans le fichier *package.xml* du package si c'est notre package personnel.
2. Aller à la racine du workspace ros2 et executer cette commande pour installer toutes les dépendances manquantes avec rosdep :
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Ajouter une dépendance au projet
Exemple : ajout de la dépendance `tf2_eigen`.  

- Ajouter dans le `package.xml`:
```xml
 <depend>tf2_eigen</depend>
```

- Ajouter dans le `CMakeLists`  
  - Dans le *find_package(...)* : 
```python
 find_package(tf2_eigen REQUIRED)
```  
   
  - Dans `ament_target_dependencies(...)`
```python
  "tf2_eigen"
```

## Mise en place de la simulation déplacement robot sous la tête laser

### Ajout de la plaque à l'URDF de l'UR

**Modifications apportées dans le package `vs_ur_description`**

- Ajout d'une fonction pour calculer l'inertie de la plaque en fonction de ses dimensions dans le fichier *ur_common.xacro* 

```xml
<!-- Inertia for the plate -->
  <xacro:macro name="rectangular_inertial" params="height width depth mass *origin">
    <inertial>
      <mass value="${mass}" />
      <xacro:insert_block name="origin" />
      <inertia ixx="${0.0833333 * mass * (width * width + depth * depth)}" ixy="0.0" ixz="0.0"
        iyy="${0.0833333 * mass * (depth * depth + height * height)}" iyz="0.0"
        izz="${0.0833333 * mass * (width * width + height * height)}" />
    </inertial>
  </xacro:macro>
```

- Ajout du *link* dans le fichier *ur_macro.xacro*
```xml
<!-- link of the rectangular plate as a tool-->
    <!-- Déclaration des propriétés -->
    <xacro:property name="plate_width" value="0.3" />
    <xacro:property name="plate_height" value="0.3" />
    <xacro:property name="plate_thickness" value="0.001" />
    <xacro:property name="plate_density" value="7600" />  <!-- en kg/m^3 -->
    <xacro:property name="plate_mass" value="${plate_density * plate_width * plate_height * plate_thickness}" />

    <!-- Utilisation dans le lien -->
    <link name="${tf_prefix}plate_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${plate_width} ${plate_height} ${plate_thickness}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${plate_width} ${plate_height} ${plate_thickness}"/>
        </geometry>
      </collision>
      <xacro:rectangular_inertial height="${plate_height}" width="${plate_width}" depth="${plate_thickness}" mass="${plate_mass}">
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
      </xacro:rectangular_inertial>
    </link>
```

- Ajout du *joint* dans le fichier *ur_macro.xacro*
```xml
<!-- joint for the plate -->
    <joint name="${tf_prefix}plate_joint" type="fixed">
      <parent link="${tf_prefix}flange"/>
      <child link="${tf_prefix}plate_link"/>
      <origin xyz="${plate_thickness} 0 0" rpy="0 ${-pi/2.0} 0"/>
    </joint>
```

- Modification du groupe Moveit  
Dans le package `vs_ur_moveit_config`, le fichier `ur_macro.srdf.xacro` dans le dossier `srdf`.

```xml
<group name="ur_manipulator">
      <chain base_link="${prefix}base_link" tip_link="${prefix}plate_link"/>
      <kinematics_plugin name="${prefix}${name}_manipulator" value="moveit_kdl_kinematics_plugin/KDLKinematicsPlugin"/>
</group>
```
Cela permet de définir la plaque comme le "end effector" du robot et donc ce sera l'origine du link de la plaque qui sera
amené à une pose ciblée lors d'une planfication de trajectoire.

### Ajout de l'enveloppe de la tête laser à Gazebo

Pour ajouter l'enveloppe de la tête laser à Gazebo, modification du fichier world personnalisé.  
Il se trouve dans le package `vs_ur_gazebo_sim` dans `worlds` : 

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <gravity>0 0 0</gravity>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Enveloppe de la tête laser avec collision solide -->
    <model name="custom_box">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.31 0.3 0.28</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.31 0.3 0.28</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>0 0.750 0.935 0 0 0</pose>
    </model>
  </world>
</sdf>
```

>
>*NB : les dimensions de l'enveloppe et son placement par rapport au robot ont pu évolués par rapport à l'exemple de code fourni*
>


### Déplacement avec MoveIt dans RViz

Utilisation de l'outil *MotionPlanning* pour réaliser un déplacement simple sans coder pour vérifier que tout fonctionne.  

Le launch file *ur_moveit.launch.py* est utilisé comme base pour réaliser le launch file *ws.launch.py* dans le package `vs_ur_ws`  

#### Joint state publisher

>
>**Erreur traitée** : 
>Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds. Requested time 1746173538.406386, but latest received state has time 0.000000.
>


Il est nécessaire de publier l'état des liaisons pour pouvoir exécuter une planification de trajectoire. Dans le cas d'une simulation (sans capteurs réels),
il est nécessaire d'initialiser et de lancer le node suivant dans le launch file :

```py
    #joint_state_node : producteur
    #Cas démo/simulation sans capteurs réels
    joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name="joint_state_publisher",
    parameters=[{"use_sim_time": False}]
    )
```

Dans le cas où le robot est directement utilisé (UR driver), le `/joint_states` doit être publié par le driver (ou contrôleur). Par exemple, dans le cas de Universal Robot ROS2 driver, il doit publier l’état automatiquement.

#### Contrôleur ROS2 et Moveit

##### Différences entre les contrôleurs ROS2 et MoveIt

| Aspect         | ros2_control                                                                 | MoveIt                                                                 |
|----------------|------------------------------------------------------------------------------|-----------------------------------------------------------------------|
| Rôle           | Piloter physiquement le robot (ou une simulation) : position, effort, vitesse | Planifier des trajectoires de mouvement dans l’espace                  |
| Contrôleurs    | Ex. joint_trajectory_controller, forward_position_controller                 | Définit quels contrôleurs utiliser pour exécuter une trajectoire planifiée |
| Communication  | Action server (/follow_joint_trajectory) exposé par un contrôleur réel       | Action client dans move_group qui envoie la trajectoire à exécuter     |
| Déploiement    | Nécessite un controller_manager en exécution                                 | Nécessite le noeud move_group et sa configuration                      |


##### Résolution de l'erreur rencontrée
>
>**Erreur traitée** : 
>Action client not connected to action server : joint_trajectory_controller/follow_joint_trajectory
>

Cela signifie que MoveIt essaie d'envoyer la trajectoire vers un contrôleur qui n'existe pas ou n'est pas lancé. Autrement dit, le contrôleur `joint_trajectory_controller` n’est pas actif, ou bien il n’a pas démarré correctement. 

1. Vérifier qu'un contrôleur est bien défini dans le fichier de configuration MoveIt. 
    - C'est notre cas, il s'agit du fichier *controllers.yaml* dans `vs_ur_moveit_config/config`.  
    - Le paramètre `use_fake_hardware` permet de choisir quel contrôleur utiliser parmi les deux définis dans *controllers.yaml*. **True** = simulation

2. Vérifier qu'un contrôleur ROS2 est bien défini dans l'URDF du robot
    - Le fichier **ros2_control** (avec macro ur_ros2_control) :

        - La déclaration du système matériel (hardware), qui peut être :
            - simulée (mock_components, gazebo_ros2_control, etc.)
            - ou réelle avec le driver ur_robot_driver/URPositionHardwareInterface

        - La configuration des joints et leurs interfaces :
            - command_interface (position et/ou velocity)
            - state_interface (position, velocity, effort)

        - Des sensors et GPIO spécifiques aux robots UR (ex : force/torque sensor, I/O, payload, etc.)


    - Le **fichier transmission** (macro ur_arm_transmission) :
        - Une définition des transmissions mécaniques entre les joints et les moteurs.
        - Utilise le type transmission_interface/SimpleTransmission (adapté pour un bras UR).
        - Indique la réduction mécanique (ici 1, donc aucun réducteur explicite modélisé).

#### Ordre de lancement des nodes
##### En mode simulation (use_fake_hardware == true) :

- **controller_manager_node** : il charge l’interface ros2_control.
- **spawner_joint_trajectory** : lance un contrôleur (ex. joint_trajectory_controller) via le manager.
- **joint_state_publisher** : publie des joint_states simulés (optionnel si un contrôleur s’en occupe déjà).
- **robot_state_publisher** : publie les TF à partir de joint_states.
- **move_group_node** : MoveIt! (il attend que le contrôleur soit dispo pour envoyer les trajectoires).
- **rviz_node** : pour visualisation.

##### En mode réel (use_fake_hardware == false) :

- (tu ne lances pas controller_manager_node ni spawner_*)
- Le driver UR ROS 2 officiel (non montré ici) publie joint_states + exécute les commandes.
- Tu gardes :
  - **robot_state_publisher**
  - **move_group_node**
  - **rviz_node**
  - et tu désactives **joint_state_publisher**.


>
>Modification par certain du tout
> Dans ur_macro.srdf.xacro, ligne 14, tool0 changé par le joint plate_joint pour le groupe ur_manipulator
>

>
>**Solution finale** : 
>Il faut lancer le launch file `ur_sim_control.launch.py` pour lancer les contrôleurs nécessaires.
>Stratégie adoptée pour le moment : création du fichier ws_sim.launch.py qui est pareil que ur_sim_moveit.launch.py.
>Ajout du node perso à lancer directement dans le launch moveit du package vs_ur_moveit_config. 
>

### Collision checking

Lors de la planification d'une trajectoire, MoveIt prend en compte les collisions avec l'environnement et les auto-collisions. Il ne propose que des trajectoires sans
collsisions.  
**Donc c'est ce qui nous faut dans notre cas d'étude.**  


### Définition d'une pose initiale pour le robot
Définition d'un node `set_initial_position` dans le package `vs_ur_gazebo_sim` et démarrer dans `ur_sim_control.launch.py`.  

```cpp
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <cmath>

using namespace std::chrono_literals;


double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

class InitialPositionPublisher : public rclcpp::Node
{
public:
  InitialPositionPublisher()
  : Node("initial_position_publisher")
  {
    RCLCPP_INFO(this->get_logger(), "INITIAL POSITION STARTED");
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);

    // Give Gazebo and controllers some time to fully start
    timer_ = this->create_wall_timer(10s, std::bind(&InitialPositionPublisher::publish_trajectory, this));
  }

private:
  void publish_trajectory()
  {
    RCLCPP_INFO(this->get_logger(), "SENDING INITIAL JOINT TRAJECTORY");
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.joint_names = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    double joint_1 = deg2rad(-84.0);
    double joint_2 = deg2rad(-127.0);
    double joint_3 = deg2rad(-73.0);
    double joint_4 = deg2rad(-71.0);
    double joint_5 = deg2rad(269.0);
    double joint_6 = deg2rad(-6.0);
    point.positions = { joint_1, joint_2, joint_3, joint_4, joint_5, joint_6 };
    point.time_from_start = rclcpp::Duration::from_seconds(5.0);

    msg.points.push_back(point);
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent initial joint trajectory.");

    // Cancel the timer so we don't resend
    timer_->cancel();
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPositionPublisher>());
  rclcpp::shutdown();
  return 0;
}
```


Ce node utilise `joint_trajectory_controller` pour mettre le robot dans une configuration intiale.  

Cette méthode est intéressante car elle : 
- est compatible avec des contrôleurs temps réel qui sont utilisés dans Gazebo
- permet de générer une transition douce vers la pose désirée via *time_from_start*
- fonctionne aussi bien avec Gazebo qu'avec des robots réels via `ros2_control`

C'est la bonne méthode pour commander un robot dans un environnement simulé ou réel où les controleurs sont actifs.  

Elle comporte également quelques limitations :  
- Si le contrôleur n'est pas encore prêt ou inactif, la commande est perdue
- Il n'y a pas de feedback sur l'exécution (sauf si on observe les topics d'état)
- Il n'y a aucune garantie que le robot atteigne réllement cette position s'il est en collision ou si la physique s'y oppose  

>
> Pourquoi ne pas réaliser une planification de trajectoire avec Moveit ?
- Avantages :
    - Permet de définir directement l’état interne de MoveIt sans attendre d’exécution.

    - Très utile pour la planification de trajectoire (pas pour l’exécution réelle dans Gazebo).

    - Peut être utilisé pour forcer un état dans des tests ou pour mettre à jour l'état visuel d’un robot dans RViz.

- Inconvénients :
    - Ne déplace pas réellement le robot dans Gazebo ou dans une simulation physique.

    - Ne met pas à jour les positions physiques dans ros2_control ou le contrôleur de trajectoire.

    - C’est une opération interne à MoveIt → elle ne passe pas par les contrôleurs.


### Node vs_ur_ws

#### Structure générale 

La ligne suivante permet de crée un Node ROS2  
```cpp
auto const node = std::make_shared<rclcpp::Node>(
  "vs_ur_ws",
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);
```
Le node est ensuite ajouté à un exécuteur 
```cpp
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
```

L'exécuteur est ensuite exécuter dans un thread:
```cpp
auto spinner = std::thread([&executor]() { executor.spin(); });
```

En ROS2, l'exécuteur est une boucle d'évènements qui : 
- écoute les messages entrants (topics, services, actions...)
- déclenche les callbacks associés
- fait tourner les timers
- gère les évènements de ROS (comme les évènements du node)  
En résumé, c'est ce qui fait tourner le node.  

Faire tourner l'exécuteur dans un thread séparé permet de faire autre chose en parallèle dans la fonction main ici.


### Stratégies envisagées 

#### Stratégie 1 : frame dynamique
La première stratégie repose sur l'utilisation d'une frame dynamique nommée *moving_frame*.  
Lorsqu'on demande d'amener le robot à une pose dans l'espace, moveit amène l'origine du repère de *l'end-effector* à cette pose.  
L'idée est alors de définir un repère dynamique (child de la plaque) pour à chaque fois amené la nouvelle origine du repère à la pose cible.  
Pour cela, cette première version de code a été réalisée : 

```cpp
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <iomanip>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <geometry_msgs/msg/pose.h>
#include <random>
#include <fstream>
#include <vector>
#include <cmath> // pour M_PI
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");


void publish_moving_frame(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster,
  double x_offset, double y_offset, double z_offset)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = node->get_clock()->now();
  tf_msg.header.frame_id = "plate_link";
  tf_msg.child_frame_id = "moving_frame";
  tf_msg.transform.translation.x = x_offset;
  tf_msg.transform.translation.y = y_offset;
  tf_msg.transform.translation.z = z_offset;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.0;
  tf_msg.transform.rotation.w = 1.0;

  broadcaster->sendTransform(tf_msg);
}



int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "vs_ur_ws",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a TransformBroadcaster
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);


  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;

  //Publication 
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [node, tf_broadcaster, &offset_x, &offset_y, &offset_z]() {
      RCLCPP_INFO(node->get_logger(), "Publishing TF: offset_x = %.3f, offset_y = %.3f, offset_z = %.3f", offset_x, offset_y, offset_z);
      publish_moving_frame(node, tf_broadcaster, offset_x, offset_y, offset_z);
    }
  );
  

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("vs_ur_ws");


  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });



  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Charger le modèle du robot
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  // Optionnel : définir explicitement le link de la plaque comme end-effector
  //move_group_interface.setEndEffectorLink("moving_frame");
 
  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();


    auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  

  //Set an initial pose
  auto const initial_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.0;
    msg.position.x = 0.0;
    msg.position.y = 0.7;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(initial_pose);

  //Set a target pose : on se place sous la tête laser à la working distance de la lentille télécentrique (215.4mm)
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.0;
    msg.position.x = 0.0;
    msg.position.y = 0.7;
    msg.position.z = 0.4;
    return msg;
  }();
  //move_group_interface.setPoseTarget(target_pose);


    // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.310;
    primitive.dimensions[primitive.BOX_Y] = 0.280;
    primitive.dimensions[primitive.BOX_Z] = 0.300;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 0.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.750;
    box_pose.position.z = 0.935;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  
  //ROS API
  // We create a publisher and wait for subscribers.
  // Note that this topic may need to be remapped in the launch file.
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  rclcpp::sleep_for(std::chrono::seconds(1)); 
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");

  // Create a plan to that target the initial pose
  auto const [success_init, plan_init] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  rclcpp::sleep_for(std::chrono::seconds(1)); 
  prompt("Intial pose ok, Press 'Next' in the RvizVisualToolsGui window to plan");

  // Execute the initial plan
  if(success_init) {
    move_group_interface.execute(plan_init);
  } else {
    RCLCPP_ERROR(logger, "Inital Planing failed!");
  }
  
  rclcpp::sleep_for(std::chrono::seconds(1)); 
  prompt("Intial pose ok, Press 'Next' in the RvizVisualToolsGui window to plan");

  //On décale le TCP de la plaque pour refaire une planification et amener ce nouveau point à la pose ciblée
  offset_x = 0.0;
  offset_y = 0.0;
  offset_z = -0.1;
  publish_moving_frame(node, tf_broadcaster, offset_x, offset_y, offset_z);
  rclcpp::sleep_for(std::chrono::seconds(1));
  prompt("TCP déplacé ! Press 'Next' in the RvizVisualToolsGui window to plan");

   //Définir la nouvelle position de départ
   move_group_interface.setStartStateToCurrentState();

  // Create a plan to that target the initial pose
  //move_group_interface.setPoseTarget(target_pose);
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  rclcpp::sleep_for(std::chrono::seconds(1)); 
  prompt("Second planing, Press 'Next' in the RvizVisualToolsGui window to plan");

  // Execute the initial plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}
```
Néanmoins, il ne fonctionne pas complètement. Certaines problèmes percistent :
1. Le repère dynamique n'a pas de parent : la définition du repère dynamique via le broadcast ne semble pas permettre de définir complètement le lien de parenter entre les deux repères et donc le repère *moving_frame* n'a pas de parent et n'est donc pas dans le *tree* des repères de TF.  
2. Même si un offset est appliqué, le robot retourne au même endroit comme si l'origine du repère n'avait pas bougée. Cela est sûrement dû au problème numéro 1.  


#### Stratégie 2 : calcul de la nouvelle pose du end-effector
L'idée de cette seconde stratégie est de calculer, en fonction du point de la plaque ciblé, la nouvelle pose à atteindre pour l'origine du repère du end-effector pour amener le point de la plaque à la pose ciblée. Cette stratégie à l'avantage d'être beaucoup plus simple car il n'y a pas de gestion dynamique des repères.

## Stratégie utilisée 

La stratégie utilisée est la stratégie 2 mentionnée dans la section **Stratégies envisagées**.  
Dans un premier temps, nous devons définir les points que nous voulons tester. Pour cela, nous définissons une grille de points (descritisation de la plaque).

### Grille
Pour savoir quelle surface nous pouvons réellement texturer sur la plaque, nous devons vérifier si nous pouvons atteindre certains points dessus. En particulier, les centres de chacun des patchs ( par défaut $80 \cdot 80mm²$) que nous pouvons "placer" dans la plaque.  
Pour cela, nous réalisons une grille de points dont chaque point correspond à un centre de patch. Pour cela, nous réalisons la fonction suivante : 

```cpp
std::vector<std::array<double, 3>> grid_definition(double longueur, double largeur, double patch_size = 0.08) {
    std::vector<std::array<double, 3>> points;

    // Top left corner of the plate
    double c1_x = -longueur / 2.0;
    double c1_y = largeur / 2.0;

    // First patch center (defined from the top left corner of the plate)
    double p_init_x = c1_x + patch_size / 2.0;
    double p_init_y = c1_y - patch_size / 2.0;

    // Number of patches along X and Y (in the plate frame)
    int n_patches_x = static_cast<int>(longueur / patch_size);
    int n_patches_y = static_cast<int>(largeur / patch_size);

    // Set up variables
    double x = p_init_x;
    double y = p_init_y;

    // Add the first point to the list
    points.push_back({x, y, 0.0});

    // Creation of the other centers : we want a "snape shape" as a trajectory 
    for (int i = 0; i <= n_patches_x; ++i) {
        if (i != 0) {
            x += patch_size;
            if (i % 2 == 0) {
                y = p_init_y;
            }
            points.push_back({x, y, 0.0});
        }

        for (int j = 0; j < n_patches_y; ++j) {
            if (i % 2 == 0) {
                y -= patch_size;
            } else {
                y += patch_size;
            }

            points.push_back({x, y, 0.0});
        }
    }

    return points;
}
```
Cette fonction nous permet de définir les centre des patchs pour n'importe quelle taille de plaque. De plus, la taille des patchs est ajustable (considérés carrés).
L'algorithme a été testé en amont via python (implémentation rapide et simple pour des essaies) et validé.  

Cette grille de point va nous permettre directement de définir les points cibles que notre effectueur doit atteindre.

### Points effecteurs cibles

La grille que nous avons définit précédemment, nous donnes les offsets selon x et y entre un point à tester et le centre de la plaque, que nous soyons dans le repère de la plaque ou dans le repère world (en considérant comme centre de la plaque la pose cible : la sortie du scanner).  

Pour la planification de trajectoire, nous devons définir le point effecteur cible dans le repère world. Ainsi, nous exprimons les points effecteurs ciblés dans ce repère. Pour cela, nous avons réaliser la fonction suivante : 

```cpp
// Compute the end effector pose based on the target pose and the point on the plate selected
geometry_msgs::msg::Pose calculate_effector_pose(const geometry_msgs::msg::Pose &target_pose, const geometry_msgs::msg::Point &point)
{
    geometry_msgs::msg::Pose effector_pose;
    effector_pose.position.x = target_pose.position.x - point.x;
    effector_pose.position.y = target_pose.position.y - point.y;
    effector_pose.position.z = target_pose.position.z - point.z;
    effector_pose.orientation = target_pose.orientation;
    return effector_pose;
}
```

Cette fonction renvoie les coordonnées des points effecteurs ciblés dans le repère world.  
Les points de la grille ayant une coordonnées des coordonnées 2D (troisième composante nulle), nous fixons directement la pose effecteur selon z world comme étant celui de la pose ciblée.  
De plus, nous imposons que l'orientation doit être la même que celle de la pose cible.  

La pose cible est définie de la façon suivante : 

```cpp
// Set a target pose in the world frame
    auto const target_pose = [robotBase_to_box_x, robotBase_to_box_y, working_distance_from_ground]() {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = robotBase_to_box_x;
        msg.position.y = robotBase_to_box_y;
        msg.position.z = working_distance_from_ground;
        return msg;
    }();
```

Les coordonnées sont exprimées dans le repère world. Cela nous permet d'obtenir les coordonnées des points cibles directement dans ce repère car nous ne faisons qu'une simple addition d'offset indépendant du repère.

De plus, nous fixons l'orientation avec un quaternion unité représentant l'absence totale de rotation. En faisant ça, nous nous assurons que nous sommes parfaitement parallèle au sol (et donc perpendiculaire à l'axe optique si nous le considérons perpendiculaire au sol) car aucune inclinaison selon roll et le pitch n'est autorisée.
De plus, il n'y a aucune rotation selon le yaw, donc par défaut, le robot va s'orienté vers l'axe x du repère world. Nous pouvons modifier cette composante si nécessaire à l'avenir.

### Vérification des repères : affichage de marqueurs 

Nous pouvons afficher des marqueurs pour vérifier les points que nous allons tester.  
Pour cela, il faut dans un premier temps instancier un publisher pour publier les marqueurs sur le topic *visualization_marker_array* :
```cpp
// Publisher for markers
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    visualization_msgs::msg::MarkerArray marker_array;
```

Puis ensuite on peut par exemple créer une fonction qu'on appelle pour publier les marqueurs dans la boucle d'itérations pour chacun des points testés :
```cpp
//Function for adding red sphere markers at a pose
void add_marker(visualization_msgs::msg::MarkerArray &marker_array,const geometry_msgs::msg::Pose &pose, const std::string &frame_id, int id, const rclcpp::Time &timestamp){
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = "test_points";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker_array.markers.push_back(marker);
}
```

**Attention** : Il est important de récupérer le timestamp de la transformation sinon un décalage temporel entre les timestamps TF et celui du marqueur apparaît. Il faut bien prendre "la version de la transformation" la plus récente pour placer le marqueur. Pour la récupérer : *timestamp = transformation.header.stamp*.  
**TODO** voir si la transformation est vraiement nécessaire


### Valeurs des liaisons

La planification de Moveit 2 nous permet de récupérer pour une planification de trajectoire, l'ensemble des valeurs des liaisons pour chaque point au cours du mouvement.  
Récupérer ces valeurs de liaisons, nous permettra par la suite de vérifier si les liaisons sont bien restées comprises dans l'interval défini par les butées articulaires.  

Pour récupérer les liaisons au cours de la planification, une fonction réalisant la planification et la récupération des liaisons a été réalisées : 

```cpp
// Function to plan a trajectory to a target pose of the end effector and return the success status and also the joint trajectory points     
std::pair<bool, std::vector<std::vector<double>>> plan_trajectory_and_joints(moveit::planning_interface::MoveGroupInterface &move_group_interface, const geometry_msgs::msg::Pose &effector_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan){

    // Plannification of the trajectory
    move_group_interface.setPoseTarget(effector_pose);
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Vector to store the joint trajectory points
     std::vector<std::vector<double>> joint_trajectory_points;

     // Check if the trajectory is empty
     if (plan.trajectory_.joint_trajectory.points.empty())
     {
         return {false, joint_trajectory_points};
     }

     const trajectory_msgs::msg::JointTrajectory& trajectory = plan.trajectory_.joint_trajectory;

     for (const auto& point : trajectory.points)
     {
        joint_trajectory_points.push_back(point.positions);
     }

     return {success, joint_trajectory_points};
}
```
Cette fonction nous renvoie si la planification a été un succès au non et les valeurs des liaisons pour tous les points de la trajectoire planifiées (si elle existe).  

## Planificateur de trajectoire

Il a été remarqué que lorsque la simulation était lancée par exemple deux fois de suite sans toucher aucun paramètre, les résultats d'accessibilité des points pouvaient être différents. La source de ce problème vient du planificateur de trajectoire. Parfois, il n'arrive pas à converger dans le temps imparti.  
Plusieurs solutions sont alors envisageables : 
- Augmenter le temps accordé au planificateur pour converger  
- Tester plusieurs fois la planification pour un même point (et s'il est une fois atteignable c'est ok)  
- Réaliser la planification de la trajectoire avec plusieurs planificateurs en parallèle  

La solution qui a été retenue est de tester plusieurs fois la planification d'un même point.  
Pour cela, une boucle *while* a été mise en place :  
```cpp
// We try to plan the trajectory several times
        while ((planning_iterations < max_planning_iterations) && (success == false))
        {
            RCLCPP_INFO(node->get_logger(), "Iteration [%d] is starting.", planning_iterations);
            auto [local_success, local_joint_trajectory_points] = plan_trajectory_and_joints(move_group_interface, effector_pose, plan);
            success = local_success;
            joint_trajectory_points = local_joint_trajectory_points;

            planning_iterations = planning_iterations + 1;
            RCLCPP_INFO(node->get_logger(), "Success : %s", success ? "true" : "false");

            if (success)
            {
                RCLCPP_INFO(node->get_logger(), "Planning succeeded.");
                break; // Exit the loop if planning is successful
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Planning failed. Retrying...");
                rclcpp::sleep_for(std::chrono::seconds(1)); // Wait before retrying
            }
        }
```

Avec les variables suivantes définies en amont dans le code :  
```cpp
//Variables for the planning
    int planning_iterations = 0;
    int max_planning_iterations =10; // Number of iterations to try to plan the trajectory
    bool success = false;
    std::vector<std::vector<double>> joint_trajectory_points;
```
Dans notre cas d'application, le planificateur `geometric::RRTConnect`(avec ses paramètres par défauts) est utilisé par défaut.  

Nous pouvons modifier le temps accordé au planificateur pour déterminer une trajectoire : 
```cpp
  node->declare_parameter("planning_time", 10.0);
  double planning_time = node->get_parameter("planning_time").as_double();
  move_group_interface.setPlanningTime(planning_time);
```
La valeur par défaut est reseignée lors de la déclaration du paramètre (première ligne).  

Des détails sur les planificateurs sont disponibles dans le fichier `Planner.md`.  

## Ajout des logs de la planification

Pour comprendre mieux ce qu'il se passe au niveau de la planification et comprendre notamment les erreurs rencontrées en cas d'échec, nous décidons d'ajouter l'affichage du code erreur dans la console ainsi que de l'écrire dans le fichier CSV.  

### Récupérer le code d'erreur
*Nous sommes dans la fonction `plan_trajectory`*.  

Pour cela, nous devons dans un premier temps nous intéresser à `MoveItErrorCode`.  
La fonction `move_group_interface.plan(plan)` retourne un objet de type `MoveItErrorCode`. C'est un objet qui contient un code entier (*val*) qui indique le résultat de ka planification. Les correspondances entre les codes erreurs et les types d'erreur sont disponibles [ici](https://docs.ros.org/en/jade/api/moveit_msgs/html/msg/MoveItErrorCodes.html).  

>
> **Code Erreur -27 :**  
> Ce code erreur n'est pas répertorié dans la documentation officielle de MoveIt. Il a été néanmoins été rencontré dans le projet. A chaque fois qu'il apparait,
> il est accompagné de l'erreur suivante : *[ompl]: ... PRM.cpp:482 ... Unable to find any valid goal states*. Cette erreur signifie qu'aucune configuration articulée
> permet d'atteindre la pose souhaitée, *i.e*, aucune solution cinématique inverse n'a été trouvée.  
> Cela peut venir de plusieurs raisons :
> 1. la pose ciblée est hors de portée du bras robot
> 2. une orientation impossible à atteindre
> 3. des collisions immédiates ou contraintes non respectées
>


Ainsi, nous définissons la variable **erreur_code** comme étant le résultat de la fonction `move_group_interface.plan(plan)`.  
Puis nous récupérons la valeur particulière de *success* (car l'ensemble du code utilise cette valeur par la suite) et nous extrayons également la valeur, sous forme d'entier, du code erreur.  

```cpp
// Plan the trajectory and extract the error code
    auto error_code = move_group_interface.plan(plan);

    // Extract the boolean success to know if the plannification is a success or not
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Extract the code error in int format
    int error_code_int = error_code.val;
```

## Tolérances
Il est possible dans MoveIt 2 d'ajuster les tolérances de :  
- la trajectoire
- de l'objectif 

Pour les ajuster, il faut les modifier directement dans le fichier `ur_controllers.yalm`.
Voici un exemple :

```xml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint: { trajectory: 0.3, goal: 0.1 }
      shoulder_lift_joint: { trajectory: 0.3, goal: 0.1 }
      elbow_joint: { trajectory: 0.3, goal: 0.1 }
      wrist_1_joint: { trajectory: 0.3, goal: 0.1 }
      wrist_2_joint: { trajectory: 0.3, goal: 0.1 }
      wrist_3_joint: { trajectory: 0.3, goal: 0.1 }
```

Le paramètre `stopped_velocity_tolerance` fait référence à la vitesse résiduelle sur les liaisons à la fin de la trajectoire. Cela permet de s'assurer que le robot s'arrête bien à la fin. Dans le cas de l'exemple, si une liaison du robot a une vitesse supérieure à 0.0.2rad/s à l'instant final, alors l'exécution est considérée comme non réussie. Ce paramètre est typiquement utile pour éviter des erreurs de glissement ou à une inertie.  

Le paramètre `goal_time` correspond au temps maximal autorisé (en secondes) pour atteindre le but final après la fin de la théorie de la trajectoire. Cela donne un petit délai supplémentaire pour compenser les imperfections du contrôle. Par exemple, si une trajectoire se termine à t=5.0s et que le goal_time=1.0s, alors le robot a t=6.0s pour satisfaire toutes les contraintes de fin.  

Le paramètre `goal` correspond à la tolérance absolue sur la position finale de chaque liaison. Cela permet de déterminer si le robot est arrivé suffisamment près de la cible. Par exemple avec goal=0.2, une liaison pivot ayant pour cible 1rad, sera considérée bonne si sa valeur est comprise dans l'interval [0.8,1.2]rad.  


## Fonctions 

### tf2::transformToEigen
Cette fonction convertit une `geometry_msgs::Pose` ROS 2 (position + quaternion) en une matrice homogène 4×4 `Eigen::Isometry3d`, qui représente une transformation dans l’espace (translation + rotation).

## tf2

tf2 suit l'évolution de tous les repères du robot au cours du temps.  
Cela permet de poser des questions comme :  
- Où est l'effecteur par rapport au repère de la base ?  
- Quelle est la pose courante du repère de ma base dans le repère monde ?  

Il y a deux tâches principales pouvant être réalisées avec tf2 :  
1. Si on veut utiliser des transformations entre des repères, le noeud doit écouter pour les transformations.  
2. Si on veut envoyer la pose relative d'un repère au reste du système, il faut créer le publier et donc créer un broadcaster.  


## TODO
Actuellement, une planification de trajectoire est réalisée pour positionner le robot dans une configuration initiale.  
Ensuite, l'idée est de déplacer le TCP de la plaque car lorsqu'on fait une planification de trajectoire, c'est l'origine du repère du link mis en effecteur
qui est amenée à la pose ciblée.  
Pour faire ça, on définit un TCP dynamique "moving_frame" qu'on vient déplacer comme on veut (dans l'idée).
Le déplacement à l'air de fonctionner mais impossible d'afficher le TCP donc pas sûr. 
De plus, la planification (qui fonctionne) à l'air d'amener le robot au même endroit donc bon...  
Problème avec la position initiale du mouvement à creuser.  
