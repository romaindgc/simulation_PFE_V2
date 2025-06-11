
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
#include <geometry_msgs/msg/pose.hpp>
#include <random>
#include <fstream>
#include <vector>
#include <cmath> // pour M_PI
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <array>
#include <string>
#include <moveit/robot_state/conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <focus_curve/image_capture_manager.hpp>
#include <focus_curve/image_processing_utils.hpp>
#include <focus_curve/csv_utils.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");
std::string package_path = ament_index_cpp::get_package_share_directory("focus_curve");
std::string csv_file = csv_utils::generate_csv_filename(package_path + "/csv");

std::vector<geometry_msgs::msg::Point> points_depth(double x, double y, double wd, double step = 0.0005, double DOF = 0.003450) {
    std::vector<geometry_msgs::msg::Point> points;

    int end_if = std::round(DOF/step);

    // Ajouter le premier point
    geometry_msgs::msg::Point first_pt;
    first_pt.x = x;
    first_pt.y = y;
    first_pt.z = wd - (end_if/2)*step; // Has to be the lowest point in the depth direction
    points.push_back(first_pt);
    RCLCPP_INFO(rclcpp::get_logger("point_list_depth"), "Point: x=%.3f, y=%.3f, z=%.3f", first_pt.x, first_pt.y, first_pt.z);

    double z = first_pt.z + step;

    for (int i = 0; i <= end_if; ++i) {
        geometry_msgs::msg::Point pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;
            points.push_back(pt);
            RCLCPP_INFO(rclcpp::get_logger("point_list_depth"), "Point: x=%.3f, y=%.3f, z=%.1f", x, y, z);

        z += step;
    }

    RCLCPP_INFO(rclcpp::get_logger("point_list_depth"), "Number of points in the grid : (%ld)", points.size());
    RCLCPP_INFO(rclcpp::get_logger("point_list_depth"), "Depth min : (%f)", points.front().z);
    RCLCPP_INFO(rclcpp::get_logger("point_list_depth"), "Depth min : (%f)", points.back().z);

    return points;
}

//Function for writting data to the CSV file
/*The 6 first columns are the joint values of the robot (J1 to J6)
The 3 next columns are the coordinates of the tested point (x,y,z) in the plate frame
The 3 next columns are the coordinates of the end effector (x,y,z) in the plate frame : target pose
The last column is the reachability of the point/end effector pose
*/
void writeToCSV(const std::vector<double>& joint_values, bool reachable, const std::string& filename, const geometry_msgs::msg::Point & tested_point, double base_to_box_x ,double base_to_box_y, double planning_time, int max_planning_iterations, int error_code,const std::vector<double>& end_effector_orientation,const std::vector<double>& end_effector_position, double focus_score, double step)
{
    std::ofstream file;
    file.open(filename, std::ios_base::app); // Mode ajout

    if (!file.is_open())
    {
        std::cerr << "Erreur d'ouverture du fichier CSV !" << std::endl;
        return;
    }

    // Write the joint values of rhe target pose 
    for (const auto& val : joint_values)
    {
        file << val << ",";
    }

    // Write the coordinates (x,y,z) of the tested point in the plate frame
    file << tested_point.x << "," << tested_point.y << "," << tested_point.z << ",";

    // Write the step
    // Write the planning time selected
    file << step << ",";

    // Write if the point is reachable or not
    file << (reachable ? "reachable" : "NOK") << ",";

    // Write the planning time selected
    file << planning_time << ",";

    // Write the number of iterations to plan the trajectory
    file << max_planning_iterations << ",";

    // Write the distance between the robot base and the box along x world
    file << base_to_box_x << ",";

    // Write the distance between the robot base and the box along y world
    file << base_to_box_y << ",";

    // Write the error code of the plannification (int format)
    file << error_code << ",";

    //Write end-effector orientation
    file << end_effector_orientation[0] << "," << end_effector_orientation[1] << "," << end_effector_orientation[2] << "," << end_effector_orientation[3] << ",";

    //Write end-effector position
    file << end_effector_position[0] << "," << end_effector_position[1] << "," << end_effector_position[2] << ",";

    //Write the focus score value
    file << focus_score << std::endl;

    file.close();
}


std::vector<double> get_end_effector_orientation(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& from_frame = "world",
    const std::string& to_frame = "ur5_plate_support")
{
    try {
        geometry_msgs::msg::TransformStamped transformStamped =
            tf_buffer->lookupTransform(from_frame, to_frame, tf2::TimePointZero);

        const auto& q = transformStamped.transform.rotation;
        return {q.x, q.y, q.z, q.w};
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(rclcpp::get_logger("TF2"), "Could not get transform: %s", ex.what());
        return {};  // return empty vector on failure
    }
}

std::vector<double> get_end_effector_position(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& from_frame = "world",
    const std::string& to_frame = "ur5_plate_support")
{
    try {
        geometry_msgs::msg::TransformStamped transformStamped =
            tf_buffer->lookupTransform(from_frame, to_frame, tf2::TimePointZero);

        const auto& t = transformStamped.transform.translation;
        return {t.x, t.y, t.z};
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(rclcpp::get_logger("TF2"), "Could not get transform: %s", ex.what());
        return {};  // retourne un vecteur vide en cas d'échec
    }
}


// Fonction pour créer une pose cible dans le repère monde
geometry_msgs::msg::Pose createTargetPose(double x, double y, double z) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;  //Same orientation than the world frame
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    return target_pose;
}


// Function to plan a trajectory to a target pose of the end effector and return:
// - success status (bool),
// - joint trajectory points (vector<vector<double>>),
// - MoveIt error code (int)

std::tuple<
    bool,                               // success
    std::vector<std::vector<double>>,   // joint_trajectory_points
    int,                                // error_code
    std::vector<double>,                // last_configuration
    std::vector<std::string>            // joint_names
>
plan_trajectory_and_joints(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    const geometry_msgs::msg::Pose &effector_pose,
    moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    // Set the target pose
    move_group_interface.setPoseTarget(effector_pose);

    // Plan the trajectory and extract the error code
    auto error_code = move_group_interface.plan(plan);
    bool success = (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    int error_code_int = error_code.val;

    std::vector<std::vector<double>> joint_trajectory_points;
    std::vector<double> last_configuration;
    std::vector<std::string> joint_names;

    // If trajectory is empty or planning failed, return early
    if (!success || plan.trajectory_.joint_trajectory.points.empty())
    {
        return {false, joint_trajectory_points, error_code_int, last_configuration, joint_names};
    }

    // Extract joint positions
    for (const auto& point : plan.trajectory_.joint_trajectory.points)
    {
        joint_trajectory_points.push_back(point.positions);
    }

    // Extract last configuration
    last_configuration = joint_trajectory_points.back();

    // Extract joint names
    joint_names = plan.trajectory_.joint_trajectory.joint_names;

    return {success, joint_trajectory_points, error_code_int, last_configuration, joint_names};
}


void execute_trajectory(moveit::planning_interface::MoveGroupInterface &move_group_interface, const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    move_group_interface.execute(plan);
}

void add_marker(visualization_msgs::msg::MarkerArray &marker_array,const geometry_msgs::msg::Pose &pose, const std::string &frame_id, int id, const rclcpp::Time &timestamp, double r, double g, double b){
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
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker_array.markers.push_back(marker);
}

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "focus_curve",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto image_manager = std::make_shared<ImageCaptureManager>(
    node, "/vimbax_camera_235062/image_raw"
    );

    // Declare parameters
    node->declare_parameter("robotBase_to_box_x", 0.0); //Distance entre le centre de la box et de la base du robot selon x world
    node->declare_parameter("robotBase_to_box_y", 0.905); //Distance entre le centre de la box et de la base du robot selon y world
    node->declare_parameter("box_to_ground", 1.45); //Camera sur trépied environ 145cm du sol
    node->declare_parameter("working_distance_under_box", 0.1497); //0.1497 215mm sous la lentille et donc à 215-65.3 = 149.7mm sous la box car elle comprend le ring de lumière
    node->declare_parameter("planning_time", 5.0);
    node->declare_parameter("max_planning_iterations", 15);
    node->declare_parameter("DOF", 0.003450 ); //Depth of field of the lens (m)
    node->declare_parameter("step", 0.0005 ); //Step between each point in the depth direction (m)

    // Get parameters
    double robotBase_to_box_x = node->get_parameter("robotBase_to_box_x").as_double();
    double robotBase_to_box_y = node->get_parameter("robotBase_to_box_y").as_double();
    double box_to_ground = node->get_parameter("box_to_ground").as_double();
    double working_distance_under_box = node->get_parameter("working_distance_under_box").as_double();
    double planning_time_init = node->get_parameter("planning_time").as_double();
    int max_planning_iterations = node->get_parameter("max_planning_iterations").as_int();
    double DOF = node->get_parameter("DOF").as_double();
    double step = node->get_parameter("step").as_double();


    // Define the working distance
    double working_distance_from_ground = box_to_ground - working_distance_under_box;

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    // Charger le modèle du robot
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

     // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    //Set the planner
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP"); 

    //Set the plannig time 
    move_group_interface.setPlanningTime(planning_time_init);

    // Set a buffer and a listerner for the transform
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Wait for the transform to be available : plate to world
    rclcpp::sleep_for(std::chrono::seconds(2));
  
    // Transform plate to world
    geometry_msgs::msg::TransformStamped transformStamped_PW;
    try {
        transformStamped_PW = tf_buffer->lookupTransform(
            "world",  // target_frame (child)
            "ur5_plate_support", // source_frame (parent) 
            tf2::TimePointZero
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node->get_logger(), "Could not get transform: %s", ex.what());
        return 0;
    }

    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);
    };

     // Define the points of the plate (e.g., corners, grid)
    std::vector<geometry_msgs::msg::Point> points = points_depth(robotBase_to_box_x, robotBase_to_box_y, working_distance_from_ground, step, 0.01);

    // Create collision object for the robot to avoid : Camera
    auto make_collision_object = [node](const std::string &frame_id, double x, double y, double z) {
        RCLCPP_INFO(node->get_logger(), "Création d'un objet de collision à la position x=%.3f, y=%.3f, z=%.3f", x, y, z);

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.310;
        primitive.dimensions[primitive.BOX_Y] = 0.280;
        primitive.dimensions[primitive.BOX_Z] = 0.300;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z + 0.150; //Given distance z in between the floor and the bottom of the box so we have to had the half the hight to place the center of the box

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    };

    rclcpp::sleep_for(std::chrono::seconds(3));


    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(make_collision_object(move_group_interface.getPlanningFrame(), robotBase_to_box_x, robotBase_to_box_y, box_to_ground));

    // Publisher for markers
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    visualization_msgs::msg::MarkerArray marker_array;

    int marker_id = 0;


    //Variables for the planning
    int planning_iterations = 0;
    bool success = false;
    int error_code = 0;
    int planning_time = planning_time_init;
    bool is_in_collision = false;
    std::vector<std::vector<double>> joint_trajectory_points;
    std::vector<double> last_configuration;
    std::vector<std::string> joint_names;
    std::vector<double> start_configuration;
    std::vector<double> end_effector_orientation;
    std::vector<double> end_effector_position;

    //Variables for image processing
    double focus_score = 0;

    prompt("Press 'Next' to capture an image.");
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    prompt("Press 'Next' to start the planning process.");

    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& point = points[i];

        // Create the target pose
        geometry_msgs::msg::Pose target_pose = createTargetPose(point.x, point.y, point.z);

        // Enter in the for loop to test the points
        RCLCPP_INFO(node->get_logger(), "Testing point [%f, %f, %f]", point.x, point.y, point.z);

        // Add marker for the point being tested
        add_marker(marker_array, target_pose, "world", marker_id++, transformStamped_PW.header.stamp, 1.0, 0.0, 0.0);
        marker_pub->publish(marker_array);
        RCLCPP_INFO(node->get_logger(), "Marker added");

        prompt("Press 'Next' to plan the trajectory.");

        // Planning the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO(node->get_logger(), "Planning trajectory...");

        // We try to plan the trajectory several times
        while ((planning_iterations < max_planning_iterations) && (success == false))
        {
            RCLCPP_INFO(node->get_logger(), "Iteration [%d] is starting.", planning_iterations);
            auto [local_success, local_joint_trajectory_points,local_error_code, local_last_config, local_joint_name] = plan_trajectory_and_joints(move_group_interface, target_pose, plan);
            success = local_success;
            joint_trajectory_points = local_joint_trajectory_points;
            error_code = local_error_code;
            last_configuration = local_last_config;
            joint_names = local_joint_name;

            planning_iterations = planning_iterations + 1;

            //Print the output in the logger
            RCLCPP_INFO(node->get_logger(), "Success : %s", success ? "true" : "false");

            // Test de collision (because we use a cartesian planner)
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            planning_scene.checkCollision(collision_request, collision_result);
            is_in_collision = collision_result.collision;
            RCLCPP_INFO(node->get_logger(), "Test CC : Curent state is  %s", is_in_collision ? "in collision" : " not in collisions");

            if (is_in_collision)
            {
                RCLCPP_INFO(node->get_logger(), "Collision CS detected : success --> false and code error = 100");
                success = false;
                error_code = 100;
            }

            if (success)
            {
                RCLCPP_INFO(node->get_logger(), "Planning succeeded.");
                break; // Exit the loop if planning is successful
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Planning failed.");
                RCLCPP_INFO(node->get_logger(), "Error code : (%d)", error_code);

                if((error_code == -6) || (error_code == -2)){
                    //Time out : increase the planning time
                    if (planning_time < 20)
                    {
                        planning_time = planning_time + 5;
                        move_group_interface.setPlanningTime(planning_time);
                        RCLCPP_WARN(node->get_logger(), "Planning time increased : new value (%d) -->", planning_time);
                    }
                }
                rclcpp::sleep_for(std::chrono::seconds(1)); // Wait before retrying
            }
        }

        if (success)
        {
            RCLCPP_INFO(node->get_logger(), "Point [%f, %f, %f] is reachable.", point.x, point.y, point.z);
            prompt("Planning succeeded. Press 'Next' to execute the trajectory.");

            prompt("Press 'Next' to execute the planned trajectory.");
            execute_trajectory(move_group_interface, plan);
            rclcpp::sleep_for(std::chrono::seconds(5));

            //Extract finale orientation of the end-effector
            end_effector_orientation = get_end_effector_orientation(tf_buffer);
            RCLCPP_INFO(node->get_logger(), "End effector oriention [%f, %f, %f, %f].", end_effector_orientation[0],end_effector_orientation[1],end_effector_orientation[2],end_effector_orientation[3]);

            //Extract the finale position of the end-effector
            end_effector_position = get_end_effector_position(tf_buffer);
            RCLCPP_INFO(node->get_logger(), "End effector position [%f, %f, %f].", end_effector_position[0],end_effector_position[1],end_effector_position[2]);

            //Capture a picture
            try {
                RCLCPP_INFO(node->get_logger(), "Waiting for image...");
                cv::Mat image = image_manager->get_next_image();

                // Sauvegarde (si besoin)
                std::string image_filename = image_utils::generate_image_filename(package_path + "/images");
                cv::imwrite(image_filename, image);
                RCLCPP_INFO(node->get_logger(), "Image saved at: %s", image_filename.c_str());

                // Mise en niveaux de gris si nécessaire
                if (image.channels() == 3)
                cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

                // Détection de cercles
                auto cercles = image_utils::circle_detection_binarized(image);
                RCLCPP_INFO(node->get_logger(), "Cercles détectés : %ld", cercles.size());

                for (size_t i = 0; i < cercles.size(); ++i)
                {
                    const auto& cir = cercles[i];
                    RCLCPP_INFO(node->get_logger(), "Cercle %ld, centre : (%f,%f), rayon : %f", i, cir[0], cir[1], cir[2]);
                }

                //Création d'une imagette autour du premier cercle trouvé
                if (!cercles.empty()) {
                    auto c = cercles.front();
                    cv::Point centre(cvRound(c[0]), cvRound(c[1]));
                    cv::Mat imagette = image_utils::extraire_imagette(image, centre, 100, 100);
                    if (!imagette.empty()) {
                        std::string imagette_filename = image_utils::generate_imagette_filename(package_path + "/images");
                        cv::imwrite(imagette_filename, imagette);
                        focus_score = image_utils::sharpness_score(imagette); // Score de netteté sur l'imagette
                        RCLCPP_INFO(node->get_logger(), "Sharpness score: %.3f", focus_score);
                    }
                }else{
                    RCLCPP_WARN(node->get_logger(), "No cercle detected");
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Image capture failed: %s", e.what());
            }


            // Write the data to CSV file
            RCLCPP_INFO(node->get_logger(), "Success : Writing data to CSV file...");
            writeToCSV(last_configuration, true, csv_file, point,robotBase_to_box_x, robotBase_to_box_y, planning_time, max_planning_iterations, error_code, end_effector_orientation,end_effector_position, focus_score, step);
            RCLCPP_INFO(node->get_logger(), "Success : Data written to CSV file.");

        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Point [%f, %f, %f] is NOT reachable.", point.x, point.y, point.z);
            i = points.size(); //We stop the loop execution

        }

        // Reset of the variables of the planning
        planning_iterations = 0;
        planning_time = planning_time_init;
        success = false;
        error_code = 0;
        move_group_interface.setPlanningTime(planning_time_init);
        focus_score =0;

        rclcpp::sleep_for(std::chrono::seconds(3));
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}