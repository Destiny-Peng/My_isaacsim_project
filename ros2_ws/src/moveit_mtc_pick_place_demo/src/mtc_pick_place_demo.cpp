#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_set>

namespace mtc = moveit::task_constructor;

namespace
{
    Eigen::Isometry3d makeGraspFrameTransform(double x, double y, double z, const std::vector<double> &rpy)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translate(Eigen::Vector3d(x, y, z));
        if (rpy.size() == 3)
        {
            transform.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));
            transform.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
            transform.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
        }
        return transform;
    }
} // namespace

class MTCPickPlaceDemo : public rclcpp::Node
{
public:
    MTCPickPlaceDemo() : Node("mtc_pick_place_demo")
    {
        arm_group_ = this->declare_parameter<std::string>("arm_group_name", "arm");
        hand_group_ = this->declare_parameter<std::string>("hand_group_name", "hand");
        hand_frame_ = this->declare_parameter<std::string>("hand_frame", "panda_panda_hand");
        world_frame_ = this->declare_parameter<std::string>("world_frame", "world");
        object_name_ = this->declare_parameter<std::string>("object_name", "demo_cube");
        support_surface_ = this->declare_parameter<std::string>("support_surface", "");

        pregrasp_ = this->declare_parameter<std::string>("pregrasp", "open");
        grasp_ = this->declare_parameter<std::string>("grasp", "close");
        enable_gripper_actions_ = this->declare_parameter<bool>("enable_gripper_actions", false);
        enable_initial_open_hand_stage_ = this->declare_parameter<bool>("enable_initial_open_hand_stage", false);
        enable_move_to_pick_stage_ = this->declare_parameter<bool>("enable_move_to_pick_stage", true);
        enable_move_to_place_stage_ = this->declare_parameter<bool>("enable_move_to_place_stage", true);
        enable_retreat_after_place_stage_ = this->declare_parameter<bool>("enable_retreat_after_place_stage", true);
        enable_forbid_hand_object_collision_after_place_ =
            this->declare_parameter<bool>("enable_forbid_hand_object_collision_after_place", true);
        enable_move_home_stage_ = this->declare_parameter<bool>("enable_move_home_stage", true);
        max_solutions_ = this->declare_parameter<int>("max_solutions", 4);
        task_timeout_sec_ = this->declare_parameter<double>("task_timeout_sec", 60.0);

        pick_xyz_ = this->declare_parameter<std::vector<double>>("pick_pose_xyz", std::vector<double>{0.55, 0.0, 0.20});
        place_xyz_ = this->declare_parameter<std::vector<double>>("place_pose_xyz", std::vector<double>{0.45, -0.25, 0.20});
        object_size_xyz_ = this->declare_parameter<std::vector<double>>("object_size_xyz", std::vector<double>{0.04, 0.04, 0.12});

        const double pick_x = this->declare_parameter<double>("pick_x", pick_xyz_[0]);
        const double pick_y = this->declare_parameter<double>("pick_y", pick_xyz_[1]);
        const double pick_z = this->declare_parameter<double>("pick_z", pick_xyz_[2]);
        pick_xyz_ = {pick_x, pick_y, pick_z};

        const double place_x = this->declare_parameter<double>("place_x", place_xyz_[0]);
        const double place_y = this->declare_parameter<double>("place_y", place_xyz_[1]);
        const double place_z = this->declare_parameter<double>("place_z", place_xyz_[2]);
        place_xyz_ = {place_x, place_y, place_z};

        const double object_size_x = this->declare_parameter<double>("object_size_x", object_size_xyz_[0]);
        const double object_size_y = this->declare_parameter<double>("object_size_y", object_size_xyz_[1]);
        const double object_size_z = this->declare_parameter<double>("object_size_z", object_size_xyz_[2]);
        object_size_xyz_ = {object_size_x, object_size_y, object_size_z};

        approach_distance_ = this->declare_parameter<double>("approach_distance", 0.10);
        approach_min_distance_ = this->declare_parameter<double>("approach_min_distance", 0.02);
        lift_distance_ = this->declare_parameter<double>("lift_distance", 0.12);
        place_retreat_distance_ = this->declare_parameter<double>("place_retreat_distance", 0.10);
        grasp_x_offset_ = this->declare_parameter<double>("grasp_x_offset", 0.0);
        grasp_y_offset_ = this->declare_parameter<double>("grasp_y_offset", 0.0);
        grasp_z_offset_ = this->declare_parameter<double>("grasp_z_offset", 0.08);
        grasp_angle_delta_ = this->declare_parameter<double>("grasp_angle_delta", M_PI / 12.0);
        grasp_max_ik_solutions_ = this->declare_parameter<int>("grasp_max_ik_solutions", 16);
        grasp_ik_ignore_collisions_ = this->declare_parameter<bool>("grasp_ik_ignore_collisions", false);
        additional_allowed_collision_links_ = this->declare_parameter<std::vector<std::string>>(
            "additional_allowed_collision_links", std::vector<std::string>{"panda_panda_link7"});
        grasp_rpy_ = this->declare_parameter<std::vector<double>>(
            "grasp_rpy", std::vector<double>{1.571, 0.785, 1.571});
        enable_allow_grasp_collision_stage_ =
            this->declare_parameter<bool>("enable_allow_grasp_collision_stage", true);

        joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
        joint_state_wait_timeout_sec_ = this->declare_parameter<double>("joint_state_wait_timeout_sec", 10.0);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_,
            rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                if (!msg->name.empty())
                {
                    joint_state_received_ = true;
                }
            });
    }

    void run()
    {
        if (!waitForJointState())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "No joint state received on '%s' within %.2f seconds. Abort MTC run.",
                         joint_state_topic_.c_str(),
                         joint_state_wait_timeout_sec_);
            return;
        }

        addCollisionObject();

        task_ = std::make_unique<mtc::Task>();
        mtc::Task &task = *task_;
        task.stages()->setName("pick_place_task");
        task.loadRobotModel(shared_from_this());

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(0.01);

        task.setProperty("group", arm_group_);
        task.setProperty("eef", hand_group_);
        task.setProperty("hand", hand_group_);
        task.setProperty("hand_grasping_frame", hand_frame_);
        task.setProperty("ik_frame", hand_frame_);
        if (task_timeout_sec_ > 0.0)
        {
            task.setTimeout(task_timeout_sec_);
        }

        mtc::Stage *initial_state_ptr = nullptr;
        {
            auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
            auto applicability_filter = std::make_unique<mtc::stages::PredicateFilter>(
                "applicability test", std::move(current_state));
            applicability_filter->setPredicate(
                [object_name = object_name_](const mtc::SolutionBase &s, std::string &comment)
                {
                    if (s.start()->scene()->getCurrentState().hasAttachedBody(object_name))
                    {
                        comment = "object with id '" + object_name + "' is already attached and cannot be picked";
                        return false;
                    }
                    return true;
                });
            task.add(std::move(applicability_filter));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(hand_group_);
            stage->setGoal(pregrasp_);
            initial_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::Connect>(
                "move to pick", mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
            stage->setTimeout(15.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage));
        }
        /****************************************************
         *                                                  *
         *               Pick Object                        *
         *                                                  *
         ***************************************************/
        mtc::Stage *pick_stage_ptr = nullptr;
        {
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            task.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
            grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
            /****************************************************
      ---- *               Approach Object                    *
             ***************************************************/
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
                stage->properties().set("marker_ns", "approach_object");
                stage->properties().set("link", hand_frame_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(approach_min_distance_, approach_distance_);
                stage->setIKFrame(hand_frame_);

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = hand_frame_;
                vec.vector.x = -1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }
            /****************************************************
      ---- *               Generate Grasp Pose                *
             ***************************************************/
            {
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose(pregrasp_);
                stage->setObject(object_name_);
                stage->setAngleDelta(grasp_angle_delta_);
                stage->setMonitoredStage(initial_state_ptr);

                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(grasp_max_ik_solutions_);
                wrapper->setMinSolutionDistance(1.0);
                // wrapper->setIgnoreCollisions(grasp_ik_ignore_collisions_);
                wrapper->setIKFrame(makeGraspFrameTransform(
                                        grasp_x_offset_,
                                        grasp_y_offset_,
                                        grasp_z_offset_,
                                        grasp_rpy_),
                                    hand_frame_);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                grasp->insert(std::move(wrapper));
            }
            /****************************************************
      ---- *               Allow Collision (hand object)   *
             ***************************************************/
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                auto collision_links = task.getRobotModel()
                                           ->getJointModelGroup(hand_group_)
                                           ->getLinkModelNamesWithCollisionGeometry();
                const std::unordered_set<std::string> robot_links(
                    task.getRobotModel()->getLinkModelNames().begin(),
                    task.getRobotModel()->getLinkModelNames().end());
                for (const auto &link_name : additional_allowed_collision_links_)
                {
                    if (robot_links.find(link_name) != robot_links.end())
                    {
                        collision_links.push_back(link_name);
                    }
                }
                std::sort(collision_links.begin(), collision_links.end());
                collision_links.erase(std::unique(collision_links.begin(), collision_links.end()), collision_links.end());
                stage->allowCollisions(object_name_, collision_links, true);
                grasp->insert(std::move(stage));
            }
            /****************************************************
      ---- *               Close Hand                      *
             ***************************************************/
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
                stage->setGroup(hand_group_);
                stage->setGoal(grasp_);
                grasp->insert(std::move(stage));
            }
            /****************************************************
      .... *               Attach Object                      *
             ***************************************************/
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
                stage->attachObject(object_name_, hand_frame_);
                grasp->insert(std::move(stage));
            }
            /****************************************************
      .... *               Allow collision (object support)   *
             ***************************************************/
            if (!support_surface_.empty())
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
                stage->allowCollisions({object_name_}, {support_surface_}, true);
                grasp->insert(std::move(stage));
            }
            /****************************************************
      .... *               Lift object                        *
             ***************************************************/
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0.03, lift_distance_);
                stage->setIKFrame(hand_frame_);
                stage->properties().set("marker_ns", "lift_object");

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = world_frame_;
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }
            /****************************************************
      .... *               Forbid collision (object support)  *
             ***************************************************/
            if (!support_surface_.empty())
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
                stage->allowCollisions({object_name_}, {support_surface_}, false);
                grasp->insert(std::move(stage));
            }

            pick_stage_ptr = grasp.get();
            task.add(std::move(grasp));
        }

        {
            auto stage = std::make_unique<mtc::stages::Connect>(
                "move to place", mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage));
        }

        {
            auto place = std::make_unique<mtc::SerialContainer>("place object");
            task.properties().exposeTo(place->properties(), {"eef", "hand", "group", "ik_frame"});
            place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
                stage->properties().set("marker_ns", "lower_object");
                stage->properties().set("link", hand_frame_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0.03, 0.13);
                stage->setIKFrame(hand_frame_);

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = world_frame_;
                vec.vector.z = -1.0;
                stage->setDirection(vec);
                place->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
                stage->properties().set("marker_ns", "place_pose");
                stage->setObject(object_name_);

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = world_frame_;
                pose.pose.orientation.w = 1.0;
                pose.pose.position.x = place_xyz_[0];
                pose.pose.position.y = place_xyz_[1];
                pose.pose.position.z = place_xyz_[2];
                stage->setPose(pose);
                stage->setMonitoredStage(pick_stage_ptr);

                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(4);
                wrapper->setIKFrame(makeGraspFrameTransform(
                                        grasp_x_offset_,
                                        grasp_y_offset_,
                                        grasp_z_offset_,
                                        grasp_rpy_),
                                    hand_frame_);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                place->insert(std::move(wrapper));
            }

            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
                stage->setGroup(hand_group_);
                stage->setGoal(pregrasp_);
                place->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
                stage->detachObject(object_name_, hand_frame_);
                place->insert(std::move(stage));
            }

            if (enable_retreat_after_place_stage_)
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("retreat after place", sampling_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setGroup(arm_group_);
                stage->setGoal("ready");
                place->insert(std::move(stage));
            }

            if (enable_forbid_hand_object_collision_after_place_)
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
                stage->allowCollisions(object_name_, *task.getRobotModel()->getJointModelGroup(hand_group_), false);
                place->insert(std::move(stage));
            }

            task.add(std::move(place));
        }

        if (enable_move_home_stage_)
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setGoal("ready");
            stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
            task.add(std::move(stage));
        }

        try
        {
            task.init();
        }
        catch (const mtc::InitStageException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Task init failed: " << e);
            return;
        }

        if (!task.plan(max_solutions_))
        {
            RCLCPP_ERROR(this->get_logger(), "MTC plan failed");
            std::stringstream ss;
            task.explainFailure(ss);
            RCLCPP_ERROR(this->get_logger(), "MTC failure detail:\n%s", ss.str().c_str());
            return;
        }

        auto solution = task.solutions().front();
        task.introspection().publishSolution(*solution);
        RCLCPP_INFO(this->get_logger(), "Published best solution to MTC introspection topics");

        // const auto result = task.execute(*solution);
        // if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Task execution failed with code: %d", result.val);
        //     return;
        // }

        // RCLCPP_INFO(this->get_logger(), "MTC pick and place completed successfully");
    }

private:
    bool waitForJointState()
    {
        if (joint_state_received_)
        {
            return true;
        }

        const auto start = std::chrono::steady_clock::now();
        rclcpp::WallRate rate(50.0);
        while (rclcpp::ok())
        {
            rclcpp::spin_some(shared_from_this());
            if (joint_state_received_)
            {
                RCLCPP_INFO(this->get_logger(), "Received joint state from topic '%s'", joint_state_topic_.c_str());
                return true;
            }

            const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed >= joint_state_wait_timeout_sec_)
            {
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    void addCollisionObject()
    {
        if (pick_xyz_.size() != 3 || place_xyz_.size() != 3 || object_size_xyz_.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter xyz vectors must all have size 3");
            return;
        }

        moveit::planning_interface::PlanningSceneInterface psi;

        moveit_msgs::msg::CollisionObject obj;
        obj.id = object_name_;
        obj.header.frame_id = world_frame_;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {
            object_size_xyz_[0],
            object_size_xyz_[1],
            object_size_xyz_[2]};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = pick_xyz_[0];
        pose.position.y = pick_xyz_[1];
        pose.position.z = pick_xyz_[2];

        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        psi.applyCollisionObject(obj);
        RCLCPP_INFO(this->get_logger(), "Added collision object '%s' in frame '%s'", object_name_.c_str(), world_frame_.c_str());
    }

    std::string arm_group_;
    std::string hand_group_;
    std::string hand_frame_;
    std::string world_frame_;
    std::string object_name_;
    std::string support_surface_;

    std::string pregrasp_;
    std::string grasp_;
    bool enable_gripper_actions_{};
    bool enable_initial_open_hand_stage_{};
    bool enable_move_to_pick_stage_{};
    bool enable_move_to_place_stage_{};
    bool enable_retreat_after_place_stage_{};
    bool enable_forbid_hand_object_collision_after_place_{};
    bool enable_move_home_stage_{};
    int max_solutions_{};
    double task_timeout_sec_{};

    std::vector<double> pick_xyz_;
    std::vector<double> place_xyz_;
    std::vector<double> object_size_xyz_;

    double approach_distance_{};
    double approach_min_distance_{};
    double lift_distance_{};
    double place_retreat_distance_{};
    double grasp_x_offset_{};
    double grasp_y_offset_{};
    double grasp_z_offset_{};
    double grasp_angle_delta_{};
    int grasp_max_ik_solutions_{};
    bool grasp_ik_ignore_collisions_{};
    std::vector<std::string> additional_allowed_collision_links_;
    std::vector<double> grasp_rpy_;
    bool enable_allow_grasp_collision_stage_{};

    std::string joint_state_topic_;
    double joint_state_wait_timeout_sec_{};
    bool joint_state_received_{false};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::unique_ptr<mtc::Task> task_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MTCPickPlaceDemo>();
    std::thread spinning_thread([node]
                                { rclcpp::spin(node); });

    rclcpp::sleep_for(std::chrono::seconds(2));
    node->run();

    // Keep introspection topics alive for RViz after planning finishes.
    spinning_thread.join();
    return 0;
}
