#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
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
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <mutex>
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

    const char *moveItErrorCodeToString(int code)
    {
        switch (code)
        {
        case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
            return "SUCCESS";
        case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
            return "FAILURE";
        case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED:
            return "PLANNING_FAILED";
        case moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
        case moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED:
            return "CONTROL_FAILED";
        case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
            return "TIMED_OUT";
        case moveit_msgs::msg::MoveItErrorCodes::PREEMPTED:
            return "PREEMPTED";
        default:
            return "UNKNOWN";
        }
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
        support_surface_ = this->declare_parameter<std::string>("support_surface", "demo_table");

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
        execute_ = this->declare_parameter<bool>("execute", false);
        keep_alive_after_execute_failure_ = this->declare_parameter<bool>("keep_alive_after_execute_failure", true);
        wait_for_execution_action_servers_ = this->declare_parameter<bool>("wait_for_execution_action_servers", true);
        execute_action_server_wait_timeout_sec_ =
            this->declare_parameter<double>("execute_action_server_wait_timeout_sec", 20.0);
        arm_controller_action_name_ =
            this->declare_parameter<std::string>("arm_controller_action_name", "arm_controller/follow_joint_trajectory");
        hand_controller_action_name_ =
            this->declare_parameter<std::string>("hand_controller_action_name", "hand_controller/gripper_cmd");
        max_solutions_ = this->declare_parameter<int>("max_solutions", 4);
        task_timeout_sec_ = this->declare_parameter<double>("task_timeout_sec", 60.0);

        pick_xyz_ = this->declare_parameter<std::vector<double>>("pick_pose_xyz", std::vector<double>{0.55, 0.0, 0.20});
        place_xyz_ = this->declare_parameter<std::vector<double>>("place_pose_xyz", std::vector<double>{0.45, -0.25, 0.20});
        object_size_xyz_ = this->declare_parameter<std::vector<double>>("object_size_xyz", std::vector<double>{0.04, 0.04, 0.12});
        support_surface_xyz_ = this->declare_parameter<std::vector<double>>(
            "support_surface_xyz", std::vector<double>{0.55, 0.0, 0.52});
        support_surface_size_xyz_ = this->declare_parameter<std::vector<double>>(
            "support_surface_size_xyz", std::vector<double>{0.8, 0.8, 0.04});

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

        const double support_surface_x = this->declare_parameter<double>("support_surface_x", support_surface_xyz_[0]);
        const double support_surface_y = this->declare_parameter<double>("support_surface_y", support_surface_xyz_[1]);
        const double support_surface_z = this->declare_parameter<double>("support_surface_z", support_surface_xyz_[2]);
        support_surface_xyz_ = {support_surface_x, support_surface_y, support_surface_z};

        const double support_surface_size_x =
            this->declare_parameter<double>("support_surface_size_x", support_surface_size_xyz_[0]);
        const double support_surface_size_y =
            this->declare_parameter<double>("support_surface_size_y", support_surface_size_xyz_[1]);
        const double support_surface_size_z =
            this->declare_parameter<double>("support_surface_size_z", support_surface_size_xyz_[2]);
        support_surface_size_xyz_ = {support_surface_size_x, support_surface_size_y, support_surface_size_z};

        approach_distance_ = this->declare_parameter<double>("approach_distance", 0.10);
        approach_min_distance_ = this->declare_parameter<double>("approach_min_distance", 0.02);
        lift_distance_ = this->declare_parameter<double>("lift_distance", 0.12);
        place_retreat_distance_ = this->declare_parameter<double>("place_retreat_distance", 0.10);
        plan_velocity_scaling_ = this->declare_parameter<double>("plan_velocity_scaling", 0.2);
        plan_acceleration_scaling_ = this->declare_parameter<double>("plan_acceleration_scaling", 0.2);
        cartesian_step_size_ = this->declare_parameter<double>("cartesian_step_size", 0.01);
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

        auto_run_on_start_ = this->declare_parameter<bool>("auto_run_on_start", true);

        run_task_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "run_task",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                response->success = this->run();
                response->message = response->success ? "MTC run completed" : "MTC run failed";
            });

        reset_and_run_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_and_run",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                if (!this->resetPlanningSceneForRetry())
                {
                    response->success = false;
                    response->message = "MTC reset_and_run failed in planning scene reset step";
                    return;
                }
                task_.reset();
                response->success = this->run();
                response->message = response->success ? "MTC reset_and_run completed" : "MTC reset_and_run failed";
            });

        reset_task_state_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_task_state",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                task_.reset();
                response->success = this->resetPlanningSceneForRetry();
                response->message = response->success ? "MTC task/planning-scene reset completed" : "MTC task/planning-scene reset failed";
            });
    }

    bool autoRunOnStart() const
    {
        return auto_run_on_start_;
    }

    bool run()
    {
        {
            std::lock_guard<std::mutex> lk(run_mutex_);
            if (run_in_progress_)
            {
                RCLCPP_WARN(this->get_logger(), "MTC run request ignored because another run is in progress");
                return false;
            }
            run_in_progress_ = true;
        }

        struct RunScopeGuard
        {
            MTCPickPlaceDemo *self;
            ~RunScopeGuard()
            {
                std::lock_guard<std::mutex> lk(self->run_mutex_);
                self->run_in_progress_ = false;
            }
        } guard{this};

        if (!waitForJointState())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "No joint state received on '%s' within %.2f seconds. Abort MTC run.",
                         joint_state_topic_.c_str(),
                         joint_state_wait_timeout_sec_);
            return false;
        }

        if (!addCollisionObject())
        {
            return false;
        }

        task_ = std::make_unique<mtc::Task>();
        mtc::Task &task = *task_;
        task.stages()->setName("pick_place_task");
        task.loadRobotModel(shared_from_this());

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
        sampling_planner->setProperty("max_velocity_scaling_factor", plan_velocity_scaling_);
        sampling_planner->setProperty("max_acceleration_scaling_factor", plan_acceleration_scaling_);

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(plan_velocity_scaling_);
        cartesian_planner->setMaxAccelerationScalingFactor(plan_acceleration_scaling_);
        cartesian_planner->setStepSize(cartesian_step_size_);

        task.setProperty("group", arm_group_);
        task.setProperty("eef", hand_group_);
        task.setProperty("hand", hand_group_);
        task.setProperty("hand_grasping_frame", hand_frame_);
        task.setProperty("ik_frame", hand_frame_);
        if (task_timeout_sec_ > 0.0)
        {
            task.setTimeout(task_timeout_sec_);
        }

        mtc::TrajectoryExecutionInfo arm_exec_info;
        arm_exec_info.controller_names = {"arm_controller"};
        mtc::TrajectoryExecutionInfo hand_exec_info;
        hand_exec_info.controller_names = {"hand_controller"};

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
            initial_state_ptr = applicability_filter.get();
            task.add(std::move(applicability_filter));
        }

        if (enable_initial_open_hand_stage_)
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(hand_group_);
            stage->setGoal(pregrasp_);
            stage->setTrajectoryExecutionInfo(hand_exec_info);
            initial_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        if (enable_move_to_pick_stage_)
        {
            auto stage = std::make_unique<mtc::stages::Connect>(
                "move to pick", mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
            stage->setTimeout(15.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->setTrajectoryExecutionInfo(arm_exec_info);
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
                stage->setTrajectoryExecutionInfo(arm_exec_info);
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
            if (enable_gripper_actions_)
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
                stage->setGroup(hand_group_);
                stage->setGoal(grasp_);
                stage->setTrajectoryExecutionInfo(hand_exec_info);
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
                stage->setTrajectoryExecutionInfo(arm_exec_info);
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

        if (enable_move_to_place_stage_)
        {
            auto stage = std::make_unique<mtc::stages::Connect>(
                "move to place", mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->setTrajectoryExecutionInfo(arm_exec_info);
            task.add(std::move(stage));
        }

        if (enable_move_to_place_stage_)
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
                stage->setTrajectoryExecutionInfo(arm_exec_info);
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

            if (enable_gripper_actions_)
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
                stage->setGroup(hand_group_);
                stage->setGoal(pregrasp_);
                stage->setTrajectoryExecutionInfo(hand_exec_info);
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
                stage->setTrajectoryExecutionInfo(arm_exec_info);
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
            stage->setTrajectoryExecutionInfo(arm_exec_info);
            task.add(std::move(stage));
        }

        try
        {
            task.init();
        }
        catch (const mtc::InitStageException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Task init failed: " << e);
            return false;
        }

        if (!task.plan(max_solutions_))
        {
            RCLCPP_ERROR(this->get_logger(), "MTC plan failed");
            std::stringstream ss;
            task.explainFailure(ss);
            RCLCPP_ERROR(this->get_logger(), "MTC failure detail:\n%s", ss.str().c_str());
            return false;
        }

        auto solution = task.solutions().front();
        task.introspection().publishSolution(*solution);
        RCLCPP_INFO(this->get_logger(), "Published best solution to MTC introspection topics");

        if (!execute_)
        {
            RCLCPP_INFO(this->get_logger(), "Execution disabled (`execute=false`); keeping node alive for RViz introspection");
            return true;
        }

        RCLCPP_INFO(this->get_logger(), "Starting task execution");
        if (wait_for_execution_action_servers_ && !waitForExecutionActionServers())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Execution action servers are not ready. Skipping execute to avoid immediate CONTROL_FAILED.");
            if (keep_alive_after_execute_failure_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Keeping node alive after execute precheck failure for RViz analysis");
            }
            return false;
        }

        const auto result = task.execute(*solution);
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Task execution failed: code=%d (%s)",
                         result.val,
                         moveItErrorCodeToString(result.val));
            if (keep_alive_after_execute_failure_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Keeping node alive after execute failure for RViz analysis (`keep_alive_after_execute_failure=true`)");
            }
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "MTC pick and place execution completed successfully");
        return true;
    }

private:
    bool resetPlanningSceneForRetry()
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        moveit_msgs::msg::PlanningScene scene_msg;
        scene_msg.is_diff = true;
        scene_msg.robot_state.is_diff = true;

        moveit_msgs::msg::AttachedCollisionObject detach;
        detach.link_name = hand_frame_;
        detach.object.id = object_name_;
        detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        scene_msg.robot_state.attached_collision_objects.push_back(detach);

        const bool detach_ok = psi.applyPlanningScene(scene_msg);
        if (!detach_ok)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to apply attached-object cleanup for '%s' on link '%s'",
                        object_name_.c_str(),
                        hand_frame_.c_str());
        }

        std::vector<std::string> ids_to_remove{object_name_};
        if (!support_surface_.empty())
        {
            ids_to_remove.push_back(support_surface_);
        }
        psi.removeCollisionObjects(ids_to_remove);

        RCLCPP_INFO(this->get_logger(), "Planning scene reset completed for retry");
        return true;
    }

    bool waitForExecutionActionServers()
    {
        if (execute_action_server_wait_timeout_sec_ <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Skipping execution action server wait because execute_action_server_wait_timeout_sec<=0");
            return true;
        }

        const auto timeout = std::chrono::duration<double>(execute_action_server_wait_timeout_sec_);
        const auto arm_ok = waitForActionServer<control_msgs::action::FollowJointTrajectory>(
            arm_controller_action_name_,
            timeout,
            "arm_controller");
        const auto hand_ok = waitForActionServer<control_msgs::action::GripperCommand>(
            hand_controller_action_name_,
            timeout,
            "hand_controller");
        return arm_ok && hand_ok;
    }

    template <typename ActionT>
    bool waitForActionServer(const std::string &action_name,
                             const std::chrono::duration<double> &timeout,
                             const char *label)
    {
        if (action_name.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Action name for %s is empty", label);
            return false;
        }

        auto client = rclcpp_action::create_client<ActionT>(shared_from_this(), action_name);
        RCLCPP_INFO(this->get_logger(),
                    "Waiting up to %.1f s for %s action server: %s",
                    timeout.count(),
                    label,
                    action_name.c_str());

        if (!client->wait_for_action_server(timeout))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Timed out waiting for %s action server: %s",
                         label,
                         action_name.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "%s action server is ready: %s", label, action_name.c_str());
        return true;
    }

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

    bool addCollisionObject()
    {
        if (pick_xyz_.size() != 3 || place_xyz_.size() != 3 || object_size_xyz_.size() != 3 ||
            support_surface_xyz_.size() != 3 || support_surface_size_xyz_.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter xyz vectors must all have size 3");
            return false;
        }

        moveit::planning_interface::PlanningSceneInterface psi;

        std::vector<moveit_msgs::msg::CollisionObject> objects;

        if (!support_surface_.empty())
        {
            moveit_msgs::msg::CollisionObject table;
            table.id = support_surface_;
            table.header.frame_id = world_frame_;

            shape_msgs::msg::SolidPrimitive table_primitive;
            table_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            table_primitive.dimensions = {
                support_surface_size_xyz_[0],
                support_surface_size_xyz_[1],
                support_surface_size_xyz_[2]};

            geometry_msgs::msg::Pose table_pose;
            table_pose.orientation.w = 1.0;
            table_pose.position.x = support_surface_xyz_[0];
            table_pose.position.y = support_surface_xyz_[1];
            table_pose.position.z = support_surface_xyz_[2];

            table.primitives.push_back(table_primitive);
            table.primitive_poses.push_back(table_pose);
            table.operation = moveit_msgs::msg::CollisionObject::ADD;
            objects.push_back(table);
        }

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
        objects.push_back(obj);

        psi.applyCollisionObjects(objects);

        if (!support_surface_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Added support surface '%s' in frame '%s'", support_surface_.c_str(), world_frame_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Added collision object '%s' in frame '%s'", object_name_.c_str(), world_frame_.c_str());
        return true;
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
    bool execute_{};
    bool keep_alive_after_execute_failure_{};
    bool wait_for_execution_action_servers_{};
    double execute_action_server_wait_timeout_sec_{};
    std::string arm_controller_action_name_;
    std::string hand_controller_action_name_;
    int max_solutions_{};
    double task_timeout_sec_{};

    std::vector<double> pick_xyz_;
    std::vector<double> place_xyz_;
    std::vector<double> object_size_xyz_;
    std::vector<double> support_surface_xyz_;
    std::vector<double> support_surface_size_xyz_;

    double approach_distance_{};
    double approach_min_distance_{};
    double lift_distance_{};
    double place_retreat_distance_{};
    double plan_velocity_scaling_{};
    double plan_acceleration_scaling_{};
    double cartesian_step_size_{};
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
    bool auto_run_on_start_{};
    bool run_in_progress_{false};
    std::mutex run_mutex_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_task_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_and_run_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_task_state_srv_;
    std::unique_ptr<mtc::Task> task_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MTCPickPlaceDemo>();
    std::thread spinning_thread([node]
                                { rclcpp::spin(node); });

    rclcpp::sleep_for(std::chrono::seconds(2));
    if (node->autoRunOnStart())
    {
        node->run();
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "auto_run_on_start=false, waiting for service-triggered run_task/reset_and_run");
    }

    // Keep introspection topics alive for RViz after planning finishes.
    spinning_thread.join();
    return 0;
}
