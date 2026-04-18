#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>

namespace mtc = moveit::task_constructor;

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

        pregrasp_ = this->declare_parameter<std::string>("pregrasp", "open");
        grasp_ = this->declare_parameter<std::string>("grasp", "close");
        enable_gripper_actions_ = this->declare_parameter<bool>("enable_gripper_actions", false);

        pick_xyz_ = this->declare_parameter<std::vector<double>>("pick_pose_xyz", std::vector<double>{0.55, 0.0, 0.20});
        place_xyz_ = this->declare_parameter<std::vector<double>>("place_pose_xyz", std::vector<double>{0.45, -0.25, 0.20});
        object_size_xyz_ = this->declare_parameter<std::vector<double>>("object_size_xyz", std::vector<double>{0.04, 0.04, 0.12});

        // Keep tutorial-style vector params, but allow simple scalar overrides from launch CLI.
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
        lift_distance_ = this->declare_parameter<double>("lift_distance", 0.12);
        place_retreat_distance_ = this->declare_parameter<double>("place_retreat_distance", 0.10);
        grasp_z_offset_ = this->declare_parameter<double>("grasp_z_offset", 0.08);
        grasp_angle_delta_ = this->declare_parameter<double>("grasp_angle_delta", M_PI / 12.0);
        grasp_max_ik_solutions_ = this->declare_parameter<int>("grasp_max_ik_solutions", 16);
        grasp_rpy_ = this->declare_parameter<std::vector<double>>(
            "grasp_rpy", std::vector<double>{1.571, 0.785, 1.571});
        enable_allow_grasp_collision_stage_ =
            this->declare_parameter<bool>("enable_allow_grasp_collision_stage", false);
    }

    void run()
    {
        addCollisionObject();

        mtc::Task task;
        task.stages()->setName("pick_place_task");
        task.loadRobotModel(shared_from_this());

        auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
        pipeline->setProperty("goal_joint_tolerance", 1e-3);

        task.setProperty("group", arm_group_);
        task.setProperty("eef", hand_group_);
        task.setProperty("ik_frame", hand_frame_);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
        mtc::Stage *current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        if (enable_gripper_actions_)
        {
            auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", pipeline);
            open_hand->setGroup(hand_group_);
            open_hand->setGoal(pregrasp_);
            task.add(std::move(open_hand));
        }

        auto move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move to pick",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_, pipeline}});
        move_to_pick->setTimeout(10.0);
        move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(move_to_pick));

        auto pick = std::make_unique<mtc::SerialContainer>("pick object");
        pick->setProperty("eef", hand_group_);
        pick->setProperty("group", arm_group_);
        pick->setProperty("ik_frame", hand_frame_);
        pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", pipeline);
            stage->properties().set("marker_ns", "approach");
            stage->properties().set("link", hand_frame_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
            stage->setMinMaxDistance(0.02, approach_distance_);
            stage->setIKFrame(hand_frame_);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            pick->insert(std::move(stage));
        }

        if (enable_allow_grasp_collision_stage_)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow hand-object collision");
            auto collision_links = task.getRobotModel()
                                       ->getJointModelGroup(hand_group_)
                                       ->getLinkModelNamesWithCollisionGeometry();
            const auto arm_links = task.getRobotModel()
                                       ->getJointModelGroup(arm_group_)
                                       ->getLinkModelNamesWithCollisionGeometry();
            collision_links.insert(collision_links.end(), arm_links.begin(), arm_links.end());
            std::sort(collision_links.begin(), collision_links.end());
            collision_links.erase(std::unique(collision_links.begin(), collision_links.end()), collision_links.end());
            stage->allowCollisions(object_name_, collision_links, true);
            pick->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose(pregrasp_);
            stage->setObject(object_name_);
            stage->setAngleDelta(grasp_angle_delta_);
            stage->setMonitoredStage(current_state_ptr);

            geometry_msgs::msg::PoseStamped grasp_pose;
            grasp_pose.header.frame_id = object_name_;
            grasp_pose.pose.position.z = grasp_z_offset_;
            if (grasp_rpy_.size() == 3)
            {
                tf2::Quaternion q;
                q.setRPY(grasp_rpy_[0], grasp_rpy_[1], grasp_rpy_[2]);
                grasp_pose.pose.orientation = tf2::toMsg(q);
            }
            else
            {
                grasp_pose.pose.orientation.w = 1.0;
            }
            stage->setPose(grasp_pose);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(grasp_max_ik_solutions_);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIgnoreCollisions(true);
            wrapper->setIKFrame(hand_frame_);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            pick->insert(std::move(wrapper));
        }

        if (enable_gripper_actions_)
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", pipeline);
            stage->setGroup(hand_group_);
            stage->setGoal(grasp_);
            pick->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object_name_, hand_frame_);
            pick->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", pipeline);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.03, lift_distance_);
            stage->setIKFrame(hand_frame_);
            stage->properties().set("marker_ns", "lift");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = world_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            pick->insert(std::move(stage));
        }

        task.add(std::move(pick));

        auto move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_, pipeline}});
        move_to_place->setTimeout(10.0);
        move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(move_to_place));

        auto place = std::make_unique<mtc::SerialContainer>("place object");
        place->setProperty("eef", hand_group_);
        place->setProperty("group", arm_group_);
        place->setProperty("ik_frame", hand_frame_);
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(object_name_);
            stage->setMonitoredStage(current_state_ptr);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = world_frame_;
            pose.pose.position.x = place_xyz_[0];
            pose.pose.position.y = place_xyz_[1];
            pose.pose.position.z = place_xyz_[2];
            pose.pose.orientation.w = 1.0;
            stage->setPose(pose);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(hand_frame_);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            place->insert(std::move(wrapper));
        }

        if (enable_gripper_actions_)
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", pipeline);
            stage->setGroup(hand_group_);
            stage->setGoal(pregrasp_);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid hand-object collision");
            stage->allowCollisions(object_name_,
                                   task.getRobotModel()->getJointModelGroup(hand_group_)->getLinkModelNamesWithCollisionGeometry(),
                                   false);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(object_name_, hand_frame_);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", pipeline);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.02, place_retreat_distance_);
            stage->setIKFrame(hand_frame_);
            stage->properties().set("marker_ns", "retreat");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = world_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        task.add(std::move(place));

        auto move_home = std::make_unique<mtc::stages::MoveTo>("move home", pipeline);
        move_home->setGroup(arm_group_);
        move_home->setGoal("ready");
        task.add(std::move(move_home));

        try
        {
            task.init();
        }
        catch (const mtc::InitStageException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Task init failed: " << e);
            return;
        }

        if (!task.plan(5))
        {
            RCLCPP_ERROR(this->get_logger(), "MTC plan failed");
            return;
        }

        auto solution = task.solutions().front();
        task.introspection().publishSolution(*solution);
        RCLCPP_INFO(this->get_logger(), "Published best solution to MTC introspection topics");

        const auto result = task.execute(*solution);
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Task execution failed with code: %d", result.val);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "MTC pick and place completed successfully");

        (void)current_state_ptr;
    }

private:
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
    std::string pregrasp_;
    std::string grasp_;
    bool enable_gripper_actions_{};

    std::vector<double> pick_xyz_;
    std::vector<double> place_xyz_;
    std::vector<double> object_size_xyz_;

    double approach_distance_{};
    double lift_distance_{};
    double place_retreat_distance_{};
    double grasp_z_offset_{};
    double grasp_angle_delta_{};
    int grasp_max_ik_solutions_{};
    std::vector<double> grasp_rpy_;
    bool enable_allow_grasp_collision_stage_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MTCPickPlaceDemo>();

    rclcpp::sleep_for(std::chrono::seconds(2));
    node->run();

    rclcpp::shutdown();
    return 0;
}
