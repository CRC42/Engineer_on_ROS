#include "manipulator_control/arm_control.h"
#include "manipulator_control/scene_generate.h"
#include <std_msgs/Float64MultiArray.h>
#include "sp_common/SingleJointWrite.h"


void pose_callback(const geometry_msgs::Pose::ConstPtr& pose_, manipulator_control::Manipulator* manipulator)
{
    manipulator->write(*pose_);      
}

void state_callback(const std_msgs::Float64MultiArray::ConstPtr& state_, manipulator_control::Manipulator* manipulator)
{
    std::vector<double> state;
    state = state_->data;
    manipulator->write(state);      
}

void single_state_callback(const sp_common::SingleJointWrite::ConstPtr& state_, manipulator_control::Manipulator* manipulator)
{
    double state;
    int num;
    state = state_->state;
    num = state_->num;
    manipulator->singlewrite(state, num);
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "trajectory_control", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(3);
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_MANIPULATOR);
    moveit::planning_interface::MoveGroupInterface grip_group_interface(PLANNING_GROUP_GRIPPER);
    manipulator_control::Manipulator manipulator_(move_group_interface, grip_group_interface);
    manipulator_control::Scene scene;
    scene.init();
    ros::NodeHandle nh; 
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("/moveit/pose_sub",10, boost::bind(&pose_callback, _1, &manipulator_));
    ros::Subscriber state_sub = nh.subscribe<std_msgs::Float64MultiArray>("/moveit/state_sub",10, boost::bind(&state_callback, _1, &manipulator_));
    ros::Subscriber single_sub = nh.subscribe<sp_common::SingleJointWrite>("/moveit/single_state_sub",10, boost::bind(&single_state_callback, _1, &manipulator_));
    spinner.start(); 
   

    if (manipulator_.init())
    {
        while(ros::ok())
        {       
            manipulator_.read();
            if (manipulator_.get_executed() == false)
            {
                manipulator_.execute();
                manipulator_.set_executed(true);
            }
            ros::spinOnce();
            sleep(1);      
        }
    }    
    return 0;
}/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_control");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

    // 创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface current_scene;

    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "seven_dof_arm_cylinder";

    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 0.2;

    // 设置障碍物的位置
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  1.0;
    pose.position.y = -1.4;
    pose.position.z =  1.4;

    // 将障碍物的属性、位置加入到障碍物的实例中
    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;

    // 创建一个障碍物的列表，把之前创建的障碍物实例加入其中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cylinder);

    // 所有障碍物加入列表后（这里只有一个障碍物），再把障碍物加入到当前的情景中，如果要删除障碍物，使用removeCollisionObjects(collision_objects)
    current_scene.addCollisionObjects(collision_objects);

    ros::shutdown();

    return 0;
}*/

