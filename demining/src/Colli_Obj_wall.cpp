#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometric_shapes/shape_operations.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "building_workspace");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    Eigen::Vector3d b(0.001, 0.001, 0.001);

    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "wall";
    shapes::Mesh* m = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl", b);
    ROS_INFO("wall mesh loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    collision_object.meshes.resize(1);
    collision_object.mesh_poses.resize(1);
    collision_object.meshes[0] = mesh;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.mesh_poses[0].position.x = -0.5;
    collision_object.mesh_poses[0].position.y = -0.5;
    collision_object.mesh_poses[0].position.z = -0.1;
    collision_object.mesh_poses[0].orientation.x = 1.57;

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_vector;
    collision_vector.push_back(collision_object);

    planning_scene_interface.planning_scene_interface.applyCollisionObject(collision_vector);
    ROS_INFO("Wall added into the world");
    move_group.attachObject(collision_object.id);
    sleep(5.0);
    ros::shutdown();
    return 0;
}