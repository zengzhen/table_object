#include <ros/ros.h>
#include <tf/tf.h>

// msgs
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/CollisionObject.h>

// pcl_ros
#include <pcl/ros/conversions.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "table_bottle_publisher");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(0.5);   //loop rate for any loops appear in this cpp

    ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    
    moveit_msgs::CollisionObject collision_object;
    // Adding table as Box & bottle as Cylinder primitive as collision objects
    collision_object.header.frame_id = "/base";

    /* The id of the object is used to identify it. */
    collision_object.id = "table_bottle";
    
    /* Define a table to add to the world. */
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.6; //fixed X
    table_primitive.dimensions[1] = 1.5; //fixed Y
    table_primitive.dimensions[2] = 1.0; //fixed Z

    /* A pose for the table (specified relative to frame_id) */
    geometry_msgs::Pose table_pose;
    table_pose.position.x = 1;
    table_pose.position.y = 0.4;
    table_pose.position.z = -0.5;

    /* Define a cylinder to add to the world. */
    shape_msgs::SolidPrimitive bottle_primitive;
    bottle_primitive.type = bottle_primitive.CYLINDER;
    bottle_primitive.dimensions.resize(2);
    bottle_primitive.dimensions[0] = 0.2; //fixed height
    bottle_primitive.dimensions[1] = 0.03; //fixed radius

    /* A pose for the bottle (specified relative to frame_id) */
    geometry_msgs::Pose bottle_pose;
    bottle_pose.position.x = 0.4;
    bottle_pose.position.y = 0.2;
    bottle_pose.position.z = 0.1;
    
    /* Define a table to add to the world. */
    shape_msgs::Plane plane_primitive;
    //  -0.00853185 -0.897844 -0.44023 0.363923
    plane_primitive.coef[0] = -0.00853185;
    plane_primitive.coef[1] = -0.897844;
    plane_primitive.coef[2] = -0.44023;
    plane_primitive.coef[3] = 0.363923;

    /* A pose for the table (specified relative to frame_id) */
    geometry_msgs::Pose plane_pose;
//     table_pose.position.x = 1;
//     table_pose.position.y = 0.4;
//     table_pose.position.z = -0.5;

    collision_object.primitives.push_back(table_primitive);
    collision_object.primitives.push_back(bottle_primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.primitive_poses.push_back(bottle_pose); 
    
//     collision_object.planes.push_back(plane_primitive);
//     collision_object.plane_poses.push_back(plane_pose);
    
    collision_object.operation = collision_object.ADD;
    
    while(collision_object_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();    
    }
    
    collision_object_publisher.publish(collision_object);
    sleep(2.0);
    
//     tf::Quaternion ori;
//     geometry_msgs::Quaternion ori_quat;
//     
//     float count = 0;
//     while (ros::ok())
//     {
//         collision_object.operation = collision_object.MOVE;
//         ori.setRPY(count, 0, 0);
//         tf::quaternionTFToMsg(ori, ori_quat);
//         bottle_pose.orientation = ori_quat;
//         //ROS_INFO("%f %f %f %f", ori_quat.x, ori_quat.y, ori_quat.z, ori_quat.w);
//         collision_object.primitive_poses[1]=bottle_pose;
//         collision_object_publisher.publish(collision_object); 
//         
// 
//         //ros::spinOnce();
//         loop_rate.sleep();
//         count=count+0.1;
//     }


    return 0;
}
