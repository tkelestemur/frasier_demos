#include <ros/package.h>
#include <point_cloud_proc/point_cloud_proc.h>
#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_openrave_controller.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tmc_msgs/Voice.h>

geometry_msgs::Vector3 force_data;

void wristSensorCb(const geometry_msgs::WrenchStampedConstPtr& data){
    force_data = data->wrench.force;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "table");
    ros::NodeHandle nh; //
    ros::AsyncSpinner spinner(4);
    spinner.start();

    FRASIEROpenRAVE rave(nh, true, true);
    FRASIERController controller(nh);
    std::string pkg_path = ros::package::getPath("frasier_demos");
    std::string pcp_config = pkg_path + "/config/floor.yaml";
    PointCloudProc pcp(nh, true, pcp_config);

    ros::Publisher talk_pub = nh.advertise<tmc_msgs::Voice>("/talk_request", 10);
    ros::Subscriber wrist_sensor_sub = nh.subscribe("/hsrb/wrist_wrench/compensated", 1, wristSensorCb);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_cli("/move_base/move", true);
    nav_cli.waitForServer();

    ros::Duration(1.0).sleep();

    controller.moveToKnownState(MOVE_STATE::PICK);
    controller.moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
    controller.graspOrRelease(GRIPPER_STATE::RELEASE);
    ros::Duration(2.0).sleep();

    point_cloud_proc::Plane table;
    bool table_segmented = pcp.segmentSinglePlane(table);
    if (table_segmented) {
        // Creating table collision object
        std::string table_name = "table";
        OpenRAVE::Vector table_size((table.max.x - table.min.x), (table.max.y - table.min.y), table.max.z);
        OpenRAVE::Transform table_pose(OpenRAVE::Vector(1, 0, 0, 0), OpenRAVE::Vector((table.max.x + table.min.x) / 2,
                                                                                      (table.max.y + table.min.y) / 2,
                                                                                      (table.max.z) / 2));

        rave.addBoxCollObj(table_size, table_pose, table_name);
    }

    tmc_msgs::Voice take_object;
    take_object.language = tmc_msgs::Voice::kEnglish;
    take_object.sentence = "Please take the Pringles from me!";

    tmc_msgs::Voice place_object;
    place_object.language = tmc_msgs::Voice::kEnglish;
    place_object.sentence = "No one took the Pringles can! I will place it on the table.";

    geometry_msgs::Pose2D home_pose;
    home_pose.x = 0.0;
    home_pose.y = 0.0;
    home_pose.theta = 0.0;

    geometry_msgs::PoseStamped home_pose_nav;
    home_pose_nav.header.frame_id = "map";
    home_pose_nav.header.stamp = ros::Time::now();
    home_pose_nav.pose.position.x = home_pose.x;
    home_pose_nav.pose.position.x = home_pose.y;
    tf::Quaternion home_q;
    home_q.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(home_q, home_pose_nav.pose.orientation);
    move_base_msgs::MoveBaseGoal home_goal;
    home_goal.target_pose = home_pose_nav;

    geometry_msgs::Pose2D handoff_pose;
    handoff_pose.x = -1.106;
    handoff_pose.y = -0.442;
    handoff_pose.theta = M_PI;

    geometry_msgs::PoseStamped handoff_pose_nav;
    handoff_pose_nav.header.frame_id = "map";
    handoff_pose_nav.header.stamp = ros::Time::now();
    handoff_pose_nav.pose.position.x = handoff_pose.x;
    handoff_pose_nav.pose.position.y = handoff_pose.y;
    tf::Quaternion handoff_q;
    handoff_q.setRPY(0.0, 0.0, M_PI);
    tf::quaternionTFToMsg(handoff_q, handoff_pose_nav.pose.orientation);
    move_base_msgs::MoveBaseGoal handoff_goal;
    handoff_goal.target_pose = handoff_pose_nav;

    geometry_msgs::Pose2D place_pose;
    place_pose.x = -1.325;
    place_pose.y = -0.160;
    place_pose.theta = M_PI;

    geometry_msgs::PoseStamped place_pose_nav;
    place_pose_nav.header.frame_id = "map";
    place_pose_nav.header.stamp = ros::Time::now();
    place_pose_nav.pose.position.x = place_pose.x;
    place_pose_nav.pose.position.y = place_pose.y;
    tf::Quaternion place_q;
    place_q.setRPY(0.0, 0.0, M_PI/2);
    tf::quaternionTFToMsg(place_q, place_pose_nav.pose.orientation);
    move_base_msgs::MoveBaseGoal place_goal;
    place_goal.target_pose = place_pose_nav;

    ///////// CLUSTER TABLETOP OBJECTS //////////
    while (ros::ok()){
        std::vector<point_cloud_proc::Object> tabletop_objects;
        bool tabletop_clustered = pcp.clusterObjects(tabletop_objects);
        if (tabletop_clustered) {
            // Create tabletop collision objects
            int object_count = 1;
            for (auto object : tabletop_objects) {
                std::string obj_name = "tabletop_" + std::to_string(object_count);
                OpenRAVE::Vector obj_size((object.max.x - object.min.x), (object.max.y - object.min.y),
                                          (object.max.z - object.min.z));
                OpenRAVE::Transform obj_pose(OpenRAVE::Vector(1, 0, 0, 0),
                                             OpenRAVE::Vector((object.max.x + object.min.x) / 2,
                                                              (object.max.y + object.min.y) / 2,
                                                              (object.max.z + object.min.z) / 2));
                rave.addBoxCollObj(obj_size, obj_pose, obj_name);
                object_count++;
            }
            Grasp grasp = rave.generateGraspPose();
            rave.drawTransform(grasp.pose);

            EEFPoseGoals eef_goals(1);
            eef_goals.no_waypoints = 10;
            eef_goals.wrt_world = true;
            eef_goals.timesteps[0] = 9;
            eef_goals.poses[0] = grasp.pose;
            trajectory_msgs::JointTrajectory traj_raw = rave.computeTrajectory(eef_goals, false);
            trajectory_msgs::JointTrajectory traj_smooth;
            rave.smoothTrajectory(traj_raw, traj_smooth);
            controller.executeWholeBodyTraj(traj_smooth);
//            ros::Duration(2.0).sleep();

            controller.graspOrRelease(GRIPPER_STATE::GRASP);
            rave.grabObject(grasp.obj_name);

            controller.moveBase(home_pose);
            controller.moveArmToKnownState(ARM_STATE::GO_CONF);
            ros::Duration(1.0).sleep();
            nav_cli.sendGoal(handoff_goal);

            nav_cli.waitForResult();
            controller.moveArmToKnownState(ARM_STATE::GIVE_CONF);
            talk_pub.publish(take_object);
            bool grasped = false;
            int handoff_wait_time = 5;
            int handoff_count_threshold = handoff_wait_time * 10;
            int handoff_count = 0;
            while (!grasped && handoff_count < handoff_count_threshold){
                std::cout << force_data << std::endl;
                if (force_data.z < -5.0){
                    controller.graspOrRelease(GRIPPER_STATE::RELEASE);
                    rave.releaseObject(grasp.obj_name);
                    grasped = true;
                }
                handoff_count++;
                ros::Duration(0.1).sleep();
            }

            if(!grasped)
            {
              talk_pub.publish(place_object);

              //No one grasped the object - move to the table/bin place position
              controller.moveArmToKnownState(ARM_STATE::GO_CONF);
              ros::Duration(1.0).sleep();
              nav_cli.sendGoal(place_goal);
              nav_cli.waitForResult();
//
//              //Place the object in the bin/table
              controller.moveArmToKnownState(ARM_STATE::PLACE_TABLE_ONE);
              ros::Duration(1.0).sleep();
              controller.moveArmToKnownState(ARM_STATE::PLACE_TABLE_TWO);
               ros::Duration(1.0).sleep();
              controller.graspOrRelease(GRIPPER_STATE::RELEASE);
              rave.releaseObject(grasp.obj_name);
              ros::Duration(0.1).sleep();
              //ros::Duration(1.0).sleep();
              controller.moveArmToKnownState(ARM_STATE::PLACE_TABLE_ONE);
              //ros::Duration(1.0).sleep();
              controller.graspOrRelease(GRIPPER_STATE::GRASP);


            }

            controller.moveArmToKnownState(ARM_STATE::GO_CONF);
            ros::Duration(1.0).sleep();
            nav_cli.sendGoal(home_goal);
            nav_cli.waitForResult();
            controller.graspOrRelease(GRIPPER_STATE::RELEASE);

            rave.removeTableObjects();
            controller.moveArmToKnownState(ARM_STATE ::GRASP_CONF);
            controller.moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
            ros::Duration(3.0).sleep();

        } else {
            std::cout << "TASK: there is no object left on the table!!!" << std::endl;
        }

    }


    ros::waitForShutdown();

}
