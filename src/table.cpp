#include <ros/package.h>
#include <point_cloud_proc/point_cloud_proc.h>
#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_openrave_controller.h>

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


//    std::vector<point_cloud_proc::Object> tabletop_objects;
//    bool tabletop_clustered = pcp.clusterObjects(tabletop_objects);
//    if (tabletop_clustered) {
//        // Create tabletop collision objects
//        int object_count = 1;
//        for (auto object : tabletop_objects) {
//            std::string obj_name = "tabletop_" + std::to_string(object_count);
//            OpenRAVE::Vector obj_size((object.max.x - object.min.x), (object.max.y - object.min.y),
//                                      (object.max.z - object.min.z));
//            OpenRAVE::Transform obj_pose(OpenRAVE::Vector(1, 0, 0, 0),
//                                         OpenRAVE::Vector((object.max.x + object.min.x) / 2,
//                                                          (object.max.y + object.min.y) / 2,
//                                                          (object.max.z + object.min.z) / 2));
//            rave.addBoxCollObj(obj_size, obj_pose, obj_name);
//            object_count++;
//        }
//        Grasp grasp = rave.generateGraspPose();
//        rave.drawTransform(grasp.pose);
//    } else {
//            std::cout << "TASK: there is no object left on the table!!!" << std::endl;
//        }


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

            controller.graspOrRelease(GRIPPER_STATE::GRASP);
            rave.grabObject(grasp.obj_name);

            OpenRAVE::Transform place_pose(OpenRAVE::Vector(0.5, -0.5, -0.5, -0.5),
                                           OpenRAVE::Vector(-0.074, 0.971, 0.35));
            rave.drawTransform(place_pose);

            EEFPoseGoals eef_goals_place(1);
            eef_goals_place.no_waypoints = 10;
            eef_goals_place.wrt_world = true;
            eef_goals_place.timesteps[0] = 9;
            eef_goals_place.poses[0] = place_pose;
            trajectory_msgs::JointTrajectory traj_place_raw = rave.computeTrajectory(eef_goals_place, false);
            trajectory_msgs::JointTrajectory traj_place_smooth;
            rave.smoothTrajectory(traj_place_raw, traj_place_smooth);
            controller.executeWholeBodyTraj(traj_place_smooth);

            controller.graspOrRelease(GRIPPER_STATE::RELEASE);
            rave.releaseObject(grasp.obj_name);

            rave.removeTableObjects();
            controller.moveToKnownState(MOVE_STATE::PICK);
            controller.moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
            ros::Duration(3.0).sleep();

        } else {
            std::cout << "TASK: there is no object left on the table!!!" << std::endl;
        }


    }


    ros::waitForShutdown();

}

