#include "genocto.hpp"


int main(int argc, char **argv) {
    
    std :: vector<std :: vector<float>> in_hull_points;

    float world_resolution;
    std::string world_file_name;
    
    ros::init(argc, argv, "generate_octomap");
    ros::NodeHandle nh("~");

    ros::Subscriber viconMarkersSub = nh.subscribe("/vicon/markers", 1, viconMarkersCallback);

    ros::Publisher convex_pub = nh.advertise<visualization_msgs::Marker>("convex_hull", 10, true);
    ros::Publisher in_hull_points_pub = nh.advertise<visualization_msgs::MarkerArray>("hull_points", 1, true);

    visualization_msgs::MarkerArray vis_pts;
    nh.getParam("/generate_octomap/world/resolution", world_resolution);
    nh.getParam("/generate_octomap/world/file_name", world_file_name);
    
    // // Load the octomap from the .bt file
    std::string package_path = ros::package::getPath("vicon_to_octomap");
    std::string file_path = package_path + "/world/" + world_file_name;

    octomap::OcTree octree(file_path);
     

    // vicon_markers = {{-1.86427, -0.547712, 0, 2}, 
    //                  {-1.53973, -0.206962, 0, 2},
    //                  {-1.81515, 0.0583257, 0, 2},
    //                  {-2.1423, -0.263839, 0, 2}, 
    //                  {-1.86427, -0.547712, 0.742779, 2},
    //                  {-1.53973, -0.206962, 0.732354, 2},
    //                  {-1.81515, 0.0583257, 0.738385, 2},
    //                  {-2.1423, -0.263839, 0.745702, 2},
    //                  {2.0, 2.0, 0.5, 0},
    //                  {2.0, 1.0, 0.5, 0},
    //                  {2.5, 1.0, 0.5, 0},
    //                  {2.5, 2.0, 0.5, 0},
    //                  {+2.0, +2.0, 0.2, 0},
    //                  {2.0, 1.0, 0.2, 0}, 
    //                  {2.5, 1.0, 0.2, 0}, 
    //                  {2.5, 2.0, 0.2, 0}, 
    //                  {-1.47983, 1.17005, 0.0579177, 11},
    //                  {-0.937562, 1.20197, 0.0381817, 11},
    //                  {-0.951067, 1.52978, 0.0381255, 11},
    //                  {-1.49617, 1.53334, 0.0457832, 11},
    //                  {-1.40594, 1.18383, 0.662371, 11},
    //                  {-1.20532, 1.1923, 0.656429, 11},
    //                  {-1.21813, 1.53791, 0.644181, 11},
    //                  {-1.41878, 1.52569, 0.657523, 11}};

    // buildPoleAndRing(0, 1, 0.8);
    // buildPoleAndRing(-2, -2, 0.8);
    // buildTrap();
    // buildPoleAndRing(0, 0, 0.8);
    // buildPoleAndRing(0, -1, 0.8);
    buildTrap();
    
    received = true;
    bool modified = false;
    ros::Rate rate(10);
    while(ros::ok()){
        std::vector<ConvexHull> hull_obstacles;
        if(received && !modified){
            
            modified = true;
            std::vector<Point3D> vecs;
            int prev_obs_id = -1, curr_obs_id, hull_cnt = 0;
            for (int i = 0; i < vicon_markers.size() ; i++) {
                std :: cout << "{" << vicon_markers[i][0] << ", " << vicon_markers[i][1] << ", " << vicon_markers[i][2] << "}," << "\n";
                curr_obs_id = vicon_markers[i][3];
                if (prev_obs_id == curr_obs_id || prev_obs_id == -1) {
                    vecs.emplace_back(vicon_markers[i][0], vicon_markers[i][1], vicon_markers[i][2]);
                }
                else{
                    if(prev_obs_id == RingInner1){
                        ring_ids.push_back(hull_cnt);
                    }
                    if(prev_obs_id == RingInner2){
                        ring_ids.push_back(hull_cnt);
                    }
                    hull_obstacles.emplace_back(ConvexHull(vecs));
                    hull_cnt++;
                    vecs.clear();
                    i--;        
                }
                prev_obs_id = curr_obs_id;
            }
            hull_obstacles.emplace_back(ConvexHull(vecs));
            
            // Set the dimensions of the OctoMap
            double x_m = 6.0;  // Width in meters
            double y_m = 6.0;  // Height in meters
            double z_m = 2.0;   // Depth in meters

            // Create an OctoMap with specified dimensions and resolution
            octomap::ColorOcTree octree(world_resolution);

            // Iterate through the 3D space and set some voxels as occupied
            for (double x = -x_m / 2.0; x < x_m / 2.0; x += world_resolution/2) {
                for (double y = -y_m / 2.0; y < y_m / 2.0; y += world_resolution/2) {
                    for (double z = 0.0; z < z_m; z += world_resolution/2) {
                        octomap::point3d point(x, y, z);
                        
                        octree.updateNode(point, false); // Set the voxel as occupied
                            
                            
                    }
                        
                }
            }
            

            // Iterate through the 3D space and set some voxels as occupied
            for (double x = -x_m / 2.0; x < x_m / 2.0; x += world_resolution/2) {
                for (double y = -y_m / 2.0; y < y_m / 2.0; y += world_resolution/2) {
                    for (double z = 0.0; z < z_m; z += world_resolution/2) {
                        octomap::point3d point(x, y, z);
                        Point3D pt(x, y, z);

                       for(int k = 0; k < hull_obstacles.size(); k++){
                            if(std::find(ring_ids.begin(), ring_ids.end(), k) != ring_ids.end()){
                                bool inner = hull_obstacles[k].Contains(pt);
                                bool outer =  hull_obstacles[k+1].Contains(pt);

                                if(!inner && outer){
                                    octree.updateNode(point, true); // Set the voxel as occupied
                                    break;    
                                }
                                k++;
                            }
                            else{
                                bool inside = hull_obstacles[k].Contains(pt); 
                                if(inside){
                                    octree.updateNode(point, true); // Set the voxel as occupied
                                    break;
                                }
                            }
                        }
                        
                    }
                }
            }
            
            // Define the iterator (Choose between leaf_iterator for occupied voxels or iterator for all voxels)
            // octomap::OcTree::iterator it;

            // // Initialize the iterator to the beginning of the octree
            // it = octree.begin();

            // // Loop through all voxels in the octomap
            // for (it = octree.begin(); it != octree.end(); ++it) {
            //     // Determine if the voxel is occupied or free
            //     if (!octree.isNodeOccupied(*it)) {
                   
            //         // Voxel is free
            //         double x = it.getX();
            //         double y = it.getY();
            //         double z = it.getZ();
            //         Point3D pt(x, y, z);
            //         octomap::OcTreeNode* node = octree.search(it.getKey());
            //         octree.pruneNode(node);
            //         for(int k = 0; k < hull_obstacles.size(); k++){
            //             bool inside = hull_obstacles[k].Contains(pt); 
            //             if(inside){
            //                 in_hull_points.push_back({x, y, z});
            //                 // Delete the children of the current node
                            
                            
            //                 it->setLogOdds(octomap::logodds(1.0));
            //                 // Expand the node to make it non-leaf
            //                 break;
            //             }
            //          }
            //          octree.expandNode(node);
            //     }
            // }
            // octree.updateInnerOccupancy();


            // Write the modified octomap to a new .bt file
            std::string output_file_path = package_path + "/world/vicon_room_obs.bt";
            if (!octree.writeBinary(output_file_path)) {
                ROS_ERROR_STREAM("Failed to write modified octomap to file: " << output_file_path);
                return 1;
            }

            output_file_path = ros::package::getPath("swarm_nav") + "/world/vicon_room_obs.bt";
            if (!octree.writeBinary(output_file_path)) {
                ROS_ERROR_STREAM("Failed to write modified octomap to file: " << output_file_path);
                return 1;
            }

            output_file_path = "/home/vivek/charlottetown/dsl/src/dsl__projects__crazyflie/crazyflie_new/crazyflie_control/world/vicon_room_obs.bt";
            if (!octree.writeBinary(output_file_path)) {
                ROS_ERROR_STREAM("Failed to write modified octomap to file: " << output_file_path);
                return 1;
            }
            ROS_INFO_STREAM("New octomap generated.");
            
        }
        else if(!received){
            ROS_INFO_STREAM("Waiting for subscriber");
        }

        visualization_msgs :: Marker marker;
        if(modified){
            // std::list<Face> vis = hull_obstacles[0].GetFaces();
            for(int i = 0; i < hull_obstacles.size(); i++){
                VizPolyhedron(hull_obstacles[i].GetFaces(), convex_pub);
                // for(auto so:hull_obstacles[i].GetFaces())
                //     vis.push_back(so);
            }
            vis_pts.markers.clear();
             for(int k = 0; k < vicon_markers.size(); k++){
                        marker = visualization_msgs :: Marker();
                        marker.header.frame_id = "/map";
                        marker.id = k;
                        marker.type = marker.SPHERE;
                        marker.action = marker.ADD;
                      
                            marker.color.a = 1.0;
                            marker.color.r = 1.0;
                            marker.color.g = 0.0;
                            marker.color.b = 0.0;                 
                            marker.scale.x = 0.02;
                            marker.scale.y = 0.02;
                            marker.scale.z = 0.02;                       
                            marker.pose.orientation.w = 1.0;
                            marker.pose.position.x = vicon_markers[k][0];
                            marker.pose.position.y = vicon_markers[k][1]; 
                            marker.pose.position.z = vicon_markers[k][2]; 
                            vis_pts.markers.push_back(marker);
                        
             }
             in_hull_points_pub.publish(vis_pts);
            // VizPolyhedron(hull_obstacles[0].GetFaces(), convex_pub);
        }
        ros::spinOnce();
        rate.sleep();
        
    }
    ros::spin();
    return 0;
}
