#include "genocto.hpp"


int main(int argc, char **argv) {
    
    float world_resolution;
    std::string world_file_name;
    
    ros::init(argc, argv, "generate_octomap");
    ros::NodeHandle nh("~");

    ros::Subscriber viconMarkersSub = nh.subscribe("/vicon/markers", 1, viconMarkersCallback);

    nh.getParam("/generate_octomap/world/resolution", world_resolution);
    nh.getParam("/generate_octomap/world/file_name", world_file_name);
    
    vicon_markers ={{-2.0, -2.0, 0.5, 0}, {-2.0, -1.0, 0.5, 0}, {-2.5, -1.0, 0.5, 0}, {-2.5, -2.0, 0.5, 0},
                    {-2.0, -2.0, 0.2, 0}, {-2.0, -1.0, 0.2, 0}, {-2.5, -1.0, 0.2, 0}, {-2.5, -2.0, 0.2, 0},
                    {+2.0, -2.0, 0.5, 1}, {+2.0, -1.0, 0.5, 1}, {+2.5, -1.0, 0.5, 1}, {+2.5, -2.0, 0.5, 1},
                    {+2.0, -2.0, 0.2, 1}, {+2.0, -1.0, 0.2, 1}, {+2.5, -1.0, 0.2, 1}, {+2.5, -2.0, 0.2, 1}};

    std::vector<ConvexHull> hull_obstacles;
    
    std::vector<Point3D> vecs;
    int prev_obs_id = -1, curr_obs_id;
    for (int i = 0; i < vicon_markers.size(); i++) {
        curr_obs_id = vicon_markers[i][3];
        if (prev_obs_id == curr_obs_id || prev_obs_id == -1) {
            vecs.emplace_back(vicon_markers[i][0], vicon_markers[i][1], vicon_markers[i][2]);
        }
        else{
            hull_obstacles.emplace_back(ConvexHull(vecs));
            vecs.clear();
            i--;        
        }
        prev_obs_id = curr_obs_id;
    }
    hull_obstacles.emplace_back(ConvexHull(vecs));
    
    // // Load the octomap from the .bt file
    std::string package_path = ros::package::getPath("vicon_to_octomap");
    std::string file_path = package_path + "/world/" + world_file_name;

    octomap::OcTreeStamped tree(world_resolution);
    if (!tree.readBinary(file_path)) {
        ROS_ERROR_STREAM("Failed to read octomap file: " << file_path);
        return 1;
    }

    // Iterate through all leaf nodes in the octomap and set them to occupied
    for (octomap::OcTreeStamped::leaf_iterator it = tree.begin_leafs(),
         end = tree.end_leafs(); it != end; ++it) {
        octomap::point3d voxel_center = it.getCoordinate();
        Point3D pt(voxel_center.x(), voxel_center.y(), voxel_center.z());
        for(int k = 0; k < hull_obstacles.size(); k++){
            bool inside = hull_obstacles[k].Contains(pt); 
            if(inside){
                it->setLogOdds(octomap::logodds(1.0));
            }
        }
    }
    tree.updateInnerOccupancy();

    // Write the modified octomap to a new .bt file
    std::string output_file_path = package_path + "/world/vicon_room_obs.bt";
    if (!tree.writeBinary(output_file_path)) {
        ROS_ERROR_STREAM("Failed to write modified octomap to file: " << output_file_path);
        return 1;
    }

    ROS_INFO_STREAM("New octomap generated.");

    return 0;
}
