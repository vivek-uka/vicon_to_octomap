#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include "convexhull.h"

int main() {
    
    float world_resolution = 0.1;
    std::vector<std::vector<double>> tests;
    tests ={{-2.0, -2.0, 0.5}, {-2.0, -1.0, 0.5}, {-2.5, -1.0, 0.5}, {-2.5, -2.0, 0.5},
                                {-2.0, -2.0, 0.1}, {-2.0, -1.0, 0.1}, {-2.5, -1.0, 0.1}, {-2.5, -2.0, 0.1}};

    std::vector<Point3D> vecs;
    for(int i =0 ; i < tests.size(); i++)
    vecs.emplace_back(tests[i][0], tests[i][1], tests[i][2]);
    
    ConvexHull hull(vecs);
    
    // Load the octomap from the .bt file
    std::string package_path = ros::package::getPath("vicon_to_octomap");
    std::string file_path = package_path + "/world/simple/simple_corridor.bt";

    octomap::OcTreeStamped tree(world_resolution);
    if (!tree.readBinary(file_path)) {
        std::cerr << "Failed to read octomap file: " << file_path << std::endl;
        return 1;
    }

    int cnt = 0;
    // Iterate through all leaf nodes in the octomap and set them to occupied
    for (octomap::OcTreeStamped::leaf_iterator it = tree.begin_leafs(),
         end = tree.end_leafs(); it != end; ++it) {
        octomap::point3d voxel_center = it.getCoordinate();
        Point3D pt(voxel_center.x(), voxel_center.y(), voxel_center.z());
        bool inside = hull.Contains(pt); 
        if(inside){
            it->setLogOdds(octomap::logodds(1.0));
        }
    }
    tree.updateInnerOccupancy();

    // Write the modified octomap to a new .bt file
    std::string output_file_path = package_path + "/world/temp_octomap.bt";
    if (!tree.writeBinary(output_file_path)) {
        std::cerr << "Failed to write modified octomap to file: " << output_file_path << std::endl;
        return 1;
    }

    std::cout << "New octomap generated." << std::endl;

    return 0;
}
