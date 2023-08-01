#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include "vicon_bridge/Marker.h"
#include "vicon_bridge/Markers.h"
#include "convexhull.h"

std::vector<std::vector<double>> vicon_markers;

enum Obstacles { UShapedBlock1=0, UShapedBlock2, UShapedBlock3, 
                 TiltedBox1, TiltedBox2, 
                 ChairBlock1, ChairBlock2, 
                 RingBlock1, RingBlock2, RingBlock3};
                 
void viconMarkersCallback(const vicon_bridge::Markers& msg) {
    vicon_markers.clear();
    std::vector<vicon_bridge::Marker>::const_iterator it;
    for (it = msg.markers.begin(); it != msg.markers.end(); ++it) {
        if (it->segment_name == "UShapedBlock1") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, UShapedBlock1 });
        } 
        else if (it->segment_name == "UShapedBlock2") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, UShapedBlock2 });
        } 
        else if (it->segment_name == "UShapedBlock3") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, UShapedBlock3 });
        } 
        else if (it->segment_name == "TiltedBox1") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, TiltedBox1 });
        } 
        else if (it->segment_name == "TiltedBox2") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, TiltedBox2 });
        }
        else if (it->segment_name == "ChairBlock1") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, ChairBlock1 });
        }
        else if (it->segment_name == "ChairBlock2") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, ChairBlock2 });
        }
        else if (it->segment_name == "RingBlock1") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, RingBlock1 });
        }
        else if (it->segment_name == "RingBlock2") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, RingBlock2 });
        }
        else if (it->segment_name == "RingBlock3") {
            vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0, RingBlock3 });
        }
        
    }
}