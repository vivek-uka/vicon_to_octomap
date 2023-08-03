#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include "vicon_bridge/Marker.h"
#include "vicon_bridge/Markers.h"
#include "convexhull.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

std::vector<std::vector<double>> vicon_markers;

enum Obstacles {Ladder = 0, Box};
bool received = false;

void viconMarkersCallback(const vicon_bridge::Markers& msg) {
    
    if(!received){
        std::vector<vicon_bridge::Marker>::const_iterator it;
        for (it = msg.markers.begin(); it != msg.markers.end(); ++it) {
            if (it->segment_name == "Ladder") {
                if(it->translation.z / 1000.0 > 0.2)
                    vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0 + 0.1, Ladder });
                else
                    vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, 0.0, Ladder });
                ROS_INFO_STREAM(it->translation.x / 1000.0 << " " << it->translation.y / 1000.0 << " " << it->translation.z / 1000.0);
            }    
            if (it->segment_name == "Box") {
                vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0 + 0.1, Box });
            }         
        }
        ROS_INFO_STREAM("Received vicon markers");

        for(int i = 0; i < 4; i++){
            vicon_markers.push_back({ vicon_markers[i][0], vicon_markers[i][1], 0.0, Box });
        }
    }
    received = true;
}

void SetMarker(visualization_msgs::Marker& scan_marker, float scale=1.f)
{ 
  scan_marker.header.frame_id = "map";
  scan_marker.header.stamp = ros::Time::now();
  scan_marker.ns = "scan";
  scan_marker.action = visualization_msgs::Marker::ADD;
  scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale;
  scan_marker.pose.orientation.x = 0.0;
  scan_marker.pose.orientation.y = 0.0;
  scan_marker.pose.orientation.z = 0.0;
  scan_marker.pose.orientation.w = 1.0;
  scan_marker.pose.position.x = 0.0;
  scan_marker.pose.position.y = 0.0;
  scan_marker.pose.position.z = 0.0;
  std_msgs::ColorRGBA scan_color;
  scan_color.a = 0.5f;
  scan_color.r = 0.f;
  scan_color.g = 0.9f;
  scan_color.b = 0.9f;
  scan_marker.color = scan_color;
}

void VizPolyhedron(const std::list<Face>& triangles, ros::Publisher &convex_pub)
{
  visualization_msgs::Marker scan_marker;
  SetMarker(scan_marker);
  scan_marker.id = 0;
  scan_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

  geometry_msgs::Point temp;
  for (const auto& triangle : triangles) 
  {
    for(int i = 0; i < 3; i++)
    {
      temp.x = triangle.vertices[i].x; 
      temp.y = triangle.vertices[i].y; 
      temp.z = triangle.vertices[i].z;
      scan_marker.points.push_back(temp);
    }
  }
  convex_pub.publish(scan_marker);
}