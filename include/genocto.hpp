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
std::vector<int> ring_ids;
std::vector<double> trap_center({0.0, 0.0, 0.75});

float trap_middle_length = 1.5;
float trap_side_length = 0.65;
int pole_cnt = 0;

float pole_radius = 0.15 * sqrt(2) / 2;
float ring_inner_radius = 0.40;
float ring_outer_radius = ring_inner_radius + 0.1;

enum Obstacles {Ladder = 0, Box, TrapBlock1, TrapBlock2, TrapBlock3, Pole1, Pole2, Pole3, RingInner1, RingInner2, RingOuter1, RingOuter2};
bool received = false;


void buildPoleAndRing(double x_center, double y_center, double z_center){
  float angles[4] = {M_PI_4, 3*M_PI/4, 5*M_PI_4, 7*M_PI_4};
  if(pole_cnt == 0){

      for(int k = 0; k < 4; k++){
          vicon_markers.push_back({x_center + pole_radius * cos(angles[k]), y_center + pole_radius * sin(angles[k]), z_center, Pole1});
          vicon_markers.push_back({x_center + pole_radius * cos(angles[k]), y_center + pole_radius * sin(angles[k]), 0.0, Pole1});
      }
      float angle = 0;
      while(angle < 360.0){
          vicon_markers.push_back({x_center - 0.42 + ring_inner_radius * cos(angle*M_PI/180), y_center+0.1, (z_center - 0.28) + ring_inner_radius * sin(angle*M_PI/180), RingInner1});
          vicon_markers.push_back({x_center - 0.42 + ring_inner_radius * cos(angle*M_PI/180), y_center-0.1, (z_center - 0.28) + ring_inner_radius * sin(angle*M_PI/180), RingInner1});
            angle += 1;
      }

        angle = 0;
      while(angle < 360.0){
          vicon_markers.push_back({x_center -0.42 + ring_outer_radius * cos(angle*M_PI/180), y_center-0.1, (z_center - 0.28) + ring_outer_radius * sin(angle*M_PI/180) , RingOuter1});
          vicon_markers.push_back({x_center -0.42 + ring_outer_radius * cos(angle*M_PI/180), y_center+0.1, (z_center - 0.28) + ring_outer_radius * sin(angle*M_PI/180) , RingOuter1});
        angle += 1;
      }
      pole_cnt++;
    }
    else if(pole_cnt == 1){
        for(int k = 0; k < 4; k++){
          vicon_markers.push_back({x_center + 1.5*pole_radius * cos(angles[k]), y_center + 1.5*pole_radius * sin(angles[k]), z_center, Pole2});
          vicon_markers.push_back({x_center + 1.5*pole_radius * cos(angles[k]), y_center + 1.5*pole_radius * sin(angles[k]), 0.0, Pole2});
        }
        pole_cnt++;
    }
    else{
        for(int k = 0; k < 4; k++){
          vicon_markers.push_back({x_center + 1.5*pole_radius * cos(angles[k]), y_center + 1.5*pole_radius * sin(angles[k]), z_center, Pole3});
          vicon_markers.push_back({x_center + 1.5*pole_radius * cos(angles[k]), y_center + 1.5*pole_radius * sin(angles[k]), 0.0, Pole3});
        }
    }
}

void buildTrap(){
  // middle block of trap
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2, trap_center[1] + 0.1, trap_center[2], TrapBlock1});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2, trap_center[1] - 0.1, trap_center[2], TrapBlock1});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2, trap_center[1] + 0.1, trap_center[2], TrapBlock1});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2, trap_center[1] - 0.1, trap_center[2], TrapBlock1});

    vicon_markers.push_back({trap_center[0] - trap_middle_length/2, trap_center[1] + 0.1, 0, TrapBlock1});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2, trap_center[1] - 0.1, 0, TrapBlock1});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2, trap_center[1] + 0.1, 0, TrapBlock1});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2, trap_center[1] - 0.1, 0, TrapBlock1});

    // left side block of trap
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 - 0.1, trap_center[1] + 0.1, trap_center[2], TrapBlock2});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 + 0.1, trap_center[1] + 0.1, trap_center[2], TrapBlock2});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 + 0.1, trap_center[1] - trap_side_length, trap_center[2], TrapBlock2});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 - 0.1, trap_center[1] - trap_side_length, trap_center[2], TrapBlock2});

    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 - 0.1, trap_center[1] + 0.1, 0, TrapBlock2});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 + 0.1, trap_center[1] + 0.1, 0, TrapBlock2});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 + 0.1, trap_center[1] - trap_side_length, 0, TrapBlock2});
    vicon_markers.push_back({trap_center[0] - trap_middle_length/2 - 0.1, trap_center[1] - trap_side_length, 0, TrapBlock2});

    // right side block of trap
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 - 0.1, trap_center[1] + 0.1, trap_center[2], TrapBlock3});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 + 0.1, trap_center[1] + 0.1, trap_center[2], TrapBlock3});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 + 0.1, trap_center[1] - trap_side_length, trap_center[2], TrapBlock3});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 - 0.1, trap_center[1] - trap_side_length, trap_center[2], TrapBlock3});

    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 - 0.1, trap_center[1] + 0.1, 0, TrapBlock3});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 + 0.1, trap_center[1] + 0.1, 0, TrapBlock3});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 + 0.1, trap_center[1] - trap_side_length, 0, TrapBlock3});
    vicon_markers.push_back({trap_center[0] + trap_middle_length/2 - 0.1, trap_center[1] - trap_side_length, 0, TrapBlock3});
}

void viconMarkersCallback(const vicon_bridge::Markers& msg) {
    
    if(!received){
        
        std::vector<vicon_bridge::Marker>::const_iterator it;
        for (it = msg.markers.begin(); it != msg.markers.end(); ++it) {
            if (it->segment_name == "Ladder") {
                if(it->translation.z / 1000.0 > 0.2)
                    vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0 + 0.1, Ladder });
                else
                    vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, 0.0, Ladder });
            }    
            if (it->segment_name == "Box") {
                vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, it->translation.z / 1000.0 + 0.1, Box });
                vicon_markers.push_back({ it->translation.x / 1000.0, it->translation.y / 1000.0, 0.0, Box });
            }
            if(it->segment_name == "Poles"){
                float x_center = it->translation.x / 1000.0;
                float y_center = it->translation.y / 1000.0;
                float z_center = it->translation.z / 1000.0;
                
                buildPoleAndRing(x_center, y_center, z_center);
                
            }
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