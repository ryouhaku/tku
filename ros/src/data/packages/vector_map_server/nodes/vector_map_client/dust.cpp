/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <math.h>
#include <iostream>
#include <iomanip>
#include <geometry_msgs/PoseStamped.h>
#include "autoware_msgs/lane.h"
#include <visualization_msgs/MarkerArray.h>
#include "vector_map/vector_map.h"

#include "vector_map_server/GetRoadEdge.h"
#include "vector_map_server/GetGutter.h"
#include "vector_map_server/GetCurb.h"
#include "vector_map_server/GetWhiteLine.h"
#include "vector_map_server/GetStopLine.h"
#include "vector_map_server/GetZebraZone.h"
#include "vector_map_server/GetCrossWalk.h"
#include "vector_map_server/GetRoadMark.h"
#include "vector_map_server/GetRoadPole.h"
#include "vector_map_server/GetRoadSign.h"
#include "vector_map_server/GetSignal.h"
#include "vector_map_server/GetStreetLight.h"
#include "vector_map_server/GetUtilityPole.h"
#include "vector_map_server/GetGuardRail.h"
#include "vector_map_server/GetSideWalk.h"
#include "vector_map_server/GetDriveOnPortion.h"
#include "vector_map_server/GetCrossRoad.h"
#include "vector_map_server/GetSideStrip.h"
#include "vector_map_server/GetCurveMirror.h"
#include "vector_map_server/GetWall.h"
#include "vector_map_server/GetFence.h"
#include "vector_map_server/GetRailCrossing.h"

using vector_map::VectorMap;
using vector_map::Category;
using vector_map::Color;
using vector_map::Key;

using vector_map::Vector;
using vector_map::Line;
using vector_map::Area;
using vector_map::Pole;
using vector_map::Signal;

using vector_map::Gutter;

using vector_map::isValidMarker;
using vector_map::createVectorMarker;
using vector_map::createLineMarker;
using vector_map::createAreaMarker;
using vector_map::createPoleMarker;

namespace
{
class VectorMapClient
{
private:
  geometry_msgs::PoseStamped pose_;
  autoware_msgs::lane waypoints_;

public:
  VectorMapClient()
  {
  }

  geometry_msgs::PoseStamped getPose() const
  {
    return pose_;
  }

  autoware_msgs::lane getWaypoints() const
  {
    return waypoints_;
  }

  void setPose(const geometry_msgs::PoseStamped& pose)
  {
    pose_ = pose;
  }

  void setWaypoints(const autoware_msgs::lane& waypoints)
  {
    waypoints_ = waypoints;
  }
};
} // namespace

// aritoshi
std::string float_to_string(double f, int digits)
{
  std::ostringstream oss;
  oss << std::setprecision(digits) << std::setiosflags(std::ios::fixed) << f;
  return oss.str();
}


visualization_msgs::Marker visualize_marker(geometry_msgs::Pose pose,float r,float g,float b,int count){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.frame_locked = true;

  marker.ns = "akihiro_" + std::to_string(count);
  marker.id = count;
  geometry_msgs::Point relative_p;
  relative_p.x = -0.1;
  //marker.pose.position = calcAbsoluteCoordinate(relative_p, pose);
  //marker.pose.position.z += 0.2;
  marker.pose = pose;

    std::cout << marker.pose.position.x << " " << marker.pose.position.y <<  " " << marker.pose.position.z << " " << marker.pose.orientation.x << " " << marker.pose.orientation.y
              << " " << marker.pose.orientation.z << " " << marker.pose.orientation.w << std::endl;

  marker.text = "akihiro";

  return marker;
}

visualization_msgs::Marker coordinateLineMarker(geometry_msgs::Point p,int id,const visualization_msgs::Marker marker, std::string s){
  visualization_msgs::Marker tmp_marker = marker;//visualize_marker(marker.pose,0.0,1.0,0.0,count);
  tmp_marker.id = id;
  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  tmp_marker.pose.position = marker.points[0];//convertGeomPointToPoint(marker.points[0]
  tmp_marker.scale.z = 0.4;
  //tmp_marker.text = "x: " + std::to_string(tmp_marker.pose.position.x) + "\ny: " + std::to_string(tmp_marker.pose.position.y) + "\nz: " + std::to_string(tmp_marker.pose.position.z);
  tmp_marker.text = s + float_to_string((sqrt((marker.pose.position.x - p.x)*(marker.pose.position.x - p.x) +
    (marker.pose.position.y - p.y)*(marker.pose.position.y - p.y))),2) + " m\n";
  tmp_marker.pose.position.z += 0.2;

  return tmp_marker;
}

visualization_msgs::Marker coordinateVectorMarker(geometry_msgs::Point p,int id,const visualization_msgs::Marker marker, std::string s){
  visualization_msgs::Marker tmp_marker = marker;//visualize_marker(marker.pose,0.0,1.0,0.0,count);
  tmp_marker.id = id;
  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  tmp_marker.pose = marker.pose;//convertGeomPointToPoint(marker.points[0]
  tmp_marker.scale.z = 0.4;
  //tmp_marker.text = "x: " + std::to_string(tmp_marker.pose.position.x) + "\ny: " + std::to_string(tmp_marker.pose.position.y) + "\nz: " + std::to_string(tmp_marker.pose.position.z);
  tmp_marker.text = s + float_to_string((sqrt((marker.pose.position.x - p.x)*(marker.pose.position.x - p.x) +
    (marker.pose.position.y - p.y)*(marker.pose.position.y - p.y))),2) + " m\n";
  tmp_marker.pose.position.z += 0.2;

  return tmp_marker;
}

visualization_msgs::Marker coordinatePoleMarker(geometry_msgs::Point p,int id,const visualization_msgs::Marker marker, std::string s){
  visualization_msgs::Marker tmp_marker = marker;//visualize_marker(marker.pose,0.0,1.0,0.0,count);
  tmp_marker.id = id;
  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  tmp_marker.pose = marker.pose;//convertGeomPointToPoint(marker.points[0]
  tmp_marker.scale.z = 0.4;
  //tmp_marker.text = "x: " + std::to_string(tmp_marker.pose.position.x) + "\ny: " + std::to_string(tmp_marker.pose.position.y) + "\nz: " + std::to_string(tmp_marker.pose.position.z);
  tmp_marker.text = s + float_to_string((sqrt((marker.pose.position.x - p.x)*(marker.pose.position.x - p.x) +
    (marker.pose.position.y - p.y)*(marker.pose.position.y - p.y))),2) + " m\n";
  tmp_marker.pose.position.z += 0.2;

  return tmp_marker;
}

visualization_msgs::Marker coordinateAreaMarker(geometry_msgs::Point p,int id,const visualization_msgs::Marker marker, std::string s){
  visualization_msgs::Marker tmp_marker = marker;//visualize_marker(marker.pose,0.0,1.0,0.0,count);
  tmp_marker.id = id;
  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  tmp_marker.pose.position = marker.points[0];//convertGeomPointToPoint(marker.points[0]
  tmp_marker.scale.z = 0.4;
  //tmp_marker.text = "x: " + std::to_string(tmp_marker.pose.position.x) + "\ny: " + std::to_string(tmp_marker.pose.position.y) + "\nz: " + std::to_string(tmp_marker.pose.position.z);
  tmp_marker.text = s + float_to_string((sqrt((marker.pose.position.x - p.x)*(marker.pose.position.x - p.x) +
    (marker.pose.position.y - p.y)*(marker.pose.position.y - p.y))),2) + " m\n";
  tmp_marker.pose.position.z += 0.2;

  return tmp_marker;
}

visualization_msgs::Marker coordinateLinkedLineMarker(geometry_msgs::Point p,int id,const visualization_msgs::Marker marker, std::string s){
  visualization_msgs::Marker tmp_marker = marker;//visualize_marker(marker.pose,0.0,1.0,0.0,count);
  tmp_marker.id = id;
  tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  tmp_marker.pose.position = marker.points[0];//convertGeomPointToPoint(marker.points[0]
  tmp_marker.scale.z = 0.4;
  //tmp_marker.text = "x: " + std::to_string(tmp_marker.pose.position.x) + "\ny: " + std::to_string(tmp_marker.pose.position.y) + "\nz: " + std::to_string(tmp_marker.pose.position.z);
  tmp_marker.text = s + float_to_string((sqrt((marker.pose.position.x - p.x)*(marker.pose.position.x - p.x) +
    (marker.pose.position.y - p.y)*(marker.pose.position.y - p.y))),2) + " m\n";
  tmp_marker.pose.position.z += 0.2;

  return tmp_marker;
}

visualization_msgs::Marker createLinkedLineMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                                  const Line& line)
{
  Area area;
  area.aid = 1; // must set valid aid
  area.slid = line.lid;
  return createAreaMarker(ns, id, color, vmap, area);
}

float distance(geometry_msgs::Point m, geometry_msgs::Point p){
  float i = (sqrt((m.x - p.x)*(m.x - p.x) + (m.y - p.y)*(m.y - p.y) + (m.z - p.z)*(m.z - p.z)));
  return i;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vector_map_client");

  ros::NodeHandle nh;
  VectorMapClient vmc;

  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vector_map_client", 10, true);

  VectorMap vmap;
  vmap.subscribe(nh,
                 Category::POINT | Category::VECTOR | Category::LINE | Category::AREA | Category::POLE |
                 Category::WHITE_LINE | Category::STOP_LINE | Category::CROSS_WALK | Category::SIGNAL,
                 ros::Duration(0)); // non-blocking

  ros::Subscriber pose_sub = nh.subscribe("current_pose", 1, &VectorMapClient::setPose, &vmc);
  ros::Subscriber waypoints_sub = nh.subscribe("final_waypoints", 1, &VectorMapClient::setWaypoints, &vmc);

  visualization_msgs::MarkerArray marker_array;
  ros::ServiceClient road_edge_cli =
    nh.serviceClient<vector_map_server::GetRoadEdge>("vector_map_server/get_road_edge");
  ros::ServiceClient gutter_cli =
    nh.serviceClient<vector_map_server::GetGutter>("vector_map_server/get_gutter");
  ros::ServiceClient curb_cli =
    nh.serviceClient<vector_map_server::GetCurb>("vector_map_server/get_curb");
  ros::ServiceClient white_line_cli =
    nh.serviceClient<vector_map_server::GetWhiteLine>("vector_map_server/get_white_line");
  ros::ServiceClient stop_line_cli =
    nh.serviceClient<vector_map_server::GetStopLine>("vector_map_server/get_stop_line");
  ros::ServiceClient zebra_zone_cli =
    nh.serviceClient<vector_map_server::GetZebraZone>("vector_map_server/get_zebra_zone");
  ros::ServiceClient cross_walk_cli =
    nh.serviceClient<vector_map_server::GetCrossWalk>("vector_map_server/get_cross_walk");
  ros::ServiceClient road_mark_cli =
    nh.serviceClient<vector_map_server::GetRoadMark>("vector_map_server/get_road_mark");
  ros::ServiceClient road_pole_cli =
    nh.serviceClient<vector_map_server::GetRoadPole>("vector_map_server/get_road_pole");
  ros::ServiceClient road_sign_cli =
    nh.serviceClient<vector_map_server::GetRoadSign>("vector_map_server/get_road_sign");
  ros::ServiceClient signal_cli =
    nh.serviceClient<vector_map_server::GetSignal>("vector_map_server/get_signal");
  ros::ServiceClient street_light_cli =
    nh.serviceClient<vector_map_server::GetStreetLight>("vector_map_server/get_street_light");
  ros::ServiceClient utility_pole_cli =
    nh.serviceClient<vector_map_server::GetUtilityPole>("vector_map_server/get_utility_pole");
  ros::ServiceClient guard_rail_cli =
    nh.serviceClient<vector_map_server::GetGuardRail>("vector_map_server/get_guard_rail");
  ros::ServiceClient side_walk_cli =
    nh.serviceClient<vector_map_server::GetSideWalk>("vector_map_server/get_side_walk");
  ros::ServiceClient drive_on_portion_cli =
    nh.serviceClient<vector_map_server::GetDriveOnPortion>("vector_map_server/get_drive_on_portion");
  ros::ServiceClient cross_road_cli =
    nh.serviceClient<vector_map_server::GetCrossRoad>("vector_map_server/get_cross_road");
  ros::ServiceClient side_strip_cli =
    nh.serviceClient<vector_map_server::GetSideStrip>("vector_map_server/get_side_strip");
  ros::ServiceClient curve_mirror_cli =
    nh.serviceClient<vector_map_server::GetCurveMirror>("vector_map_server/get_curve_mirror");
  ros::ServiceClient wall_cli =
    nh.serviceClient<vector_map_server::GetWall>("vector_map_server/get_wall");
  ros::ServiceClient fence_cli =
    nh.serviceClient<vector_map_server::GetFence>("vector_map_server/get_fence");
  ros::ServiceClient rail_crossing_cli =
    nh.serviceClient<vector_map_server::GetRailCrossing>("vector_map_server/get_rail_crossing");


  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    visualization_msgs::MarkerArray marker_array_buffer;
    int id = 0;

    vector_map_server::GetRoadEdge road_edge_srv;
    road_edge_srv.request.pose = vmc.getPose();
    road_edge_srv.request.waypoints = vmc.getWaypoints();
    if (road_edge_cli.call(road_edge_srv))
    {
      for (const auto& road_edge : road_edge_srv.response.objects.data)
      {
        if (road_edge.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(road_edge.lid));
        if (line.lid == 0)
          continue;

        visualization_msgs::Marker marker = createLinkedLineMarker("road_edge", id++, Color::RED, vmap, line);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          //marker.color.g = 0.0;
          //marker.color.r = 1.0;

          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateLinkedLineMarker(vmc.getPose().pose.position,id++, marker,"RoadEdge: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "RoadEdge" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createRoadEdgeMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& road_edge : vmap.findByFilter([](const RoadEdge& road_edge){return true;}))
      {
        if (road_edge.lid == 0)
        {
          ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] invalid road_edge: " << road_edge);
          continue;
        }

        Line line = vmap.findByKey(Key<Line>(road_edge.lid));
        if (line.lid == 0)
        {
          ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] invalid line: " << line);
          continue;
        }

        if (line.blid == 0) // if beginning line
        {
          visualization_msgs::Marker marker = createLinkedLineMarker("road_edge", id++, color, vmap, line);
          if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
          else
            ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] failed createLinkedLineMarker: " << line);
        }
      }
      return marker_array;
    }
*/
    vector_map_server::GetGutter gutter_srv;
    gutter_srv.request.pose = vmc.getPose();
    gutter_srv.request.waypoints = vmc.getWaypoints();
    if (gutter_cli.call(gutter_srv))
    {
      for (const auto& gutter : gutter_srv.response.objects.data)
      {
        if (gutter.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(gutter.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker;
        switch (gutter.type)
        {
        case Gutter::NO_COVER:
          marker = createAreaMarker("gutter", id++, Color::RED, vmap, area);
          break;
        case Gutter::COVER:
          marker = createAreaMarker("gutter", id++, Color::RED, vmap, area);
          break;
        case Gutter::GRATING:
          marker = createAreaMarker("gutter", id++, Color::RED, vmap, area);
          break;
        default:
          continue;
        }

        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          //marker.color.g = 0.0;
          //marker.color.r = 1.0;

          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker, "Gutter: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "Gutter" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createGutterMarkerArray(const VectorMap& vmap, Color no_cover_color,
                                                            Color cover_color, Color grating_color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& gutter : vmap.findByFilter([](const Gutter& gutter){return true;}))
      {
        if (gutter.aid == 0)
        {
          ROS_ERROR_STREAM("[createGutterMarkerArray] invalid gutter: " << gutter);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(gutter.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createGutterMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker;
        switch (gutter.type)
        {
        case Gutter::NO_COVER:
          marker = createAreaMarker("gutter", id++, no_cover_color, vmap, area);
          break;
        case Gutter::COVER:
          marker = createAreaMarker("gutter", id++, cover_color, vmap, area);
          break;
        case Gutter::GRATING:
          marker = createAreaMarker("gutter", id++, grating_color, vmap, area);
          break;
        default:
          ROS_ERROR_STREAM("[createGutterMarkerArray] unknown gutter.type: " << gutter);
          continue;
        }
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createGutterMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/


    vector_map_server::GetCurb curb_srv;
    curb_srv.request.pose = vmc.getPose();
    curb_srv.request.waypoints = vmc.getWaypoints();
    if (curb_cli.call(curb_srv))
    {
      for (const auto& curb : curb_srv.response.objects.data)
      {
        if (curb.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(curb.lid));
        if (line.lid == 0)
          continue;

        visualization_msgs::Marker marker = createLinkedLineMarker("curb", id++, Color::RED, vmap, line);

        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          //marker.color.g = 0.0;
          //marker.color.r = 1.0;

          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateLinkedLineMarker(vmc.getPose().pose.position,id++, marker, "Curb: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "Curb" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createCurbMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& curb : vmap.findByFilter([](const Curb& curb){return true;}))
      {
        if (curb.lid == 0)
        {
          ROS_ERROR_STREAM("[createCurbMarkerArray] invalid curb: " << curb);
          continue;
        }

        Line line = vmap.findByKey(Key<Line>(curb.lid));
        if (line.lid == 0)
        {
          ROS_ERROR_STREAM("[createCurbMarkerArray] invalid line: " << line);
          continue;
        }

        if (line.blid == 0) // if beginning line
        {
          visualization_msgs::Marker marker = createLinkedLineMarker("curb", id++, color, vmap, line);
          // XXX: The visualization_msgs::Marker::LINE_STRIP is difficult to deal with curb.width and curb.height.
          if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
          else
            ROS_ERROR_STREAM("[createCurbMarkerArray] failed createLinkedLineMarker: " << line);
        }
      }
      return marker_array;
    }
*/
    vector_map_server::GetWhiteLine white_line_srv;
    white_line_srv.request.pose = vmc.getPose();
    white_line_srv.request.waypoints = vmc.getWaypoints();
    if (white_line_cli.call(white_line_srv))
    {
      for (const auto& white_line : white_line_srv.response.objects.data)
      {
        if (white_line.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(white_line.lid));
        if (line.lid == 0)
          continue;

        visualization_msgs::Marker marker = createLineMarker("white_line", id++, Color::GREEN, vmap, line);

        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker.color.g = 0.0;
          marker.color.r = 1.0;

          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateLineMarker(vmc.getPose().pose.position,id++, marker, "WhiteLine: ");
          //marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "WhiteLine" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }

    vector_map_server::GetStopLine stop_line_srv;
    stop_line_srv.request.pose = vmc.getPose();
    stop_line_srv.request.waypoints = vmc.getWaypoints();
    if (stop_line_cli.call(stop_line_srv))
    {
      for (const auto& stop_line : stop_line_srv.response.objects.data)
      {
        if (stop_line.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(stop_line.lid));
        if (line.lid == 0)
          continue;

        visualization_msgs::Marker marker = createLineMarker("stop_line", id++, Color::RED, vmap, line);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateLineMarker(vmc.getPose().pose.position,id++, marker, "StopLine: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "StopLine" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }


    vector_map_server::GetZebraZone zebra_zone_srv;
    zebra_zone_srv.request.pose = vmc.getPose();
    zebra_zone_srv.request.waypoints = vmc.getWaypoints();
    if (zebra_zone_cli.call(zebra_zone_srv))
    {
      for (const auto& zebra_zone : zebra_zone_srv.response.objects.data)
      {
        if (zebra_zone.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(zebra_zone.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("zebra_zone", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker,"ZebraZone: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "ZebraZone" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createZebraZoneMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& zebra_zone : vmap.findByFilter([](const ZebraZone& zebra_zone){return true;}))
      {
        if (zebra_zone.aid == 0)
        {
          ROS_ERROR_STREAM("[createZebraZoneMarkerArray] invalid zebra_zone: " << zebra_zone);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(zebra_zone.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createZebraZoneMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("zebra_zone", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createZebraZoneMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetCrossWalk cross_walk_srv;
    cross_walk_srv.request.pose = vmc.getPose();
    cross_walk_srv.request.waypoints = vmc.getWaypoints();
    if (cross_walk_cli.call(cross_walk_srv))
    {
      for (const auto& cross_walk : cross_walk_srv.response.objects.data)
      {
        if (cross_walk.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(cross_walk.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("cross_walk", id++, Color::BLUE, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateLineMarker(vmc.getPose().pose.position,id++, marker, "CrossWalk: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "CrossWalk" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }

    vector_map_server::GetRoadMark road_mark_srv;
    road_mark_srv.request.pose = vmc.getPose();
    road_mark_srv.request.waypoints = vmc.getWaypoints();
    if (road_mark_cli.call(road_mark_srv))
    {
      for (const auto& road_mark : road_mark_srv.response.objects.data)
      {
        if (road_mark.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(road_mark.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("road_mark", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker,"RoadMark: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "RoadMark" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createRoadMarkMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& road_mark : vmap.findByFilter([](const RoadMark& road_mark){return true;}))
      {
        if (road_mark.aid == 0)
        {
          ROS_ERROR_STREAM("[createRoadMarkMarkerArray] invalid road_mark: " << road_mark);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(road_mark.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createRoadMarkMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("road_mark", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createRoadMarkMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetRoadPole road_pole_srv;
    road_pole_srv.request.pose = vmc.getPose();
    road_pole_srv.request.waypoints = vmc.getWaypoints();
    if (road_pole_cli.call(road_pole_srv))
    {
      for (const auto& road_pole : road_pole_srv.response.objects.data)
      {
        if (road_pole.plid == 0)
          continue;

        Pole pole = vmap.findByKey(Key<Pole>(road_pole.plid));
        if (pole.plid == 0)
          continue;

        visualization_msgs::Marker marker = createPoleMarker("road_pole", id++, Color::RED, vmap, pole);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinatePoleMarker(vmc.getPose().pose.position,id++, marker,"RoadPole: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "RoadPole" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createRoadPoleMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& road_pole : vmap.findByFilter([](const RoadPole& road_pole){return true;}))
      {
        if (road_pole.plid == 0)
        {
          ROS_ERROR_STREAM("[createRoadPoleMarkerArray] invalid road_pole: " << road_pole);
          continue;
        }

        Pole pole = vmap.findByKey(Key<Pole>(road_pole.plid));
        if (pole.plid == 0)
        {
          ROS_ERROR_STREAM("[createRoadPoleMarkerArray] invalid pole: " << pole);
          continue;
        }

        visualization_msgs::Marker marker = createPoleMarker("road_pole", id++, color, vmap, pole);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createRoadPoleMarkerArray] failed createPoleMarker: " << pole);
      }
      return marker_array;
    }
*/
    vector_map_server::GetRoadSign road_sign_srv;
    road_sign_srv.request.pose = vmc.getPose();
    road_sign_srv.request.waypoints = vmc.getWaypoints();
    if (road_sign_cli.call(road_sign_srv))
    {
      for (const auto& road_sign : road_sign_srv.response.objects.data)
      {
        if (road_sign.vid == 0)
          continue;

        Vector vector = vmap.findByKey(Key<Vector>(road_sign.vid));
        if (vector.vid == 0)
          continue;

        Pole pole;
        if (road_sign.plid != 0)
          continue;

        pole = vmap.findByKey(Key<Pole>(road_sign.plid));
        if (pole.plid == 0)
          continue;

          visualization_msgs::Marker vector_marker = createVectorMarker("road_sign", id++, Color::RED, vmap, vector);
        if (isValidMarker(vector_marker))
        {
          vector_marker.scale.x = 0.4;
          vector_marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(vector_marker);

          if (road_sign.plid != 0)
          {
            visualization_msgs::Marker pole_marker = createPoleMarker("road_sign", id++, Color::RED, vmap, pole);
            if (isValidMarker(pole_marker)){
              marker_array.markers.push_back(pole_marker);

              //aritoshi
              visualization_msgs::Marker tmp_pole_marker = coordinatePoleMarker(vmc.getPose().pose.position,id++, pole_marker,"RoadSign-Pole: ");
              marker_array_buffer.markers.push_back(tmp_pole_marker);
              std::cout << "RoadSign:Pole" << ": " << distance(tmp_pole_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;

            }

            //aritoshi
            visualization_msgs::Marker tmp_marker = coordinateVectorMarker(vmc.getPose().pose.position,id++, vector_marker,"RoadSign-Vector: ");
            marker_array_buffer.markers.push_back(tmp_marker);
            std::cout << "RoadSign:Vector" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
          }
        }
      }
    }
/*
    visualization_msgs::MarkerArray createRoadSignMarkerArray(const VectorMap& vmap, Color sign_color, Color pole_color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& road_sign : vmap.findByFilter([](const RoadSign& road_sign){return true;}))
      {
        if (road_sign.vid == 0)
        {
          ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid road_sign: " << road_sign);
          continue;
        }

        Vector vector = vmap.findByKey(Key<Vector>(road_sign.vid));
        if (vector.vid == 0)
        {
          ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid vector: " << vector);
          continue;
        }

        Pole pole;
        if (road_sign.plid != 0)
        {
          pole = vmap.findByKey(Key<Pole>(road_sign.plid));
          if (pole.plid == 0)
          {
            ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid pole: " << pole);
            continue;
          }
        }

        visualization_msgs::Marker vector_marker = createVectorMarker("road_sign", id++, sign_color, vmap, vector);
        if (isValidMarker(vector_marker))
          marker_array.markers.push_back(vector_marker);
        else
          ROS_ERROR_STREAM("[createRoadSignMarkerArray] failed createVectorMarker: " << vector);

        if (road_sign.plid != 0)
        {
          visualization_msgs::Marker pole_marker = createPoleMarker("road_sign", id++, pole_color, vmap, pole);
          if (isValidMarker(pole_marker))
            marker_array.markers.push_back(pole_marker);
          else
            ROS_ERROR_STREAM("[createRoadSignMarkerArray] failed createPoleMarker: " << pole);
        }
      }
      return marker_array;
    }
*/
    vector_map_server::GetSignal signal_srv;
    signal_srv.request.pose = vmc.getPose();
    signal_srv.request.waypoints = vmc.getWaypoints();
    if (signal_cli.call(signal_srv))
    {
      for (const auto& signal : signal_srv.response.objects.data)
      {
        if (signal.vid == 0)
          continue;

        Vector vector = vmap.findByKey(Key<Vector>(signal.vid));
        if (vector.vid == 0)
          continue;

        Pole pole;
        if (signal.plid != 0)
        {
          pole = vmap.findByKey(Key<Pole>(signal.plid));
          if (pole.plid == 0)
            continue;
        }

        visualization_msgs::Marker vector_marker;
        switch (signal.type)
        {
        case Signal::RED:
        case Signal::PEDESTRIAN_RED:
          vector_marker = createVectorMarker("signal", id++, Color::RED, vmap, vector);
          break;
        case Signal::BLUE:
        case Signal::PEDESTRIAN_BLUE:
          vector_marker = createVectorMarker("signal", id++, Color::BLUE, vmap, vector);
          break;
        case Signal::YELLOW:
          vector_marker = createVectorMarker("signal", id++, Color::YELLOW, vmap, vector);
          break;
        case Signal::OTHER:
          vector_marker = createVectorMarker("signal", id++, Color::CYAN, vmap, vector);
          break;
        default:
          continue;
        }
        if (isValidMarker(vector_marker))
        {
          vector_marker.type = visualization_msgs::Marker::CUBE;
          vector_marker.scale.x = 0.4;
          vector_marker.scale.y = 0.4;
          vector_marker.scale.z = 0.4;
          vector_marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(vector_marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateVectorMarker(vmc.getPose().pose.position,id++, vector_marker,"Signal-Vector: ");
          marker_array_buffer.markers.push_back(tmp_marker);
        }

        if (signal.plid != 0)
        {
          visualization_msgs::Marker pole_marker = createPoleMarker("signal", id++, Color::MAGENTA, vmap, pole);
          if (isValidMarker(pole_marker))
          {
            pole_marker.type = visualization_msgs::Marker::CUBE;
            pole_marker.scale.x += 0.4;
            pole_marker.scale.y += 0.4;
            pole_marker.color.a = 0.4;
            marker_array_buffer.markers.push_back(pole_marker);

            //aritoshi
            visualization_msgs::Marker tmp_marker = coordinateVectorMarker(vmc.getPose().pose.position,id++, pole_marker,"Signal-Pole: ");
            marker_array_buffer.markers.push_back(tmp_marker);
          }
        }
      }
    }

    vector_map_server::GetStreetLight street_light_srv;
    street_light_srv.request.pose = vmc.getPose();
    street_light_srv.request.waypoints = vmc.getWaypoints();
    if (street_light_cli.call(street_light_srv))
    {
      for (const auto& street_light : street_light_srv.response.objects.data)
      {
        if (street_light.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(street_light.lid));
        if (line.lid == 0)
          continue;

        Pole pole;
        if (street_light.plid != 0)
          pole = vmap.findByKey(Key<Pole>(street_light.plid));
          if (pole.plid == 0)
            continue;

        if (line.blid == 0) // if beginning line
        {
          visualization_msgs::Marker line_marker = createLinkedLineMarker("street_light", id++, Color::RED, vmap, line);
          if (isValidMarker(line_marker)){
            marker_array.markers.push_back(line_marker);

            //aritoshi
            visualization_msgs::Marker tmp_marker = coordinateLinkedLineMarker(vmc.getPose().pose.position,id++,line_marker, "StreetLight-line: ");
            marker_array_buffer.markers.push_back(tmp_marker);
            std::cout << "StreetLight:line" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
          }
        }

        if (street_light.plid != 0)
        {
          visualization_msgs::Marker pole_marker = createPoleMarker("street_light", id++, Color::RED, vmap, pole);
          if (isValidMarker(pole_marker)){
            marker_array.markers.push_back(pole_marker);

            //aritoshi
            visualization_msgs::Marker tmp_pole_marker = coordinatePoleMarker(vmc.getPose().pose.position,id++, pole_marker,"StreetLight-Pole: ");
            marker_array_buffer.markers.push_back(tmp_pole_marker);
            std::cout << "StreetLight:pole" << ": " << distance(tmp_pole_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
          }
        }

      }
    }

/*
    visualization_msgs::MarkerArray createStreetLightMarkerArray(const VectorMap& vmap, Color light_color,
                                                                 Color pole_color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& street_light : vmap.findByFilter([](const StreetLight& street_light){return true;}))
      {
        if (street_light.lid == 0)
        {
          ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid street_light: " << street_light);
          continue;
        }

        Line line = vmap.findByKey(Key<Line>(street_light.lid));
        if (line.lid == 0)
        {
          ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid line: " << line);
          continue;
        }

        Pole pole;
        if (street_light.plid != 0)
        {
          pole = vmap.findByKey(Key<Pole>(street_light.plid));
          if (pole.plid == 0)
          {
            ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid pole: " << pole);
            continue;
          }
        }

        if (line.blid == 0) // if beginning line
        {
          visualization_msgs::Marker line_marker = createLinkedLineMarker("street_light", id++, light_color, vmap, line);
          if (isValidMarker(line_marker))
            marker_array.markers.push_back(line_marker);
          else
            ROS_ERROR_STREAM("[createStreetLightMarkerArray] failed createLinkedLineMarker: " << line);
        }

        if (street_light.plid != 0)
        {
          visualization_msgs::Marker pole_marker = createPoleMarker("street_light", id++, pole_color, vmap, pole);
          if (isValidMarker(pole_marker))
            marker_array.markers.push_back(pole_marker);
          else
            ROS_ERROR_STREAM("[createStreetLightMarkerArray] failed createPoleMarker: " << pole);
        }
      }
      return marker_array;
    }
*/
    vector_map_server::GetUtilityPole utility_pole_srv;
    utility_pole_srv.request.pose = vmc.getPose();
    utility_pole_srv.request.waypoints = vmc.getWaypoints();
    if (utility_pole_cli.call(utility_pole_srv))
    {
      for (const auto& utility_pole : utility_pole_srv.response.objects.data)
      {
        if (utility_pole.plid == 0)
          continue;

        Pole pole = vmap.findByKey(Key<Pole>(utility_pole.plid));
        if (pole.plid == 0)
          continue;

        visualization_msgs::Marker marker = createPoleMarker("road_pole", id++, Color::RED, vmap, pole);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinatePoleMarker(vmc.getPose().pose.position,id++, marker,"UtilityPole: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "UtilityPole" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createUtilityPoleMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& utility_pole : vmap.findByFilter([](const UtilityPole& utility_pole){return true;}))
      {
        if (utility_pole.plid == 0)
        {
          ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] invalid utility_pole: " << utility_pole);
          continue;
        }

        Pole pole = vmap.findByKey(Key<Pole>(utility_pole.plid));
        if (pole.plid == 0)
        {
          ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] invalid pole: " << pole);
          continue;
        }

        visualization_msgs::Marker marker = createPoleMarker("utility_pole", id++, color, vmap, pole);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] failed createPoleMarker: " << pole);
      }
      return marker_array;
    }
*/
    vector_map_server::GetGuardRail guard_rail_srv;
    guard_rail_srv.request.pose = vmc.getPose();
    guard_rail_srv.request.waypoints = vmc.getWaypoints();
    if(guard_rail_cli.call(guard_rail_srv))
    {
      for (const auto& guard_rail : guard_rail_srv.response.objects.data)
      {
        if (guard_rail.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(guard_rail.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("guard_rail", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker,"GuardRail: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "GuardRail" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createGuardRailMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& guard_rail : vmap.findByFilter([](const GuardRail& guard_rail){return true;}))
      {
        if (guard_rail.aid == 0)
        {
          ROS_ERROR_STREAM("[createGuardRailMarkerArray] invalid guard_rail: " << guard_rail);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(guard_rail.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createGuardRailMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("guard_rail", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createGuardRailMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetSideWalk side_walk_srv;
    side_walk_srv.request.pose = vmc.getPose();
    side_walk_srv.request.waypoints = vmc.getWaypoints();
    if(side_walk_cli.call(side_walk_srv))
    {
      for (const auto& side_walk : side_walk_srv.response.objects.data)
      {
        if (side_walk.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(side_walk.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("side_walk", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker,"SideWalk: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "SideWalk" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createSideWalkMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& side_walk : vmap.findByFilter([](const SideWalk& side_walk){return true;}))
      {
        if (side_walk.aid == 0)
        {
          ROS_ERROR_STREAM("[createSideWalkMarkerArray] invalid side_walk: " << side_walk);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(side_walk.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createSideWalkMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("side_walk", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createSideWalkMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetDriveOnPortion drive_on_portion_srv;
    drive_on_portion_srv.request.pose = vmc.getPose();
    drive_on_portion_srv.request.waypoints = vmc.getWaypoints();
    if(drive_on_portion_cli.call(drive_on_portion_srv))
    {
      for (const auto& drive_on_portion : drive_on_portion_srv.response.objects.data)
      {
        if (drive_on_portion.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(drive_on_portion.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("drive_on_portion", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker,"DriveOnPortion: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "DriveOnPortion" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createDriveOnPortionMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& drive_on_portion : vmap.findByFilter([](const DriveOnPortion& drive_on_portion){return true;}))
      {
        if (drive_on_portion.aid == 0)
        {
          ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] invalid drive_on_portion: " << drive_on_portion);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(drive_on_portion.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("drive_on_portion", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetCrossRoad cross_road_srv;
    cross_road_srv.request.pose = vmc.getPose();
    cross_road_srv.request.waypoints = vmc.getWaypoints();
    if(cross_road_cli.call(cross_road_srv))
    {
      for (const auto& cross_road : cross_road_srv.response.objects.data)
      {
        if (cross_road.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(cross_road.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("cross_road", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker,"CrossRoad: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "CrossRoad" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createCrossRoadMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& cross_road : vmap.findByFilter([](const CrossRoad& cross_road){return true;}))
      {
        if (cross_road.aid == 0)
        {
          ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid cross_road: " << cross_road);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(cross_road.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("cross_road", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createCrossRoadMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetSideStrip side_strip_srv;
    side_strip_srv.request.pose = vmc.getPose();
    side_strip_srv.request.waypoints = vmc.getWaypoints();
    if(side_strip_cli.call(side_strip_srv))
    {
      for (const auto& side_strip : side_strip_srv.response.objects.data)
      {
        if (side_strip.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(side_strip.lid));
        if (line.lid == 0)
          continue;

        visualization_msgs::Marker marker = createLinkedLineMarker("side_strip", id++, Color::RED, vmap, line);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateLinkedLineMarker(vmc.getPose().pose.position,id++, marker, "SideStrip: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "SideStrip" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createSideStripMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& side_strip : vmap.findByFilter([](const SideStrip& side_strip){return true;}))
      {
        if (side_strip.lid == 0)
        {
          ROS_ERROR_STREAM("[createSideStripMarkerArray] invalid side_strip: " << side_strip);
          continue;
        }

        Line line = vmap.findByKey(Key<Line>(side_strip.lid));
        if (line.lid == 0)
        {
          ROS_ERROR_STREAM("[createSideStripMarkerArray] invalid line: " << line);
          continue;
        }

        if (line.blid == 0) // if beginning line
        {
          visualization_msgs::Marker marker = createLinkedLineMarker("side_strip", id++, color, vmap, line);
          if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
          else
            ROS_ERROR_STREAM("[createSideStripMarkerArray] failed createLinkedLineMarker: " << line);
        }
      }
      return marker_array;
    }
*/
    vector_map_server::GetCurveMirror curve_mirror_srv;
    curve_mirror_srv.request.pose = vmc.getPose();
    curve_mirror_srv.request.waypoints = vmc.getWaypoints();
    if(curve_mirror_cli.call(curve_mirror_srv))
    {
      for (const auto& curve_mirror : curve_mirror_srv.response.objects.data)
      {
        if (curve_mirror.vid == 0 || curve_mirror.plid == 0)
          continue;

        Vector vector = vmap.findByKey(Key<Vector>(curve_mirror.vid));
        if (vector.vid == 0)
          continue;

        Pole pole = vmap.findByKey(Key<Pole>(curve_mirror.plid));
        if (pole.plid == 0)
          continue;

        visualization_msgs::Marker vector_marker = createVectorMarker("curve_mirror", id++, Color::RED, vmap, vector);
        if (isValidMarker(vector_marker))
        {
          vector_marker.scale.x = 0.4;
          vector_marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(vector_marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateVectorMarker(vmc.getPose().pose.position,id++, vector_marker,"CurveMirror-Vector: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "CurveMirror:vector" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }

        visualization_msgs::Marker pole_marker = createPoleMarker("curve_mirror", id++, Color::RED, vmap, pole);
        if (isValidMarker(pole_marker))
        {
          pole_marker.scale.x = 0.4;
          pole_marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(pole_marker);

          //aritoshi
          visualization_msgs::Marker tmp_pole_marker = coordinatePoleMarker(vmc.getPose().pose.position,id++, pole_marker,"CurveMirror-Pole: ");
          marker_array_buffer.markers.push_back(tmp_pole_marker);
          std::cout << "CurveMirror:pole" << ": " << distance(tmp_pole_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createCurveMirrorMarkerArray(const VectorMap& vmap, Color mirror_color,
                                                                 Color pole_color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& curve_mirror : vmap.findByFilter([](const CurveMirror& curve_mirror){return true;}))
      {
        if (curve_mirror.vid == 0 || curve_mirror.plid == 0)
        {
          ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid curve_mirror: " << curve_mirror);
          continue;
        }

        Vector vector = vmap.findByKey(Key<Vector>(curve_mirror.vid));
        if (vector.vid == 0)
        {
          ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid vector: " << vector);
          continue;
        }

        Pole pole = vmap.findByKey(Key<Pole>(curve_mirror.plid));
        if (pole.plid == 0)
        {
          ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid pole: " << pole);
          continue;
        }

        visualization_msgs::Marker vector_marker = createVectorMarker("curve_mirror", id++, mirror_color, vmap, vector);
        if (isValidMarker(vector_marker))
          marker_array.markers.push_back(vector_marker);
        else
          ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] failed createVectorMarker: " << vector);

        visualization_msgs::Marker pole_marker = createPoleMarker("curve_mirror", id++, pole_color, vmap, pole);
        if (isValidMarker(pole_marker))
          marker_array.markers.push_back(pole_marker);
        else
          ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] failed createPoleMarker: " << pole);
      }
      return marker_array;
    }
*/
    vector_map_server::GetWall wall_srv;
    wall_srv.request.pose = vmc.getPose();
    wall_srv.request.waypoints = vmc.getWaypoints();
    if(wall_cli.call(wall_srv))
    {
      for (const auto& wall : wall_srv.response.objects.data)
      {
        if (wall.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(wall.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("wall", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker, "Wall: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "Wall" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createWallMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& wall : vmap.findByFilter([](const Wall& wall){return true;}))
      {
        if (wall.aid == 0)
        {
          ROS_ERROR_STREAM("[createWallMarkerArray] invalid wall: " << wall);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(wall.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createWallMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("wall", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createWallMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetFence fence_srv;
    fence_srv.request.pose = vmc.getPose();
    fence_srv.request.waypoints = vmc.getWaypoints();
    if(fence_cli.call(fence_srv))
    {
      for (const auto& fence : fence_srv.response.objects.data)
      {
        if (fence.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(fence.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("fence", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker, "Fence: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "Fence" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createFenceMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& fence : vmap.findByFilter([](const Fence& fence){return true;}))
      {
        if (fence.aid == 0)
        {
          ROS_ERROR_STREAM("[createFenceMarkerArray] invalid fence: " << fence);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(fence.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createFenceMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("fence", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createFenceMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    vector_map_server::GetRailCrossing rail_crossing_srv;
    rail_crossing_srv.request.pose = vmc.getPose();
    rail_crossing_srv.request.waypoints = vmc.getWaypoints();
    if(rail_crossing_cli.call(rail_crossing_srv))
    {
      for (const auto& rail_crossing : rail_crossing_srv.response.objects.data)
      {
        if (rail_crossing.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(rail_crossing.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("rail_crossing", id++, Color::RED, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);

          //aritoshi
          visualization_msgs::Marker tmp_marker = coordinateAreaMarker(vmc.getPose().pose.position,id++, marker,"RailCrossing: ");
          marker_array_buffer.markers.push_back(tmp_marker);
          std::cout << "RailCrossing" << ": " << distance(tmp_marker.pose.position,vmc.getPose().pose.position) << " m\n" << std::endl;
        }
      }
    }
/*
    visualization_msgs::MarkerArray createRailCrossingMarkerArray(const VectorMap& vmap, Color color)
    {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;
      for (const auto& rail_crossing : vmap.findByFilter([](const RailCrossing& rail_crossing){return true;}))
      {
        if (rail_crossing.aid == 0)
        {
          ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid rail_crossing: " << rail_crossing);
          continue;
        }

        Area area = vmap.findByKey(Key<Area>(rail_crossing.aid));
        if (area.aid == 0)
        {
          ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid area: " << area);
          continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("rail_crossing", id++, color, vmap, area);
        if (isValidMarker(marker))
          marker_array.markers.push_back(marker);
        else
          ROS_ERROR_STREAM("[createRailCrossingMarkerArray] failed createAreaMarker: " << area);
      }
      return marker_array;
    }
*/
    if (!marker_array.markers.empty())
    {
      for (auto& marker : marker_array.markers)
        marker.action = visualization_msgs::Marker::DELETE;
      marker_array_pub.publish(marker_array); // clear previous marker
    }
    marker_array = marker_array_buffer;
    marker_array_pub.publish(marker_array);

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
