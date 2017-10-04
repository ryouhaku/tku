/*
 * Normal Distributions Transform program
 * 205/04/24 tku
 * ndt_matching for ROS
 */

// number of cells
#define G_MAP_X 2000
#define G_MAP_Y 2000
#define G_MAP_Z 200
#define G_MAP_CELLSIZE 1.0

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <chrono>
<<<<<<< HEAD
=======
#include <fstream>
#include <iostream>
#include <sstream>
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
#include <string>
#include "algebra.h"
#include "ndt.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
<<<<<<< HEAD
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
=======
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"

/*grobal variables*/
NDMapPtr NDmap;
NDPtr NDs;
int NDs_num;

Point scan_points[130000];
int scan_points_num;

int is_first_time = 1;
int is_map_exist = 0;

double scan_points_weight[130000];

Point map_points[130000];
double map_points_i[130000];

int layer_select = LAYER_NUM - 1;

Posture prev_pose, prev_pose2;

// params
double g_map_center_x, g_map_center_y, g_map_center_z;
double g_map_rotation;
char g_ndmap_name[500];
int g_use_gnss;
int g_map_update = 1;
double g_ini_x, g_ini_y, g_ini_z, g_ini_roll, g_ini_pitch, g_ini_yaw;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;  // tf between base_link and localizer
<<<<<<< HEAD
static tf::Quaternion q_local_to_global;
static Eigen::Matrix4f tf_local_to_global;

void save_nd_map(char *name);
=======
static tf::Quaternion q_local_to_global, q_global_to_local;
static Eigen::Matrix4f tf_local_to_global, tf_global_to_local;
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e

static pcl::PointCloud<pcl::PointXYZ> map;
static int map_loaded = 0;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;
static double exe_time = 0.0;

static ros::Publisher localizer_pose_pub, ndt_pose_pub;
static geometry_msgs::PoseStamped localizer_pose_msg, ndt_pose_msg;

// double pose_mod(Posture *pose){
void pose_mod(Posture *pose)
{
  while (pose->theta < -M_PI)
    pose->theta += 2 * M_PI;
  while (pose->theta > M_PI)
    pose->theta -= 2 * M_PI;
  while (pose->theta2 < -M_PI)
    pose->theta2 += 2 * M_PI;
  while (pose->theta2 > M_PI)
    pose->theta2 -= 2 * M_PI;
  while (pose->theta3 < -M_PI)
    pose->theta3 += 2 * M_PI;
  while (pose->theta3 > M_PI)
    pose->theta3 -= 2 * M_PI;
}

double nrand(double n)
{
  double r;
  r = n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX);
  return r;
}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  if (map_loaded == 0)
  {
    pcl::fromROSMsg(*input, map);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    Point p;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = map_ptr->begin(); item != map_ptr->end(); item++)
    {
      p.x = (item->x - g_map_center_x) * cos(-g_map_rotation) - (item->y - g_map_center_y) * sin(-g_map_rotation);
      p.y = (item->x - g_map_center_x) * sin(-g_map_rotation) + (item->y - g_map_center_y) * cos(-g_map_rotation);
      p.z = item->z - g_map_center_z;
<<<<<<< HEAD

      add_point_map(NDmap, &p);
    }
    std::cout << "Finished loading point cloud map." << std::endl;
    save_nd_map(g_ndmap_name);
=======
      add_point_map(NDmap, &p);
    }
    std::cout << "Finished loading point cloud map." << std::endl;
    std::cout << "Map points num: " << map_ptr->size() << " points." << std::endl;

>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
    is_map_exist = 1;
    map_loaded = 1;
  }
}

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
<<<<<<< HEAD
=======
  std::cout << "call 2D pose estimate" << std::endl;
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/map", "/world", now, ros::Duration(10.0));
    listener.lookupTransform("/map", "world", now, transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                   input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  g_ini_x = input->pose.pose.position.x + transform.getOrigin().x();
  g_ini_y = input->pose.pose.position.y + transform.getOrigin().y();
  g_ini_z = input->pose.pose.position.z + transform.getOrigin().z();

<<<<<<< HEAD
  m.getRPY(g_ini_roll, g_ini_roll, g_ini_yaw);

  prev_pose.x = g_ini_x;
  prev_pose.y = g_ini_y;
  prev_pose.z = g_ini_z;
  prev_pose.theta = g_ini_roll;
  prev_pose.theta2 = g_ini_pitch;
  prev_pose.theta3 = g_ini_yaw;
=======
  m.getRPY(g_ini_roll, g_ini_pitch, g_ini_yaw);

  // global(map) to local(map center)
  Eigen::Translation3f translation(g_ini_x, g_ini_y, g_ini_z);
  Eigen::AngleAxisf rotation_x(g_ini_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(g_ini_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(g_ini_yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f global_t = (translation * rotation_z * rotation_y * rotation_x).matrix();
  Eigen::Matrix4f local_t = tf_global_to_local * global_t;

  tf::Quaternion q_g_to_l;
  tf::Matrix3x3 mat_g;
  mat_g.setValue(
      static_cast<double>(local_t(0, 0)), static_cast<double>(local_t(0, 1)), static_cast<double>(local_t(0, 2)),
      static_cast<double>(local_t(1, 0)), static_cast<double>(local_t(1, 1)), static_cast<double>(local_t(1, 2)),
      static_cast<double>(local_t(2, 0)), static_cast<double>(local_t(2, 1)), static_cast<double>(local_t(2, 2)));

  mat_g.getRotation(q_g_to_l);

  prev_pose.x = local_t(0, 3);
  prev_pose.y = local_t(1, 3);
  prev_pose.z = local_t(2, 3);
  prev_pose.theta = q_g_to_l.x();
  prev_pose.theta2 = q_g_to_l.y();
  prev_pose.theta3 = q_g_to_l.z();
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e

  prev_pose2 = prev_pose;
}

void points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  matching_start = std::chrono::system_clock::now();
  static tf::TransformBroadcaster br;
  static FILE *log_fp;
  ros::Time time;
  static tf::TransformListener listener;
  static ros::Time current_scan_time;
  current_scan_time = msg->header.stamp;

  static int iteration;

<<<<<<< HEAD
  Posture pose, bpose, initial_pose;
  static Posture key_pose;

  double e = 0;
  double x_offset, y_offset, z_offset, theta_offset;

  double distance;
=======
  Posture pose, bpose;
  double e = 0;
  double x_offset, y_offset, z_offset, theta_offset;
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e

  tf::Quaternion ndt_q, localizer_q;

  pcl::PointCloud<pcl::PointXYZ> filtered_scan;
  pcl::fromROSMsg(*msg, filtered_scan);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));

  if (!log_fp)
    log_fp = fopen("/tmp/ndt_log", "w");

  int j = 0;
  for (int i = 0; i < (int)filtered_scan_ptr->points.size(); i++)
  {
    scan_points[j].x = filtered_scan_ptr->points[i].x + nrand(0.01);
    scan_points[j].y = filtered_scan_ptr->points[i].y + nrand(0.01);
    scan_points[j].z = filtered_scan_ptr->points[i].z + nrand(0.01);
    double dist =
        scan_points[j].x * scan_points[j].x + scan_points[j].y * scan_points[j].y + scan_points[j].z * scan_points[j].z;
    if (dist < 3 * 3)
      continue;

    j++;
    if (j > 130000)
      break;
  }
  scan_points_num = j;

  /*--matching---*/
  // calc offset
  x_offset = prev_pose.x - prev_pose2.x;
  y_offset = prev_pose.y - prev_pose2.y;
  z_offset = prev_pose.z - prev_pose2.z;
  theta_offset = prev_pose.theta3 - prev_pose2.theta3;

  if (theta_offset < -M_PI)
    theta_offset += 2 * M_PI;
  if (theta_offset > M_PI)
    theta_offset -= 2 * M_PI;

  // calc estimated initial position
  pose.x = prev_pose.x + x_offset;
  pose.y = prev_pose.y + y_offset;
  pose.z = prev_pose.z + z_offset;
  pose.theta = prev_pose.theta;
  pose.theta2 = prev_pose.theta2;
  pose.theta3 = prev_pose.theta3 + theta_offset;

  // matching
  for (layer_select = 1; layer_select >= 1; layer_select -= 1)
  {
    for (j = 0; j < 100; j++)
    {
      if (layer_select != 1 && j > 2)
      {
        break;
      }
      bpose = pose;

      e = adjust3d(scan_points, scan_points_num, &pose, layer_select);

      pose_mod(&pose);

      if ((bpose.x - pose.x) * (bpose.x - pose.x) + (bpose.y - pose.y) * (bpose.y - pose.y) +
              (bpose.z - pose.z) * (bpose.z - pose.z) + 3 * (bpose.theta - pose.theta) * (bpose.theta - pose.theta) +
              3 * (bpose.theta2 - pose.theta2) * (bpose.theta2 - pose.theta2) +
              3 * (bpose.theta3 - pose.theta3) * (bpose.theta3 - pose.theta3) <
          0.00001)
      {
        break;
      }
    }
    iteration = j;

    /*gps resetting*/
    if (g_use_gnss)
    {
      static FILE *e_fp;
      if (!e_fp)
      {
        e_fp = fopen("/tmp/e_log", "w");
      }
      fprintf(e_fp, "%f\n", e);
      if (layer_select == 1 && e < 1000)
      {
        printf("reset\n");
        tf::StampedTransform gps_tf_on_world;
        try
        {
          listener.lookupTransform("world", "gps", ros::Time(0), gps_tf_on_world);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          return;
        }
        printf("set initial position by gps posiotion\n");
        pose.x = gps_tf_on_world.getOrigin().x();
        pose.y = gps_tf_on_world.getOrigin().y();
        pose.z = gps_tf_on_world.getOrigin().z() + nrand(5);
        tf::Quaternion q(gps_tf_on_world.getRotation().x(), gps_tf_on_world.getRotation().y(),
                         gps_tf_on_world.getRotation().z(), gps_tf_on_world.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        pose.theta = roll;
        pose.theta2 = pitch + 0.22;
        pose.theta3 = yaw + nrand(0.7);
        prev_pose2 = prev_pose = pose;
        printf("reset %f %f %f %f %f %f\n", pose.x, pose.y, pose.z, pose.theta, pose.theta2, pose.theta3);

        return;
      }
    }

<<<<<<< HEAD
=======
    // unti-distotion
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
    if (layer_select == 2)
    {
      double rate, xrate, yrate, dx, dy, dtheta;
      double tempx, tempy;
      int i;

      tempx = (pose.x - prev_pose.x);
      tempy = (pose.y - prev_pose.y);
      dx = tempx * cos(-prev_pose.theta3) - tempy * sin(-prev_pose.theta3);
      dy = tempx * sin(-prev_pose.theta3) + tempy * cos(-prev_pose.theta3);
      dtheta = pose.theta3 - prev_pose.theta3;
      if (dtheta < -M_PI)
      {
        dtheta += 2 * M_PI;
      }
      if (dtheta > M_PI)
      {
        dtheta -= 2 * M_PI;
      }

      rate = dtheta / (double)scan_points_num;
      xrate = dx / (double)scan_points_num;
      yrate = dy / (double)scan_points_num;

      printf("untidist x %f y %f yaw %f\n", dx, dy, dtheta);

      dx = -dx;
      dy = -dy;
      dtheta = -dtheta;
      for (i = 0; i < scan_points_num; i++)
      {
        tempx = scan_points[i].x * cos(dtheta) - scan_points[i].y * sin(dtheta) + dx;
        tempy = scan_points[i].x * sin(dtheta) + scan_points[i].y * cos(dtheta) + dy;

        scan_points[i].x = tempx;
        scan_points[i].y = tempy;

        dtheta += rate;
        dx += xrate;
        dy += yrate;
      }
    }
  }

  // localizer
  Eigen::Translation3f translation(pose.x, pose.y, pose.z);
  Eigen::AngleAxisf rotation_x(pose.theta, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(pose.theta2, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(pose.theta3, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f local_t = (translation * rotation_z * rotation_y * rotation_x).matrix();
  Eigen::Matrix4f global_t = tf_local_to_global * local_t;

  tf::Matrix3x3 mat_l;
  mat_l.setValue(
      static_cast<double>(global_t(0, 0)), static_cast<double>(global_t(0, 1)), static_cast<double>(global_t(0, 2)),
      static_cast<double>(global_t(1, 0)), static_cast<double>(global_t(1, 1)), static_cast<double>(global_t(1, 2)),
      static_cast<double>(global_t(2, 0)), static_cast<double>(global_t(2, 1)), static_cast<double>(global_t(2, 2)));

  mat_l.getRotation(localizer_q);
  localizer_pose_msg.header.frame_id = "/map";
  localizer_pose_msg.header.stamp = current_scan_time;
  localizer_pose_msg.pose.position.x = global_t(0, 3);
  localizer_pose_msg.pose.position.y = global_t(1, 3);
  localizer_pose_msg.pose.position.z = global_t(2, 3);
  localizer_pose_msg.pose.orientation.x = localizer_q.x();
  localizer_pose_msg.pose.orientation.y = localizer_q.y();
  localizer_pose_msg.pose.orientation.z = localizer_q.z();
  localizer_pose_msg.pose.orientation.w = localizer_q.w();

  // base_link
  Eigen::Matrix4f global_t2 = global_t * tf_ltob;
  tf::Matrix3x3 mat_b;  // base_link
  mat_b.setValue(
      static_cast<double>(global_t2(0, 0)), static_cast<double>(global_t2(0, 1)), static_cast<double>(global_t2(0, 2)),
      static_cast<double>(global_t2(1, 0)), static_cast<double>(global_t2(1, 1)), static_cast<double>(global_t2(1, 2)),
      static_cast<double>(global_t2(2, 0)), static_cast<double>(global_t2(2, 1)), static_cast<double>(global_t2(2, 2)));
  mat_b.getRotation(ndt_q);

  ndt_pose_msg.header.frame_id = "/map";
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose.position.x = global_t2(0, 3);
  ndt_pose_msg.pose.position.y = global_t2(1, 3);
  ndt_pose_msg.pose.position.z = global_t2(2, 3);
  ndt_pose_msg.pose.orientation.x = ndt_q.x();
  ndt_pose_msg.pose.orientation.y = ndt_q.y();
  ndt_pose_msg.pose.orientation.z = ndt_q.z();
  ndt_pose_msg.pose.orientation.w = ndt_q.w();

  localizer_pose_pub.publish(localizer_pose_msg);
  ndt_pose_pub.publish(ndt_pose_msg);

  scan_transrate(scan_points, map_points, &pose, scan_points_num);

<<<<<<< HEAD
  // update ND map
  distance = (key_pose.x - pose.x) * (key_pose.x - pose.x) + (key_pose.y - pose.y) * (key_pose.y - pose.y) +
             (key_pose.z - pose.z) * (key_pose.z - pose.z);

  if (g_map_update && (!is_map_exist || (distance > 0.1 * 0.1 && scan_points_num > 100)))
  {
    int i;
    for (i = 0; i < scan_points_num; i++)
    {
      add_point_map(NDmap, &map_points[i]);
    }

    key_pose = pose;
    is_map_exist = 1;
  }
=======
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
  prev_pose2 = prev_pose;
  prev_pose = pose;

  if (is_first_time)
  {
    prev_pose2 = prev_pose;
    is_first_time = 0;
  }

<<<<<<< HEAD
  fprintf(log_fp, "%f %f %f %f %f %f %f\n", header.stamp.toSec(),
=======
  fprintf(log_fp, "%f %f %f %f %f %f %f\n", current_scan_time.toSec(),
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
          pose.x * cos(g_map_rotation) - pose.y * sin(g_map_rotation) + g_map_center_x,
          pose.x * sin(g_map_rotation) + pose.y * cos(g_map_rotation) + g_map_center_y, pose.z + g_map_center_z,
          pose.theta, pose.theta2, pose.theta3 + g_map_rotation);

  fflush(log_fp);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(global_t2(0, 3), global_t2(1, 3), global_t2(2, 3)));
  transform.setRotation(ndt_q);

<<<<<<< HEAD
  br.sendTransform(tf::StampedTransform(transform, header.stamp, "map", "base_link"));
=======
  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e

  matching_end = std::chrono::system_clock::now();
  exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << msg->header.seq << std::endl;
  std::cout << "Number of filtered scan points: " << scan_points_num << " points." << std::endl;
  std::cout << "Number of iteration: " << iteration << std::endl;
  std::cout << "Execution time: " << exe_time << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.theta << ", " << pose.theta2 << ", "
            << pose.theta3 << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
<<<<<<< HEAD
}

/*add point to ndcell */
int add_point_covariance(NDPtr nd, PointPtr p)
{
  /*add data num*/
  nd->num++;
  nd->flag = 0; /*need to update*/

  /*calcurate means*/
  nd->m_x += p->x;
  nd->m_y += p->y;
  nd->m_z += p->z;

  /*calcurate covariances*/
  nd->c_xx += p->x * p->x;
  nd->c_yy += p->y * p->y;
  nd->c_zz += p->z * p->z;

  nd->c_xy += p->x * p->y;
  nd->c_yz += p->y * p->z;
  nd->c_zx += p->z * p->x;

  return 1;
}

int inv_check(double inv[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (isnan(inv[i][j]))
        return 0;
      if (fabs(inv[i][j]) > 1000)
        return 0;
    }
  }
  return 1;
}

/*calcurate covariances*/
int update_covariance(NDPtr nd)
{
  double a, b, c; /*for calcurate*/
  if (!nd->flag)
  { /*need calcurate?*/
    /*means*/
    nd->mean.x = a = nd->m_x / nd->num;
    nd->mean.y = b = nd->m_y / nd->num;
    nd->mean.z = c = nd->m_z / nd->num;

    /*covariances*/
    nd->covariance[0][0] = (nd->c_xx - 2 * a * nd->m_x) / nd->num + a * a;
    nd->covariance[1][1] = (nd->c_yy - 2 * b * nd->m_y) / nd->num + b * b;
    nd->covariance[2][2] = (nd->c_zz - 2 * c * nd->m_z) / nd->num + c * c;
    nd->covariance[0][1] = nd->covariance[1][0] = (nd->c_xy - nd->m_x * b - nd->m_y * a) / nd->num + a * b;
    nd->covariance[1][2] = nd->covariance[2][1] = (nd->c_yz - nd->m_y * c - nd->m_z * b) / nd->num + b * c;
    nd->covariance[2][0] = nd->covariance[0][2] = (nd->c_zx - nd->m_z * a - nd->m_x * c) / nd->num + c * a;
    nd->sign = 0;
    nd->flag = 1; /*this ND updated*/
    if (nd->num >= 5)
    {
      if (ginverse_matrix3d(nd->covariance, nd->inv_covariance))
        if (inv_check(nd->inv_covariance))
          nd->sign = 1;
    }
  }

  return 1;
}

/*add point to ndmap*/
int add_point_map(NDMapPtr ndmap, PointPtr point)
{
  double x, y, z;
  NDPtr *ndp[8];

  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x / 2;
  y = (point->y / ndmap->size) + ndmap->y / 2;
  z = (point->z / ndmap->size) + ndmap->z / 2;

  /*clipping*/
  if (x < 1 || x >= ndmap->x)
    return 0;
  if (y < 1 || y >= ndmap->y)
    return 0;
  if (z < 1 || z >= ndmap->z)
    return 0;

  /*select root ND*/
  ndp[0] = ndmap->nd + (int)x * ndmap->to_x + (int)y * ndmap->to_y + (int)z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  /*add  point to map */
  for (int i = 0; i < 8; i++)
  {
    if ((*ndp[i]) == 0)
      *ndp[i] = add_ND();
    if ((*ndp[i]) != 0)
      add_point_covariance(*ndp[i], point);
  }

  if (ndmap->next)
  {
    add_point_map(ndmap->next, point);
  }

  return 0;
}

/*get nd cell at point*/
int get_ND(NDMapPtr ndmap, PointPtr point, NDPtr *nd, int ndmode)
{
  double x, y, z;
  int i;
  NDPtr *ndp[8];

  /*mapping*/
  if (ndmode < 3)
  {
    x = (point->x / ndmap->size) + ndmap->x / 2 - 0.5;
    y = (point->y / ndmap->size) + ndmap->y / 2 - 0.5;
    z = (point->z / ndmap->size) + ndmap->z / 2 - 0.;
  }
  else
  {
    x = (point->x / ndmap->size) + ndmap->x / 2;
    y = (point->y / ndmap->size) + ndmap->y / 2;
    z = (point->z / ndmap->size) + ndmap->z / 2;
  }

  /*clipping*/
  if (x < 1 || x >= ndmap->x)
    return 0;
  if (y < 1 || y >= ndmap->y)
    return 0;
  if (z < 1 || z >= ndmap->z)
    return 0;

  /*select root ND*/
  ndp[0] = ndmap->nd + (int)x * ndmap->to_x + (int)y * ndmap->to_y + (int)z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  for (i = 0; i < 8; i++)
  {
    if (*ndp[i] != 0)
    {
      if (!(*ndp[i])->flag)
        update_covariance(*ndp[i]);
      nd[i] = *ndp[i];
    }
    else
    {
      nd[i] = NDs;
      // return 0;
    }
  }

  return 1;
}

NDPtr add_ND(void)
{
  NDPtr ndp;
  // int m;

  if (NDs_num >= MAX_ND_NUM)
  {
    printf("over flow\n");
    return 0;
  }

  ndp = NDs + NDs_num;
  NDs_num++;

  ndp->flag = 0;
  ndp->sign = 0;
  ndp->num = 0;
  ndp->m_x = 0;
  ndp->m_y = 0;
  ndp->m_z = 0;
  ndp->c_xx = 0;
  ndp->c_yy = 0;
  ndp->c_zz = 0;
  ndp->c_xy = 0;
  ndp->c_yz = 0;
  ndp->c_zx = 0;
  ndp->w = 1;
  ndp->is_source = 0;

  return ndp;
}

NDMapPtr initialize_NDmap_layer(int layer, NDMapPtr child)
{
  // int i,j,k,i2,i3,m;
  int i, j, k;
  int x, y, z;
  NDPtr *nd, *ndp;
  NDMapPtr ndmap;

  //  i2 = i3 = 0;
  //  printf("Initializing...layer %d\n",layer);

  x = (g_map_x >> layer) + 1;
  y = (g_map_y >> layer) + 1;
  z = (g_map_z >> layer) + 1;

  nd = (NDPtr *)malloc(x * y * z * sizeof(NDPtr));
  ndmap = (NDMapPtr)malloc(sizeof(NDMap));

  ndmap->x = x;
  ndmap->y = y;
  ndmap->z = z;
  ndmap->to_x = y * z;
  ndmap->to_y = z;
  ndmap->layer = layer;
  ndmap->nd = nd;
  ndmap->next = child;
  ndmap->size = g_map_cellsize * ((int)1 << layer);
  //  printf("size %f\n",ndmap->size);

  ndp = nd;

  for (i = 0; i < x; i++)
  {
    for (j = 0; j < y; j++)
    {
      for (k = 0; k < z; k++)
      {
        *ndp = 0;
        ndp++;
      }
    }
  }

  return ndmap;
}

NDMapPtr initialize_NDmap(void)
{
  int i;
  NDMapPtr ndmap;
  NDPtr null_nd;

  printf("Initialize NDmap\n");
  ndmap = 0;

  // init NDs
  NDs = (NDPtr)malloc(sizeof(NormalDistribution) * MAX_ND_NUM);
  NDs_num = 0;

  null_nd = add_ND();
  if (null_nd == 0)
  {
    return 0;
  }

  for (i = LAYER_NUM - 1; i >= 0; i--)
  {
    ndmap = initialize_NDmap_layer(i, ndmap);

    /*progress dots*/
    //    printf("layer %d\n",i);
  }

  //  printf("done\n");

  return ndmap;
}

int round_covariance(NDPtr nd)
{
  double v[3][3], a;

  eigenvecter_matrix3d(nd->covariance, v, nd->l);
  //  print_matrix3d(v);
  if (fabs(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0] - 1) > 0.1)
    printf("!1");
  if (fabs(v[0][0] * v[0][1] + v[1][0] * v[1][1] + v[2][0] * v[2][1]) > 0.01)
    printf("!01");
  if (fabs(v[0][1] * v[0][2] + v[1][1] * v[1][2] + v[2][1] * v[2][2]) > 0.01)
    printf("!02");
  if (fabs(v[0][2] * v[0][0] + v[1][2] * v[1][0] + v[2][2] * v[2][0]) > 0.01)
    printf("!03");

  a = fabs(nd->l[1] / nd->l[0]);
  if (a < 0.001)
  {
    return 0;
  }
  return 1;
}

double probability_on_ND(NDPtr nd, double xp, double yp, double zp)
{
  //  double xp,yp,zp;
  double e;

  if (nd->num < 5)
    return 0;

  e = exp((xp * xp * nd->inv_covariance[0][0] + yp * yp * nd->inv_covariance[1][1] +
           zp * zp * nd->inv_covariance[2][2] + 2.0 * xp * yp * nd->inv_covariance[0][1] +
           2.0 * yp * zp * nd->inv_covariance[1][2] + 2.0 * zp * xp * nd->inv_covariance[2][0]) /
          -2.0);

  if (e > 1)
    return 1;
  if (e < 0)
    return 0;
  return (e);
}

void load(char *name)
{
  FILE *fp;
  double x, y, z, q;
  Point p;

  fp = fopen(name, "r");
  int i = 0;
  while (fscanf(fp, "%lf %lf %lf %lf", &y, &x, &z, &q) != EOF)
  {  // x,y swaped
    p.x = (x - g_map_center_x) * cos(-g_map_rotation) - (y - g_map_center_y) * sin(-g_map_rotation);
    p.y = (x - g_map_center_x) * sin(-g_map_rotation) + (y - g_map_center_y) * cos(-g_map_rotation);
    p.z = z - g_map_center_z;
    add_point_map(NDmap, &p);
    std::cout << i << std::endl;
    i++;
  }
  std::cout << "Finished loading " << name << std::endl;
  fclose(fp);
}

void save_nd_map(char *name)
{
  int i, j, k, layer;
  NDData nddat;
  NDMapPtr ndmap;
  NDPtr *ndp;
  FILE *ofp;

  // for pcd
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p;

  // cloud.is_dense = false;
  // cloud.points.resize (cloud.width * cloud.height);
  // ros::Time stamp;
  // stamp = msg->header.stamp;
  // double now = ros::Time::now().toSec();

  ndmap = NDmap;
  ofp = fopen(name, "w");

  for (layer = 0; layer < 2; layer++)
  {
    ndp = ndmap->nd;
    for (i = 0; i < ndmap->x; i++)
    {
      for (j = 0; j < ndmap->y; j++)
      {
        for (k = 0; k < ndmap->z; k++)
        {
          if (*ndp)
          {
            update_covariance(*ndp);
            nddat.nd = **ndp;
            nddat.x = i;
            nddat.y = j;
            nddat.z = k;
            nddat.layer = layer;

            fwrite(&nddat, sizeof(NDData), 1, ofp);

            // regist the point to pcd data;
            p.x = (*ndp)->mean.x;
            p.y = (*ndp)->mean.y;
            p.z = (*ndp)->mean.z;
            cloud.points.push_back(p);
          }
          ndp++;
        }
      }
      //      printf("a\n");
    }
    ndmap = ndmap->next;
  }
  //  printf("done\n");
  fclose(ofp);

  // save pcd

  cloud.header.frame_id = "/map";
  cloud.width = cloud.points.size();
  cloud.height = 1;
  pcl::io::savePCDFileASCII("/tmp/ndmap.pcd", cloud);
  printf("NDMap points num: %d points.\n", (int)cloud.points.size());

  sensor_msgs::PointCloud2::Ptr ndmap_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud, *ndmap_ptr);
  ndmap_pub.publish(*ndmap_ptr);
}

// load ndt setting file
int load_ndt_ini(char *name)
{
  FILE *ifp;

  ifp = fopen(name, "r");
  if (!ifp)
    return 0;

  // map path
  if (fscanf(ifp, "%s", g_ndmap_name) == EOF)
    return 0;
  // map size
  if (fscanf(ifp, "%d %d %d %lf", &g_map_x, &g_map_y, &g_map_z, &g_map_cellsize) == EOF)
    return 0;
  // map center
  if (fscanf(ifp, "%lf %lf %lf %lf", &g_map_center_x, &g_map_center_y, &g_map_center_z, &g_map_rotation) == EOF)
    return 0;
  // use gnss
  if (fscanf(ifp, "%d", &g_use_gnss) == EOF)
    return 0;
  if (!g_use_gnss)
  {
    if (fscanf(ifp, "%lf %lf %lf %lf %lf %lf", &g_ini_x, &g_ini_y, &g_ini_z, &g_ini_roll, &g_ini_pitch, &g_ini_yaw) ==
        EOF)
      return 0;
  }

  //
  return 1;
}

int load_nd_map(char *name)
{
  //  int i,j,k,layer;
  NDData nddat;
  NDMapPtr ndmap[2];
  NDPtr ndp;
  FILE *ifp;
  //  FILE *logfp;

  ndmap[0] = NDmap;
  ndmap[1] = NDmap->next;

  ifp = fopen(name, "r");
  if (!ifp)
    return 0;

  while (fread(&nddat, sizeof(NDData), 1, ifp) > 0)
  {
    ndp = add_ND();
    *ndp = nddat.nd;
    *(ndmap[nddat.layer]->nd + nddat.x * ndmap[nddat.layer]->to_x + nddat.y * ndmap[nddat.layer]->to_y + nddat.z) = ndp;
    ndp->flag = 0;
    update_covariance(ndp);
  }

  printf("%d NDVoxels are loaded\n", NDs_num);
  fclose(ifp);
  return 1;
}

=======
}

>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
int main(int argc, char *argv[])
{
  std::cout << "3D NDT scan matching" << std::endl;

  ros::init(argc, argv, "ndt_matching_tku");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("init_x", g_ini_x);
  private_nh.getParam("init_y", g_ini_y);
  private_nh.getParam("init_z", g_ini_z);
  private_nh.getParam("init_roll", g_ini_roll);
  private_nh.getParam("init_pitch", g_ini_pitch);
  private_nh.getParam("init_yaw", g_ini_yaw);
  private_nh.getParam("use_gnss", g_use_gnss);

<<<<<<< HEAD
  if (nh.getParam("tf_x", _tf_x) == false)
=======
  if (!nh.getParam("tf_x", _tf_x))
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_y", _tf_y))
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_z", _tf_z))
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_roll", _tf_roll))
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_pitch", _tf_pitch))
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (!nh.getParam("tf_yaw", _tf_yaw))
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  // map path
  sprintf(g_ndmap_name, "%s", "ndmap");

  // map size
  g_map_x = G_MAP_X;
  g_map_y = G_MAP_Y;
  g_map_z = G_MAP_Z;
  g_map_cellsize = G_MAP_CELLSIZE;
  // map center
  g_map_center_x = g_ini_x;
  g_map_center_y = g_ini_y;
  g_map_center_z = g_ini_z;
  g_map_rotation = 0.0;

  Eigen::Translation3f tl_local_to_global(g_map_center_x, g_map_center_y, g_map_center_z);  // tl: translation
  Eigen::AngleAxisf rot_x_local_to_global(0.0, Eigen::Vector3f::UnitX());                   // rot: rotation
  Eigen::AngleAxisf rot_y_local_to_global(0.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_local_to_global(g_map_rotation, Eigen::Vector3f::UnitZ());
  q_local_to_global.setRPY(0.0, 0.0, g_map_rotation);
  tf_local_to_global =
      (tl_local_to_global * rot_z_local_to_global * rot_y_local_to_global * rot_x_local_to_global).matrix();
<<<<<<< HEAD
=======

  Eigen::Translation3f tl_global_to_local((-1.0) * g_map_center_x, (-1.0) * g_map_center_y,
                                          (-1.0) * g_map_center_z);      // tl: translation
  Eigen::AngleAxisf rot_x_global_to_local(0, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_global_to_local(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_global_to_local((-1.0) * g_map_rotation, Eigen::Vector3f::UnitZ());
  q_global_to_local.setRPY(0.0, 0.0, (-1.0) * g_map_rotation);
  tf_global_to_local =
      (tl_global_to_local * rot_z_global_to_local * rot_y_global_to_local * rot_x_global_to_local).matrix();
>>>>>>> 95e58aa41584be2ace4cc49446a8ee2e4d22594e

  /*initialize(clear) NDmap data*/
  NDmap = initialize_NDmap();

  // load map
  prev_pose.x = (g_ini_x - g_map_center_x) * cos(-g_map_rotation) - (g_ini_y - g_map_center_y) * sin(-g_map_rotation);
  prev_pose.y = (g_ini_x - g_map_center_x) * sin(-g_map_rotation) + (g_ini_y - g_map_center_y) * cos(-g_map_rotation);
  prev_pose.z = g_ini_z - g_map_center_z;
  prev_pose.theta = g_ini_roll;
  prev_pose.theta2 = g_ini_pitch;
  prev_pose.theta3 = g_ini_yaw - g_map_rotation;

  prev_pose2 = prev_pose;
  is_first_time = 1;

  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1000);
  localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 1000);

  ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialpose_callback);
  ros::Subscriber points_sub = nh.subscribe("filtered_points", 1000, points_callback);

  ros::spin();

  return 1;
}