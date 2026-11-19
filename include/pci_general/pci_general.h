#ifndef PCI_GENERAL_H_
#define PCI_GENERAL_H_

#include <cmath>
#include <ros1_compat/node_handle.h>
#include <planner_msgs/ros2_service_traits.hpp>
#include <ros1_compat/ros1_compat.h>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mav_msgs/conversions.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mav_msgs/default_topics.hpp>
#include <mav_msgs/msg/status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include "planner_control_interface/pci_manager.h"
#include <planner_msgs/msg/robot_status.hpp>
#include <fstream>
#include <string>

namespace explorer {

class PCIGeneral : public PCIManager {
 public:
  PCIGeneral(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  bool loadParams(const std::string ns);
  bool initialize();
  bool initMotion();
  bool executePath(const std::vector<geometry_msgs::msg::Pose> &path,
                   std::vector<geometry_msgs::msg::Pose> &modified_path,
                   ExecutionPathType path_type = ExecutionPathType::kLocalPath);
  void setState(const geometry_msgs::msg::Pose &pose);
  void setCurrentVelocity(const geometry_msgs::msg::Vector3 &vel);
  void setVelocity(double v);
  double getVelocity(ExecutionPathType path_type);
  bool goToWaypoint(geometry_msgs::msg::Pose &pose);
  bool planAhead() {return ((planner_trigger_lead_time_>0)?true:false);}
  void allocateYawAlongPath(std::vector<geometry_msgs::msg::Pose> &path) const;
  void allocateYawAlongFistSegment(std::vector<geometry_msgs::msg::Pose> &path) const;
  std::vector<std::string> split_string(std::string input_string);
  void pathFollowingCallback(const std_msgs::msg::String::ConstSharedPtr& msg);
  double calculateAbsDeltaAngle(double angle1, double angle2);

  inline void convert(const StateVec &st, geometry_msgs::msg::Pose &p)
  {
    tf::Quaternion quat;
    Eigen::Matrix3d rot_eigen;
    rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(st[3], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    rot_eigen = rot_eigen * Eigen::AngleAxisd(st[4], Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_eigen(rot_eigen);
    quat.setX(q_eigen.x());
    quat.setY(q_eigen.y());
    quat.setZ(q_eigen.z());
    quat.setW(q_eigen.w());
    tf::Vector3 origin(st[0], st[1], st[2]);
    tf::Pose poseTF(quat, origin);
    tf::poseTFToMsg(poseTF, p);
  }

  inline void convert(const geometry_msgs::msg::Pose &p, StateVec &st)
  {
    st[0] = p.position.x;
    st[1] = p.position.y;
    st[2] = p.position.z;
    Eigen::Quaterniond q;
    q.x() = p.orientation.x;
    q.y() = p.orientation.y;
    q.z() = p.orientation.z;
    q.w() = p.orientation.w;
    Eigen::Vector3d euler_custom = quaternionToEuler(q);
    st[3] = euler_custom.z();
    st[4] = euler_custom.y();
  }

  inline void convert(const std::vector<geometry_msgs::msg::Pose> &pose_path, std::vector<StateVec> &vec_path)
  {
    vec_path.clear();
    for(auto &p : pose_path)
    {
      StateVec vp;
      convert(p, vp);
      vec_path.push_back(vp);
    }
  }

  inline void convert(const std::vector<StateVec> &vec_path, std::vector<geometry_msgs::msg::Pose> &pose_path)
  {
    pose_path.clear();
    for(auto &vp : vec_path)
    {
      geometry_msgs::msg::Pose p;
      convert(vp, p);
      pose_path.push_back(p);
    }
  }

  inline void truncateAngle(double &angle)
  {
    if (angle < -M_PI)
      angle += 2 * M_PI;
    else if (angle > M_PI)
      angle -= 2*M_PI;
  }

  inline double getDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
  {
    StateVec vec1, vec2;
    convert(pose1, vec1);
    convert(pose2, vec2);
    return (vec1.head(3) - vec2.head(3)).norm();
  }

  inline double pathLength(std::vector<geometry_msgs::msg::Pose> &path)
  {
    double path_length = 0;
    for(int i=1; i<path.size(); ++i)
    {
      StateVec vec1, vec2;
      convert(path[i], vec1);
      convert(path[i-1], vec2);
      path_length += (vec1.head(3) - vec2.head(3)).norm();
    }
    return path_length;
  }

  inline void linearlyInterpolateYaw(std::vector<geometry_msgs::msg::Pose> &path)  // Keeps the yaw same for first and last pose, and interpolates the rest
  {
    if(path.size() <= 2)
      return;

    StateVec first_point, last_point;
    convert(path.front(), first_point);
    convert(path.back(), last_point);

    int sign_factor = 1;
    if(last_point[3] > first_point[3])
    {
      if(std::abs(last_point[3] - first_point[3]) >= M_PI)
      {
        sign_factor = -1;
      }
      else
      {
        sign_factor = 1;
      }
    }
    else
    {
      if(std::abs(last_point[3] - first_point[3]) >= M_PI)
      {
        sign_factor = 1;
      }
      else
      {
        sign_factor = -1;
      }
    }

    double delta_theta = last_point[3] - first_point[3];
    truncateAngle(delta_theta);
    delta_theta = std::abs(delta_theta);
    double path_length = pathLength(path);
    double prev_yaw = first_point[3];
    for(int i=1; i<path.size()-1; ++i)
    {
      double del_theta = delta_theta * getDistance(path[i], path[i-1]) / path_length;
      double new_yaw = prev_yaw + sign_factor * del_theta;
      truncateAngle(new_yaw);
      StateVec new_point;
      convert(path[i], new_point);
      new_point[3] = new_yaw;
      convert(new_point, path[i]);
      prev_yaw = new_yaw;
    }
  }

  inline Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) const {
      Eigen::Vector3d angles;

      // Roll (X-axis rotation)
      double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
      double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
      angles.x() = std::atan2(sinr_cosp, cosr_cosp);

      // Pitch (Y-axis rotation) - ensuring it's in the range [-90, 90]
      double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
      if (std::abs(sinp) >= 1)
          angles.y() = std::copysign(M_PI/2.0, sinp); // Clamp to -90 or 90 degrees
      else
          angles.y() = std::asin(sinp);

      // Yaw (Z-axis rotation)
      double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
      double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
      angles.z() = std::atan2(siny_cosp, cosy_cosp);

      return angles;
  }

  inline double signedAngularDistance(double a, double b) {
      // Normalize the difference to [-π, π]
      double diff = b - a;
      truncateAngle(diff);
      return diff;
  }

 private:

  visualization_msgs::msg::MarkerArray::SharedPtr generateTrajectoryMarkerArray(const trajectory_msgs::msg::MultiDOFJointTrajectory& traj) const;

  ros::Publisher trajectory_pub_;
  ros::Publisher path_pub_;
  ros::Publisher is_homing_pub_;
  ros::Publisher carrot_pose_pub_;
  ros::Publisher act_cam_angle_pub_;
  ros::Publisher carrot_point_pub_;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber cam_pitch_sub_;

  ros::Timer execution_timer_;

  RunModeType run_mode_;
  bool init_motion_enable_;
  double v_max_;
  double v_init_max_;
  double v_homing_max_;
  double v_narrow_env_max_;
  double yaw_rate_max_;
  double a_max_;
  double dt_;
  double planner_trigger_lead_time_;
  double path_end_dist_thr_;
  double path_end_yaw_thr_;
  double path_end_dist_scale_;
  double path_progression_dist_thr_;
  double path_progression_dist_thr_min_;
  double path_progression_yaw_thr_;
  double path_progression_yaw_thr_min_;
  double path_progression_pitch_thr_;
  std::string world_frame_id_;
  bool smooth_heading_enable_;
  double trajectory_lead_time_;
  bool smooth_homing_enable_;
  bool interpolate_traj_;
  bool use_time_from_start_;
  bool reconnect_path_;
  bool set_first_wp_speed_;
  bool pub_singple_wp_;

  int path_waypoint_ind_ = 0;
  int waypoint_carrot_slack_ = 5;

  double absolute_time_from_start_ =  0.5;

  std::string save_map_service_;
  bool is_homing_ = false;

  double init_z_takeoff_;
  double init_z_drop_;
  double init_x_forward_;

  ExecutionPathType current_path_type_;

  bool concatenate_path_enable_ = true;
  std::vector<geometry_msgs::msg::Pose> executing_path_;

  trajectory_msgs::msg::MultiDOFJointTrajectory samples_array_;
  mav_msgs::EigenTrajectoryPoint trajectory_point_;
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
  nav_msgs::msg::Path output_path_;

  double current_cam_pitch_;

  int n_seq_;
  int wp_curr_;
  // Add offset to account for constant tracking error of controller of aerial
  // robot --> for simulation only.
  const double z_control_offset = 0.0;
  double z_offset_a_ = 0.0; // const value
  double z_offset_b_ = 0.0; // slope over time
  double max_mission_time_;
  double z_off_t_start_;
  double mission_start_time_ = -1.0;
  const double kVelMax = 2.0;
  const double kVelMin = 0.2;
  void interpolatePath(const std::vector<geometry_msgs::msg::Pose> &path,
                       std::vector<geometry_msgs::msg::Pose> &path_res);
  void interpolatePath(const std::vector<geometry_msgs::msg::Pose> &path,
                       std::vector<geometry_msgs::msg::Pose> &path_res, double v_max,
                       double yaw_rate_max);

  // Track robot's status.
  ros::Subscriber status_sub_;
  ros::Subscriber wp_curr_sub_;
  ros::Publisher robot_status_pub_;
  void statusCallback(const mav_msgs::msg::Status &status);
  void wpCurrCallback(const std_msgs::msg::Int32 &wp);
  void executionTimerCallback(const ros::TimerEvent& event);
  void camPitchCallback(const sensor_msgs::msg::JointState &state);

  inline double diffPos(const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2) {
    return std::sqrt( (p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) +
                      (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y) +
                      (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z));
  }

  inline void truncateYaw(double& x) {
    if (x > M_PI)
      x -= 2 * M_PI;
    else if (x < -M_PI)
      x += 2 * M_PI;
  }

  double calculateDistance(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2);
  double getEndPointDistanceAlongPath(const std::vector<geometry_msgs::msg::Pose>& path);

  bool reconnectPath(const std::vector<geometry_msgs::msg::Pose> &path, std::vector<geometry_msgs::msg::Pose> &path_new);

};

}  // namespace explorer

#endif
