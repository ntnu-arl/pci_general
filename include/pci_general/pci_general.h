#ifndef PCI_GENERAL_H_
#define PCI_GENERAL_H_

#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Status.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "planner_control_interface/pci_manager.h"
#include "planner_msgs/RobotStatus.h"
#include <fstream>
#include <string>

namespace explorer {

class PCIGeneral : public PCIManager {
 public:
  PCIGeneral(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  bool loadParams(const std::string ns);
  bool initialize();
  bool initMotion();
  bool executePath(const std::vector<geometry_msgs::Pose> &path,
                   std::vector<geometry_msgs::Pose> &modified_path,
                   ExecutionPathType path_type = ExecutionPathType::kLocalPath);
  void setState(const geometry_msgs::Pose &pose);
  void setCurrentVelocity(const geometry_msgs::Vector3 &vel);
  void setVelocity(double v);
  double getVelocity(ExecutionPathType path_type);
  bool goToWaypoint(geometry_msgs::Pose &pose);
  bool planAhead() {return ((planner_trigger_lead_time_>0)?true:false);}
  void allocateYawAlongPath(std::vector<geometry_msgs::Pose> &path) const;
  void allocateYawAlongFistSegment(std::vector<geometry_msgs::Pose> &path) const;
  std::vector<std::string> split_string(std::string input_string);
  void pathFollowingCallback(const std_msgs::String::ConstPtr& msg);
  double calculateAbsDeltaAngle(double angle1, double angle2);

 private:

  visualization_msgs::MarkerArray::Ptr generateTrajectoryMarkerArray(const trajectory_msgs::MultiDOFJointTrajectory& traj) const;

  ros::Publisher trajectory_pub_;
  ros::Publisher path_pub_;
  ros::Publisher is_homing_pub_;
  ros::Publisher carrot_pose_pub_;

  ros::Subscriber trajectory_sub_;

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
  double path_progression_yaw_thr_;
  std::string world_frame_id_;
  bool smooth_heading_enable_;
  double trajectory_lead_time_;
  bool smooth_homing_enable_;

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
  std::vector<geometry_msgs::Pose> executing_path_;

  trajectory_msgs::MultiDOFJointTrajectory samples_array_;
  mav_msgs::EigenTrajectoryPoint trajectory_point_;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
  nav_msgs::Path output_path_;

  int n_seq_;
  int wp_curr_;
  // Add offset to account for constant tracking error of controller of aerial
  // robot --> for simulation only.
  const double z_control_offset = 0.0;
  const double kVelMax = 2.0;
  const double kVelMin = 0.2;
  void interpolatePath(const std::vector<geometry_msgs::Pose> &path,
                       std::vector<geometry_msgs::Pose> &path_res);
  void interpolatePath(const std::vector<geometry_msgs::Pose> &path,
                       std::vector<geometry_msgs::Pose> &path_res, double v_max,
                       double yaw_rate_max);

  // Track robot's status.
  ros::Subscriber status_sub_;
  ros::Subscriber wp_curr_sub_;
  ros::Publisher robot_status_pub_;
  void statusCallback(const mav_msgs::Status &status);
  void wpCurrCallback(const std_msgs::Int32 &wp);
  void executionTimerCallback(const ros::TimerEvent& event);

  inline double diffPos(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
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

  double calculateDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
  double getEndPointDistanceAlongPath(const std::vector<geometry_msgs::Pose>& path);

  bool reconnectPath(const std::vector<geometry_msgs::Pose> &path, std::vector<geometry_msgs::Pose> &path_new);

};

}  // namespace explorer

#endif
