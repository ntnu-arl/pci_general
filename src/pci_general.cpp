#include "pci_general/pci_general.h"
#include <chrono>
#include <thread>

namespace explorer
{

  PCIGeneral::PCIGeneral(const ros::NodeHandle &nh,
                           const ros::NodeHandle &nh_private)
      : PCIManager(nh, nh_private)
  {
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 10); //"command/trajectory"

    path_pub_ = nh_.advertise<nav_msgs::Path>("/gbplanner_path", 10);
    is_homing_pub_ = nh_.advertise<std_msgs::Bool>("/gbplanner_is_homing", 10);

    status_sub_ = nh_.subscribe("/matrice/status", 10, &PCIGeneral::statusCallback, this);
    robot_status_pub_ = nh_.advertise<planner_msgs::RobotStatus>("/robot_status", 10);
    carrot_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pci_carrot", 10);
    carrot_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("pci_carrot_point", 10);
    act_cam_angle_pub_ = nh_.advertise<std_msgs::Float64>("act_cam_cmd", 10);

    trajectory_sub_ = nh_.subscribe("/pci_general/path_following", 10, &PCIGeneral::pathFollowingCallback, this);
    cam_pitch_sub_ = nh_.subscribe("cam_pitch", 10, &PCIGeneral::camPitchCallback, this);

    execution_timer_ = nh_.createTimer(
        ros::Duration(0.05), &PCIGeneral::executionTimerCallback, this);

    current_path_type_ = ExecutionPathType::kLocalPath;
  }

  void PCIGeneral::statusCallback(const mav_msgs::Status &status)
  {
    planner_msgs::RobotStatus msg;
    msg.header.seq = status.header.seq;
    msg.header.stamp = status.header.stamp;
    msg.header.frame_id = status.header.frame_id;

    // @TODO Find a better conversion.
    // This applies to ARL-M100.
    const float kACoeff = 6.6;
    const float kBCoeff = -66.0;
    float battery_percent = status.battery_voltage;
    msg.time_remaining = battery_percent * kACoeff + kBCoeff;
    robot_status_pub_.publish(msg);
  }

  bool PCIGeneral::initialize()
  {
    // Init some parameters if required.
    n_seq_ = 0;
    pci_status_ = PCIStatus::kReady;
    if (run_mode_ == RunModeType::kSim)
    {
      // Interface with RotorS.
      std_srvs::Empty srv;
      bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

      // Trying to unpause Gazebo for 10 seconds.
      unsigned int i = 0;
      while (i <= 10 && !unpaused)
      {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
      }
      if (!unpaused)
      {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
      }
      else
      {
        ROS_INFO("Unpaused the Gazebo simulation.");
      }

      if (init_motion_enable_)
      {
        initMotion();
      }
    }
    else if (run_mode_ == RunModeType::kReal)
    {
      ROS_INFO("PCIGeneral::initialize: RunModeType::kReal.");
    }
    else
    {
      ROS_ERROR("PCIGeneral::initialize --> Not supported.");
    }
    return true;
  }

  bool PCIGeneral::initMotion()
  {
    n_seq_ = 0;
    ros::Duration(1.0).sleep();
    ROS_INFO("Performing initialization motion");
    ROS_INFO("Current pose: %f, %f, %f", current_pose_.position.x,
             current_pose_.position.y, current_pose_.position.z);

    std::vector<geometry_msgs::Pose> init_path;
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z;
      pose.orientation.x = current_pose_.orientation.x;
      pose.orientation.y = current_pose_.orientation.y;
      pose.orientation.z = current_pose_.orientation.z;
      pose.orientation.w = current_pose_.orientation.w;
      init_path.push_back(pose);
    }
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z + init_z_takeoff_;
      pose.orientation.x = current_pose_.orientation.x;
      pose.orientation.y = current_pose_.orientation.y;
      pose.orientation.z = current_pose_.orientation.z;
      pose.orientation.w = current_pose_.orientation.w;
      init_path.push_back(pose);
    }
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z + init_z_takeoff_ - init_z_drop_;
      pose.orientation.x = current_pose_.orientation.x;
      pose.orientation.y = current_pose_.orientation.y;
      pose.orientation.z = current_pose_.orientation.z;
      pose.orientation.w = current_pose_.orientation.w;
      init_path.push_back(pose);
    }
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x + init_x_forward_;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z + init_z_takeoff_ - init_z_drop_;
      pose.orientation.x = current_pose_.orientation.x;
      pose.orientation.y = current_pose_.orientation.y;
      pose.orientation.z = current_pose_.orientation.z;
      pose.orientation.w = current_pose_.orientation.w;
      init_path.push_back(pose);
    }

    std::vector<geometry_msgs::Pose> path_new;
    interpolatePath(init_path, path_new, v_init_max_, yaw_rate_max_);

    executing_path_ = path_new;

    n_seq_++;
    samples_array_.header.seq = n_seq_;
    samples_array_.header.stamp = ros::Time::now();
    samples_array_.header.frame_id = world_frame_id_;
    samples_array_.points.clear();
    output_path_.header = samples_array_.header;
    output_path_.poses.clear();
    output_path_.poses.reserve(path_new.size());

    double sum_time_from_start = 0.0;
    for (int i = 0; i < path_new.size(); i++)
    {
      double yaw = tf::getYaw(path_new[i].orientation);
      Eigen::Vector3d p(path_new[i].position.x,
                        path_new[i].position.y,
                        path_new[i].position.z);
      trajectory_point_.position_W.x() = p.x();
      trajectory_point_.position_W.y() = p.y();
      trajectory_point_.position_W.z() = p.z();
      trajectory_point_.setFromYaw(yaw);
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(
          trajectory_point_, &trajectory_point_msg_);
      if (i > 0)
      {
        Eigen::Vector3d p0(path_new[i - 1].position.x,
                           path_new[i - 1].position.y,
                           path_new[i - 1].position.z);
        double seg_len = (p - p0).norm();
        double yaw0 = tf::getYaw(path_new[i - 1].orientation);
        double dyaw = yaw - yaw0;
        if (dyaw > M_PI)
          dyaw -= 2 * M_PI;
        else if (dyaw < -M_PI)
          dyaw += 2 * M_PI;
        const double kEpsilon = 0.001;
        sum_time_from_start +=
            std::max(seg_len / v_init_max_, std::abs(dyaw) / yaw_rate_max_) + kEpsilon; // to avoid the Nan issue for MPC.
      }
      trajectory_point_msg_.time_from_start =
          ros::Duration(sum_time_from_start);
      samples_array_.points.push_back(trajectory_point_msg_);

      geometry_msgs::PoseStamped tmp;
      tmp.header = samples_array_.header;
      tmp.pose = path_new[i];
      output_path_.poses.push_back(tmp);
    }
    trajectory_vis_pub_.publish(generateTrajectoryMarkerArray(samples_array_));
    trajectory_pub_.publish(samples_array_);
    path_pub_.publish(output_path_);
    path_waypoint_ind_ = 0;

    pci_status_ = PCIStatus::kRunning;
    double wait_time = sum_time_from_start;

    ROS_INFO(". . . waiting initMotion()");
    ros::Duration(5.0).sleep();

    pci_status_ = PCIStatus::kReady;
    ROS_INFO("initMotion() - Done");
    return true;
  }

  std::vector<std::string> PCIGeneral::split_string(std::string input_string)
  {
    std::string::iterator new_end = std::unique(input_string.begin(), input_string.end(), [](const char &x, const char &y)
                                                { return x == y and x == ' '; });

    input_string.erase(new_end, input_string.end());

    while (input_string[input_string.length() - 1] == ' ')
    {
      input_string.pop_back();
    }

    std::vector<std::string> splits;
    char delimiter = ' ';

    size_t i = 0;
    size_t pos = input_string.find(delimiter);

    while (pos != std::string::npos)
    {
      splits.push_back(input_string.substr(i, pos - i));
      i = pos + 1;
      pos = input_string.find(delimiter, i);
    }

    splits.push_back(input_string.substr(i, std::min(pos, input_string.length()) - i + 1));
    return splits;
  }

  void PCIGeneral::pathFollowingCallback(const std_msgs::String::ConstPtr &msg)
  {
    // std::cout << "Path following cb" << std::endl;
    std::string filename = msg->data;
    std::ifstream infile(filename.c_str(), std::ifstream::in);

    ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Loading waypoints from the file: %s", filename.c_str());
    std::ifstream ifs(filename.c_str(), std::ifstream::in);

    const int kNumberElements = 7; // x,y,z,qx,qy,qz,qw
    std::vector<std::vector<double>> waypoints;

    std::string line;
    int count = 0;
    bool valid = true;
    while (!infile.eof())
    {
      std::getline(infile, line);
      ++count;
      std::vector<std::string> new_str;
      if (!line.empty())
        new_str = split_string(line);
      else
      {
        break;
      }

      if (new_str.size() != kNumberElements)
      {
        // something wrong
        ROS_ERROR("Line %d has %d fields (must be %d fields x-y-z-qx-qy-qz-qw)", count, (int)new_str.size(), kNumberElements);
        valid = false;
        break;
      }
      std::vector<double> wp;
      for (int i = 0; i < kNumberElements; i++)
      {
        double val = stod(new_str[i]);
        wp.push_back(val);
      }
      waypoints.push_back(wp);
      line = "";
    }
    infile.close();

    if (!valid)
      return;

    ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Generating %d waypoints.", (int)waypoints.size());
    std::vector<geometry_msgs::Pose> path_new, mod_path;
    for (auto &wp : waypoints)
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x + wp[0];
      pose.position.y = current_pose_.position.y + wp[1];
      pose.position.z = current_pose_.position.z + wp[2];
      double q_norm = std::sqrt(wp[3] * wp[3] + wp[4] * wp[4] + wp[5] * wp[5] + wp[6] * wp[6]);
      pose.orientation.x = wp[3] / q_norm;
      pose.orientation.y = wp[4] / q_norm;
      pose.orientation.z = wp[5] / q_norm;
      pose.orientation.w = wp[6] / q_norm;
      std::cout << "P: " << pose.position.x << " " << pose.position.y << " " << pose.position.z
                << " Q: " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << " \n";
      path_new.push_back(pose);
    }

    executePath(path_new, mod_path, ExecutionPathType::kLocalPath);

    ROS_INFO("Finished the trajectory.");
    return;
  }

  visualization_msgs::MarkerArray::Ptr PCIGeneral::generateTrajectoryMarkerArray(const trajectory_msgs::MultiDOFJointTrajectory &traj) const
  {
    auto m_arr = boost::make_shared<visualization_msgs::MarkerArray>();

    visualization_msgs::Marker me;
    me.points.resize(2);
    me.header.stamp = traj.header.stamp;
    me.header.seq = traj.header.seq;
    me.header.frame_id = traj.header.frame_id;
    me.id = 0;
    me.ns = "command_trajectory_edges";
    me.type = visualization_msgs::Marker::ARROW;
    me.action = visualization_msgs::Marker::ADD;
    me.pose.position.x = 0;
    me.pose.position.y = 0;
    me.pose.position.z = 0;
    me.pose.orientation.x = 0;
    me.pose.orientation.y = 0;
    me.pose.orientation.z = 0;
    me.pose.orientation.w = 1;
    me.scale.x = 0.125;
    me.scale.y = 0.25;
    me.scale.z = 0.5;
    // Green
    me.color.r = 0.0;
    me.color.g = 0.75;
    me.color.b = 0.0;
    me.color.a = 1.0;
    me.lifetime = ros::Duration(0.0);
    me.frame_locked = false;

    visualization_msgs::Marker mo;
    mo.header.stamp = traj.header.stamp;
    mo.header.seq = traj.header.seq;
    mo.header.frame_id = traj.header.frame_id;
    mo.id = 0;
    mo.ns = "command_trajectory_orientations";
    mo.type = visualization_msgs::Marker::ARROW;
    mo.action = visualization_msgs::Marker::ADD;
    mo.scale.x = 0.75;
    mo.scale.y = 0.175;
    mo.scale.z = 0.175;
    // Yellow
    mo.color.r = 0.75;
    mo.color.g = 0.75;
    mo.color.b = 0.0;
    mo.color.a = 1.0;
    mo.lifetime = ros::Duration(0.0);
    mo.frame_locked = false;

    for (size_t i = 0; i < traj.points.size() - 1; ++i)
    {
      m_arr->markers.empty() ?: ++me.id, ++mo.id;
      auto ti_v3 = traj.points.at(i).transforms.at(0).translation;
      auto tip1_v3 = traj.points.at(i + 1).transforms.at(0).translation;
      auto &me_p0 = me.points.at(0);
      me_p0.x = ti_v3.x;
      me_p0.y = ti_v3.y;
      me_p0.z = ti_v3.z;
      auto &me_p1 = me.points.at(1);
      me_p1.x = tip1_v3.x;
      me_p1.y = tip1_v3.y;
      me_p1.z = tip1_v3.z;
      m_arr->markers.push_back(me);

      mo.pose.position.x = ti_v3.x;
      mo.pose.position.y = ti_v3.y;
      mo.pose.position.z = ti_v3.z;
      auto ti_q = traj.points.at(i).transforms.at(0).rotation;
      mo.pose.orientation.x = ti_q.x;
      mo.pose.orientation.y = ti_q.y;
      mo.pose.orientation.z = ti_q.z;
      mo.pose.orientation.w = ti_q.w;
      m_arr->markers.push_back(mo);
    }
    m_arr->markers.empty() ?: ++mo.id;
    auto tb_v3 = traj.points.back().transforms.at(0).translation;
    mo.pose.position.x = tb_v3.x;
    mo.pose.position.y = tb_v3.y;
    mo.pose.position.z = tb_v3.z;
    auto tb_q = traj.points.back().transforms.at(0).rotation;
    mo.pose.orientation.x = tb_q.x;
    mo.pose.orientation.y = tb_q.y;
    mo.pose.orientation.z = tb_q.z;
    mo.pose.orientation.w = tb_q.w;
    m_arr->markers.push_back(mo);

    return m_arr;
  }

  bool PCIGeneral::goToWaypoint(geometry_msgs::Pose &pose)
  {
    ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Going to waypoint.");
    // From current pose to the target pose.
    std::vector<geometry_msgs::Pose> path{current_pose_, pose};
    std::vector<geometry_msgs::Pose> path_to_exe;
    executePath(path, path_to_exe, ExecutionPathType::kManualPath);
    return true;
  }

  bool PCIGeneral::executePath(
      const std::vector<geometry_msgs::Pose> &path,
      std::vector<geometry_msgs::Pose> &modified_path,
      ExecutionPathType path_type)
  {
    if(mission_start_time_ < 0)
    {
      mission_start_time_ = ros::Time::now().toSec();
    }
    if (path.size() <= 1)
      return false; // require at least 2 nodes

    ros::Duration(0.01).sleep(); // sleep to unblock the thread to get latest odometry.
    ros::spinOnce();

    // Setting velocity
    double v_max = v_max_;
    if (path_type == ExecutionPathType::kHomingPath)
    {
      is_homing_ = true;
      if (!save_map_service_.empty())
      {
        ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(save_map_service_);
        std_srvs::Trigger trig;
        if (client.call(trig))
        {
          ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "TRIGGER SAVE-MAP SERVICE.");
        }
        else
        {
          ROS_WARN_COND(global_verbosity >= Verbosity::ERROR, "FAILED TO TRIGGER SAVE-MAP SERVICE.");
        }
      }
      v_max = v_homing_max_;
    }
    else if ((path_type == ExecutionPathType::kNarrowEnvPath))
    {
      is_homing_ = false;
      v_max = v_narrow_env_max_;
    }
    else if((path_type == ExecutionPathType::kManualPath))  // Just to ensure that the planner is not triggered again
    {
      is_homing_ = true;
    }
    else {
      is_homing_ = false;
    }

    current_path_type_ = path_type;

    std::vector<geometry_msgs::Pose> path_new = path;
    if (path_type != ExecutionPathType::kManualPath)
    {
      ROS_WARN("Path type not kManual");
      // Only modify for path derived from auto mode.
      // Extend the path to current position if necessary to achieve better transition.
      if (reconnect_path_)
      {
        if (reconnectPath(path, path_new))
        {
          ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Reconnect done");
        }
        else
        {
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Unsafe to execute this path since it is too far from the current position");
          modified_path.push_back(current_pose_);
          return false;
        }
      }
      else
      {
        path_new = path;
      }

      if (smooth_heading_enable_)
      {
        if(path_type != ExecutionPathType::kHomingPath || smooth_homing_enable_) {
          allocateYawAlongPath(path_new);
        }
      }
    }

    if (path_type != ExecutionPathType::kManualPath)
    {
      linearlyInterpolateYaw(path_new);
    }
    

    // Return final path.
    modified_path = path_new;
    // Store this path for the next iteration.
    
    // Execute the path.
    // No need to interpolate the path in case of MPC.
    n_seq_++;
    std::vector<geometry_msgs::Pose> path_intp;
    
    if(interpolate_traj_)
      interpolatePath(path_new, path_intp);
    else
      path_intp = path_new;
    
    double time_passed = ros::Time::now().toSec() - mission_start_time_;
    ROS_WARN("Mission start time: %f, t now: %f", mission_start_time_, ros::Time::now().toSec());
    if(time_passed >= z_off_t_start_)
    {
      double t_ratio = (time_passed - z_off_t_start_) / (max_mission_time_ - z_off_t_start_);
      if(t_ratio > 1.0) t_ratio = 1.0;
      double z_offset = z_offset_a_ + z_offset_b_ * t_ratio;
      ROS_WARN("[PCI]: z offset: %f", z_offset);
      for (int i = 1; i < path_intp.size(); i++)
      {
        path_intp[i].position.z += z_offset;
      }
    }

    executing_path_ = path_intp;
    
    samples_array_.header.seq = n_seq_;
    samples_array_.header.stamp = ros::Time::now();
    samples_array_.header.frame_id = world_frame_id_;
    samples_array_.points.clear();
    output_path_.header = samples_array_.header;
    output_path_.poses.clear();
    output_path_.poses.reserve(path_intp.size());

    double sum_time_from_start = 0;
    Eigen::Vector3d prev_vel = current_vel_;
    double current_speed = 0.0;
    
    for (int i = 0; i < path_intp.size(); i++)
    {
      double yaw = tf::getYaw(path_intp[i].orientation);
      Eigen::Vector3d p(path_intp[i].position.x,
                        path_intp[i].position.y,
                        path_intp[i].position.z);
      trajectory_point_.position_W.x() = p.x();
      trajectory_point_.position_W.y() = p.y();
      trajectory_point_.position_W.z() = p.z();
      trajectory_point_.setFromYaw(yaw);
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(
          trajectory_point_, &trajectory_point_msg_);
      if(set_first_wp_speed_)
      {
        if(i < path_intp.size()-1)
        {
          trajectory_point_msg_.velocities.clear();
          Eigen::Vector3d p_0, p_1;
          Eigen::Vector3d vel;
          if(i > 0)
          {
            p_0 << path_intp[i-1].position.x, path_intp[i-1].position.y, path_intp[i-1].position.z;
            p_1 << path_intp[i].position.x, path_intp[i].position.y, path_intp[i].position.z;
          }
          else
          {
            p_0 << current_pose_.position.x, current_pose_.position.y, current_pose_.position.z;
            p_1 << path_intp[i].position.x, path_intp[i].position.y, path_intp[i].position.z;
          }
          Eigen::Vector3d p_2(path_intp[i+1].position.x, path_intp[i+1].position.y, path_intp[i+1].position.z);
          Eigen::Vector3d d1 = p_0 - p_1;
          Eigen::Vector3d d2 = p_2 - p_1;
          if(d1.norm() <= 0.001 || d2.norm() <= 0.001)
          {
            vel << 0.0, 0.0, 0.0;
          }
          else
          {
            double d = d2.normalized().dot(d1.normalized());
            if(d > 1.0) d = 1.0;
            else if(d < -1.0) d = -1.0;
            double theta = std::acos(d);
            truncateAngle(theta);
            double aloc_speed = std::max(0.0, (v_max - 0.0) * (2 * theta / M_PI - 1) + 0.0);
            if(i < path_intp.size()-2)
            {
              Eigen::Vector3d p_3(path_intp[i+2].position.x, path_intp[i+2].position.y, path_intp[i+2].position.z);
              vel = aloc_speed * (p_3 - p_1).normalized();
            }
            else
            {
              vel = aloc_speed * (p_2 - p_0).normalized();
            }
          }
          geometry_msgs::Twist twist;
          twist.linear.x = vel.x();
          twist.linear.y = vel.y();
          twist.linear.z = vel.z();
          trajectory_point_msg_.velocities.push_back(twist);
        }
      }
      if (i > 0)
      {
        Eigen::Vector3d p0(path_intp[i - 1].position.x,
                            path_intp[i - 1].position.y,
                            path_intp[i - 1].position.z);
        Eigen::Vector3d seg = p - p0;
        double seg_len = seg.norm();
        double yaw0 = tf::getYaw(path_intp[i - 1].orientation);
        double dyaw = yaw - yaw0;
        if (dyaw > M_PI)
          dyaw -= 2 * M_PI;
        else if (dyaw < -M_PI)
          dyaw += 2 * M_PI;

        const double kEpsilon = 0.001;
        double trans_time, end_speed;
        double current_proj_speed = current_speed;
        end_speed = current_proj_speed;
        if(false)
        {
          trans_time = seg_len / v_max_;
        }
        else
        {
          if(seg_len <= 0.0) 
          {
            trans_time = 0.0;
          }
          else
          {
            end_speed = std::sqrt(current_proj_speed*current_proj_speed + 2 * a_max_ * seg_len);
            if(end_speed > v_max_) 
            {
              end_speed = v_max_;
              double ta = (end_speed - current_proj_speed) / a_max_;
              double da = current_proj_speed * ta + 0.5 * a_max_ * ta * ta;
              double tc = (seg_len - da) / end_speed;
              trans_time = ta + tc;
            }
            else
            {
              trans_time = (end_speed - current_proj_speed) / a_max_;
            }
          }
        }
        
        double rot_time = std::abs(dyaw) / yaw_rate_max_;
        double delta_time = std::max(trans_time, rot_time);
        sum_time_from_start +=
            std::max(trans_time, rot_time) + kEpsilon; // to avoid the Nan issue for MPC.
        if(seg_len > 0.0)
        {
          current_speed = end_speed;
          prev_vel = seg.normalized() * end_speed;
        }
        else
        {
          current_speed = 0.0;
        }
      }

      trajectory_point_msg_.time_from_start =
          ros::Duration(sum_time_from_start);
      samples_array_.points.push_back(trajectory_point_msg_);

      geometry_msgs::PoseStamped tmp;
      tmp.header = samples_array_.header;
      tmp.pose = path_intp[i];
      output_path_.poses.push_back(tmp);
    }
    absolute_time_from_start_ = sum_time_from_start;
    trajectory_vis_pub_.publish(generateTrajectoryMarkerArray(samples_array_));
    trajectory_pub_.publish(samples_array_);
    path_pub_.publish(output_path_);

    std_msgs::Bool is_homing_msg; 
    is_homing_msg.data = is_homing_;
    is_homing_pub_.publish(is_homing_msg);
    pci_status_ = PCIStatus::kRunning;

    path_waypoint_ind_ = 0;
    return true;
  }


  void PCIGeneral::executionTimerCallback(const ros::TimerEvent &event)
  {
    if (force_stop_)
    {
      force_stop_ = false;
      pci_status_ = PCIStatus::kError;
      path_waypoint_ind_ = 0;
    }
    if (pci_status_ == PCIStatus::kRunning)
    {
      double dist_thr;
      double yaw_thr = path_progression_yaw_thr_;
      double pitch_thr = path_progression_yaw_thr_;

      Eigen::Quaterniond q;
      q.x() = executing_path_[path_waypoint_ind_].orientation.x;
      q.y() = executing_path_[path_waypoint_ind_].orientation.y;
      q.z() = executing_path_[path_waypoint_ind_].orientation.z;
      q.w() = executing_path_[path_waypoint_ind_].orientation.w;
      Eigen::Vector3d euler_custom = quaternionToEuler(q);
      double wp_yaw = euler_custom.z();
      double wp_pitch = euler_custom.y();

      double current_yaw = tf::getYaw(current_pose_.orientation);
      truncateYaw(current_yaw);

      Eigen::Vector3d current_pos(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
      Eigen::Vector3d p_1(executing_path_[path_waypoint_ind_].position.x, executing_path_[path_waypoint_ind_].position.y, executing_path_[path_waypoint_ind_].position.z);

      if (path_waypoint_ind_ < executing_path_.size() - 1)
      {
        Eigen::Vector3d p_2(executing_path_[path_waypoint_ind_ + 1].position.x, executing_path_[path_waypoint_ind_ + 1].position.y, executing_path_[path_waypoint_ind_ + 1].position.z);
        Eigen::Vector3d d1 = current_pos - p_1;
        Eigen::Vector3d d2 = p_2 - p_1;
        if(d1.norm() <= 0.001 || d2.norm() <= 0.001)
        {
          dist_thr = path_end_dist_thr_;
        }
        else
        {
          double theta = std::acos((p_2 - p_1).normalized().dot((current_pos - p_1).normalized()));
          truncateAngle(theta);
          dist_thr = std::max(path_progression_dist_thr_min_, (path_progression_dist_thr_ - path_progression_dist_thr_min_) * (2 * theta / M_PI - 1) + path_progression_dist_thr_min_);
        }
        Eigen::Quaterniond q_next;
        q_next.x() = executing_path_[path_waypoint_ind_+1].orientation.x;
        q_next.y() = executing_path_[path_waypoint_ind_+1].orientation.y;
        q_next.z() = executing_path_[path_waypoint_ind_+1].orientation.z;
        q_next.w() = executing_path_[path_waypoint_ind_+1].orientation.w;
        Eigen::Vector3d euler_custom_next = quaternionToEuler(q_next);
        double next_wp_yaw = euler_custom_next.z();
        double syd1 = signedAngularDistance(wp_yaw, current_yaw);
        double syd2 = signedAngularDistance(next_wp_yaw, wp_yaw);
        if(std::signbit(syd2) != std::signbit(syd1))
        {
          yaw_thr = path_progression_yaw_thr_min_;
        }
      }
      else
      {
        dist_thr = path_end_dist_thr_;
      }

      double wp_dist = calculateDistance(current_pose_, executing_path_[path_waypoint_ind_]);
      
      double delta_yaw = std::abs(wp_yaw - current_yaw);
      truncateYaw(delta_yaw);

      double delta_pitch = std::abs(wp_pitch - current_cam_pitch_);
      truncateYaw(delta_pitch);
      std::cout << "wp_dist: " << wp_dist << ", dist_thr: " << dist_thr << ", delta_yaw: " << delta_yaw << ", yaw_thr: " << yaw_thr << ", delta_pitch: " << delta_pitch << ", path_thr: " << path_progression_pitch_thr_ << std::endl;
      if (wp_dist <= dist_thr && delta_yaw <= yaw_thr && delta_pitch <= path_progression_pitch_thr_)
      {
        ++path_waypoint_ind_;
        if (path_waypoint_ind_ >= executing_path_.size())
        {
          pci_status_ = PCIStatus::kReady;
          ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "PCI: Ready to trigger the planner.");
        }
      }
      if (pci_status_ == PCIStatus::kRunning)
      {
        geometry_msgs::PoseStamped ps;
        ps.pose = executing_path_[path_waypoint_ind_];
        StateVec s;
        convert(ps.pose, s);
        s(4) = 0;
        convert(s, ps.pose);
        ps.header.frame_id = "world";
        carrot_pose_pub_.publish(ps);
        geometry_msgs::PointStamped pts;
        pts.point = ps.pose.position;
        pts.header.frame_id = "map";
        carrot_point_pub_.publish(pts);
        // Publish ps as a path
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        path.poses.push_back(ps);
        if(pub_singple_wp_)
          path_pub_.publish(path);
        double pitch_cmd = euler_custom.y();
        std_msgs::Float64 pitch_msg;
        pitch_msg.data = pitch_cmd;
        act_cam_angle_pub_.publish(pitch_msg);
      }

      // if (pci_status_ == PCIStatus::kRunning)
      // {
      //   geometry_msgs::PoseStamped ps;
      //   ps.pose = executing_path_[path_waypoint_ind_];
      //   StateVec s;
      //   convert(ps.pose, s);
      //   s(4) = 0;
      //   convert(s, ps.pose);
      //   ps.header.frame_id = "world";
      //   carrot_pose_pub_.publish(ps);
      //   geometry_msgs::PointStamped pts;
      //   pts.point = ps.pose.position;
      //   pts.header.frame_id = "map";
      //   carrot_point_pub_.publish(pts);
      //   // Publish ps as a path
      //   nav_msgs::Path path;
      //   path.header.frame_id = "world";
      //   path.header.stamp = ros::Time::now();
      //   path.poses.push_back(ps);
      //   if(pub_singple_wp_)
      //     path_pub_.publish(path);
      //   double pitch_cmd = euler_custom.y();
      //   std_msgs::Float64 pitch_msg;
      //   pitch_msg.data = pitch_cmd;
      //   act_cam_angle_pub_.publish(pitch_msg);
      // }
    }
  }

  void PCIGeneral::camPitchCallback(const sensor_msgs::JointState &state)
  {
    current_cam_pitch_ = state.position[0];
  }

  double PCIGeneral::getEndPointDistanceAlongPath(
      const std::vector<geometry_msgs::Pose> &path)
  {
    std::vector<double> dists;
    for (int i = path_waypoint_ind_; i < path.size(); ++i)
    {
      dists.push_back(calculateDistance(current_pose_, path[i]));
    }
    int closest_waypoint_ind =
        std::min_element(dists.begin(), dists.end()) - dists.begin();

    path_waypoint_ind_ = closest_waypoint_ind;

    double total_dist = 0.0;
    total_dist += calculateDistance(current_pose_, path[closest_waypoint_ind]);
    for (int i = closest_waypoint_ind + 1; i < path.size(); ++i)
    {
      total_dist += calculateDistance(path[i], path[i - 1]);
    }
    return total_dist;
  }

  double PCIGeneral::calculateDistance(const geometry_msgs::Pose &p1,
                                        const geometry_msgs::Pose &p2)
  {
    Eigen::Vector3d v1(p1.position.x, p1.position.y, p1.position.z);
    Eigen::Vector3d v2(p2.position.x, p2.position.y, p2.position.z);

    return (v1 - v2).head(3).norm();
  }

  double PCIGeneral::calculateAbsDeltaAngle(double angle1, double angle2)
  {
    truncateYaw(angle1);
    truncateYaw(angle2);
    double delta = std::abs(angle1 - angle2);
    truncateYaw(delta);
    return delta;
  }

  void PCIGeneral::interpolatePath(const std::vector<geometry_msgs::Pose> &path,
                                    std::vector<geometry_msgs::Pose> &path_res)
  {
    path_res.clear();
    if (path.size() == 0)
      return;
    if (path.size() == 1)
      path_res.push_back(path[0]);

    for (int i = 0; i < (path.size() - 1); ++i)
    {
      // Interpolate each segment.
      Eigen::Vector3d start(path[i].position.x, path[i].position.y, path[i].position.z);
      Eigen::Vector3d end(path[i + 1].position.x, path[i + 1].position.y, path[i + 1].position.z);
      Eigen::Vector3d distance = end - start;
      double yaw_start = tf::getYaw(path[i].orientation);
      double yaw_end = tf::getYaw(path[i + 1].orientation);
      double yaw_direction = yaw_end - yaw_start;
      if (yaw_direction > M_PI)
      {
        yaw_direction -= 2.0 * M_PI;
      }
      if (yaw_direction < -M_PI)
      {
        yaw_direction += 2.0 * M_PI;
      }

      double dist_norm = distance.norm();
      double disc = std::min(dt_ * v_max_ / dist_norm, dt_ * yaw_rate_max_ / abs(yaw_direction));

      bool int_flag = true;

      if (int_flag)
      {
        for (double it = 0.0; it <= 1.0; it += disc)
        {
          tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                             (1.0 - it) * start[1] + it * end[1],
                             (1.0 - it) * start[2] + it * end[2]);
          double yaw = yaw_start + yaw_direction * it;
          if (yaw > M_PI)
            yaw -= 2.0 * M_PI;
          if (yaw < -M_PI)
            yaw += 2.0 * M_PI;
          tf::Quaternion quat;
          quat.setEuler(0.0, 0.0, yaw);
          tf::Pose poseTF(quat, origin);
          geometry_msgs::Pose pose;
          tf::poseTFToMsg(poseTF, pose);
          path_res.push_back(pose);
        }
      }
    }

    path_res.push_back(path.back());
  }

  void PCIGeneral::interpolatePath(const std::vector<geometry_msgs::Pose> &path,
                                    std::vector<geometry_msgs::Pose> &path_res, double v_max,
                                    double yaw_rate_max)
  {
    for (int i = 0; i < (path.size() - 1); ++i)
    {
      // Interpolate each segment.
      Eigen::Vector3d start(path[i].position.x, path[i].position.y, path[i].position.z);
      Eigen::Vector3d end(path[i + 1].position.x, path[i + 1].position.y, path[i + 1].position.z);
      Eigen::Vector3d distance = end - start;
      double yaw_start = tf::getYaw(path[i].orientation);
      double yaw_end = tf::getYaw(path[i + 1].orientation);
      double yaw_direction = yaw_end - yaw_start;
      if (yaw_direction > M_PI)
      {
        yaw_direction -= 2.0 * M_PI;
      }
      if (yaw_direction < -M_PI)
      {
        yaw_direction += 2.0 * M_PI;
      }
      double disc = std::min(dt_ * v_max / distance.norm(), dt_ * yaw_rate_max / abs(yaw_direction));
      for (double it = 0.0; it <= 1.0; it += disc)
      {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = yaw_start + yaw_direction * it;
        if (yaw > M_PI)
          yaw -= 2.0 * M_PI;
        if (yaw < -M_PI)
          yaw += 2.0 * M_PI;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        path_res.push_back(pose);
      }
    }
  }

  void PCIGeneral::allocateYawAlongFistSegment(std::vector<geometry_msgs::Pose> &path) const
  {
    // Return if only one vertex or less.
    if (path.size() <= 1)
      return;
    // Assign heading on first segment only
    // BUT: Make sure we don't consider a micro-segment for orientation calculation
    for (int i = 1; i < path.size(); ++i)
    {
      Eigen::Vector3d dir_vec;
      dir_vec << path[i].position.x - path[i - 1].position.x,
          path[i].position.y - path[i - 1].position.y,
          path[i].position.z - path[i - 1].position.z;
      if (dir_vec.norm() > 0.5)
      { // TODO: Make parameter for this value (control how big a micro-segment is)
        double yaw_first = std::atan2(path[i].position.y - path[0].position.y,
                                      path[i].position.x - path[0].position.x);
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw_first);
        for (--i; i >= 0; --i)
        {
          path[i].orientation.x = quat.x();
          path[i].orientation.y = quat.y();
          path[i].orientation.z = quat.z();
          path[i].orientation.w = quat.w();
        }
        break;
      }
    }
  }

  void PCIGeneral::allocateYawAlongPath(std::vector<geometry_msgs::Pose> &path) const
  {
    // Return if only one vertex or less.
    if (path.size() <= 1)
      return;
    // Assign new heading along each segment except the first one.
    // double yaw_prev = tf::getYaw(path[0].orientation);
    double yaw_prev = tf::getYaw(current_pose_.orientation);
    Eigen::Vector3d prev_vel = current_vel_;
    path[0].orientation = current_pose_.orientation;
    double current_speed = 0.0;
    for (int i = 1; i < path.size(); ++i)
    {
      Eigen::Vector3d dir_vec;
      dir_vec << path[i].position.x - path[i - 1].position.x,
          path[i].position.y - path[i - 1].position.y,
          path[i].position.z - path[i - 1].position.z;
      double yaw_now = tf::getYaw(path[i].orientation);

      double dyaw = yaw_now - yaw_prev;
      if (dyaw < -M_PI)
        dyaw += 2 * M_PI;
      else if (dyaw > M_PI)
        dyaw -= 2 * M_PI;

      double trans_time, end_speed;
      double current_proj_speed = current_speed;
      end_speed = current_proj_speed;
      if (dir_vec.norm() <= 0.0)
      {
        trans_time = 0.0;
      }
      else
      {
        end_speed = std::sqrt(current_proj_speed * current_proj_speed + 2 * a_max_ * dir_vec.norm());
        if (end_speed > v_max_)
        {
          end_speed = v_max_;
          double ta = (end_speed - current_proj_speed) / a_max_;
          double da = current_proj_speed * ta + 0.5 * a_max_ * ta * ta;
          double tc = (dir_vec.norm() - da) / end_speed;
          trans_time = ta + tc;
        }
        else
        {
          trans_time = (end_speed - current_proj_speed) / a_max_;
        }
      }

      double rot_time = std::abs(dyaw) / yaw_rate_max_;

      if (dir_vec.norm() > 0.0)
      {
        current_speed = end_speed;

        prev_vel = dir_vec.normalized() * end_speed;
      }
      else
      {
        current_speed = 0.0;
      }

      if (trans_time < rot_time)
      {
        // Re-assign another heading.
        yaw_now = yaw_prev + trans_time * yaw_rate_max_ * ((dyaw >= 0) ? 1.0 : -1.0);
        // Truncate again if neccessary.
        if (yaw_now < -M_PI)
          yaw_now += 2 * M_PI;
        else if (yaw_now > M_PI)
          yaw_now -= 2 * M_PI;
        tf::Quaternion quat;
        Eigen::Quaterniond q;
        q.x() = path[i].orientation.x;
        q.y() = path[i].orientation.y;
        q.z() = path[i].orientation.z;
        q.w() = path[i].orientation.w;
        Eigen::Vector3d euler_custom = quaternionToEuler(q);
        double pitch_now = euler_custom.y();
        Eigen::Matrix3d rot_eigen;
        rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(yaw_now, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        rot_eigen = rot_eigen * Eigen::AngleAxisd(pitch_now, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q_eigen(rot_eigen);
        path[i].orientation.x = q_eigen.x();
        path[i].orientation.y = q_eigen.y();
        path[i].orientation.z = q_eigen.z();
        path[i].orientation.w = q_eigen.w();
      }
      yaw_prev = yaw_now;
      prev_vel = dir_vec.normalized() * end_speed;
    }
  }

  bool PCIGeneral::reconnectPath(const std::vector<geometry_msgs::Pose> &path,
                                  std::vector<geometry_msgs::Pose> &path_new)
  {
    path_new = path;

    // Extend the path to current position if necessary to achieve better transition.
    // Check if the path starts from current pose.
    const double kLimLow = 0.75; // all magic numbers
    const double kLimHigh = 1.5;
    Eigen::Vector3d root_pos(path[0].position.x, path[0].position.y, path[0].position.z);
    Eigen::Vector3d second_pos(path[1].position.x, path[1].position.y, path[1].position.z);
    Eigen::Vector3d cur_pos(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
    Eigen::Vector3d ext_seg;
    ext_seg = root_pos - cur_pos;
    double d_ext_seg = ext_seg.norm();
    if (d_ext_seg <= kLimLow)
    {
      ROS_WARN("Reconnect path: current pose is close to the path.");
      // no change needed.
      return true;
    }
    else if (d_ext_seg <= kLimHigh)
    {
      return true;
    }
    else if (planAhead() && concatenate_path_enable_ && (executing_path_.size() >= 1))
    {
      ROS_WARN("Reconnect path: concatenating previous path to new path.");
      // Set path is too far away from current position,
      // but if we want and assume that planner is in auto mode.
      // Extend previous path to this path for safety purpose.
      // First check if we are able to connect them.
      bool reconnect_ok = false;
      const double kDiffEps = 0.001;
      const double kPointToSegmentDistThreshold = 0.20; // deviation from the path due to tracking error in the controller.
      if (diffPos(executing_path_.back(), path.front()) <= kDiffEps)
      {
        ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Trying to connect prev path and current path.");
        // Check if current pos belongs to any segment.
        int exe_path_size = executing_path_.size();
        for (int ind = exe_path_size - 1; ind > 0; --ind)
        {
          if (diffPos(executing_path_[ind - 1], current_pose_) < kLimHigh)
          {
            path_new.insert(path_new.begin(), current_pose_);
            reconnect_ok = true;
            break;
          }
          else
          {
            // check if same direction.
            Eigen::Vector3d v1(executing_path_[ind - 1].position.x - executing_path_[ind].position.x,
                               executing_path_[ind - 1].position.y - executing_path_[ind].position.y,
                               executing_path_[ind - 1].position.z - executing_path_[ind].position.z);
            Eigen::Vector3d v2(current_pose_.position.x - executing_path_[ind].position.x,
                               current_pose_.position.y - executing_path_[ind].position.y,
                               current_pose_.position.z - executing_path_[ind].position.z);
            double v_dot_prod = v1.dot(v2);
            Eigen::Vector3d v_cross_prod = v1.cross(v2);
            if ((v_dot_prod >= 0) &&
                (v_dot_prod < v1.squaredNorm()) &&
                (v_cross_prod.squaredNorm() / v1.norm() < kPointToSegmentDistThreshold))
            {
              ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Belong to this segment");
              path_new.insert(path_new.begin(), current_pose_);
              reconnect_ok = true;
              break;
            }
            else
            {
              // keep adding old vertices until find the solution.
              path_new.insert(path_new.begin(), executing_path_[ind - 1]);
            }
          }
        }
      }
      return reconnect_ok;
    }
    else
    {
      return false;
    }
  }

  bool PCIGeneral::loadParams(const std::string ns)
  {
    std::string param_name;
    std::string parse_str;

    param_name = ns + "/run_mode";
    ros::param::get(param_name, parse_str);
    if (!parse_str.compare("kReal"))
      run_mode_ = RunModeType::kReal;
    else if (!parse_str.compare("kSim"))
      run_mode_ = RunModeType::kSim;
    else
    {
      run_mode_ = RunModeType::kSim;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No run mode setting, set it to kSim.");
    }

    param_name = ns + "/init_motion_enable";
    if (!ros::param::get(param_name, init_motion_enable_))
    {
      init_motion_enable_ = true;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No motion initialization setting, set it to True.");
    }

    param_name = ns + "/planner_trigger_lead_time";
    if (!ros::param::get(param_name, planner_trigger_lead_time_) || (planner_trigger_lead_time_ <= 0.0))
    {
      planner_trigger_lead_time_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No planner_trigger_lead_time setting, set it to 0.0 (s).");
    }
    else
    {
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "planner_trigger_lead_time_: %f", planner_trigger_lead_time_);
    }

    param_name = ns + "/world_frame_id";
    if (!ros::param::get(param_name, world_frame_id_))
    {
      world_frame_id_ = "world";
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No world_frame_id setting, set it to: %s.", world_frame_id_.c_str());
    }

    param_name = ns + "/smooth_heading_enable";
    if (!ros::param::get(param_name, smooth_heading_enable_))
    {
      smooth_heading_enable_ = true;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No smooth_heading_enable_ setting, set it to: True.");
    }

    param_name = ns + "/init_motion/z_takeoff";
    if (!ros::param::get(param_name, init_z_takeoff_))
    {
      init_z_takeoff_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No init_motion/z_takeoff setting, set it to 0.0 (s).");
    }
    param_name = ns + "/init_motion/z_drop";
    if (!ros::param::get(param_name, init_z_drop_))
    {
      init_z_drop_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No init_motion/z_drop setting, set it to 0.0 (s).");
    }
    param_name = ns + "/init_motion/x_forward";
    if (!ros::param::get(param_name, init_x_forward_))
    {
      init_x_forward_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No init_motion/x_forward setting, set it to 0.0 (s).");
    }

    param_name = ns + "/save_map_service";
    if (!ros::param::get(param_name, save_map_service_))
    {
      save_map_service_ = "";
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No save_map_service setting, set it to empty");
    }

    param_name = ns + "/RobotDynamics/v_max";
    if (!ros::param::get(param_name, v_max_))
    {
      v_max_ = 0.2;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_max setting, set it to 0.2 (m/s).");
    }

    param_name = ns + "/RobotDynamics/v_init_max";
    if (!ros::param::get(param_name, v_init_max_))
    {
      v_init_max_ = 0.2;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_max setting, set it to 0.2 (m/s).");
    }

    param_name = ns + "/RobotDynamics/v_homing_max";
    if (!ros::param::get(param_name, v_homing_max_))
    {
      v_homing_max_ = v_max_;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_max_homing setting, set it to %f (m/s).", v_homing_max_);
    }

    param_name = ns + "/RobotDynamics/v_narrow_env_max";
    if (!ros::param::get(param_name, v_narrow_env_max_))
    {
      v_narrow_env_max_ = v_max_;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_narrow_env_max setting, set it to %f (m/s).", v_narrow_env_max_);
    }

    param_name = ns + "/RobotDynamics/yaw_rate_max";
    if (!ros::param::get(param_name, yaw_rate_max_))
    {
      yaw_rate_max_ = M_PI / 8.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No yaw_rate_max setting, set it to PI/8 (rad/s)");
    }

    param_name = ns + "/RobotDynamics/dt";
    if (!ros::param::get(param_name, dt_))
    {
      dt_ = 0.1;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No dt setting, set it to 0.1 (s).");
    }

    param_name = ns + "/RobotDynamics/a_max";
    if (!ros::param::get(param_name, a_max_))
    {
      a_max_ = 1.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No a_max setting, setting it to %f (rad).", a_max_);
    }

    param_name = ns + "/path_end_dist_thr";
    if (!ros::param::get(param_name, path_end_dist_thr_))
    {
      path_end_dist_thr_ = 0.5;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_end_dist_thr setting, setting it to %f (m).", path_end_dist_thr_);
    }

    param_name = ns + "/path_end_yaw_thr";
    if (!ros::param::get(param_name, path_end_yaw_thr_))
    {
      path_end_yaw_thr_ = 0.5;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_end_yaw_thr setting, setting it to %f (rad).", path_end_yaw_thr_);
    }

    param_name = ns + "/path_end_dist_scale";
    if (!ros::param::get(param_name, path_end_dist_scale_))
    {
      path_end_dist_scale_ = 1.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_end_dist_scale setting, setting it to %f.", path_end_dist_scale_);
    }

    param_name = ns + "/path_progression_dist_thr";
    if (!ros::param::get(param_name, path_progression_dist_thr_))
    {
      path_progression_dist_thr_ = 0.5;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_progression_dist_thr setting, setting it to %f (m).", path_progression_dist_thr_);
    }

    param_name = ns + "/path_progression_yaw_thr";
    if (!ros::param::get(param_name, path_progression_yaw_thr_))
    {
      path_progression_yaw_thr_ = 0.5;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_progression_yaw_thr setting, setting it to %f (rad).", path_progression_yaw_thr_);
    }

    param_name = ns + "/path_progression_yaw_thr_min";
    if (!ros::param::get(param_name, path_progression_yaw_thr_min_))
    {
      path_progression_yaw_thr_min_ = path_progression_yaw_thr_;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_progression_yaw_thr_min setting, setting it to %f (rad).", path_progression_yaw_thr_min_);
    }

    param_name = ns + "/path_progression_pitch_thr";
    if (!ros::param::get(param_name, path_progression_pitch_thr_))
    {
      path_progression_pitch_thr_ = 0.1;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_progression_pitch_thr setting, setting it to %f (rad).", path_progression_pitch_thr_);
    }

    param_name = ns + "/trajectory_lead_time";
    if (!ros::param::get(param_name, trajectory_lead_time_))
    {
      trajectory_lead_time_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No trajectory_lead_time setting, setting it to %f (rad).", trajectory_lead_time_);
    }

    param_name = ns + "/smooth_homing_enable";
    if (!ros::param::get(param_name, smooth_homing_enable_))
    {
      smooth_homing_enable_ = smooth_heading_enable_;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No smooth_homing_enable setting, setting it to %f (rad).", smooth_homing_enable_);
    }

    param_name = ns + "/z_offset_a";
    if (!ros::param::get(param_name, z_offset_a_))
    {
      z_offset_a_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No z_offset_a setting, setting it to %f (rad).", z_offset_a_);
    }

    param_name = ns + "/z_offset_b";
    if (!ros::param::get(param_name, z_offset_b_))
    {
      z_offset_b_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No z_offset_b setting, setting it to %f (rad).", z_offset_b_);
    }

    param_name = ns + "/max_mission_time";
    if (!ros::param::get(param_name, max_mission_time_))
    {
      max_mission_time_ = 360.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No max_mission_time setting, setting it to %f (rad).", max_mission_time_);
    }

    param_name = ns + "/z_off_t_start";
    if (!ros::param::get(param_name, z_off_t_start_))
    {
      z_off_t_start_ = 0.0;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No z_off_t_start setting, setting it to %f (rad).", z_off_t_start_);
    }

    param_name = ns + "/interpolate_traj";
    if (!ros::param::get(param_name, interpolate_traj_))
    {
      interpolate_traj_ = true;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No interpolate_traj setting, setting it to %f (rad).", interpolate_traj_);
    }

    param_name = ns + "/use_time_from_start";
    if (!ros::param::get(param_name, use_time_from_start_))
    {
      use_time_from_start_ = true;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No use_time_from_start setting, setting it to %f (rad).", use_time_from_start_);
    }

    param_name = ns + "/path_progression_dist_thr_min";
    if (!ros::param::get(param_name, path_progression_dist_thr_min_))
    {
      path_progression_dist_thr_min_ = path_end_dist_thr_;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No path_progression_dist_thr_min setting, setting it to %f (rad).", path_progression_dist_thr_min_);
    }

    param_name = ns + "/reconnect_path";
    if (!ros::param::get(param_name, reconnect_path_))
    {
      reconnect_path_ = true;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No reconnect_path setting, setting it to %f (rad).", reconnect_path_);
    }

    param_name = ns + "/set_first_wp_speed";
    if (!ros::param::get(param_name, set_first_wp_speed_))
    {
      set_first_wp_speed_ = false;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No set_first_wp_speed setting, setting it to %f (rad).", set_first_wp_speed_);
    }

    param_name = ns + "/pub_singple_wp";
    if (!ros::param::get(param_name, pub_singple_wp_))
    {
      pub_singple_wp_ = false;
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No pub_singple_wp setting, setting it to %f (rad).", pub_singple_wp_);
    }

    absolute_time_from_start_ = trajectory_lead_time_;

    return true;
  }

  void PCIGeneral::setState(const geometry_msgs::Pose &pose)
  {
    current_pose_.position.x = pose.position.x;
    current_pose_.position.y = pose.position.y;
    current_pose_.position.z = pose.position.z;
    current_pose_.orientation.x = pose.orientation.x;
    current_pose_.orientation.y = pose.orientation.y;
    current_pose_.orientation.z = pose.orientation.z;
    current_pose_.orientation.w = pose.orientation.w;
  }

  void PCIGeneral::setCurrentVelocity(const geometry_msgs::Vector3 &vel)
  {
    current_vel_.x() = vel.x;
    current_vel_.y() = vel.y;
    current_vel_.z() = vel.z;
  }

  void PCIGeneral::setVelocity(double v)
  {
    if ((v >= kVelMin) && (v <= kVelMax))
    {
      ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Changed velocity from %f to %f (m/s)", v_max_, v);
      v_max_ = v;
    }
    else if (v != 0)
    {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Setting velocity is out of allowed range [%f, %f]", kVelMin, kVelMax);
    }
  }

  double PCIGeneral::getVelocity(ExecutionPathType path_type)
  {
    if (path_type == ExecutionPathType::kHomingPath)
    {
      return v_homing_max_;
    }
    else if (path_type == ExecutionPathType::kLocalPath)
    {
      return v_max_;
    }
    else if (path_type == ExecutionPathType::kGlobalPath)
    {
      return v_homing_max_;
    }
    else if (path_type == ExecutionPathType::kNarrowEnvPath)
    {
      return v_narrow_env_max_;
    }
    else
    {
      return v_max_;
    }
  }

} // namespace explorer
