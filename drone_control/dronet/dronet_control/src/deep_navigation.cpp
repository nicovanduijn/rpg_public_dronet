#include "dronet_control/deep_navigation.h"


namespace deep_navigation
{

deepNavigation::deepNavigation(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
  :   nh_(nh),
      nh_private_(nh_private),
      name_(nh_private.getNamespace())
{
  ROS_INFO("[%s]: Initializing Deep Control Node", name_.c_str());
  loadParameters();
  deep_network_sub_ = nh_.subscribe("cnn_predictions", 1, &deepNavigation::deepNetworkCallback, this);
  desired_trajectory_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &deepNavigation::desiredTrajectoryCallback, this);
  generated_trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);

  steering_angle_ = 0.0;
  probability_of_collision_ = 0.0;

  // Aggressive initialization
  desired_forward_velocity_ = max_forward_index_;
  desired_angular_velocity_ = 0.0;
}


void deepNavigation::run()
{

  ros::Duration(2.0).sleep();
  ros::Rate rate(30.0);

  while (ros::ok())
  {

    // Desired body frame velocity to world frame
    double desired_forward_velocity_m = (1.0 -  probability_of_collision_) * max_forward_index_;
    if (desired_forward_velocity_m <= 0.0)
    {
      ROS_INFO("Detected negative forward velocity! Drone will now stop!");
      desired_forward_velocity_m  = 0;
    }

    // Low pass filter the velocity and integrate it to get the position
    desired_forward_velocity_ = (1.0 - alpha_velocity_) * desired_forward_velocity_
        + alpha_velocity_ * desired_forward_velocity_m;

    // Stop if velocity is prob of collision is too high
    if (desired_forward_velocity_ < ((1 - critical_prob_coll_) * max_forward_index_))
    {
      desired_forward_velocity_ = 0.0;
    }


    // Low pass filter the angular_velocity (Remember to tune the bebop angular velocity parameters)
    desired_angular_velocity_ = (1.0 - alpha_yaw_) * desired_angular_velocity_ + alpha_yaw_ * steering_angle_;

    // Compute direction to goal
    double x_dir = (desired_trajectory_.point_2.position.x - desired_trajectory_.point_1.position.x);
    double y_dir = (desired_trajectory_.point_2.position.y - desired_trajectory_.point_1.position.y);
    double z_dir = (desired_trajectory_.point_2.position.z - desired_trajectory_.point_1.position.z);
    double mag = sqrt(x_dir*x_dir+y_dir*y_dir + z_dir*z_dir);
    if(mag >1){// only make unit if far away from goal
          x_dir /= mag;
    y_dir /= mag;
    z_dir /= mag;
    }

    // If probability of collision is less than 0.1
    if(desired_forward_velocity_ > ((1 - 0.1) * max_forward_index_)){
        generated_trajectory_.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
        generated_trajectory_.point_1.position.x = NAN;
        generated_trajectory_.point_1.position.y = NAN;
        generated_trajectory_.point_1.position.z = NAN;
        generated_trajectory_.point_1.velocity.x = x_dir;
        generated_trajectory_.point_1.velocity.y = y_dir;
        generated_trajectory_.point_1.velocity.z = z_dir;
        generated_trajectory_.point_1.acceleration_or_force.x = NAN;
        generated_trajectory_.point_1.acceleration_or_force.y = NAN;
        generated_trajectory_.point_1.acceleration_or_force.z = NAN;
        generated_trajectory_.point_1.yaw =atan2(y_dir,x_dir); 
        generated_trajectory_.point_1.yaw_rate = NAN;

        fillUnusedTrajectoryPoint(generated_trajectory_.point_2);
        fillUnusedTrajectoryPoint(generated_trajectory_.point_3);
        fillUnusedTrajectoryPoint(generated_trajectory_.point_4);
        fillUnusedTrajectoryPoint(generated_trajectory_.point_5);

        generated_trajectory_.time_horizon = {NAN, NAN, NAN, NAN, NAN};
        generated_trajectory_.point_valid = {true, false, false, false, false};
      }

    // Use Dronet's output if the probability of collision is high  
    else{
        generated_trajectory_.type = 0;
        generated_trajectory_.point_1.position.x =NAN;
        generated_trajectory_.point_1.position.y =  NAN;
        generated_trajectory_.point_1.position.z = NAN;
        generated_trajectory_.point_1.velocity.x = cos(desired_trajectory_.point_1.yaw)*desired_forward_velocity_;
        generated_trajectory_.point_1.velocity.y = sin(desired_trajectory_.point_1.yaw)*desired_forward_velocity_;
        generated_trajectory_.point_1.velocity.z = (desired_trajectory_.point_2.position.z - desired_trajectory_.point_1.position.z);
        generated_trajectory_.point_1.acceleration_or_force.x = NAN;
        generated_trajectory_.point_1.acceleration_or_force.y = NAN;
        generated_trajectory_.point_1.acceleration_or_force.z = NAN;
        generated_trajectory_.point_1.yaw = NAN;
        generated_trajectory_.point_1.yaw_rate = -desired_angular_velocity_*5;

        fillUnusedTrajectoryPoint(generated_trajectory_.point_2);
        fillUnusedTrajectoryPoint(generated_trajectory_.point_3);
        fillUnusedTrajectoryPoint(generated_trajectory_.point_4);
        fillUnusedTrajectoryPoint(generated_trajectory_.point_5);

        generated_trajectory_.time_horizon = {NAN, NAN, NAN, NAN, NAN};
        generated_trajectory_.point_valid = {true, false, false, false, false};
      }
       
ROS_INFO("desired_forward_velocity_: %.3f , vx: %.3f, vy: %.3f, vz: %.3f, vyaw: %.3f", desired_forward_velocity_, generated_trajectory_.point_1.velocity.x,
         generated_trajectory_.point_1.velocity.y, generated_trajectory_.point_1.velocity.z, generated_trajectory_.point_1.yaw_rate);

    // Publish desired state
        generated_trajectory_pub_.publish(generated_trajectory_);

    rate.sleep();

    ros::spinOnce();

  }

}

void deepNavigation::deepNetworkCallback(const dronet_perception::CNN_out::ConstPtr& msg)
{

  probability_of_collision_ = msg->collision_prob;
  steering_angle_ = msg->steering_angle;

  // Output modulation
  if (steering_angle_ < -1.0) { steering_angle_ = -1.0;}
  if (steering_angle_ > 1.0) { steering_angle_ = 1.0;}

}


void deepNavigation::loadParameters()
{

  ROS_INFO("[%s]: Reading parameters", name_.c_str()); 
  nh_private_.param<double>("alpha_velocity", alpha_velocity_, 0.3);
  nh_private_.param<double>("alpha_yaw", alpha_yaw_, 0.5);
  nh_private_.param<double>("max_forward_index", max_forward_index_, 0.2);
  nh_private_.param<double>("critical_prob", critical_prob_coll_, 0.7);

}

void deepNavigation::desiredTrajectoryCallback(const mavros_msgs::Trajectory& traj){
      desired_trajectory_ = traj;
  }

void deepNavigation::fillUnusedTrajectoryPoint(
      mavros_msgs::PositionTarget &point) {
    point.position.x = NAN;
    point.position.y = NAN;
    point.position.z = NAN;
    point.velocity.x = NAN;
    point.velocity.y = NAN;
    point.velocity.z = NAN;
    point.acceleration_or_force.x = NAN;
    point.acceleration_or_force.y = NAN;
    point.acceleration_or_force.z = NAN;
    point.yaw = NAN;
    point.yaw_rate = NAN;
  }


} // namespace deep_navigation

int main(int argc, char** argv)
{
  ros::init(argc, argv, "deep_navigation");
  deep_navigation::deepNavigation dn;

  dn.run();

  return 0;
}

