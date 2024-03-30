#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <string>
#include <map>
#include <math.h>

// Mainly Copied from https://www.utsbox.com/?page_id=523

// The biquadratic (BiQuad) filter is a digital filter that calculates
// the output signal using the previous two input signals and
// the previous two output signals for the input signal.
class BiQuadFilter
{
public:
  void init(const std::string& filter_type, std::map<std::string, float> filter_params)
  {
    // Define parameters for each filter
    float omega, alpha;
    if (filter_type == "band_pass")
      {
        omega = 2.0 * M_PI * filter_params["cutoff_frequency"] / filter_params["sampling_frequency"];
        alpha = std::sin(omega) * std::sinh(std::log(2.0) / 2.0 * filter_params["cutoff_bandwidth"] * omega / std::sin(omega));
        a0_ = 1.0 + alpha;
        a1_ = -2.0 * std::cos(omega);
        a2_ = 1.0 - alpha;
        b0_ = alpha;
        b1_ = 0.0;
        b2_ = -1 * alpha;
      }
    else if (filter_type == "low_pass")
      {
        omega = 2.0 * M_PI * filter_params["cutoff_frequency"] / filter_params["sampling_frequency"];
        alpha = std::sin(omega) / (2.0 * filter_params["q"]);
        a0_ = 1.0 + alpha;
        a1_ = -2.0 * std::cos(omega);
        a2_ = 1.0 - alpha;
        b0_ = (1.0 - std::cos(omega)) / 2.0;
        b1_ = 1.0 - std::cos(omega);
        b2_ = (1.0 - std::cos(omega)) / 2.0;
      }
    else if (filter_type == "notch")
      {
        omega = 2.0 * M_PI * filter_params["cutoff_frequency"] / filter_params["sampling_frequency"];
        alpha = std::sin(omega) * std::sinh(std::log(2.0) / 2.0 * filter_params["cutoff_bandwidth"] * omega / std::sin(omega));
        a0_ = 1.0 + alpha;
        a1_ = -2.0 * std::cos(omega);
        a2_ = 1.0 - alpha;
        b0_ = 1.0;
        b1_ = -2.0 * std::cos(omega);
        b2_ = 1.0;
      }
  }

  float apply_filter(float current_input, std::string key)
  {
    // Initial process. Store current_input for the first data.
    auto set_init_value = [&current_input, &key](std::map<std::string, float>* stored_data) {
                            if (stored_data->find(key) == stored_data->end())
                              {
                                stored_data->insert(std::make_pair(key, current_input));
                                return -1;
                              }
                            return 0;
                          };
    if (-1 == set_init_value(&last1_inputs_) || -1 == set_init_value(&last2_inputs_) ||
        -1 == set_init_value(&last1_outputs_) || -1 == set_init_value(&last2_outputs_))
        return current_input;

    // Calculate filterd IMU singal
    float current_output = (b0_ / a0_) * current_input
      + (b1_ / a0_) * last1_inputs_[key]
      + (b2_ / a0_) * last2_inputs_[key]
      - (a1_ / a0_) * last1_outputs_[key]
      - (a2_ / a0_) * last2_outputs_[key];
    // Update inputs and outputs
    last2_inputs_[key] = last1_inputs_[key];
    last1_inputs_[key] = current_input;
    last2_outputs_[key] = last1_outputs_[key];
    last1_outputs_[key] = current_output;

    return current_output;
  }

  std::vector<float> get_filter_coef()
  {
    std::vector<float> coef = {a0_, a1_, a2_, b0_, b1_, b2_};
    return coef;
  }
  
private:
  float a0_, a1_, a2_, b0_, b1_, b2_;
  // Store previous two inputs and outputs
  std::map<std::string, float> last1_inputs_;
  std::map<std::string, float> last2_inputs_;
  std::map<std::string, float> last1_outputs_;
  std::map<std::string, float> last2_outputs_;
};


class IMUFilterNode
{
public:
  IMUFilterNode()
  {
    // Filter type and parameters
    std::string filter_type;
    double sampling_frequency, cutoff_frequency, cutoff_bandwidth, q;
    // Retrieve parameters or set default values using the private node handle
    nh_private_ = ros::NodeHandle("~");
    nh_private_.param<std::string>("filter_type", filter_type, "notch");
    nh_private_.param("sampling_frequency", sampling_frequency, 200.0);
    nh_private_.param("cutoff_frequency", cutoff_frequency, 82.0);
    nh_private_.param("cutoff_bandwidth", cutoff_bandwidth, 1.0);
    nh_private_.param("q", q, 1.0);
    // Define BiQuadFilter
    std::map<std::string, float> filter_params =
      {
       {"sampling_frequency", sampling_frequency},
       {"cutoff_frequency", cutoff_frequency},
       {"cutoff_bandwidth", cutoff_bandwidth},
       {"q", q},
      };
    // Define Filter
    bqf_ = BiQuadFilter();
    bqf_.init(filter_type, filter_params);
    std::vector<float> coef = bqf_.get_filter_coef();
    ROS_INFO_STREAM(filter_type << " filter coef:");
    ROS_INFO_STREAM("a0: " << coef[0]);
    ROS_INFO_STREAM("a1: " << coef[1]);
    ROS_INFO_STREAM("a2: " << coef[2]);
    ROS_INFO_STREAM("b0: " << coef[3]);
    ROS_INFO_STREAM("b1: " << coef[4]);
    ROS_INFO_STREAM("b2: " << coef[5]);

    // Start publisher and subscriber
    int queue_size = 1;
    imu_pub_ = nh_private_.advertise<sensor_msgs::Imu>("imu_filtered", queue_size);
    imu_sub_ = nh_private_.subscribe("imu", 1000, &IMUFilterNode::imuCallback, this);
  }
  
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    sensor_msgs::Imu filtered_msg;
    // Copy original message
    filtered_msg.header = msg->header;
    filtered_msg.orientation_covariance = msg->orientation_covariance;
    filtered_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
    filtered_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

    // Apply filter to angular_velocity and linear_acceleration
    filtered_msg.orientation.x = bqf_.apply_filter(msg->orientation.x, "orientation_x");
    filtered_msg.orientation.y = bqf_.apply_filter(msg->orientation.y, "orientation_y");
    filtered_msg.orientation.z = bqf_.apply_filter(msg->orientation.z, "orientation_z");
    filtered_msg.orientation.w = bqf_.apply_filter(msg->orientation.w, "orientation_w");

    filtered_msg.linear_acceleration.x = bqf_.apply_filter(msg->linear_acceleration.x, "accel_x");
    filtered_msg.linear_acceleration.y = bqf_.apply_filter(msg->linear_acceleration.y, "accel_y");
    filtered_msg.linear_acceleration.z = bqf_.apply_filter(msg->linear_acceleration.z, "accel_z");

    filtered_msg.angular_velocity.x = bqf_.apply_filter(msg->angular_velocity.x, "gyro_x");
    filtered_msg.angular_velocity.y = bqf_.apply_filter(msg->angular_velocity.y, "gyro_y");
    filtered_msg.angular_velocity.z = bqf_.apply_filter(msg->angular_velocity.z, "gyro_z");  

    // Publish filtered IMU message
    imu_pub_.publish(filtered_msg);
  }

private:
  ros::Publisher imu_pub_;
  ros::Subscriber imu_sub_;
  BiQuadFilter bqf_;
  ros::NodeHandle nh_private_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_filter_node");

  IMUFilterNode ifn = IMUFilterNode();
  ros::spin();

  return 0;
}
