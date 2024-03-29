#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cstring>
#include <cstdio>

// MPU6886 Register Addresses and Configuration Constants
#define ACCEL_XOUT_H 0x3b
#define PWR_MGMT_1 0x6b
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1c

// Accelerometer and Gyroscope Full-Scale Selection Settings
#define ACCEL_FS_SEL_2G 0b00000000
#define GYRO_FS_SEL_250DPS 0b00000000

int fd; // File descriptor for the I2C device
const double convert_to_mss = 9.80665 / 16384.0;
const double convert_to_radian_per_sec = (250.0 / 32768.0) * (M_PI / 180.0);

// Function to initialize the MPU6886 sensor
void initialize_sensor(int fd, uint8_t accel_fs_sel, uint8_t gyro_fs_sel) {
    uint8_t buf[2];
    // Reset device
    buf[0] = PWR_MGMT_1;
    buf[1] = 0b10000000; // Reset
    write(fd, buf, 2);
    usleep(100000); // 100ms
    buf[1] = 0b00000001; // Set clock source
    write(fd, buf, 2);

    // Set accelerometer and gyroscope range
    buf[0] = ACCEL_CONFIG;
    buf[1] = accel_fs_sel;
    write(fd, buf, 2);
    buf[0] = GYRO_CONFIG;
    buf[1] = gyro_fs_sel;
    write(fd, buf, 2);
    usleep(50000); // 50ms
}

// Function to read sensor data
bool read_sensor_data(int fd, int16_t& accel_x, int16_t& accel_y, int16_t& accel_z, int16_t& gyro_x, int16_t& gyro_y, int16_t& gyro_z) {
    uint8_t reg_addr = ACCEL_XOUT_H;
    uint8_t buf[14];
    if (write(fd, &reg_addr, 1) != 1) return false;
    if (read(fd, buf, 14) != 14) return false;

    // Extract accelerometer data
    accel_x = (buf[0] << 8) | buf[1];
    accel_y = (buf[2] << 8) | buf[3];
    accel_z = (buf[4] << 8) | buf[5];

    // Extract gyroscope data
    gyro_x = (buf[8] << 8) | buf[9];
    gyro_y = (buf[10] << 8) | buf[11];
    gyro_z = (buf[12] << 8) | buf[13];

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpu6886_imu_publisher_node");
    ros::NodeHandle nh_private("~");

    std::string i2c_device, frame_id;
    int mpu6886_addr, queue_size;
    double loop_rate_hz;

    // Retrieve parameters or set default values using the private node handle
    nh_private.param<std::string>("i2c_device", i2c_device, "/dev/i2c-1");
    nh_private.param<std::string>("frame_id", frame_id, "mpu6886_frame");
    nh_private.param("mpu6886_addr", mpu6886_addr, 0x68);
    nh_private.param("queue_size", queue_size, 1);
    nh_private.param("loop_rate", loop_rate_hz, 500.0);

    // Get the node's namespace
    std::string full_namespace = ros::this_node::getNamespace();
    // Get the node's name
    std::string node_name = ros::this_node::getName();

    // Remove the first slash from the namespace if it exists
    if (!full_namespace.empty() && full_namespace.front() == '/') {
        full_namespace.erase(0, 1);
    }

    // Construct the new frame_id without the node name
    std::string clean_frame_id = full_namespace + "/" + frame_id;
    // Search and replace the node name with an empty string if it exists in the frame_id
    auto pos = clean_frame_id.find(node_name);
    if (pos != std::string::npos) {
        clean_frame_id.erase(pos, node_name.length() + 1); // +1 to also remove the following slash
    }

    // Update the frame_id with the cleaned version
    frame_id = clean_frame_id;
    if (!frame_id.empty() && frame_id.front() == '/') {
      frame_id.erase(0, 1);
    }

    // Initialize I2C device
    if ((fd = open(i2c_device.c_str(), O_RDWR)) < 0) {
        ROS_ERROR("Failed to open the I2C device");
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, mpu6886_addr) < 0) {
        ROS_ERROR("Failed to acquire bus access and/or talk to slave");
        return -1;
    }
    initialize_sensor(fd, ACCEL_FS_SEL_2G, GYRO_FS_SEL_250DPS);

    // Create a ROS publisher for the IMU data
    ros::Publisher imu_pub = nh_private.advertise<sensor_msgs::Imu>("imu", queue_size);
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = frame_id;

    ros::Rate loop_rate(loop_rate_hz); // Publishing rate in Hz
    while (ros::ok()) {
        int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
        if (read_sensor_data(fd, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)) {
            imu_msg.header.stamp = ros::Time::now();

            // Populate the IMU message
            imu_msg.linear_acceleration.x = accel_x * convert_to_mss;
            imu_msg.linear_acceleration.y = accel_y * convert_to_mss;
            imu_msg.linear_acceleration.z = accel_z * convert_to_mss;
            imu_msg.angular_velocity.x = gyro_x * convert_to_radian_per_sec;
            imu_msg.angular_velocity.y = gyro_y * convert_to_radian_per_sec;
            imu_msg.angular_velocity.z = gyro_z * convert_to_radian_per_sec;

            // Publish the IMU message
            imu_pub.publish(imu_msg);
        } else {
            ROS_ERROR("Failed to read from the sensor");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);
    return 0;
}
