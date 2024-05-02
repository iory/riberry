#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

int lockFile(int fd) {
  struct flock fl;
  fl.l_type = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len = 0;

  if (fcntl(fd, F_SETLKW, &fl) == -1) {
    std::cerr << "Locking failed: " << strerror(errno) << std::endl;
    return -1;
  }
  return 0;
}

int unlockFile(int fd) {
  struct flock fl;
  fl.l_type = F_UNLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len = 0;

  if (fcntl(fd, F_SETLK, &fl) == -1) {
    std::cerr << "Unlocking failed: " << strerror(errno) << std::endl;
    return -1;
  }
  return 0;
}

class WireCrc {
public:
  WireCrc() : seed(0) {
    table = generate_crc_table();
  }

  uint8_t calc(const std::vector<uint8_t>& data, size_t length) {
    seed = 0;
    return update(data, length);
  }

  uint8_t seed;
  std::vector<uint8_t> table;

  std::vector<uint8_t> generate_crc_table() {
    std::vector<uint8_t> table(256);
    for (int byte = 0; byte < 256; ++byte) {
      uint8_t crc = byte;
      for (int j = 0; j < 8; ++j) {
        if (crc & 0x01) {
          crc = (crc >> 1) ^ 0x8C;
        } else {
          crc >>= 1;
        }
      }
      table[byte] = crc;
    }
    return table;
  }

  uint8_t update(const std::vector<uint8_t>& data, size_t length) {
    uint8_t crc = seed;
    for (size_t i = 0; i < length; ++i) {
      uint8_t byte = data[i];
      crc = table[crc ^ byte];
    }
    return crc;
  }
};


class WireUnpacker {
public:
  enum Error {
    NONE = 0,
    INVALID_CRC = 1,
    INVALID_LENGTH = 2
  };

  explicit WireUnpacker(size_t buffer_size) : buffer_size(buffer_size), buffer(buffer_size, 0), index(0), totalLength(0),
                                              payloadLength(0), isPacketOpen(false), expectedLength(0), lastError(NONE) {
    reset();
  }

  bool hasError() const {
    return lastError != NONE;
  }

  int write(uint8_t data);

  int write_data_list(const std::vector<uint8_t>& byte_data_list);

  int write_array(const std::vector<uint8_t>& data) {
    int count = 0;
    for (uint8_t byte : data) {
      count += write(byte);
    }
    return count;
  }

  size_t available() const {
    return isPacketOpen ? 0 : payloadLength - index;
  }

  int read();

  std::vector<uint8_t> getPayload() const {
    if (payloadLength > 0) {
      return std::vector<uint8_t>(buffer.begin(), buffer.begin() + payloadLength);
    }
    return std::vector<uint8_t>();
  }

  void reset() {
    index = 0;
    totalLength = 0;
    expectedLength = 0;
    payloadLength = 0;
    isPacketOpen = false;
    numBufferLength = 0;
    lastError = NONE;
    ignoreLength = 0;
  }

private:
  size_t buffer_size;
  std::vector<uint8_t> buffer;
  size_t index;
  size_t totalLength;
  size_t payloadLength;
  bool isPacketOpen;
  size_t expectedLength;
  uint8_t expectedCrc;
  Error lastError;
  size_t numBufferLength;
  size_t ignoreLength;
  static const uint8_t frameStart = 0x02;
  static const uint8_t frameEnd = 0x04;
};


int WireUnpacker::write(uint8_t data) {
  if (totalLength >= buffer_size || hasError()) {
    return 0;
  }

  if (!isPacketOpen) {
    if (totalLength == 0 && data == frameStart) {
      isPacketOpen = true;
      totalLength += 1;
      return 1;
    }
    return 0;
  }

  if (expectedLength == 0 || numBufferLength > 0) {
    if (numBufferLength == 0) {
      numBufferLength = data;
      ignoreLength = numBufferLength == 1 ? 5 : (numBufferLength == 2 ? 6 : 7);
      totalLength += 1;
      return 1;
    }

    expectedLength = (expectedLength << 8) + data;
    numBufferLength -= 1;
    if (numBufferLength > 0) {
      totalLength += 1;
      return 1;
    }

    if (expectedLength > buffer_size) {
      isPacketOpen = false;
      lastError = INVALID_LENGTH;
      return 0;
    }

    totalLength += 1;
    return 1;
  }

  if (totalLength < (expectedLength - 1)) {
    buffer[index] = data;
    index += 1;
    totalLength += 1;
    return 1;
  }

  isPacketOpen = false;
  totalLength += 1;

  if (data != frameEnd) {
    lastError = INVALID_LENGTH;
    return 0;
  }

  payloadLength = totalLength - ignoreLength;

  WireCrc crc8;
  uint8_t crc = crc8.update(buffer, payloadLength);

  if (crc != buffer[index - 1]) {
    lastError = INVALID_CRC;
    return 0;
  }

  index = 0;
  return 1;
}

int WireUnpacker::write_data_list(const std::vector<uint8_t>& byte_data_list) {
  int count = 0;
  for (auto& data : byte_data_list) {
    count += write(data);
  }
  return count;
}

int WireUnpacker::read() {
  if (isPacketOpen || index >= payloadLength) {
    return -1;
  }
  uint8_t value = buffer[index];
  index += 1;
  return value;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "i2c_button_state_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/i2c_button_state", 1);
  ros::Duration(3.0).sleep();

  std::string i2c_device;
  std::string i2c_lock_file;
  int buffer_size;
  int i2c_addr;

  private_nh.param<std::string>("i2c_device", i2c_device, "/dev/i2c-1");
  private_nh.param("buffer_size", buffer_size, 100);
  private_nh.param("i2c_addr", i2c_addr, 0x42);
  private_nh.param<std::string>("i2c_lock_file", i2c_lock_file, "/tmp/i2c-1.lock");

  int lock_file_descriptor = open(i2c_lock_file.c_str(), O_RDWR | O_CREAT, 0666);
  int file_descriptor;

  if ((file_descriptor = open(i2c_device.c_str(), O_RDWR)) < 0) {
    ROS_ERROR("Failed to open the I2C bus");
    return -1;
  }
  if (ioctl(file_descriptor, I2C_SLAVE, i2c_addr) < 0) {
    ROS_ERROR("Failed to acquire bus access and/or talk to slave");
    close(file_descriptor);
    return -1;
  }

  std::vector<uint8_t> rxBuffer(buffer_size);

  uint8_t write_buf[5] = {2, 1, 5, 0, 4};

  WireUnpacker unpacker(buffer_size);

  while (ros::ok()) {
    ros::spinOnce();
    if (lockFile(lock_file_descriptor) != 0) {
      ROS_ERROR("Failed to lock file descriptor");
      break;
    }

    write(file_descriptor, write_buf, sizeof(write_buf));
    ros::Duration(0.05).sleep();
    ssize_t bytesRead = read(file_descriptor, rxBuffer.data(), rxBuffer.size());

    if (unlockFile(lock_file_descriptor) != 0) {
      ROS_ERROR("Failed to unlock file descriptor");
      break;
    }

    if (bytesRead <= 0) {
      ROS_ERROR("Could not get packet.");
      continue;
    }

    rxBuffer.resize(bytesRead);
    unpacker.reset();
    unpacker.write_data_list(rxBuffer);
    std::vector<uint8_t> payload = unpacker.getPayload();
    if (payload.size() != 1) {
      ROS_ERROR("Failed to read button state");
      continue;
    }
    std_msgs::Int32 button_state_msg;
    button_state_msg.data = payload[0];
    pub.publish(button_state_msg);
  }
  close(file_descriptor);
  close(lock_file_descriptor);
  return 0;
}
