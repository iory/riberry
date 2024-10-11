#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>

// The code has been adapted to ensure consistency with the FileLock behavior in a Python program.
// https://github.com/tox-dev/filelock/blob/main/src/filelock/_unix.py
class FileLock {
public:
    FileLock(const std::string& filename, int timeoutSecs = 10) : lockFile(filename), timeoutSecs(timeoutSecs), lockFileFd(-1), mode(0666) {}

    void acquire() {
        ensureDirectoryExists();
        int openFlags = O_RDWR | O_TRUNC;
        if (!std::filesystem::exists(lockFile)) {
            openFlags |= O_CREAT;
        }
        lockFileFd = open(lockFile.c_str(), openFlags, mode);
        if (lockFileFd == -1) {
            throw std::runtime_error("Failed to open lock file");
        }
        try {
            fchmod(lockFileFd, mode);
        } catch (...) {
            // Ignore PermissionError: This locked is not owned by this UID
        }

        auto start = std::chrono::steady_clock::now();
        while (true) {
            if (flock(lockFileFd, LOCK_EX | LOCK_NB) != -1) {
                break;
            }
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(timeoutSecs)) {
                close(lockFileFd);
                lockFileFd = -1;
                throw std::runtime_error("Failed to acquire file lock: timeout");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void release() {
        if (lockFileFd != -1) {
            flock(lockFileFd, LOCK_UN);
            close(lockFileFd);
            lockFileFd = -1;
        }
    }

    ~FileLock() {
        if (lockFileFd != -1) {
            release();
        }
    }

private:
    std::string lockFile;
    int timeoutSecs;
    int lockFileFd;
    mode_t mode;

    void ensureDirectoryExists() {
        std::filesystem::path dir = std::filesystem::path(lockFile).parent_path();
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);
        }
    }
};



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
  ros::Publisher pub_mode = nh.advertise<std_msgs::String>("/i2c_mode", 1);
  ros::Duration(3.0).sleep();

  std::string i2c_device;
  std::string i2c_lock_file;
  int buffer_size;
  int i2c_addr;
  double loop_rate;

  private_nh.param<std::string>("i2c_device", i2c_device, "/dev/i2c-1");
  private_nh.param("buffer_size", buffer_size, 5+100);
  private_nh.param("i2c_addr", i2c_addr, 0x42);
  private_nh.param("loop_rate", loop_rate, 10.0);
  private_nh.param<std::string>("i2c_lock_file", i2c_lock_file, "/tmp/i2c-1.lock");

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
  FileLock fileLock(i2c_lock_file.c_str());
  ssize_t bytesRead;
  ros::Rate rate(loop_rate);

  while (ros::ok()) {
    ros::spinOnce();

    try {
      fileLock.acquire();
      write(file_descriptor, write_buf, sizeof(write_buf));
      ros::Duration(0.05).sleep();
      bytesRead = read(file_descriptor, rxBuffer.data(), rxBuffer.size());
      fileLock.release();
    } catch (const std::runtime_error& e) {
      ROS_ERROR_STREAM("An error occurred: " << e.what());
      break;
    }

    if (bytesRead <= 0) {
      ROS_ERROR("Could not get packet.");
      continue;
    }

    // expected bytes no 2 1 6 0 0 4
    //         one click 2 1 6 1 94 4
    rxBuffer.resize(bytesRead);
    unpacker.reset();
    unpacker.write_data_list(rxBuffer);
    std::vector<uint8_t> payload = unpacker.getPayload();
    if (payload.size() == 0) {
      // This error occurs frequently, but after several attempts,
      // the button value becomes readable, so throttle the output.
      ROS_ERROR_THROTTLE(60.0, "Failed to read button state");
      continue;
    }

    // Publish Int32 msg
    std_msgs::Int32 button_state_msg;
    button_state_msg.data = payload[0];
    pub.publish(button_state_msg);

    // Publish String msg
    if (payload.size() > 1) {
      // Retrieve the data from payload[1] onwards as a string
      std::string extracted_string(payload.begin() + 1, payload.end());
      std_msgs::String string_msg;
      string_msg.data = extracted_string;
      pub_mode.publish(string_msg);
    }

    rate.sleep();
  }
  close(file_descriptor);
  return 0;
}
