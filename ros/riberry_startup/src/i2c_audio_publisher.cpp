#include <audio_common_msgs/AudioData.h>
#include <audio_common_msgs/AudioInfo.h>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

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


void convertAndPack(const std::vector<uint8_t>& input, std::vector<uint8_t>& result) {
  result.clear();
  for(auto byte : input) {
    int8_t signedByte = static_cast<int8_t>(byte);
    int16_t shifted = signedByte * 256;
    uint8_t highByte = static_cast<uint8_t>((shifted >> 8) & 0xFF);
    uint8_t lowByte = static_cast<uint8_t>(shifted & 0xFF);

    result.push_back(lowByte);
    result.push_back(highByte);
  }
}



std::vector<uint8_t> publishAudioSegments(ros::Publisher& pub, const std::vector<uint8_t>& convertedPayload, int sampleRate) {
  const int samplesPer10ms = sampleRate / 100; // 10ms worth of samples
  size_t totalSamples = convertedPayload.size() / 2; // Each sample is 2 bytes (int16_t)
  std::vector<uint8_t> carryOver;

  for (size_t i = 0; i < totalSamples; i += samplesPer10ms) {
    std::vector<uint8_t> segment;

    // Include any carryover from the previous segment
    segment.insert(segment.end(), carryOver.begin(), carryOver.end());
    carryOver.clear();

    // Determine the end index for this segment
    size_t end = std::min(i + samplesPer10ms, totalSamples) * 2; // Multiply by 2 for byte indexing
    for (size_t j = i * 2; j < end; ++j) {
      segment.push_back(convertedPayload[j]);
    }

    // If the segment is less than the required size, carry over the remainder to the next segment
    if (segment.size() < samplesPer10ms * 2) {
      carryOver.insert(carryOver.end(), segment.begin(), segment.end());
      continue;
    }

    // Publish the segment
    audio_common_msgs::AudioData audio_msg;
    audio_msg.data = segment;
    pub.publish(audio_msg);

    // Sleep for 10ms before publishing the next segment
    ros::Duration(0.01).sleep();
  }

  // Return any remaining carryover data
  if (!carryOver.empty()) {
    return carryOver;
  } else {
    return std::vector<uint8_t>();
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "i2c_audio_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Publisher pub = nh.advertise<audio_common_msgs::AudioData>("/audio", 1);
  ros::Publisher pub_audio_info = nh.advertise<audio_common_msgs::AudioInfo>("/audio_info", 1, true);
  audio_common_msgs::AudioInfo audio_info_msg;
  audio_info_msg.sample_rate = 8000;
  audio_info_msg.channels = 1;
  audio_info_msg.sample_format = "S16LE";
  audio_info_msg.bitrate = 128;
  audio_info_msg.coding_format = "wave";
  ros::Duration(3.0).sleep();

  pub_audio_info.publish(audio_info_msg);

  std::string i2c_device;
  std::string i2c_lock_file;
  int buffer_size;
  int i2c_addr;

  private_nh.param<std::string>("i2c_device", i2c_device, "/dev/i2c-3");
  private_nh.param<std::string>("i2c_lock_file", i2c_lock_file, "/tmp/i2c-3.lock");
  private_nh.param("buffer_size", buffer_size, 4096);
  private_nh.param("i2c_addr", i2c_addr, 0x41);

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
  std::vector<uint8_t> convertedPayload;
  std::vector<uint8_t> carryOver;

  uint8_t write_buf[5] = {2, 1, 5, 0, 4};

  WireUnpacker unpacker(buffer_size);
  FileLock fileLock(i2c_lock_file.c_str());

  ssize_t bytesRead;
  while (ros::ok()) {

    ros::Duration(0.01).sleep();

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
      ROS_ERROR("Failed to read from the audio sensor");
      continue;
    }

    rxBuffer.resize(bytesRead);
    unpacker.reset();
    unpacker.write_data_list(rxBuffer);
    std::vector<uint8_t> payload = unpacker.getPayload();
    convertAndPack(payload, convertedPayload);
    if (!convertedPayload.empty()) {
      std::vector<uint8_t> combinedPayload = carryOver;
      combinedPayload.insert(combinedPayload.end(), convertedPayload.begin(), convertedPayload.end());
      carryOver = publishAudioSegments(pub, combinedPayload, audio_info_msg.sample_rate);
    }
    ros::spinOnce();
  }
  close(file_descriptor);
  return 0;
}
