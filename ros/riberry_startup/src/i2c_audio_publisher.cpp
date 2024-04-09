#include <audio_common_msgs/AudioData.h>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>


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

int main(int argc, char **argv) {
  ros::init(argc, argv, "i2c_audio_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Publisher pub = nh.advertise<audio_common_msgs::AudioData>("/audio", 1);
  ros::Duration(3.0).sleep();

  std::string i2c_device;
  int buffer_size;
  int i2c_addr;

  private_nh.param<std::string>("i2c_device", i2c_device, "/dev/i2c-3");
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

  uint8_t write_buf[5] = {2, 1, 5, 0, 4};

  WireUnpacker unpacker(buffer_size);

  while (ros::ok()) {
    write(file_descriptor, write_buf, sizeof(write_buf));
    ros::Duration(0.01).sleep();

    ssize_t bytesRead = read(file_descriptor, rxBuffer.data(), rxBuffer.size());
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
      audio_common_msgs::AudioData audio_msg;
      audio_msg.data = convertedPayload;
      pub.publish(audio_msg);
    }
    ros::spinOnce();
  }
  close(file_descriptor);
  return 0;
}
