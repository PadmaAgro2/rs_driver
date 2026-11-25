/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/


//
// Padma: online driver that publishes each point cloud over ZMQ PUB.
//
// Usage:
//   demo_online <msop> <difop> <zmq_port> [lidar_type=RSE1] [imu_port=0] [imu_zmq_port=0] [stamp_frames=0]
//
//   lidar_type    rs_driver LidarType name, e.g. RSE1, RSAIRYLITE_ETH.
//   imu_port      UDP port the lidar sends IMU packets to. 0 = no IMU socket.
//   imu_zmq_port  ZMQ PUB port for IMU samples. 0 = IMU not published. Requires imu_port > 0.
//   stamp_frames  1 = prefix each point-cloud message with a 16-byte header (see below) and stamp the
//                 frame with its first point (decoder_param.ts_first_point = true).
//
// Wire formats (all little-endian, x86 native byte order):
//   point cloud (zmq_port), stamp_frames=0: N * 16 bytes, per point float32 x, y, z, intensity.
//                                            Identical to the feature/two_lidars output.
//   point cloud (zmq_port), stamp_frames=1: 16-byte prefix `<QQ`: uint64 frame_ts_ns, uint64 seq,
//                                            followed by the same N * 16 bytes of float32 x, y, z, intensity.
//                                            frame_ts_ns = llround(msg->timestamp * 1e9), host clock
//                                            (use_lidar_clock = false), time of the first point of the sweep.
//   IMU (imu_zmq_port): one 32-byte message per sample, `<d6f`: double timestamp (s, host clock, same clock as
//                       frame_ts_ns), float accel_x, accel_y, accel_z (m/s^2), gyro_x, gyro_y, gyro_z (rad/s).
//

#include <rs_driver/api/lidar_driver.hpp>
#include <zmq.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>

//#define ORDERLY_EXIT

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;

// The wire formats are documented as little-endian and are written with memcpy of native values.
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "ZMQ wire format assumes a little-endian host");
#endif
static_assert(sizeof(float) == 4, "float must be 4 bytes");
static_assert(sizeof(double) == 8, "double must be 8 bytes");

static const size_t kFloatsPerPoint = 4;                                    // x, y, z, intensity
static const size_t kBytesPerPoint = kFloatsPerPoint * sizeof(float);       // 16
static const size_t kFramePrefixBytes = 2 * sizeof(uint64_t);               // frame_ts_ns, seq
static const size_t kImuMsgBytes = sizeof(double) + 6 * sizeof(float);      // <d6f
static_assert(kFramePrefixBytes == 16, "frame prefix must be 16 bytes");
static_assert(kImuMsgBytes == 32, "IMU message must be 32 bytes");

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

SyncQueue<std::shared_ptr<ImuData>> free_imu_data_queue;
SyncQueue<std::shared_ptr<ImuData>> stuffed_imu_data_queue;

zmq::socket_t* g_zmq_pub = nullptr;
zmq::socket_t* g_zmq_imu_pub = nullptr;
bool g_stamp_frames = false;

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed point cloud message to the caller.
// @param msg  The stuffed point cloud message.
//
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud()
  //       below)
  stuffed_cloud_queue.push(msg);
}

//
// @brief IMU data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets a free/unused IMU data message from the caller.
//
std::shared_ptr<ImuData> driverGetIMUDataFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/imu-data-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<ImuData> msg = free_imu_data_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<ImuData>();
}

//
// @brief IMU data callback function. The caller should register it to the lidar driver.
//        Via this function, the driver returns a stuffed IMU data message to the caller.
// @param msg  The stuffed IMU data message.
//
void driverReturnImuDataToCallerCallback(const std::shared_ptr<ImuData>& msg)
{
  // Note: This callback function runs in the packet-parsing/imu-data-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see
  //       processImuData() below)
  stuffed_imu_data_queue.push(msg);
}

bool to_exit_process = false;

//
// @brief Publish each IMU sample as one 32-byte `<d6f` ZMQ message (see the file header).
//
void processImuData(void)
{
  uint32_t imu_cnt = 0;
  while (!to_exit_process)
  {
    std::shared_ptr<ImuData> msg = stuffed_imu_data_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    const double ts = msg->timestamp;  // seconds
    const float vals[6] = {
      msg->linear_acceleration_x, msg->linear_acceleration_y, msg->linear_acceleration_z,
      msg->angular_velocity_x,    msg->angular_velocity_y,    msg->angular_velocity_z,
    };

    uint8_t buf[kImuMsgBytes];
    memcpy(buf, &ts, sizeof(ts));
    memcpy(buf + sizeof(ts), vals, sizeof(vals));
    static_assert(sizeof(double) + sizeof(vals) == kImuMsgBytes, "IMU message layout mismatch");

    zmq::message_t zmq_msg(buf, kImuMsgBytes);
    g_zmq_imu_pub->send(zmq_msg, zmq::send_flags::none);

    imu_cnt++;
    if (imu_cnt % 200 == 0)
    {
      RS_MSG << "imu msg: " << imu_cnt << " ts: " << std::to_string(ts) << " accel: " << vals[0] << ", " << vals[1]
             << ", " << vals[2] << " gyro: " << vals[3] << ", " << vals[4] << ", " << vals[5] << RS_REND;
    }

    free_imu_data_queue.push(msg);
  }
}

//
// @brief exception callback function. The caller should register it to the lidar driver.
//        Via this function, the driver inform the caller that something happens.
// @param code The error code to represent the error/warning/information
//
void exceptionCallback(const Error& code)
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the
  // driver,
  //       so please DO NOT do time-consuming task here.
  RS_WARNING << code.toString() << RS_REND;
}

//
// @brief Publish each point cloud as float32 x, y, z, intensity, optionally prefixed with `<QQ` frame_ts_ns, seq.
//
void processCloud()
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();

    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the point cloud msg, even it is time-consuming.
    RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

    const size_t prefix_bytes = g_stamp_frames ? kFramePrefixBytes : 0;
    zmq::message_t zmq_msg(prefix_bytes + msg->points.size() * kBytesPerPoint);
    uint8_t* out = static_cast<uint8_t*>(zmq_msg.data());

    if (g_stamp_frames)
    {
      const uint64_t frame_ts_ns = static_cast<uint64_t>(std::llround(msg->timestamp * 1e9));
      const uint64_t seq = static_cast<uint64_t>(msg->seq);
      memcpy(out, &frame_ts_ns, sizeof(frame_ts_ns));
      memcpy(out + sizeof(frame_ts_ns), &seq, sizeof(seq));
      out += kFramePrefixBytes;
    }

    // Pack x, y, z, intensity as float32
    for (const auto& pt : msg->points)
    {
      const float p[kFloatsPerPoint] = {pt.x, pt.y, pt.z, static_cast<float>(pt.intensity)};
      memcpy(out, p, kBytesPerPoint);
      out += kBytesPerPoint;
    }

    g_zmq_pub->send(zmq_msg, zmq::send_flags::none);

    free_cloud_queue.push(msg);
  }
}

static void printUsage(const char* prog)
{
  RS_MSG << "Usage: " << prog
         << " <msop> <difop> <zmq_port> [lidar_type=RSE1] [imu_port=0] [imu_zmq_port=0] [stamp_frames=0]" << RS_REND;
  RS_MSG << "  lidar_type    e.g. RSE1, RSAIRYLITE_ETH" << RS_REND;
  RS_MSG << "  imu_port      lidar IMU UDP port, 0 = no IMU socket" << RS_REND;
  RS_MSG << "  imu_zmq_port  ZMQ PUB port for 32-byte <d6f IMU samples, 0 = off (requires imu_port > 0)" << RS_REND;
  RS_MSG << "  stamp_frames  1 = prefix point clouds with 16-byte <QQ frame_ts_ns, seq; 0 = raw xyzi" << RS_REND;
}

// Parse an unsigned integer argument in [min_val, max_val]; throws std::invalid_argument with a clear message.
static unsigned long parseUnsigned(const char* arg, const char* name, unsigned long min_val, unsigned long max_val)
{
  const std::string s(arg);
  size_t pos = 0;
  unsigned long v = 0;
  try
  {
    if (s.empty() || s[0] == '-' || s[0] == '+')
    {
      throw std::invalid_argument(s);
    }
    v = std::stoul(s, &pos, 10);
  }
  catch (const std::exception&)
  {
    throw std::invalid_argument(std::string(name) + " must be an unsigned integer, got '" + s + "'");
  }
  if (pos != s.size())
  {
    throw std::invalid_argument(std::string(name) + " must be an unsigned integer, got '" + s + "'");
  }
  if (v < min_val || v > max_val)
  {
    throw std::invalid_argument(std::string(name) + " must be in [" + std::to_string(min_val) + ", " +
                                std::to_string(max_val) + "], got " + s);
  }
  return v;
}

// Names accepted by strToLidarType() in driver_param.hpp. Checked here first because strToLidarType()
// calls exit(-1) on an unknown name.
static bool isKnownLidarType(const std::string& name)
{
  static const std::set<std::string> known = {
    "RS16", "RS32",  "RSBP", "RSHELIOS", "RSHELIOS_16P", "RS128", "RS80",   "RS48",           "RSP128",  "RSP80",
    "RSP48", "RSM1", "RSM2", "RSM3",     "RSE1",         "RSMX",  "RSEMX",  "RSAIRY", "RSAIRYLITE_ETH", "RSFAIRY",
    "RSM1_JUMBO",
  };
  return known.count(name) > 0;
}

int main(int argc, char* argv[])
{
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  if (argc > 8 || (argc >= 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")))
  {
    printUsage(argv[0]);
    return (argc > 8) ? 1 : 0;
  }

  uint16_t msop_port = 6699;
  uint16_t difop_port = 7788;
  uint16_t zmq_port = 5556;
  std::string lidar_type_str = "RSE1";
  uint16_t imu_port = 0;
  uint16_t imu_zmq_port = 0;
  bool stamp_frames = false;

  try
  {
    if (argc >= 2) msop_port = static_cast<uint16_t>(parseUnsigned(argv[1], "msop", 1, 65535));
    if (argc >= 3) difop_port = static_cast<uint16_t>(parseUnsigned(argv[2], "difop", 1, 65535));
    if (argc >= 4) zmq_port = static_cast<uint16_t>(parseUnsigned(argv[3], "zmq_port", 1, 65535));
    if (argc >= 5) lidar_type_str = argv[4];
    if (argc >= 6) imu_port = static_cast<uint16_t>(parseUnsigned(argv[5], "imu_port", 0, 65535));
    if (argc >= 7) imu_zmq_port = static_cast<uint16_t>(parseUnsigned(argv[6], "imu_zmq_port", 0, 65535));
    if (argc >= 8) stamp_frames = parseUnsigned(argv[7], "stamp_frames", 0, 1) != 0;

    if (!isKnownLidarType(lidar_type_str))
    {
      throw std::invalid_argument("unknown lidar_type '" + lidar_type_str +
                                  "' (e.g. RSE1, RSAIRYLITE_ETH; see strToLidarType in driver_param.hpp)");
    }
    if (imu_zmq_port > 0 && imu_port == 0)
    {
      throw std::invalid_argument("imu_zmq_port " + std::to_string(imu_zmq_port) + " requires imu_port > 0");
    }
    if (imu_zmq_port > 0 && imu_zmq_port == zmq_port)
    {
      throw std::invalid_argument("imu_zmq_port must differ from zmq_port");
    }
  }
  catch (const std::exception& e)
  {
    RS_ERROR << "demo_online: invalid arguments: " << e.what() << RS_REND;
    printUsage(argv[0]);
    return 1;
  }

  const LidarType lidar_type = strToLidarType(lidar_type_str);
  const bool imu_publish = (imu_zmq_port > 0);
  g_stamp_frames = stamp_frames;

  RS_INFO << "demo_online config: msop=" << msop_port << " difop=" << difop_port << " zmq_port=" << zmq_port
          << " lidar_type=" << lidarTypeToStr(lidar_type) << " imu_port=" << imu_port
          << " imu_zmq_port=" << imu_zmq_port << (imu_publish ? "" : " (IMU publish off)")
          << " stamp_frames=" << (stamp_frames ? 1 : 0) << RS_REND;

  zmq::context_t zmq_ctx(1);
  zmq::socket_t zmq_pub(zmq_ctx, zmq::socket_type::pub);
  zmq::socket_t zmq_imu_pub(zmq_ctx, zmq::socket_type::pub);
  try
  {
    zmq_pub.bind("tcp://*:" + std::to_string(zmq_port));
    if (imu_publish)
    {
      zmq_imu_pub.bind("tcp://*:" + std::to_string(imu_zmq_port));
    }
  }
  catch (const zmq::error_t& e)
  {
    RS_ERROR << "demo_online: ZMQ bind failed: " << e.what() << RS_REND;
    return 1;
  }
  g_zmq_pub = &zmq_pub;
  g_zmq_imu_pub = &zmq_imu_pub;

  RSDriverParam param;                  ///< Create a parameter object
  param.input_type = InputType::ONLINE_LIDAR;
  param.input_param.msop_port = msop_port;
  param.input_param.difop_port = difop_port;
  param.input_param.imu_port = imu_port;  ///< 0 = no IMU socket
  param.lidar_type = lidar_type;
  if (lidar_type == LidarType::RSE1)
  {
    // Keep the v1.5.17 E1R range: its decoder had DISTANCE_MIN 0.2 m / DISTANCE_MAX 200 m;
    // v1.5.20 lowered DISTANCE_MIN to 0.0 m.
    param.decoder_param.min_distance = 0.2f;
    param.decoder_param.max_distance = 200.0f;
  }
  if (stamp_frames)
  {
    param.decoder_param.ts_first_point = true;  ///< frame timestamp = first point of the sweep
  }
  param.print();

  LidarDriver<PointCloudMsg> driver;               ///< Declare the driver object
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); ///< Register the point cloud callback functions
  driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function
  if (imu_publish)
  {
    driver.regImuDataCallback(driverGetIMUDataFromCallerCallback, driverReturnImuDataToCallerCallback);
  }

  if (!driver.init(param))                         ///< Call the init function
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);
  std::thread imuData_handle_thread;
  if (imu_publish)
  {
    imuData_handle_thread = std::thread(processImuData);
  }
  driver.start();  ///< The driver thread will start
  RS_DEBUG << "RoboSense Lidar-Driver Linux online demo start......" << RS_REND;

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();
  if (imuData_handle_thread.joinable())
  {
    imuData_handle_thread.join();
  }
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
