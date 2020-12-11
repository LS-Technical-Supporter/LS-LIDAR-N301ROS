/* This file is part of lidar_n301 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIDAR_N301_DECODER_H
#define LIDAR_N301_DECODER_H

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "time.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lidar_n301_msgs/LidarN301Packet.h>
#include <lidar_n301_msgs/LidarN301Difop.h>
#include <lidar_n301_msgs/LidarN301Point.h>
#include <lidar_n301_msgs/LidarN301Scan.h>
#include <lidar_n301_msgs/LidarN301Sweep.h>
#include <std_msgs/Byte.h>

namespace lidar_n301_decoder {

// Raw lidar packet constants and structures.
static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE =
        (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double DISTANCE_MAX        = 130.0;        /**< meters */
static double DISTANCE_RESOLUTION = 0.002; /**< meters */
static const double DISTANCE_MAX_UNITS  =
        (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int     FIRINGS_PER_BLOCK = 2;

static const int     SCANS_PER_FIRING  = 16;
//static const double  BLOCK_TDURATION   = 110.592; // [µs]
static double  DSR_TOFFSET       = 2.304;   // [µs]
static double  FIRING_TOFFSET    = 55.296;  // [µs]


static const int PACKET_SIZE        = 1206;
static const int BLOCKS_PER_PACKET  = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET =
        (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static const int FIRINGS_PER_PACKET =
        FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_altitude[16] = {
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494
};

static const double cos_scan_altitude[16] = {
    std::cos(scan_altitude[ 0]), std::cos(scan_altitude[ 1]),
    std::cos(scan_altitude[ 2]), std::cos(scan_altitude[ 3]),
    std::cos(scan_altitude[ 4]), std::cos(scan_altitude[ 5]),
    std::cos(scan_altitude[ 6]), std::cos(scan_altitude[ 7]),
    std::cos(scan_altitude[ 8]), std::cos(scan_altitude[ 9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
};

static const double sin_scan_altitude[16] = {
    std::sin(scan_altitude[ 0]), std::sin(scan_altitude[ 1]),
    std::sin(scan_altitude[ 2]), std::sin(scan_altitude[ 3]),
    std::sin(scan_altitude[ 4]), std::sin(scan_altitude[ 5]),
    std::sin(scan_altitude[ 6]), std::sin(scan_altitude[ 7]),
    std::sin(scan_altitude[ 8]), std::sin(scan_altitude[ 9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
};

typedef struct{
    double distance;
    double intensity;
}point_struct;

struct PointXYZIT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

class LidarN301Decoder {
public:

    LidarN301Decoder(ros::NodeHandle& n, ros::NodeHandle& pn);
    LidarN301Decoder(const LidarN301Decoder&) = delete;
    LidarN301Decoder operator=(const LidarN301Decoder&) = delete;
    ~LidarN301Decoder() {return;}

    bool initialize();

    typedef boost::shared_ptr<LidarN301Decoder> LidarN301DecoderPtr;
    typedef boost::shared_ptr<const LidarN301Decoder> LidarN301DecoderConstPtr;

private:

    union TwoBytes {
        uint16_t distance;
        uint8_t  bytes[2];
    };

    struct RawBlock {
        uint16_t header;        ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
        uint8_t  data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        // uint32_t time_stamp;
        uint8_t time_stamp_yt[4];
        uint8_t factory[2];
        //uint16_t revolution;
        //uint8_t status[PACKET_STATUS_SIZE];
    };


    struct Firing {
        // Azimuth associated with the first shot within this firing.
        double firing_azimuth;
        double azimuth[SCANS_PER_FIRING];
        double distance[SCANS_PER_FIRING];
        double intensity[SCANS_PER_FIRING];
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();


    // Callback function for a single lidar packet.
    bool checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void packetCallback(const lidar_n301_msgs::LidarN301PacketConstPtr& msg);
	void processDifop(const lidar_n301_msgs::LidarN301Packet::ConstPtr& difop_msg);
	void TypeReceive(const std_msgs::Byte::ConstPtr& msg);
	
    // Publish data
    void publishPointCloud();

    // Publish scan Data
    void publishScan();

    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        // According to the user manual,
        // azimuth = raw_azimuth / 100.0;
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
    }

    // calc the means_point
    point_struct getMeans(std::vector<point_struct> clusters);
    
    //get gps_base time
    uint64_t get_gps_stamp(tm t);
    tm pTime;            
    
    uint64_t packet_timestamp;

    // time relavated variables
    uint64_t sweep_end_time_gps;
    uint64_t sweep_end_time_hardware;
	uint64_t last_timestamp;
	uint64_t correct_time;
	uint64_t error_time;
	
    // configuration degree base
    int point_num;
	size_t scan_size;
    double angle_base;

    // Configuration parameters
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double frequency;
    bool publish_point_cloud;
    bool use_gps_ts;
    bool filter_scan_point;
	bool gps_correct;
	uint8_t agreement_type;

    double cos_azimuth_table[6300];
    double sin_azimuth_table[6300];

    bool is_first_sweep;
	bool first_gps;
	bool correct_gps;
    double last_azimuth;
	double last_azimuth1;
    double sweep_start_time;
    double packet_start_time;
    Firing firings[FIRINGS_PER_PACKET];

    // ROS related parameters
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string fixed_frame_id;
    std::string child_frame_id;

    lidar_n301_msgs::LidarN301SweepPtr sweep_data;
	lidar_n301_msgs::LidarN301SweepPtr sweep_data1;
	lidar_n301_msgs::LidarN301DifopPtr Difop_data;
    sensor_msgs::PointCloud2 point_cloud_data;

    ros::Subscriber packet_sub;
	ros::Subscriber difop_sub_;
	ros::Subscriber type_sub;
    ros::Publisher sweep_pub;
    ros::Publisher point_cloud_pub;
    ros::Publisher scan_pub;
	ros::Publisher device_pub;
};

typedef LidarN301Decoder::LidarN301DecoderPtr LidarN301DecoderPtr;
typedef LidarN301Decoder::LidarN301DecoderConstPtr LidarN301DecoderConstPtr;
typedef PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

} // end namespace lidar_n301_decoder

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_n301_decoder::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                          uint8_t, intensity,
                                          intensity)(double, timestamp, timestamp))

#endif
