/*
 * This file is part of lidar_n301 driver.
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

#ifndef LIDAR_N301_DRIVER_H
#define LIDAR_N301_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lidar_n301_msgs/LidarN301Packet.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>

namespace lidar_n301_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
static uint16_t PACKET_SIZE = 1206;

class LidarN301Driver {
public:

    LidarN301Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LidarN301Driver();

    bool initialize();
    bool polling();
	void difopPoll();
	void serialPoll();

    typedef boost::shared_ptr<LidarN301Driver> LidarN301DriverPtr;
    typedef boost::shared_ptr<const LidarN301Driver> LidarN301DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lidar_n301_msgs::LidarN301PacketPtr& msg);
	int getDifopPacket(lidar_n301_msgs::LidarN301PacketPtr& msg);
	bool SendPacketToLidar(bool power_switch);
	
    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;
	int socket_id_difop;
	bool first_time;
	bool add_multicast;
	bool serial_switch;
	unsigned char difop_data[1206];
	std::string group_ip;
	std::string serial_device;
	
    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;
    ros::Publisher packet_pub;
	ros::Publisher type_pub;
	ros::Publisher difop_output_;
	
	ros::Publisher read_pub;
	ros::Publisher write_sub;
	
	boost::shared_ptr<boost::thread> difop_thread;
	boost::shared_ptr<boost::thread> serial_thread;
	
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

};

typedef LidarN301Driver::LidarN301DriverPtr LidarN301DriverPtr;
typedef LidarN301Driver::LidarN301DriverConstPtr LidarN301DriverConstPtr;

} // namespace lidar_driver

#endif // _LIDAR_N301_DRIVER_H_
