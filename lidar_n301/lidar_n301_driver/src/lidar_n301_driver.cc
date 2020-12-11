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
#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <lidar_n301_driver/lidar_n301_driver.h>

namespace lidar_n301_driver {

LidarN301Driver::LidarN301Driver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
	first_time(true),
	socket_id_difop(-1),
    socket_id(-1){
    return;
}

LidarN301Driver::~LidarN301Driver() {
    (void) close(socket_id);
	(void) close(socket_id_difop);
    return;
}

bool LidarN301Driver::loadParameters() {

	pnh.param("frame_id", frame_id, std::string("lidar"));
	pnh.param("device_ip", device_ip_string, std::string("192.168.1.222"));
	pnh.param<int>("device_port", UDP_PORT_NUMBER, 2368);
	pnh.param<bool>("add_multicast", add_multicast, false);
	pnh.param<std::string>("group_ip", group_ip, "224.1.1.2");
	
	inet_aton(device_ip_string.c_str(), &device_ip);
	ROS_INFO_STREAM("Opening UDP socket: address " << device_ip_string);
	ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
	return true;
}

bool LidarN301Driver::createRosIO() {

  // ROS diagnostics
  diagnostics.setHardwareID("Lidar_N301");

  // n301 publishs 20*16 thousands points per second.
  // Each packet contains 12 blocks. And each block
  // contains 30 points. Together provides the
  // packet rate.
  const double diag_freq = 16*20000.0 / (12*30);
  diag_max_freq = diag_freq;
  diag_min_freq = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
	  diag_topic.reset(new TopicDiagnostic(
						 "lidar_packets", diagnostics,
						 FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
						 TimeStampStatusParam()));

	// Output
	packet_pub = nh.advertise<lidar_n301_msgs::LidarN301Packet>(
			"lidar_packet", 100);
		
	//std::string output_difop_topic;
	//pnh.param("output_difop_topic", output_difop_topic, std::string("lidar_packet_difop"));
	difop_output_ = nh.advertise<lidar_n301_msgs::LidarN301Packet>("lidar_packet_difop", 100);
	
	//serial_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&LidarN301Driver::serialPoll, this)));
	
	//difop_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&LidarN301Driver::difopPoll, this)));
	type_pub = nh.advertise<std_msgs::Byte>("agreement_type", 100);
	  
	return true;
}

bool LidarN301Driver::openUDPPort() {
	socket_id_difop = socket(PF_INET, SOCK_DGRAM, 0);
    socket_id = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_id == -1) {
        perror("socket");
        return false;
    }
	if (socket_id_difop == -1) {
        perror("socket_difop");
        return false;
    }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(UDP_PORT_NUMBER);      // short, in network byte order
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(socket_id, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        perror("bind");                 // TODO: ROS_ERROR errno
        return false;
    }
	
	/*my_addr.sin_port = htons(1);
	if (bind(socket_id_difop, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        perror("bind");                 // TODO: ROS_ERROR errno
        return false;
    }*/
	
	if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
			group.imr_interface.s_addr =  htonl(INADDR_ANY);
			//group.imr_interface.s_addr = inet_addr("192.168.1.125");

            if (setsockopt(socket_id, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                perror("Adding multicast group error ");
                close(socket_id);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }
		
    if (fcntl(socket_id, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        perror("non-block");
        return false;
    }
	/*if (fcntl(socket_id_difop, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        perror("non-block-difop");
        return false;
    }*/

    return true;
}

bool LidarN301Driver::initialize() {
    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create all ROS IO...");
        return false;
    }

    if (!openUDPPort()) {
        ROS_ERROR("Cannot open UDP port...");
        return false;
    }
    ROS_INFO("Initialised lidar n301 without error");
    return true;
}

int LidarN301Driver::getDifopPacket(lidar_n301_msgs::LidarN301PacketPtr& packet) {
    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = socket_id_difop;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 3000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
            {
                if (errno != EINTR)
                    ROS_ERROR("poll() error: %s", strerror(errno));
                return 1;
            }
            if (retval == 0)            // poll() timeout?
            {
                ROS_WARN("lidar_difop poll() timeout");
                return 1;
            }
            if ((fds[0].revents & POLLERR)
                    || (fds[0].revents & POLLHUP)
                    || (fds[0].revents & POLLNVAL)) // device error?
            {
                ROS_ERROR("poll() reports lidar error");
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(socket_id_difop, &packet->data[0], PACKET_SIZE,  0,
                (sockaddr*) &sender_address, &sender_address_len);
        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                ROS_INFO("recvfail");
                return 1;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if( device_ip_string != "" && sender_address.sin_addr.s_addr != device_ip.s_addr )
			{
                continue;
			}
            else
                break; //done
        }
    }
    return 0;
}

int LidarN301Driver::getPacket( lidar_n301_msgs::LidarN301PacketPtr& packet) {

    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = socket_id;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 2000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.
        // poll() until input available
        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
            {
                if (errno != EINTR)
                    ROS_ERROR("poll() error: %s", strerror(errno));
                return 1;
            }
            if (retval == 0)            // poll() timeout?
            {
                ROS_WARN("lidar poll() timeout");
                return 1;
            }
            if ((fds[0].revents & POLLERR)
                    || (fds[0].revents & POLLHUP)
                    || (fds[0].revents & POLLNVAL)) // device error?
            {
                ROS_ERROR("poll() reports lidar error");
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(socket_id, &packet->data[0], PACKET_SIZE,  0,
                (sockaddr*) &sender_address, &sender_address_len);
        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                ROS_INFO("recvfail");
                return 1;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if( device_ip_string != "" && sender_address.sin_addr.s_addr != device_ip.s_addr )
                continue;
            else
                break; //done
        }

    }

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred.
    double time2 = ros::Time::now().toSec();
    packet->stamp = ros::Time((time2 + time1) / 2.0);

    return 0;
}


bool LidarN301Driver::polling()
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lidar_n301_msgs::LidarN301PacketPtr packet(
                new lidar_n301_msgs::LidarN301Packet());
				
	std_msgs::Byte msg;
	struct timeval tv;
	int last_usec,now_usec;
	
	
	while (true)
    {
        // keep reading until full packet received
        int rc = getPacket(packet);
        if (rc == 0) break;       // got a full packet?
		if(rc == 1)
		{
			msg.data = 2;//no link
			type_pub.publish(msg);
		}
        if (rc < 0) return false; // end of file reached?
    }
	
	//Determine the protocol type based on the packet interval
	//////////////////////////////////////////
	if(first_time)
	{
		first_time = false;
		gettimeofday(&tv,NULL);
		last_usec = tv.tv_sec * 1000 + tv.tv_usec/1000;
		
		for(int i=0;i<200;i++)
		{
			while (true)
			{
				// keep reading until full packet received
				int rc = getPacket(packet);
				if (rc == 0) break;       // got a full packet?
				if (rc < 0) return false; // end of file reached?
			}	
		}
		gettimeofday(&tv,NULL);
		now_usec = tv.tv_sec * 1000 + tv.tv_usec/1000;
		
		if(now_usec - last_usec > 2000)//100*18ms/2
		{
			msg.data = 1;//v3.0/v4.0
			type_pub.publish(msg);
		}
	}
	//////////////////////////////////////////////
	

    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full lidar scan.");
	
	 if (packet->data[0] != 0xa5 || packet->data[1] != 0xff || packet->data[2] != 0x00 || packet->data[3] != 0x5a)
	 {
			packet_pub.publish(*packet);
	 }
    else
	{
			for(int i=0; i <1206;i++)
				difop_data[i] = packet->data[i];
			difop_output_.publish(*packet);
	}

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic->tick(packet->stamp);
    diagnostics.update();

    return true;
}

void LidarN301Driver::difopPoll()
{
    // reading and publishing scans as fast as possible.
	lidar_n301_msgs::LidarN301PacketPtr packets(
                new lidar_n301_msgs::LidarN301Packet());
				
    while (ros::ok())
    {
        // keep reading
       // lidar_c32_msgs::LidarC32Packet difop_packet_msg;
		 int rc = getDifopPacket(packets);
        if (rc == 0)
        {
				// std::cout << "Publishing a difop data." << std::endl;
				ROS_DEBUG("Publishing a difop data.");
				difop_output_.publish(*packets);
        }
        if (rc < 0)
            return;  // end of file reached?
        ros::spinOnce();
    }
}


void LidarN301Driver::serialPoll()
{
	char str1[100] = "open";
	char str2[100] = "close";
	
	ros::Rate loop_rate(500); 	  
    while (ros::ok())
    {
		char str[100] = "";
		fgets(str,100,stdin);
		
		if(strncmp(str,str1,4) == 0)
		{
			SendPacketToLidar(true);
		}
		else if(strncmp(str,str2,5) == 0)
		{ 
			SendPacketToLidar(false);
		}
    }
	ros::spinOnce();
	loop_rate.sleep(); 
}

bool LidarN301Driver::SendPacketToLidar(bool power_switch)
{
	int socketid;
	unsigned char config_data[46];
	int data_port = difop_data[12]*256 + difop_data[13];

	memset(config_data,0,46);
	config_data[0] = 0xff;
	config_data[1] = 0x00;
	config_data[2] = 0x00;
	config_data[3] = 0xa2;
	config_data[4] = 0xff;
	config_data[5] = 0xff;
	config_data[6] = 0x00;
	config_data[7] = 0x00;
	config_data[8] = 0xa3;
	
	memcpy(config_data+9,difop_data+8,4);//ip
	memcpy(config_data+13,difop_data+8,3);//gateway
	config_data[16] = 0x01;
	
	memcpy(config_data+19,difop_data+14,4);
	memcpy(config_data+17,difop_data+12,2);//port
	memcpy(config_data+23,difop_data+18,2);
	memcpy(config_data+29,difop_data+20,2);//speed
	
	if(power_switch)
		config_data[32] = 0x0b;
	
	config_data[42] = 0xa5;
	config_data[43] = 0x5a;
	config_data[44] = 0xf0;
	config_data[45] = 0x0f;
	
    sockaddr_in addrSrv;
    socketid = socket(2, 2, 0);
    addrSrv.sin_addr.s_addr = inet_addr(device_ip_string.c_str());
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(data_port);
	
    sendto(socketid, (const char*)config_data, 46, 0, (struct sockaddr *)&addrSrv, sizeof(addrSrv));
    return 0;
}

} // namespace lidar_driver
