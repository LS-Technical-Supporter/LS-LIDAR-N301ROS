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

#include <ros/ros.h>

#include <lidar_n301_decoder/lidar_n301_decoder.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_n301_decoder_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    lidar_n301_decoder::LidarN301DecoderPtr decoder(
                new lidar_n301_decoder::LidarN301Decoder(nh, pnh));

    if (!decoder->initialize()) {
        ROS_INFO("Cannot initialize the decoder...");
        return -1;
    }

  ros::spin();

    return 0;
}
