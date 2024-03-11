// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"

class WemosPublisher : public rclcpp::Node
{
public:
    WemosPublisher()
        : Node("wemos_publisher"), io_(), port_(io_), pattern_("H:(\\d+\\.\\d+);T:(\\d+\\.\\d+)")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        std::string port_name = this->get_parameter("port").as_string();
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        publisherT_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
        publisherH_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("humidity", 10);
        port_.open(port_name);
        port_.set_option(boost::asio::serial_port_base::baud_rate(9600));

        read_data();
        boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));
    }

private:
    void handler(const boost::system::error_code &error, size_t bytes_transferred)
    {
        buf_[bytes_transferred] = 0;
        auto message = std_msgs::msg::String();
        std::string str(buf_);

        std::smatch match;
        if (std::regex_search(str, match, pattern_))
        {
            // Extract the matched values
            double hValue = std::stod(match[1].str());
            double tValue = std::stod(match[2].str());
            auto relative_humidity_msg = sensor_msgs::msg::RelativeHumidity();
            relative_humidity_msg.header.stamp = this->now();       // Set the timestamp
            relative_humidity_msg.header.frame_id = "sensor_frame"; // Set the frame ID
            relative_humidity_msg.relative_humidity = hValue;       // Set the value
            publisherH_->publish(relative_humidity_msg);            // and publish
            auto temperature_msg = sensor_msgs::msg::Temperature();
            temperature_msg.header.stamp = this->now();       // Set the timestamp
            temperature_msg.header.frame_id = "sensor_frame"; // Set the frame ID
            temperature_msg.temperature = tValue;             // Set the value
            publisherT_->publish(temperature_msg);            // and publish
        }

        size_t pos = str.rfind("\r\n");
        if (pos != std::string::npos)
        {
            str.replace(pos, 2, ""); // Replace 2 characters with an empty string
        }
        message.data = str;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        read_data();
    }

    void read_data()
    {
        port_.async_read_some(boost::asio::buffer(buf_, 512),
                              boost::bind(&WemosPublisher::handler,
                                          this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
    }

    boost::asio::io_service io_;
    boost::asio::serial_port port_;

    char buf_[512];
    std::regex pattern_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisherT_;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr publisherH_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.append_parameter_override("port", "/dev/ttyUSB0");
    rclcpp::spin(std::make_shared<WemosPublisher>());
    rclcpp::shutdown();
    return 0;
}
