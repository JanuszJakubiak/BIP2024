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

#include <memory>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class WemosSubscriber : public rclcpp::Node
{
public:
  WemosSubscriber()
  : Node("str_subscriber"), io_(), port_(io_)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&WemosSubscriber::topic_callback, this, _1));
    port_.open("/dev/ttyUSB0");
    port_.set_option(boost::asio::serial_port_base::baud_rate(9600));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    boost::system::error_code ec;
    if (port_.is_open() and msg->data.size()>0) {
      //port_.write_some(boost::asio::buffer(msg->data.c_str(), msg->data.size()), ec);
      std::string s = msg->data;
      port_.write_some(boost::asio::buffer(s+"\n"),ec);
    }
  }

  mutable boost::asio::io_service io_;
  mutable boost::asio::serial_port port_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WemosSubscriber>());
  rclcpp::shutdown();
  return 0;
}
