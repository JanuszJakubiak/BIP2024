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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//using namespace boost;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WemosPublisher : public rclcpp::Node
{    
    boost::asio::io_service io_;
    boost::asio::serial_port port_;

public:
  WemosPublisher()
  : Node("wemos_publisher"), io_(),port_(io_) 
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    port_.open("/dev/ttyUSB0");
    port_.set_option(boost::asio::serial_port_base::baud_rate(9600));

    read_data();
    boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));
  }

private:
  
  void handler(const boost::system::error_code &error, size_t bytes_transferred)
  {
      buf_[bytes_transferred] = 0;
      std::cout << bytes_transferred << " bytes: " << buf_ << std::endl;
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

  char buf_[512];
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WemosPublisher>());
  rclcpp::shutdown();
  return 0;
}
