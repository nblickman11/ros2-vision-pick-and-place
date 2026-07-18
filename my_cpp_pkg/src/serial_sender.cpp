#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SerialSender : public rclcpp::Node
{
public:
    SerialSender() : Node("serial_sender")
    {
	
        serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            rclcpp::shutdown();
            return;
        }

        configure_serial();
        
	// Create subscription, call member function on message
        sub_ = this->create_generic_subscription(
            "/arm/joint_cmd",
            "std_msgs/msg/String",
            rclcpp::QoS(10),
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                this->send_command(msg);  // call your member function
            });
	
	//sub_ = this->create_subscription<std_msgs::msg::String>(
         //     "/arm/joint_cmd", 10,
           //   std::bind(&SerialSender::send_command, this, std::placeholders::_1));

        //timer_ = this->create_wall_timer(
            //1s, std::bind(&SerialSender::send_command, this));
    }

    ~SerialSender()
    {
        if (serial_fd_ >= 0)
            close(serial_fd_);
    }

private:
    int serial_fd_;
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::shared_ptr<rclcpp::GenericSubscription> sub_;

    void configure_serial()
    {
        struct termios tty{};
        tcgetattr(serial_fd_, &tty);

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

        tty.c_lflag = 0;
        tty.c_iflag = 0;
        tty.c_oflag = 0;

        tcsetattr(serial_fd_, TCSANOW, &tty);
    }

    void send_command(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
	    RCLCPP_INFO(this->get_logger(), "send_command starting");
        std_msgs::msg::String str_msg;
        rclcpp::Serialization<std_msgs::msg::String> serializer;
        serializer.deserialize_message(msg.get(), &str_msg);

        RCLCPP_INFO(this->get_logger(), "%s", str_msg.data.c_str());
 	
	// Your custom handling
        std::cout << "Joint command received: " << str_msg.data << std::endl;
         
        //const char *cmd = "MOVE 50 170 90\n";
       
        //std::string cmd = str_msg.data + "\n";
	//std::string cmd = "MOVE 50 170 90\n";
	ssize_t bytes_written = write(serial_fd_, cmd.c_str(), cmd.size());

	if (bytes_written < 0) {
	        std::cout << "Joint command received: " << std::endl;
	}
	
	RCLCPP_INFO(this->get_logger(), "Sent command to Arduino");
        
            // --- Read any response from Arduino ---
            char buf[128];
            int n = read(serial_fd_, buf, sizeof(buf) - 1); // read up to 127 bytes
            if (n > 0)
            {
                buf[n] = '\0'; // null-terminate string
                RCLCPP_INFO(this->get_logger(), "Received from Arduino: %s", buf);
            }
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialSender>());
    rclcpp::shutdown();
    return 0;
}


