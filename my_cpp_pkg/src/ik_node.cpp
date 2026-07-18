#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

class IKNode : public rclcpp::Node
{
public:
    IKNode() : Node("ik_node")
    {
        //sub_ = this->create_subscription<geometry_msgs::msg::Point>(
          //  "/arm/target_pose", 10,
          //  std::bind(&IKNode::on_target, this, std::placeholders::_1));

        // Create subscription, call member function on message
        sub_ = this->create_generic_subscription(
            "/arm/target_xyz",
            "geometry_msgs/msg/Point",
	    rclcpp::QoS(10),
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                this->on_target(msg);  // call your member function
            });

        //pub_ = this->create_publisher<std_msgs::msg::String>(
            //"/arm/joint_cmd", 10);
	
	pub_ = this->create_generic_publisher(
	    "/arm/joint_cmd",
	    "std_msgs/msg/String",
	    rclcpp::QoS(10));

    }

//void init_publisher()
//{
 //   pub_ = this->create_generic_publisher(
  //      "/arm/joint_cmd",
    //    "std_msgs/msg/String",
      //  rclcpp::QoS(10));

    //RCLCPP_INFO(this->get_logger(), "Publisher created");

//}

private:
    // ==========================
    // ARM CONSTANTS (cm / degrees)
    // ==========================
    const double L1 = 7.75;   // Shoulder → Elbow
    const double L2 = 11.0;   // Elbow → Wrist

    const double BASE_ZERO = 0.0;
    const double SHOULDER_ZERO = 260.0;
    const double ELBOW_ZERO = 160.0;

    const bool ELBOW_DOWN = true;

    rclcpp::GenericSubscription::SharedPtr sub_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::GenericPublisher::SharedPtr pub_;

    void on_target(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        geometry_msgs::msg::Point target;
        rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
        serializer.deserialize_message(msg.get(), &target);

        double x = target.x;
        double y = target.y;
        double z = target.z;

        // ==========================
        // 1. Base yaw
        // ==========================
        double theta_base = std::atan2(y, x);  // radians

        // ==========================
        // 2. Geometry in shoulder plane
        // ==========================
        double l = std::sqrt(x * x + y * y);
        double h = std::sqrt(l * l + z * z);

        // ==========================
        // 3. Elbow angle
        // ==========================
        double cos_theta_elbow =
            (h * h - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

        // Clamp to [-1, 1]
        if (cos_theta_elbow > 1.0) cos_theta_elbow = 1.0;
        if (cos_theta_elbow < -1.0) cos_theta_elbow = -1.0;

        double theta_elbow =
            ELBOW_DOWN ? std::acos(cos_theta_elbow)
                       : -std::acos(cos_theta_elbow);

        // ==========================
        // 4. Shoulder angle
        // ==========================
        double theta_shoulder =
            std::atan2(z, l) -
            std::atan2(L2 * std::sin(theta_elbow),
                       L1 + L2 * std::cos(theta_elbow));

        // ==========================
        // 5. Convert to degrees
        // ==========================
        double theta_base_deg = theta_base * 180.0 / M_PI;
        double theta_shoulder_deg = theta_shoulder * 180.0 / M_PI;
        double theta_elbow_deg = theta_elbow * 180.0 / M_PI;

        // ==========================
        // 6. Map to servo values
        // ==========================
        double servoBase = BASE_ZERO + theta_base_deg;
        double servoShoulder = SHOULDER_ZERO + theta_shoulder_deg;
        double servoElbow = ELBOW_ZERO - theta_elbow_deg;

        // ==========================
        // 7. Publish command
        // ==========================
        //std_msgs::msg::String cmd;
        //cmd.data =
        //    "MOVE " + std::to_string(static_cast<int>(std::round(servoBase))) +
          //  " " + std::to_string(static_cast<int>(std::round(servoShoulder))) +
          //  " " + std::to_string(static_cast<int>(std::round(servoElbow)));

        //pub_->publish(cmd);

	// 1️⃣ Create the typed message
	auto cmd = std::make_shared<std_msgs::msg::String>();
	cmd->data =
	    "MOVE " + std::to_string(static_cast<int>(std::round(servoBase))) +
	    " " + std::to_string(static_cast<int>(std::round(servoShoulder))) +
	    " " + std::to_string(static_cast<int>(std::round(servoElbow)));

	// 2️⃣ Serialize the message
	rclcpp::SerializedMessage serialized_msg;
	rclcpp::Serialization<std_msgs::msg::String> serializer2;
	serializer2.serialize_message(cmd.get(), &serialized_msg);

	// 3️⃣ Publish via generic publisher
	pub_->publish(serialized_msg);

        RCLCPP_INFO(this->get_logger(),
            "IK -> Base: %.2f  Shoulder: %.2f  Elbow: %.2f",
            servoBase, servoShoulder, servoElbow);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IKNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
