#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>

class WrenchMovement : public rclcpp::Node {
	public:
		WrenchMovement()
		: Node("wrench_movement_node"),
			force_magnitude_(10000.0),
			damping_gain_(500.0)
		{
			gz_node_ = std::make_shared<gz::transport::Node>();

			wrench_.mutable_force()->set_x(0.0);
			wrench_.mutable_force()->set_y(0.0);
			wrench_.mutable_force()->set_z(0.0);
			wrench_.mutable_torque()->set_x(0.0);
			wrench_.mutable_torque()->set_y(0.0);
			wrench_.mutable_torque()->set_z(0.0);

			keypress_sub_ = this->create_subscription<std_msgs::msg::Int32>(
				"/keyboard/keypress", 10,
				std::bind(&WrenchMovement::keypress_callback, this, std::placeholders::_1)
			);

			timer_ = this->create_wall_timer(
				std::chrono::milliseconds(100),
				std::bind(&WrenchMovement::timer_callback, this)
			);

			bool ok = gz_node_->Subscribe("/world/pool/model/high_level_robosub/twist", &WrenchMovement::twist_callback, this);
			if (!ok) {
				RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to twist topic.");
			}

			gz_node_pub_ = gz_node_->Advertise<gz::msgs::EntityWrench>("/world/pool/wrench");
			if (!gz_node_pub_) {
				RCLCPP_ERROR(this->get_logger(), "Failed to advertise on /world/pool/wrench");
			}	
		}

	private:
		void twist_callback(const gz::msgs::Twist &msg) {
			double wx = msg.angular().x();  // rotation about X
			//double wy = msg.angular().y();  // allowed: rotation about Y
			double wz = msg.angular().z();  // rotation about Z
		
			// Compute damping torque (proportional negative feedback)
			gz::msgs::Vector3d torque;
			torque.set_x(-damping_gain_ * wx);
			torque.set_y(0.0); // allow rotation about Y
			torque.set_z(-damping_gain_ * wz);
			
			RCLCPP_INFO(this->get_logger(), "Torque - x: %.2f, y: %.2f, z: %.2f", torque.x(), torque.y(), torque.z());
			wrench_.mutable_torque()->CopyFrom(torque);
		}

		void keypress_callback(const std_msgs::msg::Int32::SharedPtr msg)
		{
			int key_code = msg->data;
			//RCLCPP_INFO(this->get_logger(), "Key code: %d", key_code);

			// Reset wrench to zero before applying a new one
			wrench_.mutable_force()->set_x(0.0);
			wrench_.mutable_force()->set_y(0.0);
			wrench_.mutable_force()->set_z(0.0);

			switch (key_code) {
				case 'W':
					wrench_.mutable_force()->set_x(force_magnitude_);
					break;
				case 'S':
					wrench_.mutable_force()->set_x(-force_magnitude_);
					break;
				case 'A':
					wrench_.mutable_force()->set_y(force_magnitude_);
					break;
				case 'D':
					wrench_.mutable_force()->set_y(-force_magnitude_);
					break;
				case ' ':
					wrench_.mutable_force()->set_z(force_magnitude_);
					break;
				case 'V':
					wrench_.mutable_force()->set_z(-force_magnitude_);
					break;
				default:
					break;
			}
		}

		void timer_callback()
		{
			gz::msgs::EntityWrench msg;

			msg.mutable_wrench()->CopyFrom(wrench_);
			msg.mutable_entity()->set_name("high_level_robosub");
			msg.mutable_entity()->set_type(gz::msgs::Entity::MODEL);

			if (!(gz_node_pub_.Publish(msg))) {
				RCLCPP_ERROR(this->get_logger(), "Failed to publish wrench message.");
			} else {
				const auto &torque = wrench_.torque();
				RCLCPP_INFO(this->get_logger(), "Torque - x: %.2f, y: %.2f, z: %.2f", torque.x(), torque.y(), torque.z());
			}
		}

		gz::msgs::Wrench wrench_;
		double force_magnitude_;
		double damping_gain_;

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr keypress_sub_;

		std::shared_ptr<gz::transport::Node> gz_node_;
		gz::transport::Node::Publisher gz_node_pub_;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WrenchMovement>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
