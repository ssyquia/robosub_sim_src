#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <gz/transport/Node.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/quaternion.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/pose.pb.h>

#include <cmath>

class WrenchMovement : public rclcpp::Node {
	public:
		WrenchMovement()
		: Node("wrench_movement_node"),
			force_magnitude_(10000.0),
			damping_gain_(5000.0)
		{
			prev_time_ = this->now();
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
			
			
			bool ok = gz_node_->Subscribe("/model/high_level_robosub/pose", &WrenchMovement::pose_callback, this);
			if (!ok) {
				RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to pose topic.");
			}
			
			gz_node_pub_ = gz_node_->Advertise<gz::msgs::EntityWrench>("/world/pool/wrench");
			if (!gz_node_pub_) {
				RCLCPP_ERROR(this->get_logger(), "Failed to advertise on /world/pool/wrench");
			}	
		}

	private:
		void pose_callback(const gz::msgs::Pose_V &msg) {
			rclcpp::Time current_time = this->now();
			double dt = (current_time - prev_time_).seconds();

			if (!first_time_ && dt > 0.0) {
				gz::msgs::Pose pose = msg.pose(0);
				gz::msgs::Quaternion orientation = pose.orientation();
				gz::math::Quaterniond q_current(orientation.w(), orientation.x(), orientation.y(), orientation.z());
				
				// Quaternion delta
				gz::math::Quaterniond q_delta = q_current * prev_orientation_.Inverse();

				// Convert quaternion delta to axis-angle
				gz::math::Vector3d axis;
				double angle;
				q_delta.AxisAngle(axis, angle);

				// Estimated angular velocity (rad/s)
				gz::math::Vector3d angular_velocity = axis * (angle / dt);

				// Apply damping torque: τ = -kD * ω
				gz::math::Vector3d damping_torque = -damping_gain_ * angular_velocity;

				// Send as gz::msgs
				gz::msgs::Wrench wrench;
				wrench_.mutable_torque()->set_x(damping_torque.X());
				wrench_.mutable_torque()->set_y(damping_torque.Y());
				wrench_.mutable_torque()->set_z(damping_torque.Z());

				// Update for next time
				prev_orientation_ = q_current;
				prev_time_ = current_time;
			} else {
				gz::msgs::Pose pose = msg.pose(0);
				gz::msgs::Quaternion orientation = pose.orientation();
				prev_orientation_ = gz::math::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z());
				prev_time_ = current_time;
				first_time_ = false;
			}
		}

		void keypress_callback(const std_msgs::msg::Int32::SharedPtr msg)
		{
			int key_code = msg->data;

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
			}
		}

		gz::msgs::Wrench wrench_;
		double force_magnitude_;
		double damping_gain_;

		gz::math::Quaterniond prev_orientation_;
		rclcpp::Time prev_time_;
		bool first_time_ = true;

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
