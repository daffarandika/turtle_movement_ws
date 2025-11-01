#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <turtlesim/msg/pose.hpp>
#include "turtle_interfaces/action/go_to_goal.hpp"

using Twist = geometry_msgs::msg::Twist;
using Pose = turtlesim::msg::Pose;
using GoToGoal = turtle_interfaces::action::GoToGoal;
using GoalHandleGoToGoal = rclcpp_action::ServerGoalHandle<GoToGoal>;
using namespace std::chrono_literals;

class TurtleGoToGoalActionServer : public rclcpp::Node {
public:
	TurtleGoToGoalActionServer() : Node("go_to_goal") {
		this->declare_parameter<std::string>("turtle_name", "turtle1");
		this->declare_parameter<double>("kp_angular", 2.0);
		this->declare_parameter<double>("ki_angular", 0.1);
		this->declare_parameter<double>("kd_angular", 0.5);
		this->declare_parameter<double>("kp_linear", 1.0);
		this->declare_parameter<double>("distance_tolerance", 0.3);
		this->declare_parameter<double>("angle_tolerance", 0.05);
		this->declare_parameter<double>("near_goal_distance", 4.0);

		std::string turtle_name = this->get_parameter("turtle_name").as_string();

		this->publisher_ = this->create_publisher<Twist>(turtle_name + "/cmd_vel", 10);
		this->subscriber_ = this->create_subscription<Pose>(
			turtle_name + "/pose",
			10,
			std::bind(&TurtleGoToGoalActionServer::pose_callback, this, std::placeholders::_1)
		);
		this->action_server_ = rclcpp_action::create_server<GoToGoal>(
			this,
			"turtle_go_to_goal",
			std::bind(&TurtleGoToGoalActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&TurtleGoToGoalActionServer::handle_cancel, this, std::placeholders::_1),
			std::bind(&TurtleGoToGoalActionServer::handle_accepted, this, std::placeholders::_1)
		);

		RCLCPP_INFO(this->get_logger(), "Go To Goal mulai");
		RCLCPP_INFO(this->get_logger(), "PID Param - kp: %.2f, ki: %.2f, kd: %.2f", 
							this->get_parameter("kp_angular").as_double(),
							this->get_parameter("ki_angular").as_double(),
							this->get_parameter("kd_angular").as_double());
	}

private:
	rclcpp::Publisher<Twist>::SharedPtr publisher_;
	rclcpp::Subscription<Pose>::SharedPtr subscriber_;
	rclcpp_action::Server<GoToGoal>::SharedPtr action_server_;
	Pose currPose_;

	void pose_callback(const Pose& msg) {
		this->currPose_ = msg;
	}

	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & uuid,
		std::shared_ptr<const GoToGoal::Goal> goal)
	{
		(void)uuid;
		RCLCPP_INFO(this->get_logger(), "Request: x=%.2f, y=%.2f, theta=%.2f", 
							goal->x, goal->y, goal->theta);
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleGoToGoal> goal_handle)
	{
		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<GoalHandleGoToGoal> goal_handle) {
		std::thread{std::bind(&TurtleGoToGoalActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
	}

	void execute(const std::shared_ptr<GoalHandleGoToGoal> goal_handle) {
		rclcpp::Rate loop_rate(100);
		const auto goal = goal_handle->get_goal();
		auto feedback = std::make_shared<GoToGoal::Feedback>();
		auto result = std::make_shared<GoToGoal::Result>();

		double distance_tolerance = this->get_parameter("distance_tolerance").as_double();
		double angle_tolerance = this->get_parameter("angle_tolerance").as_double();
		double near_goal_distance = this->get_parameter("near_goal_distance").as_double();

		double kp_angular = this->get_parameter("kp_angular").as_double();
		double ki_angular = this->get_parameter("ki_angular").as_double();
		double kd_angular = this->get_parameter("kd_angular").as_double();

		double kp_linear = this->get_parameter("kp_linear").as_double();

		double integral_angular = 0.0;
		double previous_error_angular = 0.0;
		double dt = 0.01;

		double integral_limit = 1.0;

		while (rclcpp::ok()) {
			if (goal_handle->is_canceling()) {
				auto stop_twist = Twist();
				stop_twist.linear.x = 0.0;
				stop_twist.angular.z = 0.0;
				this->publisher_->publish(stop_twist);

				result->success = false;
				goal_handle->canceled(result);
				return;
			}

			double distance_to_goal = std::sqrt(
				std::pow((goal->x - this->currPose_.x), 2) + 
				std::pow((goal->y - this->currPose_.y), 2)
			);
			double angle_to_goal = std::atan2(
				goal->y - this->currPose_.y, 
				goal->x - this->currPose_.x
			);
			double angle_error = angle_to_goal - this->currPose_.theta;

			// Normalize angle_error to [-π, π]
			while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
			while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

			feedback->distance_to_goal = distance_to_goal;
			feedback->angle_error = angle_error;
			goal_handle->publish_feedback(feedback);


			auto newTwist = Twist();

			if (distance_to_goal <= distance_tolerance) {
				newTwist.linear.x = 0.0;
				newTwist.angular.z = 0.0;
				this->publisher_->publish(newTwist);

				result->success = true;
				result->final_x = this->currPose_.x;
				result->final_y = this->currPose_.y;
				result->final_theta = this->currPose_.theta;
				goal_handle->succeed(result);

				RCLCPP_INFO(this->get_logger(), "GOAL TERCAPAI!");
				return;
			}
			else if (std::abs(angle_error) > angle_tolerance) {
				double p_term = kp_angular * angle_error;

				integral_angular += angle_error * dt;
				if (integral_angular > integral_limit) integral_angular = integral_limit;
				if (integral_angular < -integral_limit) integral_angular = -integral_limit;
				double i_term = ki_angular * integral_angular;

				double derivative = (angle_error - previous_error_angular) / dt;
				double d_term = kd_angular * derivative;

				newTwist.angular.z = p_term + i_term + d_term;
				newTwist.linear.x = 0.0;

				previous_error_angular = angle_error;
			}
			else {
				newTwist.linear.x = kp_linear * distance_to_goal;
				newTwist.angular.z = 0.0;

				integral_angular = 0.0;
				previous_error_angular = 0.0;
			}	
			this->publisher_->publish(newTwist);
			loop_rate.sleep();
		}
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TurtleGoToGoalActionServer>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
