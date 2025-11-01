#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>
#include "turtle_interfaces/action/go_to_goal.hpp"

using GoToGoal = turtle_interfaces::action::GoToGoal;
using GoalHandleGoToGoal = rclcpp_action::ClientGoalHandle<GoToGoal>;
using Empty = std_srvs::srv::Empty;
using namespace std::chrono_literals;

class Paint : public rclcpp::Node {
public:
    Paint() : Node("turtle_painter"), current_goal_index_(0), clear_called_(false) {

        goals_ = {
            {5.0, 7.0, 0.0},
            {3.0, 8.0, 0.0},
            {3.0, 2.0, 0.0},
            {5.0, 3.0, 0.0},
            {5.0, 7.0, 0.0},
            {5.0, 3.0, 0.0},
            {6.0, 4.0, 0.0},
            {6.0, 6.0, 0.0},
            {9.3, 4.3, 0.0},
            {9.3, 2.0, 0.0},
            {6.0, 4.0, 0.0},
            {6.0, 6.0, 0.0},
            {8.0, 5.0, 0.0},
            {9.3, 5.8, 0.0},
            {9.3, 8.0, 0.0},
            {3.0, 11.0, 0.0},
            {3.0, 9.3, 0.0},
            {5.0, 8.0, 0.0},
            {5.0, 10.1, 0.0},
            {5.0, 8.0, 0.0},
            {9.3, 5.8, 0.0},
            {7.0, 7.0, 0.0},
            {5.0, 5.5, 0.0},
            {5.0, 7.0, 0.0}
        };
        

        reset_client_ = this->create_client<Empty>("/reset");
        clear_client_ = this->create_client<Empty>("/clear");
        
        action_client_ = rclcpp_action::create_client<GoToGoal>(
            this, 
            "/turtle_go_to_goal"
        );
        
        RCLCPP_INFO(this->get_logger(), "Paint node");
        
        init_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&Paint::check_services, this)
        );
    }

private:
    struct Goal {
        double x;
        double y;
        double theta;
    };
    
    rclcpp::Client<Empty>::SharedPtr reset_client_;
    rclcpp::Client<Empty>::SharedPtr clear_client_;
    rclcpp_action::Client<GoToGoal>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::vector<Goal> goals_;
    size_t current_goal_index_;
    bool clear_called_;
    bool initialized_;
    
    void check_services() {
        if (reset_client_->service_is_ready() && 
            clear_client_->service_is_ready() && 
            action_client_->action_server_is_ready()) {
            
            RCLCPP_INFO(this->get_logger(), "Service ready smw");
            init_timer_->cancel(); 
            start_sequence();
        } else {
            RCLCPP_INFO(this->get_logger(), "waiting service");
        }
    }
    
    void start_sequence() {
        call_reset_service();
    }
    
    void call_reset_service() {
        auto request = std::make_shared<Empty::Request>();
        
        auto response_callback = [this](rclcpp::Client<Empty>::SharedFuture future) {
            try {
                auto response = future.get();
                clear_called_ = false;
                current_goal_index_ = 0;
                send_next_goal();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Gagal: %s", e.what());
            }
        };
        
        reset_client_->async_send_request(request, response_callback);
    }
    
    void call_clear_service() {
        auto request = std::make_shared<Empty::Request>();
        
        auto response_callback = [this](rclcpp::Client<Empty>::SharedFuture future) {
            try {
                auto response = future.get();
                clear_called_ = true;
                send_next_goal();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Gagal: %s", e.what());
            }
        };
        
        clear_client_->async_send_request(request, response_callback);
    }
    
    void send_next_goal() {
        if (current_goal_index_ >= goals_.size()) {
            RCLCPP_INFO(this->get_logger(), "Selamat berjuang, sukses!");
            return;
        }
        
        if (current_goal_index_ == 1 && !clear_called_) {
            call_clear_service();
            return;
        }
        
        auto goal = goals_[current_goal_index_];
        
        auto goal_msg = GoToGoal::Goal();
        goal_msg.x = goal.x;
        goal_msg.y = goal.y;
        goal_msg.theta = goal.theta;
        
        auto send_goal_options = rclcpp_action::Client<GoToGoal>::SendGoalOptions();
        
        send_goal_options.feedback_callback =
            [this](
                GoalHandleGoToGoal::SharedPtr,
                const std::shared_ptr<const GoToGoal::Feedback> feedback)
            {
                // mengurangi service call
                static int feedback_counter = 0;
                if (feedback_counter++ % 50 == 0) {
                    RCLCPP_INFO(this->get_logger(), "[Goal %zu/%zu] Distance: %.2f, Angle error: %.2f",
                               current_goal_index_ + 1,
                               goals_.size(),
                               feedback->distance_to_goal,
                               feedback->angle_error);
                }
            };
        
        send_goal_options.result_callback =
            [this](const GoalHandleGoToGoal::WrappedResult & result)
            {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "[Goal %zu/%zu] Sukses! Position: (%.2f, %.2f, %.2f)",
                                   current_goal_index_ + 1,
                                   goals_.size(),
                                   result.result->final_x,
                                   result.result->final_y,
                                   result.result->final_theta);
                        current_goal_index_++;
                        send_next_goal();
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "[Goal %zu/%zu] gagal",
                                    current_goal_index_ + 1, goals_.size());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "[Goal %zu/%zu] gagal",
                                   current_goal_index_ + 1, goals_.size());
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "[Goal %zu/%zu] gagal",
                                    current_goal_index_ + 1, goals_.size());
                        break;
                }
            };
        
        RCLCPP_INFO(this->get_logger(), "Mengirim goal %zu/%zu: (%.2f, %.2f, %.2f)",
                   current_goal_index_ + 1,
                   goals_.size(),
                   goal.x, goal.y, goal.theta);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto paint = std::make_shared<Paint>();
    rclcpp::spin(paint);
    rclcpp::shutdown();
    return 0;
}
