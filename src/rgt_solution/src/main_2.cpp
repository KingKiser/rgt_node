#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "CircularBuffer.h" // 모듈화된 헤더 포함
#include <algorithm>
#include <numeric>
#include <string>

class SensorFilterNode : public rclcpp::Node {
public:
    SensorFilterNode() : Node("sensor_filter_node"), temp_buffer_(5) {
        // "raw_temperature" 토픽을 통해 데이터를 입력받음
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "raw_temperature", 10, 
            std::bind(&SensorFilterNode::topic_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "=== 로봇 센서 데이터 필터링 노드 기동 ===");
    }

private:
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // 1. 데이터 주입 (CircularBuffer의 오버라이트 로직 작동)
        temp_buffer_.push_back(msg->data);
        RCLCPP_INFO(this->get_logger(), "데이터 수신: %.1f", msg->data);

        // 2. 과제 요구사항 출력 (데이터가 가득 찼을 때 기준)
        if (temp_buffer_.size() == 5) {
            RCLCPP_INFO(this->get_logger(), "---------------------------------------");
            RCLCPP_INFO(this->get_logger(), "tempBuffer.size() = %zu", temp_buffer_.size());
            RCLCPP_INFO(this->get_logger(), "tempBuffer.capacity() = %zu", temp_buffer_.capacity());
            RCLCPP_INFO(this->get_logger(), "tempBuffer.empty() = %s", temp_buffer_.empty() ? "true" : "false");

            // 이터레이터를 이용한 버퍼 순회 및 문자열 조합
            std::string buffer_str = "[ ";
            for (auto val : temp_buffer_) {
                buffer_str += std::to_string(val).substr(0, 4) + " ";
            }
            buffer_str += "]";
            RCLCPP_INFO(this->get_logger(), "현재 버퍼 내용: %s", buffer_str.c_str());

            // STL 알고리즘: max_element (최대값)
            auto it_max = std::max_element(temp_buffer_.begin(), temp_buffer_.end());
            double maxTemp = (it_max != temp_buffer_.end()) ? *it_max : 0.0;
            RCLCPP_INFO(this->get_logger(), "maxTemp = %.1f", maxTemp);
            
            // STL 알고리즘: accumulate (평균값 계산)
            double sum = std::accumulate(temp_buffer_.begin(), temp_buffer_.end(), 0.0);
            RCLCPP_INFO(this->get_logger(), "avgTemp = %.2f", sum / temp_buffer_.size());
            RCLCPP_INFO(this->get_logger(), "---------------------------------------");
        }
    }

    CircularBuffer<double> temp_buffer_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFilterNode>());
    rclcpp::shutdown();
    return 0;
}

//ros2 topic pub /raw_temperature std_msgs/msg/Float64 "{data: 26.1}"