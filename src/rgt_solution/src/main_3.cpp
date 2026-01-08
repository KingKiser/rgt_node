#include <memory>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ParallelProcessor.h"

class ParallelProcessorNode : public rclcpp::Node {
public:
    ParallelProcessorNode() : Node("parallel_processor_node"), processor_(4) {
        run_logic();
    }

private:
    void run_logic() {
        // 1. 데이터 생성 (기존 main_3와 동일)
        std::vector<int> pixelData(1000000);
        std::iota(pixelData.begin(), pixelData.end(), 0);

        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "병렬 처리 시작 (Threads: 4, Data: 1,000,000)");
        
        auto start_time = std::chrono::high_resolution_clock::now();

        // 2. 병렬 연산 수행 (밝기 조절 필터)
        auto brightenedImage = processor_.parallel_map(pixelData, [](int pixel) {
            // 기존 main_3의 sleep_for 시뮬레이션 유지
            std::this_thread::sleep_for(std::chrono::microseconds(1)); 
            return std::min(255, pixel + 50);
        });

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // 3. 기존 main_3와 동일한 형식으로 결과 출력
        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "// brightenedImage 결과 (소요 시간: %ld ms)", duration);
        RCLCPP_INFO(this->get_logger(), "brightenedImage[0] = %d    // 0 + 50", brightenedImage[0]);
        RCLCPP_INFO(this->get_logger(), "brightenedImage[1] = %d    // 1 + 50", brightenedImage[1]);
        RCLCPP_INFO(this->get_logger(), "brightenedImage[100] = %d  // 100 + 50", brightenedImage[100]);
        RCLCPP_INFO(this->get_logger(), "brightenedImage[999999] = %d // min(255, 999999 + 50)", brightenedImage[999999]);
        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        
        RCLCPP_INFO(this->get_logger(), "모든 병렬 처리가 성공적으로 완료되었습니다.");
        RCLCPP_INFO(this->get_logger(), "==========================================");
    }

    ParallelProcessor<int> processor_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParallelProcessorNode>();
    
    // 연산 결과 출력 후 바로 종료
    rclcpp::shutdown();
    return 0;
}