#pragma once

#include <future>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

class ServiceManager
{
    public:
        ServiceManager();

        template<typename FutureT, typename WaitTimeT>
        std::future_status wait_for_result_blocking(FutureT &future, WaitTimeT time_to_wait)
        {
            auto end = std::chrono::steady_clock::now() + time_to_wait;
            std::chrono::milliseconds wait_period(100);
            std::future_status status = std::future_status::timeout;

            do {
                auto now = std::chrono::steady_clock::now();
                auto time_left = end - now;

                if (time_left <= std::chrono::seconds(0)) {break;}

                status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
            }
            while (rclcpp::ok() && status != std::future_status::ready);

            return status;
        }

        template<typename FutureT>
        void wait_for_result_async(FutureT& future, std::chrono::milliseconds timeout, std::function<void(std::future_status)> callback)
        {
            auto start_time = std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());

            auto timer = create_wall_timer(
                std::chrono::milliseconds(100),
                [this, &future, timeout, start_time, callback]() mutable
                    {
                        auto elapsed = std::chrono::steady_clock::now() - *start_time;

                        // Check timeout
                        if (elapsed >= timeout) {
                            timer_->cancel();
                            callback(std::future_status::timeout);
                            return;
                        }

                        // Poll the future (non-blocking, 0ms wait)
                        auto status = future.wait_for(std::chrono::milliseconds(0));

                        if (status == std::future_status::ready) {
                            timer_->cancel();
                            callback(std::future_status::ready);
                        }
                    }
                );

            timer_ = timer; // store to keep it alive + allow cancel
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
};