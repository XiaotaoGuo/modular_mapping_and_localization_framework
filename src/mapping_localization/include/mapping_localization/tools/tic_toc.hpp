/*
 * @Description: 用来测试运行时间
 * @Created Date: 2020-03-01 18:12:03
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_TOOLS_TIC_TOC_HPP_
#define MAPPING_LOCALIZATION_TOOLS_TIC_TOC_HPP_

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ros/time.h>

namespace mapping_localization {
class TicToc {
public:
    TicToc() { tic(); }

    void tic() { start = std::chrono::system_clock::now(); }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        start = std::chrono::system_clock::now();
        return elapsed_seconds.count();
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

class RosTicToc {
public:
    RosTicToc() { tic(); }

    void tic() { start = ros::Time::now(); }

    double toc() {
        end = ros::Time::now();
        ros::Duration duration = (end - start);
        return duration.toSec();
    }

private:
    ros::Time start, end;
};

class Timer {
public:
    Timer(const std::string &name) : name_(name) {}

    void tic() { tictoc_.tic(); }

    void toc() {
        double cost = tictoc_.toc();
        min_cost = std::min(cost, min_cost);
        max_cost = std::max(cost, max_cost);
        average_cost_ = (average_cost_ * count_ + cost) / (count_ + 1);
        count_++;
    }

    int getCount() const { return count_; }

    friend std::ostream &operator<<(std::ostream &output, const Timer &timer) {
        output << "Timer " << timer.name_ << "==> average: " << timer.average_cost_
               << " seconds, min: " << timer.min_cost << " seconds, max: " << timer.max_cost
               << " seconds, count: " << timer.count_;

        return output;
    }

private:
    RosTicToc tictoc_;
    std::string name_;
    double average_cost_ = 0.0;
    double min_cost = 1000.0;
    double max_cost = 0.0;
    int count_ = 0;
};
}  // namespace mapping_localization
#endif
