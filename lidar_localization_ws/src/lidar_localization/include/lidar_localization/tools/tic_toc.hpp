/*
 * @Description: 用来测试运行时间
 * @Created Date: 2020-03-01 18:12:03
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:31:51
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_TOOLS_TIC_TOC_HPP_
#define LIDAR_LOCALIZATION_TOOLS_TIC_TOC_HPP_

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace lidar_localization {
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
}  // namespace lidar_localization
#endif
