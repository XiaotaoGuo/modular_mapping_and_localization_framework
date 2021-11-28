/*
 * @Description: 读写文件管理
 * @Created Date: 2020-02-24 19:22:53
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:31:20
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <string>

namespace lidar_localization {
class FileManager {
public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool InitDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path);
};
}  // namespace lidar_localization

#endif
