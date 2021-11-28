/*
 * @Description: 读写文件管理
 * @Created Date: 2020-02-24 19:22:53
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define MAPPING_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <string>

namespace mapping_localization {
class FileManager {
public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool InitDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path);
};
}  // namespace mapping_localization

#endif
