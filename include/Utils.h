#pragma once

#include <vector>
#include <string>

namespace Utils
{
    bool getImageStamps(const std::string &folder_path,
                        std::vector<std::uint64_t> &time_stamps);
    bool getImageStampsImageList(const std::string &image_list_file,
                                 std::vector<std::uint64_t> &time_stamps,
                                 std::vector<std::string> &original_stamps);
}