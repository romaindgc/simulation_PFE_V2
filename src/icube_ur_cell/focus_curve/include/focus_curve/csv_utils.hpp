#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

namespace csv_utils {

inline std::string generate_csv_filename(const std::string& base_path, const std::string& base_name = "data", const std::string& extension = ".csv") {
    static int csv_counter = 0;  // compteur unique pour cette fonction

    // Récupérer l’heure actuelle
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::stringstream ss;
    ss << base_path << "/" << base_name << "_"
       << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
       << "_" << csv_counter++
       << extension;

    return ss.str();
}

}
