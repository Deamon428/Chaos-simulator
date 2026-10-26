#include "logger.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

std::string csvEscape(const std::string& s) {
    if (s.find(',') == std::string::npos && s.find('"') == std::string::npos) return s;
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += "\"\"";
        out += c;
    }
    out += '"';
    return out;
}

}  // namespace

ResultsLogger::ResultsLogger(const std::string& path) : path_(path) {
    std::ifstream in(path);
    if (!in) return;  // new file; header written on first append
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line.rfind("run_id,", 0) == 0) continue;
        size_t comma = line.find(',');
        if (comma == std::string::npos) continue;
        try {
            int id = std::stoi(line.substr(0, comma));
            if (id >= next_id_) next_id_ = id + 1;
        } catch (...) {
            // Ignore malformed rows.
        }
    }
}

void ResultsLogger::append(const Row& row) {
    bool need_header = false;
    {
        std::ifstream in(path_);
        need_header = !in || in.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream out(path_, std::ios::app);
    if (!out) throw std::runtime_error("logger: cannot write '" + path_ + "'");
    if (need_header) {
        out << "run_id,video,start_x,start_y,end_x,end_y,magnet_id,dist_px,duration_s,"
               "detection_ratio,valid,note\n";
    }
    out << row.run_id << ',' << csvEscape(row.video) << ',' << row.start_x << ',' << row.start_y
        << ',' << row.end_x << ',' << row.end_y << ',' << row.magnet_id << ',' << row.dist_px
        << ',' << row.duration_s << ',' << row.detection_ratio << ',' << row.valid << ','
        << csvEscape(row.note) << '\n';
    if (row.run_id >= next_id_) next_id_ = row.run_id + 1;
}

void ResultsLogger::writeTrajectory(const std::string& path, const std::string& video,
                                    const std::string& records) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("logger: cannot write '" + path + "'");
    out << records;
}
