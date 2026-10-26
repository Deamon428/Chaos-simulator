// CSV logging: one row per run in results.csv plus a per-run trajectory file.
#pragma once

#include <string>

class ResultsLogger {
public:
    explicit ResultsLogger(const std::string& path);

    // Next run id (max existing id + 1); written into each new row.
    int nextRunId() const { return next_id_; }

    struct Row {
        int run_id;
        std::string video;
        double start_x, start_y;
        double end_x, end_y;
        int magnet_id;        // nearest magnet (attractor); -1 when unsettled/invalid
        double dist_px;       // distance to that magnet
        double duration_s;
        double detection_ratio;
        int valid;            // 1/0
        std::string note;     // e.g. "unsettled", "poor detection"
    };

    void append(const Row& row);

    static void writeTrajectory(const std::string& path, const std::string& video,
                                const std::string& records);

private:
    std::string path_;
    int next_id_ = 1;
};
