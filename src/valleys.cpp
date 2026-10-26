#include "valleys.h"

#include <algorithm>
#include <cmath>

std::vector<cv::Point2d> computeEquilibria(const std::vector<cv::Point2d>& magnets) {
    // Attraction setup: the bob is pulled onto a magnet, so each magnet is an
    // equilibrium point. (Repulsion setup would return the edge midpoints:
    // sorted angularly around the centroid, push_back((a + b) * 0.5).)
    return magnets;
}

int classifyAttractor(const std::vector<cv::Point2d>& equilibria, const cv::Point2d& p,
                      double& dist) {
    int best = -1;
    double best_d = 0;
    for (size_t i = 0; i < equilibria.size(); ++i) {
        double d = std::hypot(p.x - equilibria[i].x, p.y - equilibria[i].y);
        if (best < 0 || d < best_d) {
            best_d = d;
            best = static_cast<int>(i);
        }
    }
    dist = best_d;
    return best;
}
