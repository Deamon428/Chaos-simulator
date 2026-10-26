// Equilibrium points and classification.
//
// The base magnets ATTRACT the pendulum magnet, so the equilibrium points sit
// AT the magnets: whichever magnet the bob settles closest to wins.
#pragma once

#include <opencv2/core.hpp>
#include <vector>

// With attraction, the equilibrium points are the magnet positions themselves.
// (Kept as a function so the repulsion variant — edge midpoints — is a
// one-line change if the rig ever flips back.)
std::vector<cv::Point2d> computeEquilibria(const std::vector<cv::Point2d>& magnets);

// Nearest equilibrium point to p. Returns the index and the distance; -1 if empty.
int classifyAttractor(const std::vector<cv::Point2d>& equilibria, const cv::Point2d& p,
                      double& dist);
