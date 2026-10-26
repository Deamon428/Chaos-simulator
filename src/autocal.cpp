#include "autocal.h"

#include <opencv2/geometry/2d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>

#include "video_source.h"

namespace {

struct Cluster {
    std::vector<cv::Point2d> pts;
    int frames_with_hit = 0;
    cv::Point2d mean() const {
        cv::Point2d s(0, 0);
        for (const auto& p : pts) s += p;
        return s * (1.0 / pts.size());
    }
};

// Greedy clustering of per-frame blob centroids; returns clusters present in
// at least `min_frac` of frames.
std::vector<Cluster> stableClusters(const std::vector<std::vector<cv::Point2d>>& per_frame,
                                    double radius, double min_frac) {
    const int total = static_cast<int>(per_frame.size());
    std::vector<Cluster> clusters;
    for (int fi = 0; fi < total; ++fi) {
        const auto& cands = per_frame[fi];
        std::vector<bool> used(cands.size(), false);
        for (auto& cl : clusters) {
            cv::Point2d m = cl.mean();
            bool hit = false;
            for (size_t i = 0; i < cands.size(); ++i) {
                if (!used[i] && cv::norm(cands[i] - m) < radius) {
                    cl.pts.push_back(cands[i]);
                    used[i] = true;
                    hit = true;
                }
            }
            if (hit) cl.frames_with_hit++;
        }
        for (size_t i = 0; i < cands.size(); ++i) {
            if (used[i]) continue;
            Cluster cl;
            cl.pts.push_back(cands[i]);
            cl.frames_with_hit = 1;
            clusters.push_back(std::move(cl));
        }
    }

    std::vector<Cluster> good;
    for (auto& cl : clusters)
        if (cl.frames_with_hit >= min_frac * total) good.push_back(std::move(cl));
    std::sort(good.begin(), good.end(),
              [](const Cluster& a, const Cluster& b) { return a.frames_with_hit > b.frames_with_hit; });
    return good;
}

// Scale-free surround checks: ring and core radii follow the blob size, so
// they work at any provisional zoom.
double ringBrightness(const cv::Mat& gray, const cv::Point2d& p, double blob_diam) {
    int r0 = std::max(4, static_cast<int>(0.55 * blob_diam));
    int r1 = std::max(6, static_cast<int>(1.30 * blob_diam));
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8U);
    cv::circle(mask, p, r1, 255, -1);
    cv::circle(mask, p, r0, 0, -1);
    return cv::mean(gray, mask)[0];
}

double discP90(const cv::Mat& gray, const cv::Point2d& p, int r) {
    cv::Mat disc = cv::Mat::zeros(gray.size(), CV_8U);
    cv::circle(disc, p, r, 255, -1);
    std::vector<uchar> vals;
    for (int y = 0; y < gray.rows; ++y)
        for (int x = 0; x < gray.cols; ++x)
            if (disc.at<uchar>(y, x)) vals.push_back(gray.at<uchar>(y, x));
    if (vals.empty()) return 0;
    std::sort(vals.begin(), vals.end());
    return vals[static_cast<size_t>(0.9 * (vals.size() - 1))];
}

// Normalized hub-distance fingerprint of the canonical layout: the three
// hub-to-magnet distances divided by the mean side length, sorted descending.
// A fixed physical property of the rig — floor junk cannot match it.
std::array<double, 3> canonicalFingerprint(const SetupConfig& cfg) {
    std::vector<cv::Point2d> canon = cfg.canonical_magnets.empty()
                                         ? SetupConfig::defaultCanonicalMagnets()
                                         : cfg.canonical_magnets;
    cv::Point2d hub(400.0, 450.0);
    double hs[3];
    for (int i = 0; i < 3; ++i) hs[i] = cv::norm(canon[i] - hub);
    double mean_side = (cv::norm(canon[0] - canon[1]) + cv::norm(canon[1] - canon[2]) +
                        cv::norm(canon[2] - canon[0])) / 3.0;
    for (int i = 0; i < 3; ++i) hs[i] /= mean_side;
    std::sort(hs, hs + 3, std::greater<double>());
    return {hs[0], hs[1], hs[2]};
}

// Penalty: how far a candidate triple's sorted normalized hub-dists are from
// the canonical fingerprint.
double fingerprintPenalty(const std::array<cv::Point2d, 3>& mags,
                          const cv::Point2d& hub, const std::array<double, 3>& ref) {
    double hs[3];
    for (int i = 0; i < 3; ++i) hs[i] = cv::norm(mags[i] - hub);
    double mean_side = (cv::norm(mags[0] - mags[1]) + cv::norm(mags[1] - mags[2]) +
                        cv::norm(mags[2] - mags[0])) / 3.0;
    for (int i = 0; i < 3; ++i) hs[i] /= mean_side;
    std::sort(hs, hs + 3, std::greater<double>());
    return std::abs(hs[0] - ref[0]) + std::abs(hs[1] - ref[1]) + std::abs(hs[2] - ref[2]);
}

}  // namespace

AutoCalResult autoCalibrate(const std::string& video_path, const SetupConfig& cfg,
                            int label_offset, int first_frame) {
    AutoCalResult out;
    VideoSource src(video_path);
    if (!src.isOpened()) {
        out.error = src.error();
        return out;
    }

    const int total = src.frameCount();
    const int n = std::max(5, cfg.autocal_frames);
    const int board = cfg.board_size;
    // Provisional scale: image px -> board px. The real zoom is ~2-4 (the
    // ~450 px board in the image maps to the 800 px canonical board); all
    // scoring below is scale-free, this only sets the search bands.
    const double prov_zoom = 3.0;
    const cv::Point2d board_center(400.0, 450.0);

    // --- Pass 0: rough hub position per frame (largest white blob fully
    // inside the frame). The tape may be merged with a resting bob, but the
    // centroid stays within ~30 px of the tape — good enough to center the
    // provisional warp. ---
    std::vector<cv::Mat> frames;
    std::vector<cv::Point2d> hub0_per_frame;
    for (int k = 0; k < n; ++k) {
        double lo = first_frame + 0.03 * (total - first_frame);
        int idx = static_cast<int>(lo + (total - lo) * (0.97 * k / (n - 1.0)));
        if (!src.seek(idx)) continue;
        cv::Mat frame;
        if (!src.read(frame)) continue;
        frames.push_back(frame);

        cv::Mat hsv, white;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, 150), cv::Scalar(180, 50, 255), white);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(white, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double best_area = 0;
        cv::Point2d best_c;
        bool found = false;
        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area <= best_area) continue;
            if (area < 150 || area > 40000) continue;
            cv::Rect bb = cv::boundingRect(c);
            if (bb.x < 3 || bb.y < 3 || bb.br().x > frame.cols - 3 ||
                bb.br().y > frame.rows - 3)
                continue;
            // The tape is compact; a tape+bob union or a paper wedge is
            // elongated. Reject eccentric blobs.
            if (area >= 20) {
                cv::RotatedRect ell = cv::fitEllipse(c);
                double ecc = ell.size.width / ell.size.height;
                if (ecc < 1) ecc = 1 / ecc;
                if (ecc > 1.8) continue;
            }
            cv::Moments m = cv::moments(c);
            if (m.m00 <= 0) continue;
            best_area = area;
            best_c = {m.m10 / m.m00, m.m01 / m.m00};
            found = true;
        }
        if (found) hub0_per_frame.push_back(best_c);
    }
    if (frames.empty() || hub0_per_frame.size() < frames.size() / 2) {
        out.error = "autocal: could not locate the hub tape in most frames";
        return out;
    }
    std::vector<double> hx, hy;
    for (const auto& h : hub0_per_frame) {
        hx.push_back(h.x);
        hy.push_back(h.y);
    }
    std::sort(hx.begin(), hx.end());
    std::sort(hy.begin(), hy.end());
    cv::Point2d hub_img(hx[hx.size() / 2], hy[hy.size() / 2]);
    // Keep only frames whose hub agrees with the median (angled intro/outro
    // frames see a different scene entirely).
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<cv::Point2d> agree;
        for (const auto& h : hub0_per_frame)
            if (cv::norm(h - hub_img) < 45.0) agree.push_back(h);
        if (agree.size() < 3) break;
        hub0_per_frame = agree;
        hx.clear();
        hy.clear();
        for (const auto& h : hub0_per_frame) {
            hx.push_back(h.x);
            hy.push_back(h.y);
        }
        std::sort(hx.begin(), hx.end());
        std::sort(hy.begin(), hy.end());
        hub_img = {hx[hx.size() / 2], hy[hy.size() / 2]};
    }
    if (hub0_per_frame.size() < 3) {
        out.error = "autocal: hub position not stable across frames";
        return out;
    }
    if (getenv("AUTOCAL_DEBUG"))
        fprintf(stderr, "[autocal] hub_img (%.0f,%.0f)\n", hub_img.x, hub_img.y);

    // --- Pass 1: magnet candidates in provisional board space, centered on
    // the hub ---
    std::vector<std::vector<cv::Point2d>> per_frame;
    cv::Mat keep_frame;
    for (int k = 0; k < static_cast<int>(frames.size()); ++k) {
        const cv::Mat& frame = frames[k];
        cv::Mat A = cv::Mat::eye(3, 3, CV_64F);
        A.at<double>(0, 0) = prov_zoom;
        A.at<double>(1, 1) = prov_zoom;
        A.at<double>(0, 2) = board_center.x - hub_img.x * prov_zoom;
        A.at<double>(1, 2) = board_center.y - hub_img.y * prov_zoom;
        cv::Mat warped;
        cv::warpPerspective(frame, warped, A, {board, board});
        if (keep_frame.empty()) keep_frame = warped.clone();

        cv::Mat hsv, mask, gray;
        cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, cfg.magnet_v_lower),
                    cv::Scalar(180, cfg.magnet_s_upper, cfg.magnet_v_upper), mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Point2d> cands;
        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            // Provisional-space band: magnets are ~350 px^2 on the board at
            // zoom 1.0; the band covers provisional zooms 1.5-4.5.
            if (area < 60 || area > 1600) {
                if (getenv("AUTOCAL_DEBUG") && k == 0 && area > 30)
                    fprintf(stderr, "[autocal] blob a%.0f REJ-size\n", area);
                continue;
            }
            cv::Moments m = cv::moments(c);
            if (m.m00 <= 0) continue;
            cv::Point2d p(m.m10 / m.m00, m.m01 / m.m00);
            double hub_dist = cv::norm(p - board_center);
            // Magnets ring the hub; corner junk (floor, paper, wood ends) is
            // farther out.
            if (hub_dist < 120 || hub_dist > 460) continue;
            double diam = 2.0 * std::sqrt(area / CV_PI);
            double ring = ringBrightness(gray, p, diam);
            if (ring > cfg.magnet_dark_ring_v) {
                if (getenv("AUTOCAL_DEBUG") && k == 0)
                    fprintf(stderr, "[autocal] blob (%.0f,%.0f) a%.0f REJ-ring %.0f\n", p.x, p.y, area, ring);
                continue;
            }
            double p90 = discP90(gray, p, std::max(3, static_cast<int>(0.45 * diam)));
            if (p90 > cfg.magnet_bright_core_v) {
                if (getenv("AUTOCAL_DEBUG") && k == 0)
                    fprintf(stderr, "[autocal] blob (%.0f,%.0f) a%.0f REJ-core %.0f\n", p.x, p.y, area, p90);
                continue;
            }
            cands.push_back(p);
        }
        per_frame.push_back(std::move(cands));
    }

    std::vector<Cluster> cands = stableClusters(per_frame, 45.0, 0.15);
    if (getenv("AUTOCAL_DEBUG"))
        for (const auto& cl : cands) {
            cv::Point2d m = cl.mean();
            fprintf(stderr, "[autocal] cand (%.0f,%.0f) support %d/%d\n", m.x, m.y,
                    cl.frames_with_hit, static_cast<int>(per_frame.size()));
        }
    if (cands.size() < 3) {
        out.error = "autocal: found " + std::to_string(cands.size()) +
                    " magnet candidates, need 3 (tune magnet_* thresholds)";
        return out;
    }
    if (cands.size() > 12) cands.resize(12);
    const std::array<double, 3> fp_ref = canonicalFingerprint(cfg);

    // --- Choose the most-equilateral well-supported triple. The real magnet
    // triangle is nearly equilateral at ANY provisional zoom. ---
    double best_score = 0;
    bool found_tri = false;
    std::array<int, 3> best_idx{};
    for (int a = 0; a < static_cast<int>(cands.size()); ++a)
        for (int b = a + 1; b < static_cast<int>(cands.size()); ++b)
            for (int c = b + 1; c < static_cast<int>(cands.size()); ++c) {
                cv::Point2d pa = cands[a].mean(), pb = cands[b].mean(), pc = cands[c].mean();
                double d01 = cv::norm(pa - pb), d12 = cv::norm(pb - pc), d20 = cv::norm(pc - pa);
                double dmin = std::min({d01, d12, d20});
                double dmax = std::max({d01, d12, d20});
                if (dmin < 150) continue;
                double support = std::min({cands[a].frames_with_hit, cands[b].frames_with_hit,
                                           cands[c].frames_with_hit});
                cv::Point2d tri_c = (pa + pb + pc) / 3.0;
                double d_center =
                    cv::norm(tri_c - board_center) / ((d01 + d12 + d20) / 3.0);
                double fp = fingerprintPenalty({pa, pb, pc}, board_center, fp_ref);
                double score =
                    (dmax / dmin - 1.0) + 1.2 * d_center + 2.0 * fp - 0.01 * support;
                if (!found_tri || score < best_score) {
                    found_tri = true;
                    best_score = score;
                    best_idx = {a, b, c};
                }
            }
    if (!found_tri) {
        out.error = "autocal: no plausible magnet triangle among candidates";
        return out;
    }

    std::array<cv::Point2d, 3> mags{cands[best_idx[0]].mean(), cands[best_idx[1]].mean(),
                                    cands[best_idx[2]].mean()};

    // --- Hub: the provisional warp is centered on the median tape blob, so
    // the anchor IS the hub. (A "largest white blob near the centroid"
    // refinement proved unreliable — it sometimes locks onto wood highlights.) ---
    cv::Point2d hub = board_center;
    bool have_hub = true;

    // --- Label the trio so it matches the physical rig across videos. The
    // hub's barycentric coordinates inside the magnet triangle are a fixed
    // physical fingerprint; the detected magnets are assigned to canonical
    // slots M0/M1/M2 by matching those coordinates against the reference
    // measured on the reference video. ---
    auto barycentric = [&](const cv::Point2d& p, const std::array<cv::Point2d, 3>& tri) {
        // Solve p = c + u*(tri0-c) + v*(tri1-c); returns barycentric weights
        // with respect to (tri0, tri1, tri2).
        cv::Point2d c = (tri[0] + tri[1] + tri[2]) / 3.0;
        cv::Point2d e0 = tri[0] - c, e1 = tri[1] - c, d = p - c;
        double det = e0.x * e1.y - e0.y * e1.x;
        if (std::abs(det) < 1e-6) return std::array<double, 3>{1.0 / 3, 1.0 / 3, 1.0 / 3};
        double u = (d.x * e1.y - e1.x * d.y) / det;  // weight of tri[0]
        double v = (e0.x * d.y - d.x * e0.y) / det;  // weight of tri[1]
        // p - c = u*e0 + v*e1 with e0+e1+e2 = 0, so the weight of tri[2] is
        // not 1-u-v: redistribute through the centroid.
        double w2 = (1.0 - u - v) / 3.0;
        return std::array<double, 3>{w2 + u, w2 + v, w2};
    };
    std::array<double, 3> w = barycentric(hub, mags);

    std::vector<double> ref = cfg.layout_ref_bary;
    std::array<int, 3> assign{0, 1, 2};  // assign[slot] = magnet index
    if (ref.size() >= 2) {
        double ref_w0 = 1.0 - ref[0] - ref[1];
        double best = -1;
        std::array<int, 3> best_assign{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                if (j == i) continue;
                int k = 3 - i - j;
                // slot0 <- magnet i (weight w[i]), slot1 <- magnet j, slot2 <- magnet k
                double err = std::abs(w[i] - ref_w0) + std::abs(w[j] - ref[0]) +
                             std::abs(w[k] - ref[1]);
                if (best < 0 || err < best) {
                    best = err;
                    best_assign = {i, j, k};
                }
            }
        assign = best_assign;
    } else {
        // No reference measured yet: fall back to hub-distance ranking.
        std::array<int, 3> order{0, 1, 2};
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return cv::norm(mags[a] - hub) < cv::norm(mags[b] - hub);
        });
        assign = order;
    }
    std::array<cv::Point2d, 3> ranked{};
    for (int r = 0; r < 3; ++r) ranked[r] = mags[assign[r]];
    if (label_offset % 3 != 0) {
        std::array<cv::Point2d, 3> rotated{};
        for (int r = 0; r < 3; ++r) rotated[(r + label_offset) % 3] = ranked[r];
        ranked = rotated;
    }
    if (getenv("AUTOCAL_DEBUG")) {
        fprintf(stderr, "[autocal] hub (%.0f,%.0f) found=%d bary=(%.2f,%.2f,%.2f)\n", hub.x,
                hub.y, have_hub, w[0], w[1], w[2]);
        for (int r = 0; r < 3; ++r)
            fprintf(stderr, "[autocal] M%d (%.0f,%.0f) hub-dist %.0f\n", r, ranked[r].x,
                    ranked[r].y, cv::norm(ranked[r] - hub));
    }

    // --- Final homography: the 3 magnets -> canonical layout. ---
    std::vector<cv::Point2f> src_pts, dst_pts;
    std::vector<cv::Point2d> canon = cfg.canonical_magnets.empty()
                                         ? SetupConfig::defaultCanonicalMagnets()
                                         : cfg.canonical_magnets;
    for (int i = 0; i < 3; ++i) {
        src_pts.push_back(ranked[i]);
        dst_pts.push_back(canon[i]);
    }
    out.homography = cv::getAffineTransform(src_pts, dst_pts);
    cv::Mat homography = cv::Mat::eye(3, 3, CV_64F);
    out.homography.convertTo(homography(cv::Rect(0, 0, 3, 2)), CV_64F);
    out.homography = homography;

    // --- Static blobs for tracking rejection: stable magnet-band clusters
    // away from the magnets (tape fragments, wood highlights, floor spots). ---
    for (const auto& cl : cands) {
        bool on_magnet = false;
        for (const auto& m : ranked)
            if (cv::norm(cl.mean() - m) < 40.0) on_magnet = true;
        if (!on_magnet) out.static_blobs_img.push_back(cl.mean());
    }

    out.magnets_img = std::vector<cv::Point2d>(ranked.begin(), ranked.end());
    out.sample_frame = keep_frame;
    out.ok = true;
    return out;
}


// ---------------------------------------------------------------------------
// Single-image calibration (photo upload feature)
// ---------------------------------------------------------------------------
PhotoCalResult photoCalibrate(const cv::Mat& image, const SetupConfig& cfg,
                              int label_offset) {
    PhotoCalResult out;
    if (image.empty()) { out.error = "empty image"; return out; }
    const int board = cfg.board_size;
    const cv::Point2d board_center(400.0, 450.0);
    const double prov_zoom = 3.0;

    // Hub: the largest white blob fully inside the frame.
    cv::Mat hsv0, white;
    cv::cvtColor(image, hsv0, cv::COLOR_BGR2HSV);
    cv::inRange(hsv0, cv::Scalar(0, 0, 150), cv::Scalar(180, 50, 255), white);
    std::vector<std::vector<cv::Point>> hub_contours;
    cv::findContours(white, hub_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best_area = 0;
    bool have_hub = false;
    for (const auto& c : hub_contours) {
        double area = cv::contourArea(c);
        if (area <= best_area || area < 150 || area > 40000) continue;
        cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        cv::Rect bb = cv::boundingRect(c);
        if (bb.x < 3 || bb.y < 3 || bb.br().x > image.cols - 3 ||
            bb.br().y > image.rows - 3)
            continue;
        best_area = area;
        out.hub_image = {m.m10 / m.m00, m.m01 / m.m00};
        have_hub = true;
    }
    if (!have_hub) {
        out.error = "hub tape not found — is this a top-down photo of the board?";
        return out;
    }

    // Provisional warp about the hub, then magnet candidates in board space.
    cv::Mat prov = cv::Mat::eye(3, 3, CV_64F);
    prov.at<double>(0, 0) = prov_zoom;
    prov.at<double>(1, 1) = prov_zoom;
    prov.at<double>(0, 2) = board_center.x - out.hub_image.x * prov_zoom;
    prov.at<double>(1, 2) = board_center.y - out.hub_image.y * prov_zoom;
    cv::Mat warped;
    cv::warpPerspective(image, warped, prov, {board, board});

    cv::Mat hsv, mask, gray;
    cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, cfg.magnet_v_lower),
                cv::Scalar(180, cfg.magnet_s_upper, cfg.magnet_v_upper), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point2d> cands;
    const bool dbg = getenv("PHOTO_DEBUG") != nullptr;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 60 || area > 1600) {
            if (dbg && area > 40)
                fprintf(stderr, "[photo] blob a%.0f REJ-size\n", area);
            continue;
        }
        cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        cv::Point2d p(m.m10 / m.m00, m.m01 / m.m00);
        double hub_dist = cv::norm(p - board_center);
        if (hub_dist < 120 || hub_dist > 460) {
            if (dbg) fprintf(stderr, "[photo] blob (%.0f,%.0f) a%.0f REJ-hubdist %.0f\n",
                             p.x, p.y, area, hub_dist);
            continue;
        }
        double diam = 2.0 * std::sqrt(area / CV_PI);
        double ring = ringBrightness(gray, p, diam);
        if (ring > cfg.magnet_dark_ring_v) {
            if (dbg) fprintf(stderr, "[photo] blob (%.0f,%.0f) a%.0f REJ-ring %.0f\n",
                             p.x, p.y, area, ring);
            continue;
        }
        double p90 = discP90(gray, p, std::max(3, static_cast<int>(0.45 * diam)));
        if (p90 > cfg.magnet_bright_core_v) {
            if (dbg) fprintf(stderr, "[photo] blob (%.0f,%.0f) a%.0f REJ-core %.0f\n",
                             p.x, p.y, area, p90);
            continue;
        }
        if (dbg) fprintf(stderr, "[photo] blob (%.0f,%.0f) a%.0f ACCEPT\n", p.x, p.y, area);
        cands.push_back(p);
    }
    if (cands.size() < 3) {
        out.error = "found " + std::to_string(cands.size()) +
                    " magnet candidates in the photo, need 3";
        return out;
    }
    if (cands.size() > 12) cands.resize(12);

    // Most-equilateral hub-centered triple matching the rig's hub-distance
    // fingerprint, as in the video path.
    const std::array<double, 3> fp_ref = canonicalFingerprint(cfg);
    double best_score = 0;
    bool found_tri = false;
    std::array<int, 3> best_idx{};
    for (int a = 0; a < static_cast<int>(cands.size()); ++a)
        for (int b = a + 1; b < static_cast<int>(cands.size()); ++b)
            for (int c2 = b + 1; c2 < static_cast<int>(cands.size()); ++c2) {
                cv::Point2d pa = cands[a], pb = cands[b], pc = cands[c2];
                double d01 = cv::norm(pa - pb), d12 = cv::norm(pb - pc),
                       d20 = cv::norm(pc - pa);
                double dmin = std::min({d01, d12, d20});
                double dmax = std::max({d01, d12, d20});
                if (dmin < 150) continue;
                cv::Point2d tri_c = (pa + pb + pc) / 3.0;
                double d_center =
                    cv::norm(tri_c - board_center) / ((d01 + d12 + d20) / 3.0);
                double fp = fingerprintPenalty({pa, pb, pc}, board_center, fp_ref);
                double score = (dmax / dmin - 1.0) + 1.2 * d_center + 2.0 * fp;
                if (!found_tri || score < best_score) {
                    found_tri = true;
                    best_score = score;
                    best_idx = {a, b, c2};
                }
            }
    if (!found_tri) {
        out.error = "no plausible magnet triangle in the photo";
        return out;
    }
    std::array<cv::Point2d, 3> mags{cands[best_idx[0]], cands[best_idx[1]],
                                    cands[best_idx[2]]};

    // Label via the hub barycentric fingerprint (same as the video path).
    auto barycentric = [&](const cv::Point2d& p) {
        cv::Point2d c = (mags[0] + mags[1] + mags[2]) / 3.0;
        cv::Point2d e0 = mags[0] - c, e1 = mags[1] - c, d = p - c;
        double det = e0.x * e1.y - e0.y * e1.x;
        if (std::abs(det) < 1e-6)
            return std::array<double, 3>{1.0 / 3, 1.0 / 3, 1.0 / 3};
        double u = (d.x * e1.y - e1.x * d.y) / det;
        double v = (e0.x * d.y - d.x * e0.y) / det;
        double w2 = (1.0 - u - v) / 3.0;
        return std::array<double, 3>{w2 + u, w2 + v, w2};
    };
    std::array<double, 3> w = barycentric(board_center);
    std::vector<double> ref = cfg.layout_ref_bary;
    std::array<int, 3> assign{0, 1, 2};
    if (ref.size() >= 2) {
        double ref_w0 = 1.0 - ref[0] - ref[1];
        double best = -1;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                if (j == i) continue;
                int k = 3 - i - j;
                double err = std::abs(w[i] - ref_w0) + std::abs(w[j] - ref[0]) +
                             std::abs(w[k] - ref[1]);
                if (best < 0 || err < best) { best = err; assign = {i, j, k}; }
            }
    } else {
        std::array<int, 3> order{0, 1, 2};
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return cv::norm(mags[a] - board_center) < cv::norm(mags[b] - board_center);
        });
        assign = order;
    }
    std::array<cv::Point2d, 3> ranked{};
    for (int r = 0; r < 3; ++r) ranked[r] = mags[assign[r]];
    if (label_offset % 3 != 0) {
        std::array<cv::Point2d, 3> rotated{};
        for (int r = 0; r < 3; ++r) rotated[(r + label_offset) % 3] = ranked[r];
        ranked = rotated;
    }

    std::vector<cv::Point2d> canon = cfg.canonical_magnets.empty()
                                         ? SetupConfig::defaultCanonicalMagnets()
                                         : cfg.canonical_magnets;
    std::vector<cv::Point2f> src_pts, dst_pts;
    for (int i = 0; i < 3; ++i) {
        src_pts.push_back(ranked[i]);
        dst_pts.push_back(canon[i]);
    }
    cv::Mat aff23 = cv::getAffineTransform(src_pts, dst_pts);        // prov -> canonical
    cv::Mat aff = cv::Mat::eye(3, 3, CV_64F);
    aff23.convertTo(aff(cv::Rect(0, 0, 3, 2)), CV_64F);
    cv::Mat H = prov * aff;                                          // photo -> canonical

    // Annotated photo: map canonical magnets back through the inverse.
    cv::Mat H23(2, 3, CV_64F);
    H(cv::Rect(0, 0, 3, 2)).copyTo(H23);
    cv::Mat Hinv2x3;
    cv::invertAffineTransform(H23, Hinv2x3);   // OpenCV 5: 2x3 in, 2x3 out
    out.annotated = image.clone();
    auto toPhoto = [&](const cv::Point2d& p) {
        double X = Hinv2x3.at<double>(0, 0) * p.x + Hinv2x3.at<double>(0, 1) * p.y +
                   Hinv2x3.at<double>(0, 2);
        double Y = Hinv2x3.at<double>(1, 0) * p.x + Hinv2x3.at<double>(1, 1) * p.y +
                   Hinv2x3.at<double>(1, 2);
        return cv::Point2d(X, Y);
    };
    const cv::Scalar red(0, 0, 255), blue(255, 0, 0);
    for (int i = 0; i < 3; ++i) {
        cv::Point2d q = toPhoto(canon[i]);
        cv::circle(out.annotated, q, std::max(6, image.cols / 150), red, 2);
        cv::putText(out.annotated, "M" + std::to_string(i), q + cv::Point2d(12, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, blue, 2);
    }
    cv::Point2d hq = toPhoto(cv::Point2d(400, 450));
    cv::rectangle(out.annotated, hq - cv::Point2d(5, 5), hq + cv::Point2d(5, 5),
                  blue, 2);

    out.homography = H;
    out.magnets_image.assign(ranked.begin(), ranked.end());
    out.ok = true;
    return out;
}
