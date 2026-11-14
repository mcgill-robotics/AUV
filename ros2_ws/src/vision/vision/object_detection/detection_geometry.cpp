#include "detection_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace vision {

namespace {

constexpr double kWaterRefractiveIndex = 1.333;

// Python int() truncates toward zero; the ROI math below relies on that.
int trunc_int(double v) { return static_cast<int>(v); }

int clamp_int(int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); }

}  // namespace

double apply_snells_law_depth(double z_raw, double u, double fx, double depth_scale, double n_w) {
    if (fx == 0.0) {
        return z_raw * depth_scale;
    }

    const double r_a_x = std::max(std::abs(u / fx), 1e-6);
    const double cos_a_x = 1.0 / std::sqrt(1.0 + r_a_x * r_a_x);
    const double sin_a_x = r_a_x * cos_a_x;

    const double sin_w_x = sin_a_x / n_w;
    const double cos_w_x = std::sqrt(1.0 - sin_w_x * sin_w_x);

    const double dynamic_scale = depth_scale * (std::pow(cos_w_x, 3) / std::pow(cos_a_x, 3));
    return z_raw * dynamic_scale;
}

std::pair<double, double> apply_snells_law_lateral(double z_true, double u, double v, double fx, double fy,
                                                   double n_w) {
    if (fx == 0.0 || fy == 0.0) {
        return {0.0, 0.0};
    }

    const double ux = u / fx;
    const double vy = v / fy;
    const double r_a = std::sqrt(ux * ux + vy * vy);
    if (r_a < 1e-5) {
        return {ux * z_true, vy * z_true};
    }

    const double sin_a = r_a / std::sqrt(1.0 + r_a * r_a);
    const double sin_w = sin_a / n_w;
    const double tan_w = sin_w / std::sqrt(1.0 - sin_w * sin_w);

    const double r_true = z_true * tan_w;
    return {r_true * ux / r_a, r_true * vy / r_a};
}

double percentile(std::vector<float>& values, double pct) {
    const double idx = pct / 100.0 * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(idx));
    std::nth_element(values.begin(), values.begin() + lo, values.end());
    const double v_lo = values[lo];
    if (lo + 1 >= values.size()) {
        return v_lo;
    }
    const double v_hi = *std::min_element(values.begin() + lo + 1, values.end());
    return v_lo + (v_hi - v_lo) * (idx - static_cast<double>(lo));
}

std::optional<RoiDepth> depth_from_roi(const cv::Mat& depth, const BBoxXYXY& bbox, bool nearest,
                                       double shrink_ratio, double nearest_percentile,
                                       double min_valid_fraction) {
    int x1_r, y1_r, x2_r, y2_r;
    if (!nearest) {
        // Shrink only for median mode to reduce background contamination; the low
        // percentile of nearest mode already ignores far background pixels.
        const double w_box = bbox.x2 - bbox.x1;
        const double h_box = bbox.y2 - bbox.y1;
        x1_r = trunc_int(bbox.x1 + w_box * (1.0 - shrink_ratio) / 2.0);
        y1_r = trunc_int(bbox.y1 + h_box * (1.0 - shrink_ratio) / 2.0);
        x2_r = trunc_int(bbox.x2 - w_box * (1.0 - shrink_ratio) / 2.0);
        y2_r = trunc_int(bbox.y2 - h_box * (1.0 - shrink_ratio) / 2.0);
    } else {
        x1_r = trunc_int(bbox.x1);
        y1_r = trunc_int(bbox.y1);
        x2_r = trunc_int(bbox.x2);
        y2_r = trunc_int(bbox.y2);
    }

    // Same clamping as the Python node (end index clamped to size-1, then used exclusively).
    x1_r = clamp_int(x1_r, 0, depth.cols - 1);
    x2_r = clamp_int(x2_r, 0, depth.cols - 1);
    y1_r = clamp_int(y1_r, 0, depth.rows - 1);
    y2_r = clamp_int(y2_r, 0, depth.rows - 1);

    const int roi_w = std::max(0, x2_r - x1_r);
    const int roi_h = std::max(0, y2_r - y1_r);
    const int roi_size = roi_w * roi_h;

    std::vector<float> valid;
    valid.reserve(static_cast<std::size_t>(roi_size));
    for (int y = y1_r; y < y1_r + roi_h; ++y) {
        const float* row = depth.ptr<float>(y);
        for (int x = x1_r; x < x1_r + roi_w; ++x) {
            const float d = row[x];
            if (std::isfinite(d) && d > 0.1f) {
                valid.push_back(d);
            }
        }
    }

    const int min_valid = std::max(1, trunc_int(min_valid_fraction * roi_size));
    if (static_cast<int>(valid.size()) < min_valid) {
        return std::nullopt;
    }

    double mean = 0.0;
    for (float d : valid) mean += d;
    mean /= static_cast<double>(valid.size());
    double variance = 0.0;
    for (float d : valid) variance += (d - mean) * (d - mean);
    variance /= static_cast<double>(valid.size());

    const double z_raw = percentile(valid, nearest ? nearest_percentile : 50.0);
    return RoiDepth{z_raw, variance};
}

const std::string& DepthEstimationConfig::mode_for(const std::string& label) const {
    const auto it = mode_by_label.find(label);
    return it != mode_by_label.end() ? it->second : default_mode;
}

std::optional<Position3D> compute_3d_position(const Compute3DInput& in, const DepthEstimationConfig& cfg,
                                              const CameraIntrinsics& K, const RefractionParams& refraction) {
    const std::string& mode = cfg.mode_for(in.label);
    const double n_w = refraction.lateral_scale;

    std::optional<Eigen::Vector3d> cam_pos;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.1;  // default 10cm variance

    // 1. Custom depth calculation
    if (mode != "zed_3d") {
        const BBoxXYXY& b = in.bbox;
        const double bbox_cx = (b.x1 + b.x2) / 2.0;
        const double bbox_cy = (b.y1 + b.y2) / 2.0;
        const double bbox_h = std::max(1.0, b.y2 - b.y1);
        const int margin = cfg.border_margin_px;
        const bool touching_top_or_bottom = (b.y1 <= margin) || (b.y2 >= in.img_h - margin);

        const bool is_depth_map_mode = mode == "depth_map_median" || mode == "depth_map_nearest";
        const auto to_pos = mode.find("_to_");

        if (is_depth_map_mode && in.depth != nullptr) {
            const auto roi = depth_from_roi(*in.depth, b, mode == "depth_map_nearest",
                                            cfg.depth_map_shrink_ratio, cfg.depth_map_nearest_percentile,
                                            cfg.depth_map_min_valid_fraction);
            if (roi) {
                const double u = bbox_cx - K.cx;
                const double v = bbox_cy - K.cy;
                const double z_m = apply_snells_law_depth(roi->z_raw, u, K.fx, refraction.depth_scale, n_w)
                                   + refraction.depth_offset;
                const auto [x_m, y_m] = apply_snells_law_lateral(z_m, u, v, K.fx, K.fy, n_w);
                cam_pos = Eigen::Vector3d(z_m, -x_m, -y_m);

                const double depth_var = std::max(
                    0.001, roi->variance * refraction.depth_scale * refraction.depth_scale);
                cov = Eigen::Matrix3d::Identity() * depth_var;
            }
        } else if (mode == "known_height" && cfg.known_heights.count(in.label)) {
            // Only trust known height when the box is fully visible and the AUV is roughly level.
            const double pitch_deg = std::abs(in.auv_pitch * 180.0 / M_PI);
            const double roll_deg = std::abs(in.auv_roll * 180.0 / M_PI);
            if (!touching_top_or_bottom && pitch_deg <= cfg.max_pitch_deg && roll_deg <= cfg.max_roll_deg) {
                const double h_m = cfg.known_heights.at(in.label);
                const double tan_a_half = (bbox_h / 2.0) / K.fy;
                const double sin_a_half = tan_a_half / std::sqrt(1.0 + tan_a_half * tan_a_half);
                const double sin_w_half = sin_a_half / kWaterRefractiveIndex;
                const double tan_w_half = sin_w_half / std::sqrt(1.0 - sin_w_half * sin_w_half);

                if (tan_w_half > 0.0) {
                    const double z_m = h_m / (2.0 * tan_w_half) + refraction.depth_offset;
                    const double u = bbox_cx - K.cx;
                    const double v = bbox_cy - K.cy;
                    const auto [x_m, y_m] = apply_snells_law_lateral(z_m, u, v, K.fx, K.fy, n_w);
                    cam_pos = Eigen::Vector3d(z_m, -x_m, -y_m);
                }
            }
        } else if (to_pos != std::string::npos && in.T_world_camera) {
            // "{top,center,bottom}_to_{plane}": intersect the pixel ray with a horizontal plane.
            const std::string proj_type = mode.substr(0, to_pos);
            const std::string plane_name = mode.substr(to_pos + 4);
            const auto plane_it = cfg.known_planes.find(plane_name);
            if (plane_it != cfg.known_planes.end()) {
                const double plane_z = plane_it->second;
                std::optional<double> px_y;
                if (proj_type == "bottom") {
                    if (b.y2 < in.img_h - margin) px_y = b.y2;
                } else if (proj_type == "top") {
                    if (b.y1 > margin) px_y = b.y1;
                } else if (proj_type == "center") {
                    if (!touching_top_or_bottom) px_y = bbox_cy;
                }

                if (px_y) {
                    const double u = bbox_cx - K.cx;
                    const double v = *px_y - K.cy;
                    // True ray direction at depth 1 after refraction, in ROS camera frame.
                    const auto [x_ray, y_ray] = apply_snells_law_lateral(1.0, u, v, K.fx, K.fy, n_w);
                    const Eigen::Vector4d v_c(1.0, -x_ray, -y_ray, 0.0);

                    const Eigen::Vector4d v_w = *in.T_world_camera * v_c;
                    const Eigen::Vector4d p0_w = *in.T_world_camera * Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);

                    double t = -1.0;
                    if (std::abs(v_w.z()) > 1e-6) {
                        t = (plane_z - p0_w.z()) / v_w.z();
                    }
                    if (t > 0.0) {  // intersection strictly in front of the camera
                        cam_pos = t * v_c.head<3>();
                    }
                }
            }
        }
    }

    // 2. Fallback to the ZED SDK 3D point
    if (!cam_pos && in.zed_object) {
        const Eigen::Vector3d& p = in.zed_object->position;
        if (!p.allFinite() || p.x() <= 0.0) {
            return std::nullopt;
        }

        // Recover the raw pixel angles from the ZED point (X forward, Y left, Z up).
        const double u = (-p.y() / p.x()) * K.fx;
        const double v = (-p.z() / p.x()) * K.fy;

        double z_m = apply_snells_law_depth(p.x(), u, K.fx, refraction.depth_scale, n_w)
                     + refraction.depth_offset;
        z_m = std::max(z_m, 0.01);
        const auto [x_m, y_m] = apply_snells_law_lateral(z_m, u, v, K.fx, K.fy, n_w);
        cam_pos = Eigen::Vector3d(z_m, -x_m, -y_m);

        const auto& c = in.zed_object->covariance;
        cov << c[0], c[1], c[2],
               c[1], c[3], c[4],
               c[2], c[4], c[5];
        cov *= refraction.depth_scale * refraction.depth_scale;
    }

    if (!cam_pos) {
        return std::nullopt;
    }

    // Distance/size-dependent noise keeps object_map's Mahalanobis gating from becoming
    // hypersensitive at close range. TODO: use the actual object's size.
    constexpr double max_physical_size = 0.5;
    const double dynamic_noise = 0.3 + 0.1 * cam_pos->norm() + 0.2 * max_physical_size;
    cov += Eigen::Matrix3d::Identity() * dynamic_noise;

    return Position3D{*cam_pos, cov};
}

}  // namespace vision
