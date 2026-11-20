#pragma once

// Camera-frame 3D localisation of 2D detections, ported from
// vision/object_detection/utils.py and the former Python FrontCamObjectDetectorNode._compute_3d_position.
// Kept free of ROS / CUDA / ZED so it builds and is unit-tested on every machine.

#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace vision {

struct BBoxXYXY {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

// Flat-port refraction: scale ZED depth by the 1D horizontal tangential virtual depth.
double apply_snells_law_depth(double z_raw, double u, double fx, double depth_scale = 1.42, double n_w = 1.333);

// Flat-port refraction: un-project a pixel offset (u, v) at true depth z_true into metric (x, y).
std::pair<double, double> apply_snells_law_lateral(double z_true, double u, double v, double fx, double fy,
                                                   double n_w = 1.333);

// numpy.percentile (linear interpolation) and numpy.median equivalents. `values` is reordered.
double percentile(std::vector<float>& values, double pct);

struct RoiDepth {
    double z_raw;     // median or nearest-percentile depth in metres
    double variance;  // population variance of the valid ROI pixels
};

// Robust depth inside a bbox of a CV_32FC1 metre depth map (NaN/inf/<=0.1 m ignored).
// nearest=false -> median of the bbox shrunk by shrink_ratio, nearest=true -> `nearest_percentile`.
std::optional<RoiDepth> depth_from_roi(const cv::Mat& depth, const BBoxXYXY& bbox, bool nearest,
                                       double shrink_ratio, double nearest_percentile,
                                       double min_valid_fraction);

struct DepthEstimationConfig {
    std::string default_mode = "depth_map_median";
    std::unordered_map<std::string, std::string> mode_by_label;
    std::unordered_map<std::string, double> known_heights;
    std::unordered_map<std::string, double> known_planes;
    double depth_map_shrink_ratio = 0.8;
    double depth_map_nearest_percentile = 15.0;
    double depth_map_min_valid_fraction = 0.05;
    double max_pitch_deg = 20.0;
    double max_roll_deg = 20.0;
    int border_margin_px = 0;

    const std::string& mode_for(const std::string& label) const;
};

struct RefractionParams {
    double depth_scale = 1.0;
    double lateral_scale = 1.333;  // water refractive index used for lateral un-projection
    double depth_offset = 0.0;
};

// 3D point and covariance from the ZED SDK custom-box pipeline (zed_3d mode / fallback).
// position is in the ZED camera frame (RIGHT_HANDED_Z_UP_X_FWD); covariance is the
// packed upper triangle [xx, xy, xz, yy, yz, zz].
struct ZedObject3D {
    Eigen::Vector3d position;
    std::array<double, 6> covariance;
};

struct Position3D {
    Eigen::Vector3d position;  // ROS camera frame: X forward, Y left, Z up
    Eigen::Matrix3d covariance;
};

struct Compute3DInput {
    std::string label;
    BBoxXYXY bbox;
    int img_h = 0;
    std::optional<Eigen::Matrix4d> T_world_camera;
    double auv_roll = 0.0;
    double auv_pitch = 0.0;
    const cv::Mat* depth = nullptr;  // CV_32FC1 metres, may be null (sim without ZED)
    std::optional<ZedObject3D> zed_object;
};

std::optional<Position3D> compute_3d_position(const Compute3DInput& in, const DepthEstimationConfig& cfg,
                                              const CameraIntrinsics& K, const RefractionParams& refraction);

}  // namespace vision
