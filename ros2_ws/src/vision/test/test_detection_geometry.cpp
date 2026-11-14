// Parity tests for detection_geometry against the Python implementation in
// vision/object_detection/utils.py. Golden values were produced by running the
// original Python functions (and numpy.median / numpy.percentile / numpy.var).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "detection_geometry.hpp"

using namespace vision;

namespace {

constexpr double kTol = 1e-9;

DepthEstimationConfig make_config() {
    DepthEstimationConfig cfg;
    cfg.default_mode = "depth_map_median";
    cfg.mode_by_label = {
        {"gate", "depth_map_nearest"},
        {"octagon", "center_to_surface"},
        {"pipe", "bottom_to_floor"},
        {"buoy", "known_height"},
        {"bin", "zed_3d"},
    };
    cfg.known_heights = {{"buoy", 0.6}};
    cfg.known_planes = {{"surface", 0.0}, {"floor", -4.0}};
    cfg.depth_map_shrink_ratio = 0.8;
    cfg.depth_map_nearest_percentile = 10.0;
    cfg.depth_map_min_valid_fraction = 0.25;
    cfg.border_margin_px = 5;
    return cfg;
}

const CameraIntrinsics kK{500.0, 500.0, 320.0, 240.0};
const RefractionParams kRefraction{1.42, 1.333, 0.05};

}  // namespace

TEST(SnellsLaw, DepthMatchesPython) {
    struct Case { double z, u, fx, scale, n, expected; };
    const Case cases[] = {
        {2.0, 0.0, 500.0, 1.42, 1.333, 2.84000000000186},
        {2.0, 300.0, 500.0, 1.42, 1.333, 3.53624895518099},
        {5.5, -250.0, 520.0, 1.0, 1.34, 6.36619086252094},
        {3.0, 10.0, 0.0, 1.2, 1.333, 3.6},
    };
    for (const auto& c : cases) {
        EXPECT_NEAR(apply_snells_law_depth(c.z, c.u, c.fx, c.scale, c.n), c.expected, kTol);
    }
}

TEST(SnellsLaw, LateralMatchesPython) {
    struct Case { double z, u, v, fx, fy, n, x, y; };
    const Case cases[] = {
        {2.0, 0.0, 0.0, 500.0, 500.0, 1.333, 0.0, 0.0},
        {2.0, 300.0, -120.0, 500.0, 505.0, 1.333, 0.82799269885819, -0.327917900537897},
        {4.0, -250.0, 200.0, 520.0, 520.0, 1.34, -1.3279389996042, 1.06235119968336},
        {1.0, 0.0001, 0.0, 500.0, 500.0, 1.333, 2e-07, 0.0},
    };
    for (const auto& c : cases) {
        const auto [x, y] = apply_snells_law_lateral(c.z, c.u, c.v, c.fx, c.fy, c.n);
        EXPECT_NEAR(x, c.x, kTol);
        EXPECT_NEAR(y, c.y, kTol);
    }
    const auto [x0, y0] = apply_snells_law_lateral(1.0, 10.0, 10.0, 0.0, 500.0);
    EXPECT_EQ(x0, 0.0);
    EXPECT_EQ(y0, 0.0);
}

TEST(Percentile, MatchesNumpy) {
    const std::vector<float> values = {
        5.27721262f, 2.52340031f, 0.807301402f, 0.623957276f, 6.59952688f, 7.34566689f, 5.04976845f,
        5.97122431f, 4.57718754f, 7.5130434f, 6.61890173f, 0.520538747f, 6.93053198f, 0.751891792f,
        5.97241592f, 1.81741714f, 6.97384214f, 4.56095934f, 2.74783921f, 3.67015409f, 0.712397516f,
        1.43212461f, 5.52968311f, 5.35392141f, 5.11538839f, 3.3775816f, 7.97907448f, 7.85626507f,
        5.64156485f, 5.37844467f, 5.66335058f, 3.41691065f, 1.51322377f, 5.91116238f, 4.44015741f,
        2.82681417f, 4.14376497f};
    auto v = values;
    EXPECT_NEAR(percentile(v, 50.0), 5.04976844787598, 1e-6);
    v = values;
    EXPECT_NEAR(percentile(v, 10.0), 0.785137557983398, 1e-6);
    v = values;
    EXPECT_NEAR(percentile(v, 15.0), 1.46456427574158, 1e-6);

    std::vector<float> even = {4.0f, 1.0f, 3.0f, 2.0f};
    EXPECT_NEAR(percentile(even, 50.0), 2.5, 1e-9);  // numpy.median averages the middle pair
}

TEST(DepthFromRoi, IgnoresInvalidPixelsAndShrinksForMedian) {
    cv::Mat depth(100, 100, CV_32FC1, cv::Scalar(9.0f));
    depth(cv::Rect(40, 40, 20, 20)).setTo(3.0f);  // object
    depth.at<float>(50, 50) = std::numeric_limits<float>::quiet_NaN();
    depth.at<float>(51, 51) = 0.05f;  // below 0.1 m, ignored

    // bbox 35..65 shrunk by 0.8 -> 38..62: mostly object, background ring is a minority.
    const auto roi = depth_from_roi(depth, {35, 35, 65, 65}, false, 0.8, 10.0, 0.25);
    ASSERT_TRUE(roi.has_value());
    EXPECT_FLOAT_EQ(roi->z_raw, 3.0);
    EXPECT_GT(roi->variance, 0.0);

    // Nearest mode picks the closest surface even when background dominates.
    const auto near = depth_from_roi(depth, {0, 0, 99, 99}, true, 0.8, 10.0, 0.01);
    ASSERT_TRUE(near.has_value());
    EXPECT_FLOAT_EQ(near->z_raw, 9.0);  // only ~4% of pixels are object -> p10 is background
    const auto near_small = depth_from_roi(depth, {30, 30, 70, 70}, true, 0.8, 10.0, 0.01);
    ASSERT_TRUE(near_small.has_value());
    EXPECT_FLOAT_EQ(near_small->z_raw, 3.0);
}

TEST(DepthFromRoi, RejectsWhenTooFewValidPixels) {
    cv::Mat depth(50, 50, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    depth.at<float>(25, 25) = 2.0f;
    EXPECT_FALSE(depth_from_roi(depth, {10, 10, 40, 40}, false, 0.8, 10.0, 0.25).has_value());
}

TEST(Compute3D, DepthMapMedianUsesSnellCorrection) {
    cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(2.0f));
    Compute3DInput in;
    in.label = "unlisted";  // falls back to default_mode = depth_map_median
    in.bbox = {400, 200, 480, 280};
    in.img_h = 480;
    in.depth = &depth;

    const auto res = compute_3d_position(in, make_config(), kK, kRefraction);
    ASSERT_TRUE(res.has_value());

    const double u = 440.0 - kK.cx;
    const double v = 240.0 - kK.cy;
    const double z = apply_snells_law_depth(2.0, u, kK.fx, 1.42, 1.333) + 0.05;
    const auto [x, y] = apply_snells_law_lateral(z, u, v, kK.fx, kK.fy, 1.333);
    EXPECT_NEAR(res->position.x(), z, kTol);
    EXPECT_NEAR(res->position.y(), -x, kTol);
    EXPECT_NEAR(res->position.z(), -y, kTol);

    // Uniform depth -> variance clamped to 0.001, plus the dynamic noise term.
    const double noise = 0.3 + 0.1 * res->position.norm() + 0.2 * 0.5;
    EXPECT_NEAR(res->covariance(0, 0), 0.001 + noise, kTol);
    EXPECT_NEAR(res->covariance(0, 1), 0.0, kTol);
}

TEST(Compute3D, RayPlaneIntersectsFloor) {
    // Camera 1 m below the surface, level, looking along world +X.
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(2, 3) = -1.0;

    Compute3DInput in;
    in.label = "pipe";  // bottom_to_floor, floor at -4 m
    in.bbox = {300, 250, 340, 400};
    in.img_h = 480;
    in.T_world_camera = T;

    const auto res = compute_3d_position(in, make_config(), kK, kRefraction);
    ASSERT_TRUE(res.has_value());
    EXPECT_NEAR(res->position.z(), -3.0, 1e-9);  // hits the plane 3 m below the camera
    EXPECT_GT(res->position.x(), 0.0);

    // A pixel above the horizon never reaches the floor.
    in.bbox = {300, 50, 340, 100};
    EXPECT_FALSE(compute_3d_position(in, make_config(), kK, kRefraction).has_value());
}

TEST(Compute3D, BorderTouchingBoxIsRejectedForPlaneProjection) {
    Compute3DInput in;
    in.label = "pipe";
    in.bbox = {300, 250, 340, 478};  // bottom within the 5 px margin
    in.img_h = 480;
    in.T_world_camera = Eigen::Matrix4d::Identity();
    EXPECT_FALSE(compute_3d_position(in, make_config(), kK, kRefraction).has_value());
}

TEST(Compute3D, KnownHeightRespectsAttitudeLimits) {
    Compute3DInput in;
    in.label = "buoy";
    in.bbox = {300, 200, 340, 260};
    in.img_h = 480;

    const auto level = compute_3d_position(in, make_config(), kK, kRefraction);
    ASSERT_TRUE(level.has_value());
    const double tan_a = 30.0 / kK.fy;
    const double sin_w = (tan_a / std::sqrt(1.0 + tan_a * tan_a)) / 1.333;
    const double tan_w = sin_w / std::sqrt(1.0 - sin_w * sin_w);
    EXPECT_NEAR(level->position.x(), 0.6 / (2.0 * tan_w) + 0.05, kTol);

    in.auv_pitch = 30.0 * M_PI / 180.0;
    EXPECT_FALSE(compute_3d_position(in, make_config(), kK, kRefraction).has_value());
}

TEST(Compute3D, ZedFallbackAndInvalidPoints) {
    Compute3DInput in;
    in.label = "bin";  // zed_3d
    in.bbox = {300, 200, 340, 260};
    in.img_h = 480;
    in.zed_object = ZedObject3D{Eigen::Vector3d(3.0, 0.5, -0.2), {0.01, 0.0, 0.0, 0.02, 0.0, 0.03}};

    const auto res = compute_3d_position(in, make_config(), kK, kRefraction);
    ASSERT_TRUE(res.has_value());
    const double u = (-0.5 / 3.0) * kK.fx;
    const double z = apply_snells_law_depth(3.0, u, kK.fx, 1.42, 1.333) + 0.05;
    EXPECT_NEAR(res->position.x(), z, kTol);
    const double noise = 0.3 + 0.1 * res->position.norm() + 0.1;
    EXPECT_NEAR(res->covariance(1, 1), 0.02 * 1.42 * 1.42 + noise, kTol);

    in.zed_object->position = Eigen::Vector3d(std::nan(""), 0.0, 0.0);
    EXPECT_FALSE(compute_3d_position(in, make_config(), kK, kRefraction).has_value());
    in.zed_object->position = Eigen::Vector3d(-1.0, 0.0, 0.0);
    EXPECT_FALSE(compute_3d_position(in, make_config(), kK, kRefraction).has_value());
}
