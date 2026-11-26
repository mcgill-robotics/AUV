// Parity tests for detection_geometry against the Python implementation in
// vision/object_detection/utils.py. Golden values were produced by running the
// original Python functions (and numpy.median / numpy.percentile / numpy.var).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

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

// ---------------------------------------------------------------------------
// Down camera. Golden values from the Python DownCamObjectDetectorNode logic
// (cv2 4.11 moments / findContours / minAreaRect, tf_transformations quaternions).
// ---------------------------------------------------------------------------

namespace {

cv::Mat rotated_rect_mask() {
    cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);
    const std::vector<cv::Point> poly = {{200, 150}, {420, 230}, {390, 310}, {170, 230}};
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(255));
    return mask;
}

}  // namespace

TEST(DownCamMask, CentroidAndYawMatchPython) {
    const auto g = analyze_down_cam_mask(rotated_rect_mask(), {170, 150, 420, 310}, 640, 480, 10);
    EXPECT_NEAR(g.center_x, 294.9455691837841, 1e-6);
    EXPECT_NEAR(g.center_y, 229.9802965371164, 1e-6);
    ASSERT_TRUE(g.yaw.has_value());
    EXPECT_NEAR(*g.yaw, 0.3487709869922537, 1e-6);
}

TEST(DownCamMask, NoYawWhenTouchingBorderOrWithoutMask) {
    const auto border = analyze_down_cam_mask(rotated_rect_mask(), {5, 150, 420, 310}, 640, 480, 10);
    EXPECT_NEAR(border.center_x, 294.9455691837841, 1e-6);
    EXPECT_FALSE(border.yaw.has_value());

    const auto no_mask = analyze_down_cam_mask(cv::Mat(), {100, 100, 200, 140}, 640, 480, 10);
    EXPECT_DOUBLE_EQ(no_mask.center_x, 150.0);
    EXPECT_DOUBLE_EQ(no_mask.center_y, 120.0);
    EXPECT_FALSE(no_mask.yaw.has_value());
    EXPECT_FLOAT_EQ(no_mask.box[2].x, 200.0f);
}

TEST(DownCamProjection, MatchesPython) {
    const CameraIntrinsics K{692.8, 692.8, 320.0, 240.0};
    Eigen::Matrix4d T;
    T << -0.014980912390756801, -0.7175040505719174, 0.6963932148411786, 2.9029066382920936,
        0.05631138611640302, 0.6947605828651856, 0.717033304868384, -1.4986196312200568,
        -0.9983008564844517, 0.04995668033125288, 0.029995500207383924, -0.8124889428811639,
        0.0, 0.0, 0.0, 1.0;

    const auto table = project_down_cam_to_plane(410.0, 300.0, K, 1.333, T, -2.1 + 0.67);
    ASSERT_TRUE(table.has_value());
    EXPECT_NEAR(table->x(), 0.6143886377294329, 1e-9);
    EXPECT_NEAR(table->y(), -0.05955875132322846, 1e-9);
    EXPECT_NEAR(table->z(), -0.03970583421548563, 1e-9);

    const auto floor = project_down_cam_to_plane(100.0, 50.0, K, 1.333, T, -2.1);
    ASSERT_TRUE(floor.has_value());
    EXPECT_NEAR(floor->x(), 1.3125993404469836, 1e-9);
    EXPECT_NEAR(floor->y(), 0.3013105811106241, 1e-9);
    EXPECT_NEAR(floor->z(), 0.260222774595539, 1e-9);

    // Plane above the camera: intersection is behind it.
    EXPECT_FALSE(project_down_cam_to_plane(320.0, 240.0, K, 1.333, T, 0.5).has_value());
}

TEST(DownCamOrientation, MatchesPythonQuaternionMultiply) {
    const struct { double yaw, x, y, z, w; } cases[] = {
        {0.0, 0.5, -0.5, -0.4999999999999999, 0.5000000000000001},
        {0.6, 0.6254283478934728, -0.3299081412321332, -0.6254283478934727, 0.32990814123213325},
        {-2.3, -0.2521382496881819, -0.6606256905723391, 0.2521382496881818, 0.6606256905723392},
    };
    for (const auto& c : cases) {
        const auto q = down_cam_orientation_from_yaw(c.yaw);
        EXPECT_NEAR(q.x(), c.x, 1e-12) << c.yaw;
        EXPECT_NEAR(q.y(), c.y, 1e-12) << c.yaw;
        EXPECT_NEAR(q.z(), c.z, 1e-12) << c.yaw;
        EXPECT_NEAR(q.w(), c.w, 1e-12) << c.yaw;
    }
}
