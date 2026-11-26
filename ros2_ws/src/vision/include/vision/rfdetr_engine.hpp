#pragma once

// Thin RF-DETR TensorRT wrapper shared by the front and down camera detectors.
// Replaces utils.load_model / utils.get_detections from the Python nodes.
// TensorRT / CUDA headers stay behind the PIMPL so nodes don't need them.

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "detection_geometry.hpp"

namespace vision {

struct Detection2D {
    BBoxXYXY bbox;  // pixels in the input image
    int class_id = -1;
    float score = 0.0f;
    // Segmentation models only: CV_8UC1 mask (0/255) the size of the input image. Empty otherwise.
    cv::Mat mask;
};

struct RfDetrEngineOptions {
    // Model package directory containing weights.onnx and/or engine.plan.
    std::filesystem::path model_dir;
    // rfdetr_build executable used to compile weights.onnx -> engine.plan on this GPU.
    std::filesystem::path engine_builder;
    std::string precision = "fp16";
    // Logit slot dropped as "no object". Our datasets are 0-indexed with no background
    // slot (gate == 0), so nothing is dropped; COCO-style exports would use 0.
    int background_class_index = -1;
    std::function<void(const std::string&)> log_info = [](const std::string&) {};
};

class RfDetrEngine {
public:
    explicit RfDetrEngine(const RfDetrEngineOptions& options);
    ~RfDetrEngine();

    RfDetrEngine(const RfDetrEngine&) = delete;
    RfDetrEngine& operator=(const RfDetrEngine&) = delete;

    // `image` is BGR (OpenCV convention); it is converted to RGB on the GPU.
    std::vector<Detection2D> detect(const cv::Mat& image, float threshold);

    int input_size() const;
    // True when the engine has a "masks" output (RF-DETR segmentation export).
    bool has_masks() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vision
