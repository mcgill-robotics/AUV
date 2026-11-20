#include "rfdetr_engine.hpp"

#include <cstdlib>
#include <stdexcept>

#include "rfdetr/core/cuda_preprocess.cuh"
#include "rfdetr/core/postprocess.hpp"
#include "rfdetr/core/trt_session.hpp"

namespace fs = std::filesystem;

namespace vision {

namespace {

// Rebuild when the engine is missing or older than the ONNX it came from
// (replaces the MD5 cache check in utils.load_model).
bool engine_is_stale(const fs::path& onnx, const fs::path& engine) {
    if (!fs::exists(engine)) return true;
    return fs::exists(onnx) && fs::last_write_time(onnx) > fs::last_write_time(engine);
}

fs::path ensure_engine(const RfDetrEngineOptions& opts) {
    const fs::path onnx = opts.model_dir / "weights.onnx";
    // Own file name: the Python inference_models backend writes its own engine.plan in the same folder.
    const fs::path engine = opts.model_dir / ("rfdetr_cpp_" + opts.precision + ".engine");

    if (!engine_is_stale(onnx, engine)) return engine;
    if (!fs::exists(onnx)) {
        throw std::runtime_error("No " + engine.filename().string() + " or weights.onnx in " + opts.model_dir.string());
    }
    if (!fs::exists(opts.engine_builder)) {
        throw std::runtime_error("Engine builder not found: " + opts.engine_builder.string());
    }

    opts.log_info("Building TensorRT engine from " + onnx.string() + " (first run takes ~1 min)...");
    const std::string cmd = "\"" + opts.engine_builder.string() + "\" --onnx \"" + onnx.string()
                            + "\" --engine \"" + engine.string() + "\" --precision " + opts.precision;
    if (std::system(cmd.c_str()) != 0 || !fs::exists(engine)) {
        throw std::runtime_error("Failed to build TensorRT engine with: " + cmd);
    }
    opts.log_info("Wrote " + engine.string());
    return engine;
}

}  // namespace

struct RfDetrEngine::Impl {
    rfdetr::TrtSession session;
    std::unique_ptr<rfdetr::ImagePreprocessor> preprocessor;
    std::string input_name = "input";
    std::string dets_name;
    std::string labels_name;
    int input_size = 0;
    rfdetr::PostprocessParams pp;
    std::vector<float> h_dets;
    std::vector<float> h_labels;
    // Segmentation models only (third "masks" output, [1, queries, mask_h, mask_w] logits).
    std::string masks_name;
    int mask_h = 0;
    int mask_w = 0;
    std::vector<float> h_masks;

    Impl(const fs::path& engine_path, int background_class_index) : session(engine_path) {
        const auto* in = session.find(input_name);
        if (in == nullptr) {
            in = &session.bindings().at(session.input_indices().at(0));
            input_name = in->name;
        }
        if (in->shape.nbDims != 4 || in->shape.d[2] != in->shape.d[3]) {
            throw std::runtime_error("RF-DETR engine input must be square NCHW");
        }
        input_size = static_cast<int>(in->shape.d[2]);

        const auto* dets = session.find("dets");
        const auto* labels = session.find("labels");
        if (dets == nullptr || labels == nullptr) {
            const auto& outs = session.output_indices();
            if (outs.size() < 2) throw std::runtime_error("RF-DETR engine has fewer than 2 outputs");
            dets = &session.bindings()[outs[0]];
            labels = &session.bindings()[outs[1]];
        }
        dets_name = dets->name;
        labels_name = labels->name;

        const auto* masks = session.find("masks");
        if (masks == nullptr && session.output_indices().size() >= 3) {
            masks = &session.bindings()[session.output_indices()[2]];
        }
        if (masks != nullptr && masks->shape.nbDims >= 4) {
            masks_name = masks->name;
            mask_h = static_cast<int>(masks->shape.d[masks->shape.nbDims - 2]);
            mask_w = static_cast<int>(masks->shape.d[masks->shape.nbDims - 1]);
        }

        pp.num_queries = static_cast<int>(dets->shape.d[dets->shape.nbDims - 2]);
        pp.num_classes_with_bg = static_cast<int>(labels->shape.d[labels->shape.nbDims - 1]);
        pp.topk = pp.num_queries;
        pp.bg_class_index = background_class_index;
        h_dets.resize(static_cast<std::size_t>(pp.num_queries) * 4);
        h_labels.resize(static_cast<std::size_t>(pp.num_queries) * pp.num_classes_with_bg);
        if (!masks_name.empty()) {
            h_masks.resize(static_cast<std::size_t>(pp.num_queries) * mask_h * mask_w);
        }

        // ImageNet mean/std (defaults) match our models' inference_config.json.
        preprocessor = std::make_unique<rfdetr::ImagePreprocessor>(input_size, /*src_is_bgr=*/true);
    }
};

RfDetrEngine::RfDetrEngine(const RfDetrEngineOptions& options) {
    const fs::path engine = ensure_engine(options);
    impl_ = std::make_unique<Impl>(engine, options.background_class_index);
    options.log_info("Loaded RF-DETR engine " + engine.string() + " (input " + std::to_string(impl_->input_size)
                     + "px, " + std::to_string(impl_->pp.num_classes_with_bg) + " class logits)");

    // Warm-up flushes lazy CUDA / TensorRT initialisation out of the first real frame.
    cv::Mat warmup(impl_->input_size, impl_->input_size, CV_8UC3, cv::Scalar(0, 0, 0));
    (void)detect(warmup, 1.0f);
}

RfDetrEngine::~RfDetrEngine() = default;

int RfDetrEngine::input_size() const { return impl_->input_size; }

bool RfDetrEngine::has_masks() const { return !impl_->masks_name.empty(); }

std::vector<Detection2D> RfDetrEngine::detect(const cv::Mat& image, float threshold) {
    if (image.empty() || image.type() != CV_8UC3) {
        throw std::invalid_argument("RfDetrEngine::detect expects a non-empty CV_8UC3 BGR image");
    }

    auto& s = impl_->session;
    impl_->preprocessor->process(image, static_cast<float*>(s.device_buffer(impl_->input_name)), s.stream());
    s.infer();
    s.get_output_f32(impl_->dets_name, impl_->h_dets.data(), impl_->h_dets.size());
    s.get_output_f32(impl_->labels_name, impl_->h_labels.data(), impl_->h_labels.size());

    impl_->pp.threshold = threshold;
    std::vector<int> query_idx;
    rfdetr::Detections raw = rfdetr::decode_detections_with_queries(
        impl_->h_dets.data(), impl_->h_labels.data(), image.cols, image.rows, impl_->pp, query_idx);
    if (has_masks()) {
        // Full-resolution CV_8UC1 masks (0/255), upsampled and thresholded on the GPU.
        s.get_output_f32(impl_->masks_name, impl_->h_masks.data(), impl_->h_masks.size());
        rfdetr::decode_masks(impl_->h_masks.data(), impl_->mask_h, impl_->mask_w, query_idx, image.cols, image.rows,
                             raw, s.stream());
    }

    std::vector<Detection2D> out;
    out.reserve(raw.size());
    for (auto& d : raw) {
        out.push_back({{d.box.x1, d.box.y1, d.box.x2, d.box.y2}, d.class_id, d.score, std::move(d.mask)});
    }
    return out;
}

}  // namespace vision
