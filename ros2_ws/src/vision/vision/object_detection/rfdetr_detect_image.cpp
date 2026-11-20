// Smoke test for RfDetrEngine without ROS:
//   ros2 run vision rfdetr_detect_image <model_dir> <image> [threshold] [out.jpg]
// Prints every detection and writes an annotated copy of the image.

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rfdetr_engine.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " <model_dir> <image> [threshold=0.5] [out=annotated.jpg]\n";
        return 2;
    }
    const fs::path model_dir = argv[1];
    const std::string image_path = argv[2];
    const float threshold = argc > 3 ? std::stof(argv[3]) : 0.5f;
    const std::string out_path = argc > 4 ? argv[4] : "annotated.jpg";

    std::vector<std::string> class_names;
    std::ifstream names_file(model_dir / "class_names.txt");
    for (std::string line; std::getline(names_file, line);) {
        if (!line.empty()) class_names.push_back(line);
    }

    vision::RfDetrEngineOptions opts;
    opts.model_dir = model_dir;
    // Look next to the invoked path first: with --symlink-install, /proc/self/exe resolves into build/.
    opts.engine_builder = fs::absolute(argv[0]).parent_path() / "rfdetr_build";
    if (!fs::exists(opts.engine_builder)) {
        opts.engine_builder = fs::canonical("/proc/self/exe").parent_path() / "rfdetr_build";
    }
    opts.log_info = [](const std::string& msg) { std::cout << msg << "\n"; };
    vision::RfDetrEngine engine(opts);

    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "cannot read " << image_path << "\n";
        return 1;
    }

    const auto detections = engine.detect(image, threshold);
    std::cout << detections.size() << " detection(s)\n";
    for (const auto& d : detections) {
        const std::string label = d.class_id >= 0 && d.class_id < static_cast<int>(class_names.size())
                                      ? class_names[d.class_id]
                                      : "class_" + std::to_string(d.class_id);
        std::cout << "  " << label << " " << d.score << " [" << d.bbox.x1 << ", " << d.bbox.y1 << ", "
                  << d.bbox.x2 << ", " << d.bbox.y2 << "]\n";

        const cv::Point tl(static_cast<int>(d.bbox.x1), static_cast<int>(d.bbox.y1));
        const cv::Point br(static_cast<int>(d.bbox.x2), static_cast<int>(d.bbox.y2));
        cv::rectangle(image, tl, br, cv::Scalar(0, 255, 0), 2);
        cv::putText(image, label + " " + cv::format("%.2f", d.score), tl + cv::Point(0, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    cv::imwrite(out_path, image);
    std::cout << "wrote " << out_path << "\n";
    return 0;
}
