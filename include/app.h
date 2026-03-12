#ifndef APP_H
#define APP_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "image_stitcher.h"
#include "sensor_data_interface.h"
#include "stitching_param_generator.h"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"

class App {
public:
    App(const std::vector<std::string>& video_files, const std::string& output_folder, const std::string& file_name, double fps, bool dry_run, bool use_lir, bool use_calibration, const std::string& feature_method, bool use_feature_mask);
    void run_stitching();

private:
    SensorDataInterface sensor_data_interface_;
    ImageStitcher image_stitcher_;
    cv::VideoWriter video_writer_;
    std::string output_folder_;
    std::string file_name_;
    double fps_;
    int total_cols_;
    bool dry_run_;
    bool use_lir_;
    bool use_calibration_;
    std::string feature_method_;  // "SIFT", "ORB", or "LSD"
    bool use_feature_mask_;
    cv::Ptr<cv::detail::Blender> blender_;
    cv::Ptr<cv::detail::ExposureCompensator> exposure_compensator_;
    std::vector<cv::Point> corners_;
    std::vector<cv::Size> warped_sizes_;
    std::vector<cv::Size> sizes_;
    std::vector<cv::UMat> seam_masks_;  // Precomputed seam masks for smooth blending
};

#endif // APP_H