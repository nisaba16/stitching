// Created by s1nh.org.

#ifndef IMAGE_STITCHING_STITCHING_PARAM_GENERATER_H
#define IMAGE_STITCHING_STITCHING_PARAM_GENERATER_H

#include "opencv2/opencv.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"

class StitchingParamGenerator {
public:
  explicit StitchingParamGenerator(const std::vector<cv::Mat>& image_vector, bool use_calibration = true, const std::string& feature_method = "SIFT", bool use_feature_mask = false);

  void GetReprojParams(std::vector<cv::UMat>& undist_xmap_vector,
                       std::vector<cv::UMat>& undist_ymap_vector,
                       std::vector<cv::UMat>& reproj_xmap_vector,
                       std::vector<cv::UMat>& reproj_ymap_vector,
                       std::vector<cv::Rect>& projected_image_roi_refined_vect);

  void InitCameraParam();

  void InitWarper();


  void InitUndistortMap();
  cv::Ptr<cv::detail::Blender> GetBlender() { return blender_; }
  std::vector<cv::Point> GetCorners() { return corners_; }
  std::vector<cv::Size> GetWarpedSizes() { return image_warped_size_vector_; }
  cv::Ptr<cv::detail::ExposureCompensator> GetExposureCompensator() { return exposure_compensator_; }
  std::vector<cv::UMat> GetSeamMasks() { return mask_warped_vector_; }

private:
  bool use_calibration_;
  std::string feature_method_;  // "SIFT", "ORB", or "LSD"
  bool use_feature_mask_;
  // Default command line args
  std::vector<cv::String> img_names;
  bool try_cuda = false;
  float conf_thresh = 0.3f;  // Very low threshold to accept weak matches
  float match_conf = 0.4f;  // Standard matching threshold
  std::string matcher_type = "homography";
  std::string estimator_type = "homography";
  std::string ba_cost_func = "ray";  // Better for line alignment
  std::string ba_refine_mask = "_____";  // Refine all parameters (overridden to ____x when using calibration)
  cv::detail::WaveCorrectKind wave_correct = cv::detail::WAVE_CORRECT_HORIZ;
  bool save_graph = false;
  std::string save_graph_to;
  std::string warp_type = "cylindrical";  // cylindrical works cleanly for side-by-side cameras at large angles
  int expos_comp_type = cv::detail::ExposureCompensator::GAIN_BLOCKS;
  int expos_comp_nr_feeds = 1;
  int expos_comp_nr_filtering = 2;
  int expos_comp_block_size = 32;
  std::string seam_find_type = "dp_colorgrad";  // Better for exposure/color differences at seam
  int blend_type = cv::detail::Blender::MULTI_BAND;
  int timelapse_type = cv::detail::Timelapser::AS_IS;
  float blend_strength = 5;
  std::string result_name = "../results/result.jpg";
  bool timelapse = false;
  int range_width = -1;

  // Variables
  size_t num_img_;
  cv::Point full_image_size_;

  std::vector<cv::Mat> image_vector_;
  std::vector<cv::UMat> mask_vector_, mask_warped_vector_;
  std::vector<cv::Size> image_size_vector_, image_warped_size_vector_;
  std::vector<cv::UMat> reproj_xmap_vector_, reproj_ymap_vector_;
  std::vector<cv::UMat> undist_xmap_vector_, undist_ymap_vector_;

  std::vector<cv::detail::CameraParams> camera_params_vector_;
  std::vector<cv::Rect> projected_image_roi_refined_vect_;
  cv::Ptr<cv::detail::RotationWarper> rotation_warper_;
  cv::Ptr<cv::detail::Timelapser> timelapser_;
  cv::Ptr<cv::detail::Blender> blender_;
  cv::Ptr<cv::detail::ExposureCompensator> exposure_compensator_;
  std::vector<cv::Point> corners_;
};

#endif //IMAGE_STITCHING_STITCHING_PARAM_GENERATER_H

