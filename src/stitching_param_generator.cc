// Created by s1nh.org on 2020/11/13.
// Modified from samples/cpp/stitching_detailed.cpp

#include "stitching_param_generator.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cmath>

#define ENABLE_LOG 0
#define LOG(msg) std::cout << msg
#define LOGLN(msg) std::cout << msg << std::endl

using namespace std;
using namespace cv;
using namespace cv::detail;

StitchingParamGenerator::StitchingParamGenerator(
    const std::vector<cv::Mat>& image_vector, bool use_calibration, bool use_sift, bool use_feature_mask) {

  std::cout << "[StitchingParamGenerator] Initializing..." << std::endl;
  std::cout << "[StitchingParamGenerator] Use calibration files: " << (use_calibration ? "yes" : "no (on-the-fly)") << std::endl;
  std::cout << "[StitchingParamGenerator] Feature detector: " << (use_sift ? "SIFT (accurate)" : "ORB (fast)") << std::endl;
  std::cout << "[StitchingParamGenerator] Use feature mask: " << (use_feature_mask ? "yes" : "no") << std::endl;

  use_calibration_ = use_calibration;
  use_sift_ = use_sift;
  use_feature_mask_ = use_feature_mask;
  num_img_ = image_vector.size();

  image_vector_ = image_vector;
  mask_vector_ = std::vector<cv::UMat>(num_img_);
  mask_warped_vector_ = std::vector<cv::UMat>(num_img_);
  image_size_vector_ = std::vector<cv::Size>(num_img_);
  image_warped_size_vector_ = std::vector<cv::Size>(num_img_);
  reproj_xmap_vector_ = std::vector<cv::UMat>(num_img_);
  reproj_ymap_vector_ = std::vector<cv::UMat>(num_img_);
  camera_params_vector_ =
      std::vector<cv::detail::CameraParams>(num_img_);

  projected_image_roi_refined_vect_ = std::vector<cv::Rect>(num_img_);

  for (size_t img_idx = 0; img_idx < num_img_; img_idx++) {
    image_size_vector_[img_idx] = image_vector_[img_idx].size();
  }

  // Initialize undistortion maps
  undist_xmap_vector_ = std::vector<cv::UMat>(num_img_);
  undist_ymap_vector_ = std::vector<cv::UMat>(num_img_);

  if (use_calibration_) {
    InitUndistortMap();
    // NOTE: Do NOT apply undistortion to image_vector_ here!
    // The undistortion maps will be combined with reprojection maps
    // and applied during warping in image_stitcher
    
    // Load camera intrinsics (K, focal) from calibration but estimate rotation from features
    camera_params_vector_ = std::vector<cv::detail::CameraParams>(num_img_);
    for (size_t i = 0; i < num_img_; i++) {
      std::string param_file = "../params/camchain_" + std::to_string(i) + ".yaml";
      std::ifstream test_file(param_file);
      if (!test_file.good()) {
        param_file = "params/camchain_" + std::to_string(i) + ".yaml";
      }
      test_file.close();
      
      cv::FileStorage fs_read(param_file, cv::FileStorage::READ);
      if (!fs_read.isOpened()) {
        fprintf(stderr, "%s:%d:Failed to load camera params from '%s'\n", __FILE__, __LINE__, param_file.c_str());
        return;
      }
      
      cv::Mat K;
      double focal;
      
      // Read intrinsics only (K, focal) - ignore the R matrix from calibration
      fs_read["KMat"] >> K;
      fs_read["focal"] >> focal;
      
      // Set intrinsics, rotation will be estimated from features in InitCameraParam()
      camera_params_vector_[i].focal = focal;
      camera_params_vector_[i].aspect = 1.0;
      camera_params_vector_[i].ppx = K.at<double>(0, 2);
      camera_params_vector_[i].ppy = K.at<double>(1, 2);
      camera_params_vector_[i].R = cv::Mat::eye(3, 3, CV_32F);  // Identity - will be estimated
      
      std::cout << "[StitchingParamGenerator] Loaded camera intrinsics #" << i << " from " << param_file << std::endl;
      std::cout << "  focal: " << focal << std::endl;
      std::cout << "  K:\n" << K << std::endl;
      std::cout << "  R: Will be estimated from features" << std::endl;
    }
  } else {
    // Create identity maps for undistortion (no distortion correction)
    for (size_t img_idx = 0; img_idx < num_img_; ++img_idx) {
      cv::Mat xmap(image_size_vector_[img_idx], CV_32FC1);
      cv::Mat ymap(image_size_vector_[img_idx], CV_32FC1);
      for (int y = 0; y < image_size_vector_[img_idx].height; ++y) {
        for (int x = 0; x < image_size_vector_[img_idx].width; ++x) {
          xmap.at<float>(y, x) = static_cast<float>(x);
          ymap.at<float>(y, x) = static_cast<float>(y);
        }
      }
      xmap.copyTo(undist_xmap_vector_[img_idx]);
      ymap.copyTo(undist_ymap_vector_[img_idx]);
    }
  }

  // ALWAYS run InitCameraParam to perform bundle adjustment and refine camera positions
  // Even with calibration files, this refines rotation based on actual feature matches
  InitCameraParam();

  InitWarper();




  std::cout << "[StitchingParamGenerator] Initialized." << std::endl;
}



void StitchingParamGenerator::InitCameraParam() {
    // Feature detection - choose between SIFT and ORB
    Ptr<Feature2D> finder;
    if (use_sift_) {
        finder = SIFT::create();
        LOGLN("Using SIFT feature detector (slower, more accurate)");
    } else {
        finder = ORB::create(1000);  // 1000 features
        LOGLN("Using ORB feature detector (faster, less accurate)");
    }
    std::vector<ImageFeatures> features(num_img_);

    std::vector<cv::UMat> feature_masks(num_img_);
    if (use_feature_mask_) {
        for (int i = 0; i < num_img_; ++i) {
            int rows = image_vector_[i].rows;
            int cols = image_vector_[i].cols;
            cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8U);
            
            // FIELD FOCUS: Use the lower 90% of image (field area)
            // Top 10% is typically sky/buildings - exclude completely
            int field_start_y = rows * 0.10;
            cv::rectangle(mask, cv::Rect(0, field_start_y, cols, rows - field_start_y), 
                         cv::Scalar(255), cv::FILLED);
            
            // Detect line features within the field area
            cv::Mat gray;
            cv::cvtColor(image_vector_[i], gray, cv::COLOR_BGR2GRAY);
            
            // Detect white lines (field markings) - they are bright
            // Use adaptive threshold for better detection
            cv::Mat white_mask;
            cv::threshold(gray, white_mask, 180, 255, cv::THRESH_BINARY);
            
            // Detect edges for field lines using Canny
            cv::Mat edges;
            cv::Canny(gray, edges, 30, 100);
            
            // Apply field region mask to both detections
            cv::Mat field_white, field_edges;
            cv::bitwise_and(white_mask, mask, field_white);
            cv::bitwise_and(edges, mask, field_edges);
            
            // Dilate to create regions around lines
            cv::dilate(field_edges, field_edges, cv::Mat(), cv::Point(-1, -1), 8);
            cv::dilate(field_white, field_white, cv::Mat(), cv::Point(-1, -1), 8);
            
            // Final mask = field region (no buildings)
            cv::Mat combined_mask = mask.clone();
            
            combined_mask.copyTo(feature_masks[i]);
            
            // DEBUG: Save mask visualization with overlay on original
            cv::Mat mask_overlay;
            image_vector_[i].copyTo(mask_overlay);
            // Draw mask boundary
            cv::line(mask_overlay, cv::Point(0, field_start_y), cv::Point(cols, field_start_y), 
                    cv::Scalar(0, 0, 255), 3);
            cv::putText(mask_overlay, "Feature detection area (below red line)", 
                       cv::Point(10, field_start_y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                       cv::Scalar(0, 255, 0), 2);
            std::string mask_path = "../results/debug_feature_mask_" + std::to_string(i) + ".jpg";
            cv::imwrite(mask_path, mask_overlay);
            std::cout << "[DEBUG] Saved feature mask: " << mask_path << std::endl;
        }
    } else {
        for (int i = 0; i < num_img_; ++i) {
            feature_masks[i].create(image_vector_[i].size(), CV_8U);
            feature_masks[i].setTo(Scalar::all(255));
        }
    }

    // Detect features for each image WITH mask
    for (int i = 0; i < num_img_; ++i) {
        // Use mask to focus on field area and line features
        cv::Mat mask_mat;
        feature_masks[i].copyTo(mask_mat);
        computeImageFeatures(finder, image_vector_[i], features[i], mask_mat);
        features[i].img_idx = i;
        
        if (features[i].keypoints.size() < 10) {
            LOGLN("Failed to find enough features in camera " << i + 1 
                  << " (" << features[i].keypoints.size() << " found)");
        }
        LOGLN("Features in image #" << i + 1 << ": " << features[i].keypoints.size());
        
        // DEBUG: Save image with detected features
        cv::Mat features_vis;
        cv::drawKeypoints(image_vector_[i], features[i].keypoints, features_vis, 
                         cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string features_path = "../results/debug_features_" + std::to_string(i) + ".jpg";
        cv::imwrite(features_path, features_vis);
        std::cout << "[DEBUG] Saved features visualization: " << features_path << std::endl;
    }

    // Match features between images
    LOGLN("Pairwise matching");
    std::vector<MatchesInfo> pairwise_matches;
    Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
    if (matcher_type == "affine")
      matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);
    else if (range_width == -1)
      matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
    else
      matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda,
                                                    match_conf);
    (*matcher)(features, pairwise_matches);
    matcher->collectGarbage();

    // DEBUG: Save feature matches visualization
    for (const auto& match_info : pairwise_matches) {
        if (match_info.src_img_idx < match_info.dst_img_idx && 
            match_info.src_img_idx >= 0 && match_info.dst_img_idx < num_img_) {
            cv::Mat matches_vis;
            std::vector<cv::DMatch> good_matches;
            for (size_t k = 0; k < match_info.matches.size(); ++k) {
                if (match_info.inliers_mask.size() > k && match_info.inliers_mask[k]) {
                    good_matches.push_back(match_info.matches[k]);
                }
            }
            cv::drawMatches(image_vector_[match_info.src_img_idx], features[match_info.src_img_idx].keypoints,
                           image_vector_[match_info.dst_img_idx], features[match_info.dst_img_idx].keypoints,
                           good_matches, matches_vis, cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0));
            
            // Add text with match info
            std::string info = "Matches: " + std::to_string(good_matches.size()) + 
                              " | Confidence: " + std::to_string(match_info.confidence);
            cv::putText(matches_vis, info, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
            
            std::string matches_path = "../results/debug_matches_" + 
                                       std::to_string(match_info.src_img_idx) + "_" + 
                                       std::to_string(match_info.dst_img_idx) + ".jpg";
            cv::imwrite(matches_path, matches_vis);
            std::cout << "[DEBUG] Saved matches visualization: " << matches_path 
                      << " (" << good_matches.size() << " inliers, confidence: " 
                      << match_info.confidence << ")" << std::endl;
        }
    }

    // Check if we have any good matches
    bool has_good_match = false;
    for (const auto& match_info : pairwise_matches) {
        // Only warn for actual image pairs (not self-matches)
        if (match_info.src_img_idx != match_info.dst_img_idx) {
            if (match_info.confidence < conf_thresh) {
                LOGLN("Warning: Low confidence in feature matches between images " 
                      << match_info.src_img_idx + 1 << " and " << match_info.dst_img_idx + 1 
                      << " (confidence: " << match_info.confidence << ", threshold: " << conf_thresh << ")");
            } else {
                has_good_match = true;
                LOGLN("Good match found between images " 
                      << match_info.src_img_idx + 1 << " and " << match_info.dst_img_idx + 1
                      << " (confidence: " << match_info.confidence << ")");
            }
        }
    }

    if (!has_good_match) {
        LOGLN("Error: No good matches found between images. Try:");
        LOGLN("  - Ensuring images have overlapping regions (20-40% overlap recommended)");
        LOGLN("  - Using images with more distinctive features");
        LOGLN("  - Adjusting match_conf (current: " << match_conf << ") or conf_thresh (current: " << conf_thresh << ")");
    }

    if (save_graph) {
      LOGLN("Saving matches graph...");
      ofstream f(save_graph_to.c_str());
      f << matchesGraphAsString(img_names, pairwise_matches, conf_thresh);
    }

    // Save calibrated intrinsics before estimation (HomographyBasedEstimator overwrites them)
    std::vector<cv::Mat> calibrated_K_matrices(num_img_);
    std::vector<double> calibrated_focals(num_img_);
    std::vector<double> calibrated_ppx(num_img_);
    std::vector<double> calibrated_ppy(num_img_);
    std::vector<double> calibrated_aspect(num_img_);
    if (use_calibration_) {
        for (int i = 0; i < num_img_; ++i) {
            calibrated_K_matrices[i] = camera_params_vector_[i].K().clone();
            calibrated_focals[i] = camera_params_vector_[i].focal;
            calibrated_ppx[i] = camera_params_vector_[i].ppx;
            calibrated_ppy[i] = camera_params_vector_[i].ppy;
            calibrated_aspect[i] = camera_params_vector_[i].aspect;
            std::cout << "[InitCameraParam] Saved calibrated K for camera " << i << ":\n" << calibrated_K_matrices[i] << std::endl;
        }
    }

    // Estimate camera parameters
    Ptr<Estimator> estimator = makePtr<HomographyBasedEstimator>();
    if (!(*estimator)(features, pairwise_matches, camera_params_vector_)) {
        LOGLN("Error: Failed to estimate camera positions. Check camera setup and overlap.");
        assert(false);
    }

    // Restore calibrated intrinsics if using calibration files.
    // The estimator call above overwrites all camera parameters, but we want to keep
    // the calibrated focal length and principal point. The rotation `R` estimated
    // by the estimator is what we want to keep and refine.
    if (use_calibration_) {
        std::cout << "[InitCameraParam] Restoring calibrated intrinsics after estimation" << std::endl;
        for (int i = 0; i < num_img_; ++i) {
            cv::Mat estimated_R = camera_params_vector_[i].R.clone();
            camera_params_vector_[i].focal = calibrated_focals[i];
            camera_params_vector_[i].ppx = calibrated_ppx[i];
            camera_params_vector_[i].ppy = calibrated_ppy[i];
            camera_params_vector_[i].aspect = calibrated_aspect[i];
            camera_params_vector_[i].R = estimated_R;
            std::cout << "[InitCameraParam] Camera " << i << " - Restored K, kept estimated R" << std::endl;
        }
    }

    for (auto& i : camera_params_vector_) {
        Mat R;
        i.R.convertTo(R, CV_32F);
        i.R = R;
    }
    Ptr<detail::BundleAdjusterBase> adjuster;
    
    if (ba_cost_func == "reproj")
        adjuster = makePtr<detail::BundleAdjusterReproj>();
    else if (ba_cost_func == "ray")
        adjuster = makePtr<detail::BundleAdjusterRay>();
    else if (ba_cost_func == "affine")
        adjuster =
            makePtr<detail::BundleAdjusterAffinePartial>();
    else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>();
    else {
        std::cout << "Unknown bundle adjustment cost function: '"
                  << ba_cost_func
                  << "'.\n";
        assert(false);
    }
    adjuster->setConfThresh(conf_thresh);
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    
    // When using calibration, we only want to refine the rotation.
    // When not using calibration, refine all parameters.
    std::string mask = use_calibration_ ? "xx_xx" : ba_refine_mask;
    if (mask[0] == 'x') refine_mask(0, 0) = 1;
    if (mask[1] == 'x') refine_mask(0, 1) = 1;
    if (mask[2] == 'x') refine_mask(0, 2) = 1;
    if (mask[3] == 'x') refine_mask(1, 1) = 1;
    if (mask[4] == 'x') refine_mask(1, 2) = 1;
    adjuster->setRefinementMask(refine_mask);
    
    std::cout << "[InitCameraParam] Bundle adjustment mask: " << mask << std::endl;
    if (!(*adjuster)(features, pairwise_matches, camera_params_vector_)) {
        std::cout << "Camera parameters adjusting failed.\n";
        assert(false);
    }
    


    std::vector<Mat> rmats;
    for (auto& i : camera_params_vector_)
        rmats.push_back(i.R.clone());
    
    waveCorrect(rmats, wave_correct);
    for (size_t i = 0; i < camera_params_vector_.size(); ++i) {
        camera_params_vector_[i].R = rmats[i];
    }
    
    for (size_t i = 0; i < camera_params_vector_.size(); ++i) {
        LOGLN("Initial camera intrinsics #"
                  << i + 1 << ":\nK:\n"
                  << camera_params_vector_[i].K()
                  << "\nR:\n" << camera_params_vector_[i].R);
    }
}

void StitchingParamGenerator::InitWarper() {

  std::vector<double> focals;
  float median_focal_length;
  reproj_xmap_vector_ = std::vector<UMat>(num_img_);

  for (size_t i = 0; i < camera_params_vector_.size(); ++i) {
    LOGLN("Camera #" << i + 1 << ":\nK:\n" << camera_params_vector_[i].K()
                     << "\nR:\n" << camera_params_vector_[i].R);
    focals.push_back(camera_params_vector_[i].focal);
  }
  sort(focals.begin(), focals.end());
  if (focals.size() % 2 == 1)
    median_focal_length = static_cast<float>(focals[focals.size() / 2]);
  else
    median_focal_length =
        static_cast<float>(focals[focals.size() / 2 - 1] +
            focals[focals.size() / 2]) * 0.5f;

  Ptr<WarperCreator> warper_creator;
  if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0) {
    if (warp_type == "plane")
      warper_creator = makePtr<cv::PlaneWarper>();
    else if (warp_type == "cylindrical")
      warper_creator = makePtr<cv::CylindricalWarper>();
    else if (warp_type == "spherical")
      warper_creator = makePtr<cv::SphericalWarper>();
  } else {
    if (warp_type == "plane")
      warper_creator = makePtr<cv::PlaneWarper>();
    else if (warp_type == "affine")
      warper_creator = makePtr<cv::AffineWarper>();
    else if (warp_type == "cylindrical")
      warper_creator = makePtr<cv::CylindricalWarper>();
    else if (warp_type == "spherical")
      warper_creator = makePtr<cv::SphericalWarper>();
    else if (warp_type == "fisheye")
      warper_creator = makePtr<cv::FisheyeWarper>();
    else if (warp_type == "stereographic")
      warper_creator = makePtr<cv::StereographicWarper>();
    else if (warp_type == "compressedPlaneA2B1")
      warper_creator = makePtr<cv::CompressedRectilinearWarper>(2.0f, 1.0f);
    else if (warp_type == "compressedPlaneA1.5B1")
      warper_creator = makePtr<cv::CompressedRectilinearWarper>(1.5f, 1.0f);
    else if (warp_type == "compressedPlanePortraitA2B1")
      warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(2.0f, 1.0f);
    else if (warp_type == "compressedPlanePortraitA1.5B1")
      warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(1.5f, 1.0f);
    else if (warp_type == "paniniA2B1")
      warper_creator = makePtr<cv::PaniniWarper>(2.0f, 1.0f);
    else if (warp_type == "paniniA1.5B1")
      warper_creator = makePtr<cv::PaniniWarper>(1.5f, 1.0f);
    else if (warp_type == "paniniPortraitA2B1")
      warper_creator = makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f);
    else if (warp_type == "paniniPortraitA1.5B1")
      warper_creator = makePtr<cv::PaniniPortraitWarper>(1.5f, 1.0f);
    else if (warp_type == "mercator")
      warper_creator = makePtr<cv::MercatorWarper>();
    else if (warp_type == "transverseMercator")
      warper_creator = makePtr<cv::TransverseMercatorWarper>();
  }
  if (!warper_creator) {
    std::cout << "Can't create the following warper '" << warp_type << "'\n";
    assert(false);
  }
  rotation_warper_ =
      warper_creator->create(static_cast<float>(median_focal_length));
  LOGLN("warped_image_scale: " << median_focal_length);

  corners_ = std::vector<cv::Point>(num_img_);

  // DEBUG: Save camera parameters to file for analysis
  std::ofstream debug_cam("../results/debug_camera_params.txt");
  debug_cam << "=== Camera Parameters Debug ===" << std::endl;
  debug_cam << "Median focal length: " << median_focal_length << std::endl;
  debug_cam << "Warp type: " << warp_type << std::endl << std::endl;

  for (int img_idx = 0; img_idx < num_img_; ++img_idx) {
    Mat_<float> K;
    camera_params_vector_[img_idx].K().convertTo(K, CV_32F);
    
    debug_cam << "Camera " << img_idx << ":" << std::endl;
    debug_cam << "  Focal: " << camera_params_vector_[img_idx].focal << std::endl;
    debug_cam << "  K matrix:\n" << K << std::endl;
    debug_cam << "  R matrix:\n" << camera_params_vector_[img_idx].R << std::endl;
    
    Rect rect = rotation_warper_->buildMaps(image_size_vector_[img_idx], K,
                                       camera_params_vector_[img_idx].R,
                                       reproj_xmap_vector_[img_idx],
                                       reproj_ymap_vector_[img_idx]);
    Point point(rect.x, rect.y);
    corners_[img_idx] = point;
    
    debug_cam << "  Build maps rect: " << rect << std::endl << std::endl;
  }
  debug_cam.close();
  std::cout << "[DEBUG] Saved camera parameters: ../results/debug_camera_params.txt" << std::endl;


  // Prepare images masks
  for (int img_idx = 0; img_idx < num_img_; ++img_idx) {
    mask_vector_[img_idx].create(image_vector_[img_idx].size(), CV_8U);
    mask_vector_[img_idx].setTo(Scalar::all(255));
    remap(mask_vector_[img_idx],
          mask_warped_vector_[img_idx],
          reproj_xmap_vector_[img_idx],
          reproj_ymap_vector_[img_idx],
          INTER_NEAREST);
    image_warped_size_vector_[img_idx] = mask_warped_vector_[img_idx].size();
  }

  timelapser_ = Timelapser::createDefault(timelapse_type);
  
  // Setup blender
  blender_ = Blender::createDefault(blend_type);
  cv::detail::FeatherBlender* feather_blender = dynamic_cast<cv::detail::FeatherBlender*>(blender_.get());
  if (feather_blender) {
      feather_blender->setSharpness(1.f / blend_strength);
  }
  blender_->prepare(corners_, image_warped_size_vector_);

  std::vector<cv::Rect> projected_image_roi_vect = std::vector<cv::Rect>(num_img_);

  // Sanity check limits - ROI should not be larger than reasonable multiples of input
  const int MAX_ROI_DIMENSION = 10000;  // Maximum reasonable output dimension
  const int MAX_ROI_OFFSET = 50000;     // Maximum reasonable offset from origin
  
  // DEBUG: Open ROI debug file
  std::ofstream debug_roi("../results/debug_roi_info.txt");
  debug_roi << "=== ROI Debug Information ===" << std::endl;
  
  Point roi_tl_bias(999999, 999999);
  for (int i = 0; i < num_img_; ++i) {
    // Update corner and size
    Size sz = image_vector_[i].size();
    Mat K;
    camera_params_vector_[i].K().convertTo(K, CV_32F);
    Rect roi = rotation_warper_->warpRoi(sz, K, camera_params_vector_[i].R);
    std::cout << "roi" << roi << std::endl;
    
    debug_roi << "Image " << i << ":" << std::endl;
    debug_roi << "  Input size: " << sz << std::endl;
    debug_roi << "  Raw warpRoi: " << roi << std::endl;
    
    // DEBUG: Save warped image visualization
    cv::Mat warped_vis;
    cv::Point corner = rotation_warper_->warp(image_vector_[i], K, 
                                              camera_params_vector_[i].R,
                                              cv::INTER_LINEAR, cv::BORDER_CONSTANT, warped_vis);
    // Scale down for visualization if too large
    if (warped_vis.cols > 2000 || warped_vis.rows > 2000) {
      double scale = std::min(2000.0 / warped_vis.cols, 2000.0 / warped_vis.rows);
      cv::resize(warped_vis, warped_vis, cv::Size(), scale, scale);
    }
    std::string warped_path = "../results/debug_warped_" + std::to_string(i) + ".jpg";
    cv::imwrite(warped_path, warped_vis);
    std::cout << "[DEBUG] Saved warped image: " << warped_path << " (corner: " << corner << ")" << std::endl;
    debug_roi << "  Warp corner: " << corner << std::endl;
    
    // Sanity check: detect unreasonable ROI values indicating bad homography
    if (std::abs(roi.x) > MAX_ROI_OFFSET || std::abs(roi.y) > MAX_ROI_OFFSET ||
        roi.width > MAX_ROI_DIMENSION || roi.height > MAX_ROI_DIMENSION ||
        roi.width <= 0 || roi.height <= 0) {
      
      debug_roi << "  [FAILED] Sanity check failed!" << std::endl;
      debug_roi.close();
      
      std::cerr << "[ERROR] ROI sanity check failed for image " << i << "!" << std::endl;
      std::cerr << "  ROI: " << roi.width << "x" << roi.height 
                << " at (" << roi.x << ", " << roi.y << ")" << std::endl;
      std::cerr << "  This indicates bad camera calibration or feature matching." << std::endl;
      std::cerr << "  Check ../results/debug_*.jpg and ../results/debug_*.txt for diagnostics." << std::endl;
      std::cerr << "  Possible fixes:" << std::endl;
      std::cerr << "    1. Re-calibrate cameras with better checkerboard images" << std::endl;
      std::cerr << "    2. Ensure sufficient overlap between images (20-40%)" << std::endl;
      std::cerr << "    3. Try different feature detector (SIFT vs ORB)" << std::endl;
      std::cerr << "    4. Check that input images are not corrupted" << std::endl;
      throw std::runtime_error("ROI sanity check failed - invalid warping result");
    }
    
    debug_roi << "  [OK] Passed sanity check" << std::endl;
    
    roi_tl_bias.x = min(roi.tl().x, roi_tl_bias.x);
    roi_tl_bias.y = min(roi.tl().y, roi_tl_bias.y);
    projected_image_roi_vect[i] = roi;
  }
  
  debug_roi << std::endl << "ROI top-left bias: " << roi_tl_bias << std::endl;
  
  full_image_size_ = Point(0, 0);
  Point y_range = Point(-9999999, 999999);
  for (int i = 0; i < num_img_; ++i) {
    projected_image_roi_vect[i] -= roi_tl_bias;
    Point tl = projected_image_roi_vect[i].tl();
    Point br = projected_image_roi_vect[i].br();

    debug_roi << "Image " << i << " adjusted ROI: " << projected_image_roi_vect[i] << std::endl;
    
    full_image_size_.x = max(br.x, full_image_size_.x);
    full_image_size_.y = max(br.y, full_image_size_.y);
    y_range.x = max(y_range.x, tl.y);
    y_range.y = min(y_range.y, br.y);
  }
  
  debug_roi << std::endl << "Full image size: " << full_image_size_ << std::endl;
  debug_roi << "Y range: " << y_range << std::endl;
  debug_roi.close();
  std::cout << "[DEBUG] Saved ROI info: ../results/debug_roi_info.txt" << std::endl;
  
  // Sanity check final output dimensions
  if (full_image_size_.x > MAX_ROI_DIMENSION || full_image_size_.y > MAX_ROI_DIMENSION) {
    std::cerr << "[ERROR] Output dimensions too large: " << full_image_size_.x 
              << "x" << full_image_size_.y << std::endl;
    std::cerr << "  Maximum allowed: " << MAX_ROI_DIMENSION << "x" << MAX_ROI_DIMENSION << std::endl;
    std::cerr << "  This indicates bad camera parameters or feature matching." << std::endl;
    throw std::runtime_error("Output dimensions exceed safe limits");
  }
  for (int i = 0; i < num_img_; ++i) {
    Rect rect = projected_image_roi_vect[i];
    rect.height =
        rect.height - (rect.br().y - y_range.y + y_range.x - rect.tl().y);
    rect.y = y_range.x - rect.y;
    
    // Post-processing sanity check
    if (rect.width <= 0 || rect.height <= 0 || 
        rect.width > MAX_ROI_DIMENSION || rect.height > MAX_ROI_DIMENSION) {
      std::cerr << "[ERROR] Processed ROI sanity check failed for image " << i << "!" << std::endl;
      std::cerr << "  ROI after processing: " << rect.width << "x" << rect.height 
                << " at (" << rect.x << ", " << rect.y << ")" << std::endl;
      throw std::runtime_error("Processed ROI sanity check failed");
    }
    
    projected_image_roi_vect[i] = rect;
    projected_image_roi_refined_vect_[i] = rect;
  }

  for (int i = 0; i < num_img_ - 1; ++i) {
    Rect rect_left = projected_image_roi_refined_vect_[i];
    Rect rect_right = projected_image_roi_vect[i + 1];
    
    // Calculate overlap region
    int overlap_start = projected_image_roi_vect[i + 1].tl().x;
    int overlap_end = projected_image_roi_vect[i].br().x;
    int overlap_width = overlap_end - overlap_start;
    
    if (overlap_width > 0) {
      // Use weighted seam placement based on image content
      // Place seam closer to center of overlap for better blending
      // This is more robust than simple /2 division
      float seam_ratio = 0.5f;  // Can be adjusted based on image analysis
      
      // For asymmetric overlaps, adjust seam position
      int left_roi_width = projected_image_roi_vect[i].width;
      int right_roi_width = projected_image_roi_vect[i + 1].width;
      if (left_roi_width > 0 && right_roi_width > 0) {
        // Weight towards the image with more content
        float weight = static_cast<float>(left_roi_width) / (left_roi_width + right_roi_width);
        seam_ratio = 0.4f + weight * 0.2f;  // Range [0.4, 0.6] for stability
      }
      
      int offset = static_cast<int>(overlap_width * seam_ratio);
      
      rect_left.width -= offset;
      rect_right.width -= offset;
      rect_right.x = offset;
    } else {
      // No overlap - use images as-is
      LOGLN("Warning: No overlap detected between images " << i << " and " << i+1);
    }
    
    projected_image_roi_refined_vect_[i] = rect_left;
    projected_image_roi_refined_vect_[i + 1] = rect_right;
  }
}

void StitchingParamGenerator::InitUndistortMap() {
  std::vector<double> cam_focal_vector(num_img_);

  std::vector<cv::UMat> k_vector(num_img_);
  std::vector<std::vector<double>> d_vector(num_img_);
  cv::Size resolution;

  undist_xmap_vector_ = std::vector<cv::UMat>(num_img_);
  undist_ymap_vector_ = std::vector<cv::UMat>(num_img_);

  for (size_t i = 0; i < num_img_; i++) {
    // Try both paths to support running from build/ or project root
    std::string param_file = "../params/camchain_" + std::to_string(i) + ".yaml";
    std::ifstream test_file(param_file);
    if (!test_file.good()) {
      param_file = "params/camchain_" + std::to_string(i) + ".yaml";
    }
    test_file.close();
    
    cv::FileStorage fs_read(param_file, cv::FileStorage::READ);
    if (!fs_read.isOpened()) {
      fprintf(stderr, "%s:%d:loadParams failed. '%s' does not exist\n", __FILE__, __LINE__, param_file.c_str());
      return;
    }
    cv::Mat K;
    fs_read["KMat"] >> K;
    K.copyTo(k_vector[i]);
    fs_read["D"] >> d_vector[i];
    fs_read["focal"] >> cam_focal_vector[i];
    fs_read["resolution"] >> resolution;
    
    std::cout << "[InitUndistortMap] Loaded calibration for camera " << i << " from " << param_file << std::endl;
  }

  for (size_t i = 0; i < num_img_; i++) {
    cv::Mat K_float, R_identity, newCameraMatrix;
    k_vector[i].getMat(cv::ACCESS_READ).convertTo(K_float, CV_32F);
    
    // Use identity rotation for undistortion (no rotation needed during undistortion)
    R_identity = cv::Mat::eye(3, 3, CV_32F);
    
    // Compute optimal new camera matrix to preserve all pixels
    // alpha=1 keeps all source pixels, alpha=0 crops to valid region
    newCameraMatrix = cv::getOptimalNewCameraMatrix(K_float, d_vector[i], resolution, 1.0, resolution);
    
    // Generate undistortion maps
    cv::initUndistortRectifyMap(
        K_float, d_vector[i], R_identity, newCameraMatrix, resolution,
        CV_32FC1, undist_xmap_vector_[i], undist_ymap_vector_[i]);
    
    std::cout << "[InitUndistortMap] Created undistortion map for camera " << i << std::endl;
  }
}

void StitchingParamGenerator::GetReprojParams(
    std::vector<cv::UMat>& undist_xmap_vector,
    std::vector<cv::UMat>& undist_ymap_vector,
    std::vector<cv::UMat>& reproj_xmap_vector,
    std::vector<cv::UMat>& reproj_ymap_vector,
    std::vector<cv::Rect>& projected_image_roi_refined_vect) {

  undist_xmap_vector = undist_xmap_vector_;
  undist_ymap_vector = undist_ymap_vector_;
  reproj_xmap_vector = reproj_xmap_vector_;
  reproj_ymap_vector = reproj_ymap_vector_;
  projected_image_roi_refined_vect = projected_image_roi_refined_vect_;
  std::cout << "[GetReprojParams] projected_image_roi_vect_refined: " << std::endl;

  size_t i = 0;
  for (auto& roi : projected_image_roi_refined_vect) {
    std::cout << "[GetReprojParams] roi [" << i << ": "
              << roi.width << "x"
              << roi.height << " from ("
              << roi.x << ", "
              << roi.y << ")]" << std::endl;
    i++;
    if (roi.width < 0 || roi.height < 0 || roi.x < 0 || roi.y < 0) {
      std::cout << "StitchingParamGenerator did not find a suitable feature point under the current parameters, "
                << "resulting in an incorrect ROI. "
                << "Please use \"opencv/stitching_detailed\" to find the correct parameters. "
                << "(see https://docs.opencv.org/4.8.0/d8/d19/tutorial_stitcher.html)" << std::endl;
      assert (false);
    }
  }
}