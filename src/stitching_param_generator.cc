// Created by s1nh.org on 2020/11/13.
// Modified from samples/cpp/stitching_detailed.cpp

#include "stitching_param_generator.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cmath>
#include "opencv2/stitching/detail/seam_finders.hpp"

#define ENABLE_LOG 0
#define LOG(msg) std::cout << msg
#define LOGLN(msg) std::cout << msg << std::endl

using namespace std;
using namespace cv;
using namespace cv::detail;

StitchingParamGenerator::StitchingParamGenerator(
    const std::vector<cv::Mat>& image_vector, bool use_calibration, const std::string& feature_method, bool use_feature_mask, const std::string& debug_folder) {

  std::cout << "[StitchingParamGenerator] Initializing..." << std::endl;
  std::cout << "[StitchingParamGenerator] Use calibration files: " << (use_calibration ? "yes" : "no (on-the-fly)") << std::endl;
  std::cout << "[StitchingParamGenerator] Feature detector: " << feature_method << std::endl;
  std::cout << "[StitchingParamGenerator] Use feature mask: " << (use_feature_mask ? "yes" : "no") << std::endl;

  use_calibration_ = use_calibration;
  feature_method_ = feature_method;
  use_feature_mask_ = use_feature_mask;
  debug_folder_ = debug_folder;
  std::system(("mkdir -p \"" + debug_folder_ + "\"").c_str());
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
      cv::Size yaml_resolution;

      // Read intrinsics (K, focal) from calibration
      fs_read["KMat"] >> K;
      fs_read["focal"] >> focal;
      fs_read["resolution"] >> yaml_resolution;

      // Scale K and focal from calibration resolution to actual image resolution
      cv::Size actual_size = image_size_vector_[i];
      if (yaml_resolution.width > 0 && yaml_resolution.height > 0 &&
          (actual_size.width != yaml_resolution.width || actual_size.height != yaml_resolution.height)) {
          double sx = static_cast<double>(actual_size.width)  / yaml_resolution.width;
          double sy = static_cast<double>(actual_size.height) / yaml_resolution.height;
          K.at<double>(0, 0) *= sx;  // fx
          K.at<double>(1, 1) *= sy;  // fy
          K.at<double>(0, 2) *= sx;  // cx
          K.at<double>(1, 2) *= sy;  // cy
          focal *= sx;  // focal scales with fx
          std::cout << "[InitCameraParam] Scaled K/focal by (" << sx << ", " << sy
                    << ") from " << yaml_resolution << " → " << actual_size << std::endl;
      }

      // Read the stereo rotation seed: cam0=identity, cam1=R from stereo calibration.
      // This gives the bundle adjuster a correct starting point for the inter-camera angle.
      cv::Mat stereo_R = cv::Mat::eye(3, 3, CV_32F);
      cv::Mat stereo_R_raw;
      fs_read["StereoRMat"] >> stereo_R_raw;
      if (!stereo_R_raw.empty()) {
          stereo_R_raw.convertTo(stereo_R, CV_32F);

          // The stereo R contains the true 53° yaw plus small pitch/roll
          // components (~1.7°) that are calibration noise — the cameras are
          // physically mounted at the same height on a fixed rig.
          // Those residual components produce a ~33px vertical seam offset.
          // Fix: reconstruct R as a pure yaw rotation (preserve azimuth, zero pitch/roll).
          float yaw = std::atan2(stereo_R.at<float>(0, 2), stereo_R.at<float>(0, 0));
          float cy = std::cos(yaw), sy = std::sin(yaw);
          stereo_R = (cv::Mat_<float>(3, 3) <<
              cy,  0.f, sy,
              0.f, 1.f, 0.f,
             -sy,  0.f, cy);
          std::cout << "[InitCameraParam] Reconstructed stereo R as pure yaw: "
                    << yaw * 180.f / CV_PI << "°  (removed pitch/roll noise)" << std::endl;
      }

      // Set intrinsics and seed rotation; bundle adjuster will refine the rotation.
      camera_params_vector_[i].focal = focal;
      camera_params_vector_[i].aspect = 1.0;
      camera_params_vector_[i].ppx = K.at<double>(0, 2);
      camera_params_vector_[i].ppy = K.at<double>(1, 2);
      camera_params_vector_[i].R = stereo_R;
      
      std::cout << "[StitchingParamGenerator] Loaded camera intrinsics #" << i << " from " << param_file << std::endl;
      std::cout << "  focal: " << focal << std::endl;
      std::cout << "  K:\n" << K << std::endl;
      std::cout << "  R seed:\n" << stereo_R << std::endl;
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
    // Feature detection - choose between SIFT, ORB, and AKAZE
    Ptr<Feature2D> finder;
    if (feature_method_ == "SIFT") {
        // nfeatures=0 means unlimited; use contrastThreshold/edgeThreshold
        // to keep quality up while allowing more detections.
        finder = SIFT::create(/*nfeatures=*/0, /*nOctaveLayers=*/5,
                              /*contrastThreshold=*/0.03, /*edgeThreshold=*/15, /*sigma=*/1.6);
        LOGLN("Using SIFT (unlimited, contrast=0.03, 5 octave layers)");
    } else if (feature_method_ == "AKAZE") {
        // Lower the detection threshold (default 0.001) so we get more
        // keypoints on the low-texture grass.  4 octaves + 5 layers for
        // richer scale coverage.
        finder = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3,
                               /*threshold=*/0.0005f, /*nOctaves=*/4, /*nOctaveLayers=*/5);
        LOGLN("Using AKAZE feature detector (threshold=0.0005, 4 oct × 5 layers)");
    } else {
        finder = ORB::create(3000);  // 3000 features
        LOGLN("Using ORB feature detector (3000 features)");
    }
    std::vector<ImageFeatures> features(num_img_);

    // When using calibration, detect features on UNDISTORTED images so that
    // K^-1 * [u,v,1] gives correct ray directions for bundle adjustment.
    // Without this, the strong barrel distortion (k1=-0.33) causes the BA
    // to collapse the 53° rotation toward identity (a spurious minimum).
    std::vector<cv::Mat> feature_images(num_img_);
    if (use_calibration_) {
        for (int i = 0; i < num_img_; ++i) {
            cv::remap(image_vector_[i], feature_images[i],
                      undist_xmap_vector_[i], undist_ymap_vector_[i],
                      cv::INTER_LINEAR);
        }
        std::cout << "[InitCameraParam] Using undistorted images for feature detection" << std::endl;
    } else {
        for (int i = 0; i < num_img_; ++i) {
            feature_images[i] = image_vector_[i];
        }
    }

    // CLAHE preprocessing: dramatically improves feature detection on
    // uniform surfaces (grass, sky) by boosting local contrast.
    // Applied per-channel on the Lab L channel to preserve colour.
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(/*clipLimit=*/6.0, /*tileGridSize=*/cv::Size(8, 8));
        for (int i = 0; i < num_img_; ++i) {
            cv::Mat lab, channels[3];
            cv::cvtColor(feature_images[i], lab, cv::COLOR_BGR2Lab);
            cv::split(lab, channels);
            clahe->apply(channels[0], channels[0]);
            cv::merge(channels, 3, lab);
            cv::cvtColor(lab, feature_images[i], cv::COLOR_Lab2BGR);
        }
        std::cout << "[InitCameraParam] Applied CLAHE contrast enhancement for feature detection" << std::endl;
    }

    std::vector<cv::UMat> feature_masks(num_img_);
    if (use_feature_mask_) {
        for (int i = 0; i < num_img_; ++i) {
            int rows = feature_images[i].rows;
            int cols = feature_images[i].cols;
            cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8U);
            
            // FIELD FOCUS: Use the lower 90% of image (field area)
            // Top 10% is typically sky/buildings - exclude completely
            int field_start_y = rows * 0.10;
            cv::rectangle(mask, cv::Rect(0, field_start_y, cols, rows - field_start_y), 
                         cv::Scalar(255), cv::FILLED);
            
            // Detect line features within the field area
            cv::Mat gray;
            cv::cvtColor(feature_images[i], gray, cv::COLOR_BGR2GRAY);
            
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
            feature_images[i].copyTo(mask_overlay);
            // Draw mask boundary
            cv::line(mask_overlay, cv::Point(0, field_start_y), cv::Point(cols, field_start_y), 
                    cv::Scalar(0, 0, 255), 3);
            cv::putText(mask_overlay, "Feature detection area (below red line)", 
                       cv::Point(10, field_start_y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                       cv::Scalar(0, 255, 0), 2);
            std::string mask_path = debug_folder_ + "/debug_feature_mask_" + std::to_string(i) + ".jpg";
            cv::imwrite(mask_path, mask_overlay);
            std::cout << "[DEBUG] Saved feature mask: " << mask_path << std::endl;
        }
    } else if (num_img_ == 2) {
        // Overlap-aware masking: compute the fraction of each image that
        // overlaps with the other camera using calibrated yaw + FOV.
        // overlap_frac = (fov_h - |yaw|) / fov_h, clamped to [10%, 50%].
        // Falls back to 20% when calibration is unavailable.
        float overlap_frac = 0.20f;
        if (use_calibration_ && camera_params_vector_[1].focal > 0 &&
            !camera_params_vector_[1].R.empty()) {
            float yaw = std::abs(std::atan2(
                camera_params_vector_[1].R.at<float>(0, 2),
                camera_params_vector_[1].R.at<float>(0, 0)));
            float fov_h = 2.f * std::atan2(
                image_size_vector_[0].width * 0.5f,
                static_cast<float>(camera_params_vector_[0].focal));
            float computed = (fov_h > 0.f) ? std::max(0.f, (fov_h - yaw) / fov_h) : 0.20f;
            overlap_frac = std::max(0.10f, std::min(0.50f, computed));
            std::cout << "[InitCameraParam] Overlap mask: yaw=" << yaw * 180.f / CV_PI
                      << "° fov_h=" << fov_h * 180.f / CV_PI
                      << "° → overlap=" << overlap_frac * 100.f << "%" << std::endl;
        } else {
            std::cout << "[InitCameraParam] Overlap mask: using default 20%" << std::endl;
        }

        int rows0 = feature_images[0].rows, cols0 = feature_images[0].cols;
        int overlap_px_0 = static_cast<int>(cols0 * overlap_frac);
        cv::Mat overlap0 = cv::Mat::zeros(rows0, cols0, CV_8U);
        cv::rectangle(overlap0, cv::Rect(cols0 - overlap_px_0, 0, overlap_px_0, rows0),
                      cv::Scalar(255), cv::FILLED);

        int rows1 = feature_images[1].rows, cols1 = feature_images[1].cols;
        int overlap_px_1 = static_cast<int>(cols1 * overlap_frac);
        cv::Mat overlap1 = cv::Mat::zeros(rows1, cols1, CV_8U);
        cv::rectangle(overlap1, cv::Rect(0, 0, overlap_px_1, rows1),
                      cv::Scalar(255), cv::FILLED);

        overlap0.copyTo(feature_masks[0]);
        overlap1.copyTo(feature_masks[1]);

        std::cout << "[InitCameraParam] 2-camera overlap mask: " << overlap_frac * 100.0f
                  << "% (cam0 right " << overlap_px_0 << "px, cam1 left "
                  << overlap_px_1 << "px)" << std::endl;

        // Save overlap mask debug visualizations
        for (int i = 0; i < num_img_; ++i) {
            cv::Mat vis;
            feature_images[i].copyTo(vis);
            cv::Mat mask_mat_dbg;
            feature_masks[i].copyTo(mask_mat_dbg);
            cv::Mat mask_rgb;
            cv::cvtColor(mask_mat_dbg, mask_rgb, cv::COLOR_GRAY2BGR);
            cv::addWeighted(vis, 0.7, mask_rgb, 0.3, 0, vis);
            std::string path = debug_folder_ + "/debug_overlap_mask_" + std::to_string(i) + ".jpg";
            cv::imwrite(path, vis);
            std::cout << "[DEBUG] Saved overlap mask: " << path << std::endl;
        }
    } else {
        for (int i = 0; i < num_img_; ++i) {
            feature_masks[i].create(feature_images[i].size(), CV_8U);
            feature_masks[i].setTo(Scalar::all(255));
        }
    }

    // Detect features for each image WITH mask
    for (int i = 0; i < num_img_; ++i) {
        // Use mask to focus on field area and line features
        cv::Mat mask_mat;
        feature_masks[i].copyTo(mask_mat);
        computeImageFeatures(finder, feature_images[i], features[i], mask_mat);
        features[i].img_idx = i;
        
        if (features[i].keypoints.size() < 10) {
            LOGLN("Failed to find enough features in camera " << i + 1 
                  << " (" << features[i].keypoints.size() << " found)");
        }
        LOGLN("Features in image #" << i + 1 << ": " << features[i].keypoints.size());
        
        // DEBUG: Save image with detected features
        cv::Mat features_vis;
        cv::drawKeypoints(feature_images[i], features[i].keypoints, features_vis, 
                         cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string features_path = debug_folder_ + "/debug_features_" + std::to_string(i) + ".jpg";
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
            cv::drawMatches(feature_images[match_info.src_img_idx], features[match_info.src_img_idx].keypoints,
                           feature_images[match_info.dst_img_idx], features[match_info.dst_img_idx].keypoints,
                           good_matches, matches_vis, cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0));
            
            // Add text with match info
            std::string info = "Matches: " + std::to_string(good_matches.size()) + 
                              " | Confidence: " + std::to_string(match_info.confidence);
            cv::putText(matches_vis, info, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
            
            std::string matches_path = debug_folder_ + "/debug_matches_" + 
                                       std::to_string(match_info.src_img_idx) + "_" + 
                                       std::to_string(match_info.dst_img_idx) + ".jpg";
            cv::imwrite(matches_path, matches_vis);
            std::cout << "[DEBUG] Saved matches visualization: " << matches_path 
                      << " (" << good_matches.size() << " inliers, confidence: " 
                      << match_info.confidence << ")" << std::endl;
        }
    }

    // Check if we have enough inliers for stable BA.
    // BundleAdjusterRay needs geometric diversity — 6 collinear matches on a
    // single white line are not enough; the LM solver degenerates.
    const int MIN_BA_INLIERS = 20;
    bool has_good_match = false;
    int total_inliers = 0;
    for (const auto& match_info : pairwise_matches) {
        if (match_info.src_img_idx != match_info.dst_img_idx) {
            int inliers = 0;
            for (size_t k = 0; k < match_info.inliers_mask.size(); ++k)
                if (match_info.inliers_mask[k]) ++inliers;
            total_inliers += inliers;
            if (match_info.confidence < conf_thresh) {
                LOGLN("Warning: Low confidence between images "
                      << match_info.src_img_idx + 1 << " and " << match_info.dst_img_idx + 1
                      << " (confidence: " << match_info.confidence
                      << ", inliers: " << inliers << ")");
            } else {
                LOGLN("Match found between images "
                      << match_info.src_img_idx + 1 << " and " << match_info.dst_img_idx + 1
                      << " (confidence: " << match_info.confidence
                      << ", inliers: " << inliers << ")");
            }
        }
    }
    has_good_match = (total_inliers >= MIN_BA_INLIERS);
    std::cout << "[InitCameraParam] Total inliers: " << total_inliers
              << " (need " << MIN_BA_INLIERS << " for BA)" << std::endl;

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
    std::vector<double> calibrated_focals(num_img_);
    std::vector<double> calibrated_ppx(num_img_);
    std::vector<double> calibrated_ppy(num_img_);
    std::vector<double> calibrated_aspect(num_img_);
    if (use_calibration_) {
        for (int i = 0; i < num_img_; ++i) {
            calibrated_focals[i] = camera_params_vector_[i].focal;
            calibrated_ppx[i] = camera_params_vector_[i].ppx;
            calibrated_ppy[i] = camera_params_vector_[i].ppy;
            calibrated_aspect[i] = camera_params_vector_[i].aspect;
        }
    }

    if (use_calibration_) {
        // We already have calibrated intrinsics AND a stereo-rotation seed loaded in
        // camera_params_vector_. Skip HomographyBasedEstimator entirely — it would
        // overwrite our seeded rotations with its own homography-derived guess.
        // Bundle adjustment below will refine the rotation from the seed.
        std::cout << "[InitCameraParam] Calibration mode: skipping estimator, using stereo R seed" << std::endl;
        for (auto& cam : camera_params_vector_) {
            cv::Mat R;
            cam.R.convertTo(R, CV_32F);
            cam.R = R;
        }
    } else {
        // No calibration: estimate camera parameters from scratch using feature matches
        Ptr<Estimator> estimator = makePtr<HomographyBasedEstimator>();
        if (!(*estimator)(features, pairwise_matches, camera_params_vector_)) {
            LOGLN("Error: Failed to estimate camera positions. Check camera setup and overlap.");
            assert(false);
        }
        for (auto& cam : camera_params_vector_) {
            cv::Mat R;
            cam.R.convertTo(R, CV_32F);
            cam.R = R;
        }
    }

    // Restore calibrated intrinsics if using calibration files.
    if (use_calibration_) {
        std::cout << "[InitCameraParam] Restoring calibrated intrinsics after estimation" << std::endl;
        for (int i = 0; i < num_img_; ++i) {
            camera_params_vector_[i].focal = calibrated_focals[i];
            camera_params_vector_[i].ppx = calibrated_ppx[i];
            camera_params_vector_[i].ppy = calibrated_ppy[i];
            camera_params_vector_[i].aspect = calibrated_aspect[i];
            std::cout << "[InitCameraParam] Camera " << i << " - Restored K, kept estimated R" << std::endl;
        }
    }

    Ptr<detail::BundleAdjusterBase> adjuster;
    if (use_calibration_) {
        if (has_good_match) {
            // Ray-based BA: minimises the angle between corresponding rays.
            // Correct model for a pure-rotation rig; more stable than reproj
            // when matches are sparse (no depth ambiguity to destabilise it).
            adjuster = makePtr<detail::BundleAdjusterRay>();
            adjuster->setConfThresh(conf_thresh);
            // Refine only rotation (ray BA has no intrinsic refinement mask).
            Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
            adjuster->setRefinementMask(refine_mask);
            std::cout << "[InitCameraParam] Calibration mode: refining rotation via ray BA" << std::endl;
        } else {
            // No reliable matches (typical on low-texture grass): skip BA entirely
            // and trust the calibrated R, K from the YAML files.
            adjuster = makePtr<detail::NoBundleAdjuster>();
            std::cout << "[InitCameraParam] Calibration mode: no good matches, skipping BA — using calibrated params as-is" << std::endl;
        }
    } else {
        if (ba_cost_func == "reproj")
            adjuster = makePtr<detail::BundleAdjusterReproj>();
        else if (ba_cost_func == "ray")
            adjuster = makePtr<detail::BundleAdjusterRay>();
        else if (ba_cost_func == "affine")
            adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
        else if (ba_cost_func == "no")
            adjuster = makePtr<NoBundleAdjuster>();
        else {
            std::cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
            assert(false);
        }
        adjuster->setConfThresh(conf_thresh);
        Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
        if (ba_refine_mask[0] == 'x') refine_mask(0, 0) = 1;
        if (ba_refine_mask[1] == 'x') refine_mask(0, 1) = 1;
        if (ba_refine_mask[2] == 'x') refine_mask(0, 2) = 1;
        if (ba_refine_mask[3] == 'x') refine_mask(1, 1) = 1;
        if (ba_refine_mask[4] == 'x') refine_mask(1, 2) = 1;
        adjuster->setRefinementMask(refine_mask);
    }
    // Save seed rotations before BA (to detect if BA goes off the rails)
    std::vector<cv::Mat> seed_rotations(num_img_);
    if (use_calibration_) {
        for (int i = 0; i < num_img_; ++i) {
            seed_rotations[i] = camera_params_vector_[i].R.clone();
        }
    }

    if (!(*adjuster)(features, pairwise_matches, camera_params_vector_)) {
        std::cout << "[InitCameraParam] BA failed — reverting to seed rotations.\n";
        if (use_calibration_) {
            for (int i = 0; i < num_img_; ++i)
                camera_params_vector_[i].R = seed_rotations[i].clone();
        }
    }

    // Post-BA rotation sanity check: for a calibrated stereo rig the inter-camera
    // rotation is physically fixed.  If BA drifted more than MAX_ROT_DEV degrees
    // from the calibration seed, feature matching was unreliable and we fall back
    // to the known-good calibrated rotation.
    const double MAX_ROT_DEV_DEG = 12.0;
    if (use_calibration_) {
        bool rotation_reverted = false;
        for (int i = 0; i < num_img_; ++i) {
            cv::Mat R_ba, R_seed;
            camera_params_vector_[i].R.convertTo(R_ba, CV_64F);
            seed_rotations[i].convertTo(R_seed, CV_64F);

            // Angular deviation = arccos((trace(R_seed^T * R_ba) - 1) / 2)
            cv::Mat R_diff = R_seed.t() * R_ba;
            double trace_val = R_diff.at<double>(0,0) + R_diff.at<double>(1,1) + R_diff.at<double>(2,2);
            double cos_angle = std::max(-1.0, std::min(1.0, (trace_val - 1.0) / 2.0));
            double deviation_deg = std::acos(cos_angle) * 180.0 / CV_PI;

            std::cout << "[InitCameraParam] Camera " << i
                      << " rotation deviation from seed: " << deviation_deg << " deg" << std::endl;

            if (deviation_deg > MAX_ROT_DEV_DEG) {
                std::cout << "[InitCameraParam] WARNING: Camera " << i
                          << " rotation deviated " << deviation_deg
                          << "° from calibrated seed (threshold: " << MAX_ROT_DEV_DEG
                          << "°). Reverting to calibrated rotation." << std::endl;
                rotation_reverted = true;
            }
        }

        if (rotation_reverted) {
            std::cout << "[InitCameraParam] Reverting ALL cameras to calibrated seed rotations "
                      << "(and ppx/ppy — BA result not trustworthy)" << std::endl;
            for (int i = 0; i < num_img_; ++i) {
                camera_params_vector_[i].R = seed_rotations[i].clone();
                camera_params_vector_[i].ppx = calibrated_ppx[i];
                camera_params_vector_[i].ppy = calibrated_ppy[i];
            }
        }
    }

    // Restore only focal and aspect (which we locked). Keep the BA-refined ppx/ppy.
    if (use_calibration_) {
        for (int i = 0; i < num_img_; ++i) {
            std::cout << "[InitCameraParam] Camera " << i 
                      << " ppx: " << calibrated_ppx[i] << " -> " << camera_params_vector_[i].ppx
                      << ", ppy: " << calibrated_ppy[i] << " -> " << camera_params_vector_[i].ppy << std::endl;
            camera_params_vector_[i].focal  = calibrated_focals[i];
            camera_params_vector_[i].aspect = calibrated_aspect[i];
        }
        std::cout << "[InitCameraParam] Restored focal/aspect, kept BA-refined ppx/ppy" << std::endl;
    }

    std::vector<Mat> rmats;
    for (auto& i : camera_params_vector_)
        rmats.push_back(i.R.clone());
    
    // Wave correction levels both cameras to the same horizontal plane.
    // Apply in all modes — the small pitch residual in the stereo R (~1.7°)
    // causes a 33-pixel vertical offset at the seam without it.
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
  std::ofstream debug_cam(debug_folder_ + "/debug_camera_params.txt");
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


  // Prepare images masks and warped samples for exposure compensation
  std::vector<UMat> images_warped(num_img_);
  for (int img_idx = 0; img_idx < num_img_; ++img_idx) {
    mask_vector_[img_idx].create(image_vector_[img_idx].size(), CV_8U);
    mask_vector_[img_idx].setTo(Scalar::all(255));
    remap(mask_vector_[img_idx],
          mask_warped_vector_[img_idx],
          reproj_xmap_vector_[img_idx],
          reproj_ymap_vector_[img_idx],
          INTER_NEAREST);
    image_warped_size_vector_[img_idx] = mask_warped_vector_[img_idx].size();

    // Warp the sample image for exposure compensation computation
    Mat K;
    camera_params_vector_[img_idx].K().convertTo(K, CV_32F);
    rotation_warper_->warp(image_vector_[img_idx], K,
                           camera_params_vector_[img_idx].R,
                           INTER_LINEAR, BORDER_CONSTANT, images_warped[img_idx]);
  }

  // Sub-pixel alignment: find the residual vertical (and horizontal) shift
  // between the two warped images in their overlap zone.
  // Strategy: try line matching first (geometry-based, best for football pitches
  // with painted line markings), fall back to phase correlation if insufficient lines.
  if (num_img_ == 2) {
      cv::Mat wc0, wc1;
      images_warped[0].copyTo(wc0);
      images_warped[1].copyTo(wc1);

      // Grayscale copies for correlation
      cv::Mat wg0, wg1;
      cv::cvtColor(wc0, wg0, cv::COLOR_BGR2GRAY);
      cv::cvtColor(wc1, wg1, cv::COLOR_BGR2GRAY);

      // Overlap zone in panorama coordinates
      int x0s = corners_[0].x, x0e = x0s + wg0.cols;
      int x1s = corners_[1].x, x1e = x1s + wg1.cols;
      int ols = std::max(x0s, x1s);
      int ole = std::min(x0e, x1e);

      if (ole > ols + 50) {
          int c0 = ols - x0s;
          int c1 = ols - x1s;
          int olw = ole - ols;
          int h   = std::min(wg0.rows, wg1.rows);
          int y0off = (wg0.rows - h) / 2;
          int y1off = (wg1.rows - h) / 2;

          // ── Phase correlation ─────────────────────────────────────────────
          cv::Mat s0f, s1f;
          wg0(cv::Rect(c0, y0off, olw, h)).convertTo(s0f, CV_32F);
          wg1(cv::Rect(c1, y1off, olw, h)).convertTo(s1f, CV_32F);
          cv::Mat hann;
          cv::createHanningWindow(hann, s0f.size(), CV_32F);
          double pc_response;
          cv::Point2d pc_shift = cv::phaseCorrelate(s0f, s1f, hann, &pc_response);
          std::cout << "[Align] Phase-correlation: dx=" << pc_shift.x
                    << " dy=" << pc_shift.y << " response=" << pc_response << std::endl;

          // ── Line matching ─────────────────────────────────────────────────
          // Extract colour overlap strips for line detection
          cv::Mat ov0 = wc0(cv::Rect(c0, y0off, olw, h)).clone();
          cv::Mat ov1 = wc1(cv::Rect(c1, y1off, olw, h)).clone();

          // Threshold for white painted lines
          cv::Mat wb0, wb1;
          cv::threshold(wg0(cv::Rect(c0, y0off, olw, h)), wb0, 200, 255, cv::THRESH_BINARY);
          cv::threshold(wg1(cv::Rect(c1, y1off, olw, h)), wb1, 200, 255, cv::THRESH_BINARY);

          // HoughLinesP on white-pixel mask
          std::vector<cv::Vec4i> segs0, segs1;
          cv::HoughLinesP(wb0, segs0, 1, CV_PI / 180.0, 20, 50, 20);
          cv::HoughLinesP(wb1, segs1, 1, CV_PI / 180.0, 20, 50, 20);
          std::cout << "[Align] Line segments: cam0=" << segs0.size()
                    << " cam1=" << segs1.size() << std::endl;

          // Match lines by angle similarity, collect dy between midpoints
          auto seg_angle = [](const cv::Vec4i& s) {
              return std::atan2(float(s[3]-s[1]), float(s[2]-s[0])) * 180.f / CV_PI;
          };
          auto seg_midy = [](const cv::Vec4i& s) {
              return (s[1] + s[3]) * 0.5f;
          };

          const float ANGLE_TOL = 8.0f;
          std::vector<float> dy_candidates;
          for (const auto& s0 : segs0) {
              float a0 = seg_angle(s0);
              float best_da = ANGLE_TOL + 1.f;
              float best_dy = 0.f;
              for (const auto& s1 : segs1) {
                  float da = std::abs(a0 - seg_angle(s1));
                  if (da > 90.f) da = 180.f - da;
                  if (da < best_da) {
                      best_da = da;
                      best_dy = (seg_midy(s0) + y0off) - (seg_midy(s1) + y1off);
                  }
              }
              if (best_da < ANGLE_TOL)
                  dy_candidates.push_back(best_dy);
          }
          std::cout << "[Align] Matched line pairs: " << dy_candidates.size() << std::endl;

          // ── Pick best correction ──────────────────────────────────────────
          int apply_dx = 0, apply_dy = 0;
          bool used_lines = false;

          if (dy_candidates.size() >= 3) {
              // Geometry-based: use median dy from matched field lines
              std::sort(dy_candidates.begin(), dy_candidates.end());
              float median_dy = dy_candidates[dy_candidates.size() / 2];
              apply_dy = static_cast<int>(std::round(median_dy));
              used_lines = true;
              std::cout << "[Align] Using line matching: dy=" << apply_dy
                        << " (from " << dy_candidates.size() << " pairs)" << std::endl;

              // Save debug image: detected lines in both overlap strips
              cv::Mat vis0 = ov0.clone(), vis1 = ov1.clone();
              for (const auto& s : segs0)
                  cv::line(vis0, {s[0],s[1]}, {s[2],s[3]}, {0,255,0}, 2);
              for (const auto& s : segs1)
                  cv::line(vis1, {s[0],s[1]}, {s[2],s[3]}, {0,255,0}, 2);
              cv::Mat vis;
              cv::vconcat(vis0, vis1, vis);
              std::string dbg_path = debug_folder_ + "/debug_line_align.jpg";
              cv::imwrite(dbg_path, vis);
              std::cout << "[Align] Saved line debug: " << dbg_path << std::endl;

          } else if (pc_response > 0.05) {
              // Intensity-based fallback
              apply_dx = static_cast<int>(std::round(pc_shift.x));
              apply_dy = static_cast<int>(std::round(pc_shift.y));
              std::cout << "[Align] Using phase correlation: dx=" << apply_dx
                        << " dy=" << apply_dy << std::endl;
          } else {
              std::cout << "[Align] No reliable alignment signal — keeping calibration corners" << std::endl;
          }

          if (apply_dx != 0 || apply_dy != 0) {
              corners_[1].x += apply_dx;
              corners_[1].y += apply_dy;
              if (std::abs(apply_dy) > 0)
                  image_warped_size_vector_[1].height = std::max(
                      1, image_warped_size_vector_[1].height - std::abs(apply_dy));
              std::cout << "[Align] Applied: used_lines=" << used_lines
                        << " dx=" << apply_dx << " dy=" << apply_dy << std::endl;
          }
      }
  }

  // Compute exposure compensation gains from warped sample images
  exposure_compensator_ = detail::ExposureCompensator::createDefault(expos_comp_type);
  detail::GainCompensator* gain_comp = dynamic_cast<detail::GainCompensator*>(exposure_compensator_.get());
  detail::BlocksGainCompensator* block_comp = dynamic_cast<detail::BlocksGainCompensator*>(exposure_compensator_.get());
  if (gain_comp) {
      gain_comp->setNrFeeds(expos_comp_nr_feeds);
  }
  if (block_comp) {
      block_comp->setNrFeeds(expos_comp_nr_feeds);
      block_comp->setBlockSize(expos_comp_block_size, expos_comp_block_size);
      block_comp->setNrGainsFilteringIterations(expos_comp_nr_filtering);
      std::cout << "[InitWarper] Using block-based gain compensation (block=" << expos_comp_block_size << ")" << std::endl;
  }
  exposure_compensator_->feed(corners_, images_warped, mask_warped_vector_);
  if (gain_comp) {
      for (int i = 0; i < num_img_; ++i) {
          std::cout << "[InitWarper] Exposure gain for camera " << i << ": " << gain_comp->gains()[i] << std::endl;
      }
  }

  // Apply seam finding for smooth blending transitions
  // This finds the optimal cut path between images to minimize visible seams
  Ptr<SeamFinder> seam_finder;
  if (seam_find_type == "no")
      seam_finder = makePtr<NoSeamFinder>();
  else if (seam_find_type == "voronoi")
      seam_finder = makePtr<VoronoiSeamFinder>();
  else if (seam_find_type == "gc_color")
      seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
  else if (seam_find_type == "gc_colorgrad")
      seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
  else if (seam_find_type == "dp_color")
      seam_finder = makePtr<DpSeamFinder>(DpSeamFinder::COLOR);
  else if (seam_find_type == "dp_colorgrad")
      seam_finder = makePtr<DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
  else
      seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);

  // Convert warped images to float for seam finding
  std::vector<UMat> images_warped_f(num_img_);
  for (int i = 0; i < num_img_; ++i)
      images_warped[i].convertTo(images_warped_f[i], CV_32F);

  seam_finder->find(images_warped_f, corners_, mask_warped_vector_);
  std::cout << "[InitWarper] Applied seam finder: " << seam_find_type << std::endl;

  // Dilate the seam masks slightly to create smoother gradient transitions
  // This gives the multi-band blender more overlap to work with
  for (int i = 0; i < num_img_; ++i) {
      cv::dilate(mask_warped_vector_[i], mask_warped_vector_[i],
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  }

  timelapser_ = Timelapser::createDefault(timelapse_type);
  
  // Setup blender
  blender_ = Blender::createDefault(blend_type);
  cv::detail::MultiBandBlender* mb_blender = dynamic_cast<cv::detail::MultiBandBlender*>(blender_.get());
  if (mb_blender) {
      // More bands = wider & smoother gradient transition between images.
      // 7 bands gives a very wide, smooth blend zone that hides seams well.
      // Each band represents a frequency level - more bands = smoother exponential falloff.
      int num_bands = 7;
      mb_blender->setNumBands(num_bands);
      std::cout << "[InitWarper] Using multi-band blender with " << mb_blender->numBands() << " bands" << std::endl;
  }
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
  std::ofstream debug_roi(debug_folder_ + "/debug_roi_info.txt");
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
    std::string warped_path = debug_folder_ + "/debug_warped_" + std::to_string(i) + ".jpg";
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
    cv::Mat K_float, R_identity;
    k_vector[i].getMat(cv::ACCESS_READ).convertTo(K_float, CV_32F);

    // Scale K from calibration resolution to actual image resolution.
    // Calibration was performed at a different resolution than the recording —
    // focal length and principal point are in calibration-pixel units and must
    // be rescaled before use with full-size frames.
    cv::Size actual_size = image_size_vector_[i];
    double sx = static_cast<double>(actual_size.width)  / resolution.width;
    double sy = static_cast<double>(actual_size.height) / resolution.height;
    K_float.at<float>(0, 0) *= sx;  // fx
    K_float.at<float>(1, 1) *= sy;  // fy
    K_float.at<float>(0, 2) *= sx;  // cx
    K_float.at<float>(1, 2) *= sy;  // cy
    std::cout << "[InitUndistortMap] Scaling K by (" << sx << ", " << sy
              << ") from " << resolution << " → " << actual_size << std::endl;

    // Store the scaled K back so InitCameraParam can read it cleanly
    // (both functions load from YAML independently, so we scale in both places)
    k_vector[i] = cv::UMat();
    K_float.copyTo(k_vector[i]);

    R_identity = cv::Mat::eye(3, 3, CV_32F);

    cv::initUndistortRectifyMap(
        K_float, d_vector[i], R_identity, K_float, actual_size,
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