#include "image_stitcher.h"
#include <iostream>

void ImageStitcher::SetParams(
    std::vector<cv::UMat>& undist_xmap_vector,
    std::vector<cv::UMat>& undist_ymap_vector,
    std::vector<cv::UMat>& reproj_xmap_vector,
    std::vector<cv::UMat>& reproj_ymap_vector) {

    std::cout << "[ImageStitcher] Setting params..." << std::endl;
    num_img_ = undist_xmap_vector.size();

    // Combine two remap operator (For speed up a little)
    final_xmap_vector_.resize(num_img_);
    final_ymap_vector_.resize(num_img_);

    for (size_t img_idx = 0; img_idx < num_img_; ++img_idx) {
        remap(undist_xmap_vector[img_idx],
              final_xmap_vector_[img_idx],
              reproj_xmap_vector[img_idx],
              reproj_ymap_vector[img_idx],
              cv::INTER_LINEAR);
        remap(undist_ymap_vector[img_idx],
              final_ymap_vector_[img_idx],
              reproj_xmap_vector[img_idx],
              reproj_ymap_vector[img_idx],
              cv::INTER_LINEAR);
    }
    std::cout << "[ImageStitcher] Setting params... Done." << std::endl;
}

void ImageStitcher::warpImage(
    const cv::UMat& image,
    int img_idx,
    cv::UMat& warped_image,
    cv::UMat& warped_mask) {

    // Remap with edge replication to avoid black pixels from saturation
    remap(image,
          warped_image,
          final_xmap_vector_[img_idx],
          final_ymap_vector_[img_idx],
          cv::INTER_LINEAR,
          cv::BORDER_REPLICATE);

    // Create a mask for the warped image
    warped_mask.create(warped_image.size(), CV_8U);
    warped_mask.setTo(cv::Scalar::all(255));
}
