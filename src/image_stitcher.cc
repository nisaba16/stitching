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
        // When reproj coords fall outside the undist map, use -1 as the
        // sentinel value.  A negative coordinate is out-of-bounds for the
        // subsequent remap, so those pixels correctly become black / masked.
        remap(undist_xmap_vector[img_idx],
              final_xmap_vector_[img_idx],
              reproj_xmap_vector[img_idx],
              reproj_ymap_vector[img_idx],
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT,
              cv::Scalar(-1));
        remap(undist_ymap_vector[img_idx],
              final_ymap_vector_[img_idx],
              reproj_xmap_vector[img_idx],
              reproj_ymap_vector[img_idx],
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT,
              cv::Scalar(-1));
    }
    std::cout << "[ImageStitcher] Setting params... Done." << std::endl;
}

void ImageStitcher::warpImage(
    const cv::UMat& image,
    int img_idx,
    cv::UMat& warped_image,
    cv::UMat& warped_mask) {

    // Use BORDER_CONSTANT so out-of-bounds pixels become black
    // instead of being smeared from the nearest edge pixel.
    remap(image,
          warped_image,
          final_xmap_vector_[img_idx],
          final_ymap_vector_[img_idx],
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);

    // Build a proper validity mask: remap a solid-white image through
    // the same maps.  Pixels whose source falls outside the original
    // image will be 0 (black) thanks to BORDER_CONSTANT, giving the
    // blender an accurate region to work with.
    cv::UMat white_src(image.size(), CV_8U, cv::Scalar(255));
    remap(white_src,
          warped_mask,
          final_xmap_vector_[img_idx],
          final_ymap_vector_[img_idx],
          cv::INTER_NEAREST,
          cv::BORDER_CONSTANT);

    // Erode the mask to remove the interpolation fringe at edges.
    // Bilinear interpolation mixes valid pixels with black out-of-bounds
    // pixels in a ~2px band, creating dark/discolored borders.
    cv::erode(warped_mask, warped_mask,
              cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
}
