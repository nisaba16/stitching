#ifndef IMAGE_STITCHER_H
#define IMAGE_STITCHER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/stitching/detail/blenders.hpp"

class ImageStitcher {
public:
    void SetParams(
        std::vector<cv::UMat>& undist_xmap_vector,
        std::vector<cv::UMat>& undist_ymap_vector,
        std::vector<cv::UMat>& reproj_xmap_vector,
        std::vector<cv::UMat>& reproj_ymap_vector
    );

    void warpImage(
        const cv::UMat& image,
        int img_idx,
        cv::UMat& warped_image,
        cv::UMat& warped_mask
    );

private:
    int num_img_;
    std::vector<cv::UMat> final_xmap_vector_;
    std::vector<cv::UMat> final_ymap_vector_;
};

#endif // IMAGE_STITCHER_H