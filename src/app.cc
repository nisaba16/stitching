#include "app.h"

#include <iostream>
#include <thread>
#include <stack>
#include <climits>
#include <cstdlib>
#include <algorithm>
#include <opencv2/videoio.hpp>
#include "stitching_param_generator.h"
#include <opencv2/imgproc.hpp>
#include <chrono>

App::App(const std::vector<std::string>& video_files, const std::string& output_folder, const std::string& file_name, double fps, bool dry_run, bool use_lir, bool use_calibration, const std::string& feature_method, bool use_feature_mask)
    : sensor_data_interface_(video_files), output_folder_(output_folder), file_name_(file_name), fps_(fps), dry_run_(dry_run), use_lir_(use_lir), use_calibration_(use_calibration), feature_method_(feature_method), use_feature_mask_(use_feature_mask) {

    sensor_data_interface_.InitVideoCapture();
    
    // Use detected FPS from input video if available, otherwise use provided fps
    double detected_fps = sensor_data_interface_.getFPS();
    if (detected_fps > 0) {
        std::cout << "[App] Using detected FPS: " << detected_fps << " (provided: " << fps_ << ")" << std::endl;
        fps_ = detected_fps;
    }

    std::vector<cv::UMat> first_image_vector(sensor_data_interface_.num_img_);
    std::vector<cv::Mat> first_mat_vector(sensor_data_interface_.num_img_);
    std::vector<cv::UMat> reproj_xmap_vector;
    std::vector<cv::UMat> reproj_ymap_vector;
    std::vector<cv::UMat> undist_xmap_vector;
    std::vector<cv::UMat> undist_ymap_vector;
    std::vector<cv::Rect> image_roi_vect;

    sensor_data_interface_.get_initial_images(first_image_vector);

    for (size_t i = 0; i < sensor_data_interface_.num_img_; ++i) {
        first_image_vector[i].copyTo(first_mat_vector[i]);
    }

    StitchingParamGenerator stitching_param_generator(first_mat_vector, use_calibration_, feature_method_, use_feature_mask_);

    stitching_param_generator.GetReprojParams(
        undist_xmap_vector,
        undist_ymap_vector,
        reproj_xmap_vector,
        reproj_ymap_vector,
        image_roi_vect
    );

    image_stitcher_.SetParams(
        undist_xmap_vector,
        undist_ymap_vector,
        reproj_xmap_vector,
        reproj_ymap_vector
    );
    
    exposure_compensator_ = stitching_param_generator.GetExposureCompensator();
    blender_ = stitching_param_generator.GetBlender();
    corners_ = stitching_param_generator.GetCorners();
    warped_sizes_ = stitching_param_generator.GetWarpedSizes();

    // Calculate the actual bounding box of all ROIs (they may overlap)
    int min_x = INT_MAX, max_x = 0;
    int min_y = INT_MAX, max_y = 0;
    for (size_t i = 0; i < sensor_data_interface_.num_img_; ++i) {
        const auto& roi = image_roi_vect[i];
        min_x = std::min(min_x, roi.x);
        max_x = std::max(max_x, roi.x + roi.width);
        min_y = std::min(min_y, roi.y);
        max_y = std::max(max_y, roi.y + roi.height);
        std::cout << "[App] ROI[" << i << "]: " << roi.width << "x" << roi.height 
                  << " at (" << roi.x << ", " << roi.y << ")" << std::endl;
    }
    total_cols_ = max_x - min_x;

    // Calculate output dimensions for info
    int frame_width = total_cols_;
    int frame_height = max_y - min_y;
    
    std::cout << "[App] Output frame size: " << frame_width << "x" << frame_height << std::endl;
    std::cout << "[App] Memory required: ~" << (frame_width * frame_height * 3 / 1024 / 1024) << " MB" << std::endl;
}

// Helper function to check if a pixel is similar to reference color
bool isSimilarColor(const cv::Vec3b& pixel, const cv::Vec3b& reference, int tolerance = 0) {
    return std::abs(pixel[0] - reference[0]) <= tolerance &&
           std::abs(pixel[1] - reference[1]) <= tolerance &&
           std::abs(pixel[2] - reference[2]) <= tolerance;
}


// Optimized version of findLargestInteriorRectangle
cv::Rect findLargestInteriorRectangle(const cv::Mat& image, bool use_approximate = false, bool debug = false) {
    cv::Mat binary;
    cv::cvtColor(image, binary, cv::COLOR_BGR2GRAY);
    
    cv::GaussianBlur(binary, binary, cv::Size(5, 5), 0);
    cv::threshold(binary, binary, 1, 255, cv::THRESH_BINARY);

    int padding_h = use_approximate ? 50 : 0;   
    int padding_v = use_approximate ? 150 : 0;

    // Find the largest rectangle
    cv::Rect largest_rect(0, 0, 0, 0);
    int max_area = 0;
        
    // Pre-compute horizontal histograms for each row
    std::vector<std::vector<int>> h_lengths(binary.rows, std::vector<int>(binary.cols));
    
    // First row
    for(int x = 0; x < binary.cols; x++) {
        h_lengths[0][x] = binary.at<uchar>(0, x) == 255 ? 1 : 0;
    }
    
    // Remaining rows
    for(int y = 1; y < binary.rows; y++) {
        for(int x = 0; x < binary.cols; x++) {
            if(binary.at<uchar>(y, x) == 255) {
                h_lengths[y][x] = h_lengths[y-1][x] + 1;
            } else {
                h_lengths[y][x] = 0;
            }
        }
    }

    // For each row, find largest rectangle
    for(int y = 0; y < binary.rows; y++) {
        std::vector<int> heights = h_lengths[y];
        std::stack<int> s;
        int i = 0;
        
        while(i < binary.cols) {
            if(s.empty() || heights[s.top()] <= heights[i]) {
                s.push(i++);
            } else {
                int height = heights[s.top()];
                s.pop();
                int width = s.empty() ? i : i - s.top() - 1;
                int area = height * width;
                if(area > max_area) {
                    max_area = area;
                    largest_rect = cv::Rect(
                        s.empty() ? 0 : s.top() + 1,
                        y - height + 1,
                        width,
                        height
                    );
                }
            }
        }
        
        while(!s.empty()) {
            int height = heights[s.top()];
            s.pop();
            int width = s.empty() ? i : i - s.top() - 1;
            int area = height * width;
            if(area > max_area) {
                max_area = area;
                largest_rect = cv::Rect(
                    s.empty() ? 0 : s.top() + 1,
                    y - height + 1,
                    width,
                    height
                );
            }
        }
    }

    // Add padding if in approximate mode
    if (use_approximate && !largest_rect.empty()) {
        int new_x = std::max(0, largest_rect.x - padding_h);
        int new_y = std::max(0, largest_rect.y - padding_v);
        int new_width = std::min(image.cols - new_x, largest_rect.width + 2 * padding_h);
        int new_height = std::min(image.rows - new_y, largest_rect.height + 2 * padding_v);
        largest_rect = cv::Rect(new_x, new_y, new_width, new_height);
    }

    // If in debug mode, create a visualization
    if (debug) {
        cv::Mat debug_vis;
        cv::cvtColor(binary, debug_vis, cv::COLOR_GRAY2BGR);  // Convert to BGR for colored rectangle
        
        // Draw the largest rectangle in red
        if (!largest_rect.empty()) {
            cv::rectangle(debug_vis, largest_rect, cv::Scalar(0, 0, 255), 2);  // Red color, thickness 2
        }
        
        cv::imwrite("binary_mask_with_rect.png", debug_vis);
        cv::imwrite("binary_mask.png", binary);
        
        // Also save the original frame with rectangle for reference
        cv::Mat original_with_rect = image.clone();
        if (!largest_rect.empty()) {
            cv::rectangle(original_with_rect, largest_rect, cv::Scalar(0, 0, 255), 2);
        }
        cv::imwrite("original_with_rect.png", original_with_rect);
    } else {
        cv::imwrite("binary_mask.png", binary);
    }

    return largest_rect;
}

void App::run_stitching() {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<cv::UMat> image_vector(sensor_data_interface_.num_img_);
    cv::Rect crop_rect;
    cv::Mat cropped_frame;
    cv::Mat blended_frame;
    cv::UMat warped_image, warped_mask;
    
    // Safeguard limits to prevent excessively large output videos
    const int MAX_OUTPUT_WIDTH = 4096;       // Maximum width in pixels
    const int MAX_OUTPUT_HEIGHT = 2160;      // Maximum height in pixels (4K)
    const long long MAX_OUTPUT_PIXELS = 8294400LL;  // Maximum total pixels (~4K: 3840x2160)
    double scale_factor = 1.0;

    // Rewind videos to beginning since get_initial_images consumed first frame
    sensor_data_interface_.rewind_videos();

    // Get first frame for stitching
    sensor_data_interface_.get_image_vector(image_vector);

    // Process single frame for dry run
    for (size_t img_idx = 0; img_idx < sensor_data_interface_.num_img_; ++img_idx) {
        image_stitcher_.warpImage(
            image_vector[img_idx],
            img_idx,
            warped_image,
            warped_mask
        );
        // Apply exposure compensation to equalize brightness between cameras
        exposure_compensator_->apply(img_idx, corners_[img_idx], warped_image, warped_mask);
        // Blender::feed() requires CV_16SC3
        cv::UMat warped_16s;
        warped_image.convertTo(warped_16s, CV_16SC3);
        blender_->feed(warped_16s, warped_mask, corners_[img_idx]);
    }

    cv::Mat blended_mask;
    blender_->blend(blended_frame, blended_mask);
    // blend() outputs CV_16SC3 — convert back to CV_8UC3 for display/save
    blended_frame.convertTo(blended_frame, CV_8UC3);
    // Re-prepare blender for subsequent frames
    blender_->prepare(corners_, warped_sizes_);


    if (dry_run_) {
        // Save the full panorama before cropping
        std::string full_image_path = output_folder_ + "/dry_run_full_" + file_name_ + ".jpg";
        std::cout << "[App] Saving dry run full image to: " << full_image_path << std::endl;
        if (!cv::imwrite(full_image_path, blended_frame)) {
            std::cerr << "[App] Failed to save full panorama image!" << std::endl;
        }
    }

    // Apply LIR if requested
    if (use_lir_) {
        crop_rect = findLargestInteriorRectangle(blended_frame, true, dry_run_);  // Enable debug output in dry run
        cropped_frame = blended_frame(crop_rect);
        
        if (dry_run_) {
            // Save additional debug information
            std::cout << "LIR Debug Info:" << std::endl;
            std::cout << "Crop rectangle: x=" << crop_rect.x << ", y=" << crop_rect.y 
                      << ", width=" << crop_rect.width << ", height=" << crop_rect.height << std::endl;
            std::cout << "Original image size: " << blended_frame.cols << "x" << blended_frame.rows << std::endl;
            
            // Calculate and output the crop percentage
            double crop_area = crop_rect.width * crop_rect.height;
            double total_area = blended_frame.cols * blended_frame.rows;
            double crop_percentage = (crop_area / total_area) * 100.0;
            std::cout << "Crop percentage: " << crop_percentage << "% of original area" << std::endl;
        }
    } else {
        cropped_frame = blended_frame;  // Use full image without cropping
    }

    // Safeguard against excessively large frames
    // Check width, height, and total pixel count
    long long total_pixels = static_cast<long long>(cropped_frame.cols) * cropped_frame.rows;
    
    if (cropped_frame.cols > MAX_OUTPUT_WIDTH || 
        cropped_frame.rows > MAX_OUTPUT_HEIGHT || 
        total_pixels > MAX_OUTPUT_PIXELS) {
        
        // Calculate scale factor based on the most restrictive constraint
        double width_scale = static_cast<double>(MAX_OUTPUT_WIDTH) / cropped_frame.cols;
        double height_scale = static_cast<double>(MAX_OUTPUT_HEIGHT) / cropped_frame.rows;
        double pixel_scale = std::sqrt(static_cast<double>(MAX_OUTPUT_PIXELS) / total_pixels);
        
        scale_factor = std::min({width_scale, height_scale, pixel_scale, 1.0});
        
        cv::Mat scaled_frame;
        cv::resize(cropped_frame, scaled_frame, cv::Size(), scale_factor, scale_factor, cv::INTER_AREA);
        
        std::cout << "[App] WARNING: Frame size exceeds safe limits!" << std::endl;
        std::cout << "[App]   Original: " << cropped_frame.cols << "x" << cropped_frame.rows 
                  << " (" << (total_pixels / 1000000.0) << " MP)" << std::endl;
        std::cout << "[App]   Scaling down by " << scale_factor << " to " 
                  << scaled_frame.cols << "x" << scaled_frame.rows << std::endl;
        
        cropped_frame = scaled_frame;
    }

    if (dry_run_) {
        // Save the final panorama (cropped or full)
        std::string image_path = output_folder_ + "/dry_run_" + file_name_ + ".jpg";
        std::cout << "[App] Saving dry run final image to: " << image_path << std::endl;
        if (!cv::imwrite(image_path, cropped_frame)) {
            std::cerr << "[App] Failed to save final panorama image!" << std::endl;
        } else {
            std::cout << "[App] Dry run complete. Image saved successfully." << std::endl;
        }
        return;
    }

    // Initialize video writer with appropriate dimensions
    cv::Size frame_size(cropped_frame.cols, cropped_frame.rows);

    // Write to a temporary AVI file with MJPG codec (fast, well-supported)
    // We'll re-encode with FFmpeg at the end for proper H.264 compression
    std::string temp_output = output_folder_ + "/.tmp_" + file_name_ + ".avi";
    std::string final_output = output_folder_ + "/" + file_name_ + ".mp4";

    video_writer_.open(temp_output,
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                      fps_,
                      frame_size);

    if (!video_writer_.isOpened()) {
        throw std::runtime_error("Could not open the temporary video file for write: " + temp_output);
    }

    // Estimate and warn about potential file size
    double total_frames = sensor_data_interface_.getTotalFrames();
    double bytes_per_frame = frame_size.width * frame_size.height * 3;  // RGB uncompressed
    double estimated_raw_size_mb = (bytes_per_frame * total_frames) / (1024.0 * 1024.0);
    double estimated_compressed_size_mb = estimated_raw_size_mb * 0.05;  // H.264 typically ~5% of raw
    
    std::cout << "[App] Video output info:" << std::endl;
    std::cout << "[App]   Resolution: " << frame_size.width << "x" << frame_size.height << std::endl;
    std::cout << "[App]   Total frames: " << total_frames << std::endl;
    std::cout << "[App]   Estimated final size: ~" << static_cast<int>(estimated_compressed_size_mb) << " MB" << std::endl;
    
    if (estimated_compressed_size_mb > 1024) {
        std::cout << "[App]   WARNING: Output video may exceed 1 GB!" << std::endl;
    }

    // Write first frame
    // Ensure frame is in correct format (CV_8U) before writing
    if (cropped_frame.type() != CV_8UC3) {
        cropped_frame.convertTo(cropped_frame, CV_8UC3);
    }
    video_writer_.write(cropped_frame);

    // Continue with remaining frames for normal run
    size_t frame_count = 1;

    while (!dry_run_) {
        sensor_data_interface_.get_image_vector(image_vector);

        if (sensor_data_interface_.all_videos_finished()) {
            break;
        }

        for (size_t img_idx = 0; img_idx < sensor_data_interface_.num_img_; ++img_idx) {
            image_stitcher_.warpImage(
                image_vector[img_idx],
                img_idx,
                warped_image,
                warped_mask
            );
            // Apply exposure compensation to equalize brightness between cameras
            exposure_compensator_->apply(img_idx, corners_[img_idx], warped_image, warped_mask);
            // Blender::feed() requires CV_16SC3
            cv::UMat warped_16s;
            warped_image.convertTo(warped_16s, CV_16SC3);
            blender_->feed(warped_16s, warped_mask, corners_[img_idx]);
        }

        blender_->blend(blended_frame, blended_mask);
        // blend() outputs CV_16SC3 — convert back to CV_8UC3
        blended_frame.convertTo(blended_frame, CV_8UC3);
        // Re-prepare blender for next frame (blend() consumes its internal buffers)
        blender_->prepare(corners_, warped_sizes_);
        cropped_frame = use_lir_ ? blended_frame(crop_rect) : blended_frame;

        // Apply same scaling as the first frame
        if (scale_factor != 1.0) {
            cv::Mat scaled_frame;
            cv::resize(cropped_frame, scaled_frame, cv::Size(), scale_factor, scale_factor, cv::INTER_AREA);
            cropped_frame = scaled_frame;
        }
        
        // Ensure frame is in correct format (CV_8U) before writing
        if (cropped_frame.type() != CV_8UC3) {
            cropped_frame.convertTo(cropped_frame, CV_8UC3);
        }
        video_writer_.write(cropped_frame);

        frame_count++;
        if (frame_count % static_cast<size_t>(fps_ * 5) == 0) {
            auto current_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_seconds = current_time - start_time;
            double fps = frame_count / elapsed_seconds.count();
            double progress_percentage = (frame_count / total_frames) * 100;
            double estimated_total_time = total_frames / fps;
            double time_remaining = estimated_total_time - elapsed_seconds.count();

            std::cout << "PROGRESS:" << static_cast<int>(progress_percentage) << std::endl;
            std::cout << "FPS: " << fps << ", Progress: " << progress_percentage << "%"
                      << ", Time Remaining: " << time_remaining << " seconds" << std::endl;
        }
    }

    if (video_writer_.isOpened()) {
        video_writer_.release();
    }

    // Re-encode with FFmpeg for proper H.264 compression
    // CRF 23 = visually transparent quality, good compression ratio
    // preset "medium" = good balance between speed and compression
    std::string ffmpeg_cmd = "ffmpeg -y -i \"" + temp_output + "\" "
                             "-c:v libx264 -crf 23 -preset medium "
                             "-pix_fmt yuv420p "
                             "-movflags +faststart "
                             "\"" + final_output + "\"";

    std::cout << "[App] Re-encoding with FFmpeg for compression..." << std::endl;
    std::cout << "[App] Command: " << ffmpeg_cmd << std::endl;

    int ret = std::system(ffmpeg_cmd.c_str());
    if (ret != 0) {
        std::cerr << "[App] WARNING: FFmpeg re-encoding failed (exit code " << ret << ")." << std::endl;
        std::cerr << "[App] Renaming raw file as output instead." << std::endl;
        std::rename(temp_output.c_str(), final_output.c_str());
    } else {
        // Remove temporary file
        std::remove(temp_output.c_str());
        std::cout << "[App] Re-encoding complete: " << final_output << std::endl;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 11) {  // At least 10 arguments + program name
        std::cerr << "Usage: " << argv[0] 
                  << " <output_folder> <file_name> <fps> <dry_run> <use_lir> <use_calibration> <feature_method> <use_feature_mask> <video_file1> <video_file2> ..." 
                  << std::endl;
        std::cerr << "  dry_run: true/false" << std::endl;
        std::cerr << "  use_lir: true/false" << std::endl;
        std::cerr << "  use_calibration: true (use YAML files) / false (calibrate on-the-fly)" << std::endl;
        std::cerr << "  feature_method: SIFT (accurate) / ORB (fast) / AKAZE (binary, robust)" << std::endl;
        std::cerr << "  use_feature_mask: true/false" << std::endl;
        return 1;
    }

    std::string output_folder = argv[1];
    std::string file_name = argv[2];
    double fps = std::stod(argv[3]);
    bool dry_run = (std::string(argv[4]) == "true");
    bool use_lir = (std::string(argv[5]) == "true");
    bool use_calibration = (std::string(argv[6]) == "true");
    std::string feature_method = argv[7];  // "SIFT", "ORB", or "LSD"
    bool use_feature_mask = (std::string(argv[8]) == "true");
    
    std::vector<std::string> video_files;
    for (int i = 9; i < argc; ++i) {
        video_files.push_back(argv[i]);
    }

    try {
        App app(video_files, output_folder, file_name, fps, dry_run, use_lir, use_calibration, feature_method, use_feature_mask);
        app.run_stitching();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
