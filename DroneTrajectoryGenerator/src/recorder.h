#ifndef _RECORDER_H_
#define _RECORDER_H_

#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <utility>
#include <mutex>

// AirSim includes
#include "common/Common.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "utils.h"

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace msr::airlib;

// Record data (GT poses, images, depth) from airsim client to file
class Recorder {
public:
    Recorder(const std::string &out_dir_="/media/nate/Elements/the_world")
            : out_dir(out_dir_) {

        assert(target_gt_rate >= target_cam_rate);

        // Setup file IO
        cam_dir = out_dir + "/cam0";
        depth_dir = out_dir + "/depth0";
        std::filesystem::create_directories(out_dir);
        std::filesystem::create_directories(cam_dir);
        std::filesystem::create_directories(depth_dir);

        std::string gt_fname = curr_dir + "/groundtruth.txt";
        gt_file.open(gt_fname);
        if (gt_file.fail()) {
            fprintf(stderr, "Unable to open GT file %s!!", gt_fname.c_str());
            std::exit(EXIT_FAILURE);
        }
        // Position, quat orientation (ItoG Hamilton or GtoI JPL), velocity,
        // acceleration, angular velocity
        gt_file << "# timestamp(ns) tx ty tz qx qy qz qw vx vy vz az ay az wx wy wz" << std::endl;
    }
    
    ~Recorder() {
        if (gt_file.open())
            gt_file.close();
    }

    void record() {
        client_.simPause(true);
        write_intrinsics();
        while (!finished()) {
            if (curr_sim_time == next_gt_time) {
                write_gt();
                next_gt_time += 1e9 / target_gt_rate;
            }          
            if (curr_sim_time == next_cam_time) {
                write_rgbd();
                next_cam_time += 1e9 / target_cam_rate;
            }

            // Now increment the current sim time to jump
            // to whichever next sensor time comes next
            long next_sim_time = std::min(next_gt_time, next_cam_time);
            long dt = next_sim_time - curr_sim_time;
            assert(dt > 0); // Bug if fail 
            client_.simContinueForTime(1e-9 * (double)dt);
            curr_sim_time = next_sim_time;
        }
    }
 
    inline bool finished() const {
        std::unique_lock<std::mutex> mtx(lock_);
        return finished_;
    }

    inline void finish() {
        std::unique_lock<std::mutex> mtx(lock_);
        finished_ = true;
    }   

protected:

    void write_intrinsics() {
        
    }

    void write_gt() {
        Kinematics::State gt = client_.simGetGroundTruthKinematics();
        //# timestamp(ns) tx ty tz qx qy qz qw vx vy vz az ay az wx wy wz
        gt_file << curr_sim_time << " ";
        gt_file << gt.pose.position.x() << " " 
                << gt.pose.position.y() << " " 
                << gt.pose.position.z() << " ";
        gt_file << gt.pose.orientation.x() << " " 
                << gt.pose.orientation.y() << " " 
                << gt.pose.orientation.z() << " "
                << gt.pose.orientation.w() << " ";
        gt_file << gt.twist.linear.x() << " " 
                << gt.twist.linear.y() << " " 
                << gt.twist.linear.z() << " ";
        gt_file << gt.accelerations.linear.x() << " " 
                << gt.accelerations.linear.y() << " " 
                << gt.accelerations.linear.z() << " ";
        gt_file << gt.twist.angular.x() << " " 
                << gt.twist.angular.y() << " " 
                << gt.twist.angular.z() << std::endl;

        //printf("DEBUG: curr_sim_time=%ld, gt_time=%ld\n",
        //        curr_sim_time, gt.time_stamp);
    }

    void write_rgbd() {
        std::vector<ImageRequest> request = {
                ImageRequest("cam0", ImageType::Scene), 
                ImageRequest("cam0", ImageType::DepthPlanner, true) 
        };
        const std::vector<ImageResponse> &responses = client_.simGetImages(request);
        std::string time_str = std::to_string(curr_sim_time);
        for (const ImageResponse &response : responses) {
            if (response.pixels_as_float) {
                assert((int)response.image_data_float.size() 
                        == response.height * response.width);
                cv::Mat depth(response.height, response.width, CV_32FC1, 
                        response.image_data_float.data());
                // Convert to uint16 in mm for PNG
                depth = 1000 * depth;
                depth.convertTo(depth, CV_16UC1);
                printf("DEBUG: curr_sim_time=%ld, depth_time=%ld\n",
                        curr_sim_time, response.time_stamp);
                cv::imwrite(depth_dir + "/" + time_str + ".png", depth);
            } else {
                assert((int)response.image_data_uint8.size() 
                        == response.height * response.width * 3);
                cv::Mat image(response.height, response.width, CV_8UC3, 
                        response.image_data_uint8.data());
                printf("DEBUG: curr_sim_time=%ld, image_time=%ld\n",
                        curr_sim_time, response.time_stamp);
                cv::imwrite(cam_dir + "/" + time_str + ".jpg", image);
            }
        }
    }

    // Writing info
    std::string out_dir;
    std::string curr_dir;
    std::string cam_dir, depth_dir;
    std::ofstream gt_file;
    bool intr_written = false;
    
    long target_gt_rate = 1000;
    long target_cam_rate = 30;
    // We have to make our own timestamps (nanosec)
    long curr_sim_time = 0;
    long next_gt_time = 0;
    long next_cam_time = 0;

    msr::airlib::MultirotorRpcLibClient client_;

    std::mutex lock_;
    bool finished_;
};

#endif
