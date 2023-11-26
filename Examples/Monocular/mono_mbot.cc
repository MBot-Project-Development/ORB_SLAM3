/**
* This file is adapted from mono_realsense_t265.cc, a part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;


void init_camera(cv::VideoCapture& video, int sensor_id, int width, int height, int framerate){
    std::string camera_str = "nvarguscamerasrc sensor_id=";
    camera_str += std::to_string(sensor_id);
    camera_str += " ! video/x-raw(memory:NVMM), width=";
    camera_str += std::to_string(width);
    camera_str += ", height=";
    camera_str += std::to_string(height);
    camera_str += ", format=(string)NV12, framerate=";
    camera_str += std::to_string(framerate);
    camera_str += "/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    // std::cout << "Camera string is \n" << camera_str << "\n";
    video = cv::VideoCapture(camera_str);
}

bool b_continue_session;

void exit_loop_handler(int s){
   cout << "Finishing session" << endl;
   b_continue_session = false;
}

int main(int argc, char **argv)
{
    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: ./mono_mbot path_to_vocabulary path_to_settings (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4)
    {
        file_name = string(argv[argc-1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    cv::VideoCapture cap;
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, false); //true);
    try{
        init_camera(cap, 0, 640, 480, 10);
        if(!cap.isOpened()){
            std::cerr << "Error opening the video capture." << std::endl;
            cap.release();
            return -1;
        }

        cout.precision(17);

        /*cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl;
        cout << "IMU data in the sequence: " << nImu << endl << endl;*/

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        
        float imageScale = SLAM.GetImageScale();

        cv::Mat imCV;

        int width_img = 640; 
        int height_img = 480;

        double t_resize = 0.f;
        double t_track = 0.f;

        while(b_continue_session)
        {
            cap >> imCV;
            auto time = std::chrono::system_clock::now();
            auto durationSinceEpoch = time.time_since_epoch();
            double timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(durationSinceEpoch).count();
            if(imCV.empty()){
                cout << "Frame is empty, breaking...\n";
                break;
            }
            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
                int width = imCV.cols * imageScale;
                int height = imCV.rows * imageScale;
                cv::resize(imCV, imCV, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }


#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
#endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(imCV, timestamp_ms);

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        }
    }catch(const std::exception& ex){
        std::cout << "Error caught, exiting: " << ex.what() << "\n";
        cap.release();
        SLAM.Shutdown();
        return 1;
    }
    cap.release();
    SLAM.Shutdown();

    return 0;
}
