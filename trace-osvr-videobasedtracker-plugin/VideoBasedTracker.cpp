/** @file
    @brief Implementation

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// 	http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include "VideoBasedTracker.h"
#include <CameraDistortionModel.h>
#include <UndistortMeasurements.h>
#include <cvToEigen.h>
#include <osvr/Util/CSV.h>
#include <osvr/Util/EigenCoreGeometry.h>

// Library/third-party includes
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Standard includes
#include <algorithm>
#include <fstream>
#include <iostream>

namespace osvr {
namespace vbtracker {

    VideoBasedTracker::VideoBasedTracker(ConfigParams const &params)
        : m_params(params), m_blobExtractor(params.blobParams) {}

    // This version requires YOU to add your beacons! You!
    void VideoBasedTracker::addSensor(
        LedIdentifierPtr &&identifier, CameraParameters const &camParams,
        std::function<void(BeaconBasedPoseEstimator &)> const &beaconAdder,
        size_t requiredInliers, size_t permittedOutliers) {
        m_camParams = camParams;
        m_identifiers.emplace_back(std::move(identifier));
        m_estimators.emplace_back(new BeaconBasedPoseEstimator(
            camParams.createUndistortedVariant(), requiredInliers,
            permittedOutliers, m_params));
        m_led_groups.emplace_back();
        beaconAdder(*m_estimators.back());
        m_assertInvariants();
    }

    void VideoBasedTracker::addSensor(
        LedIdentifierPtr &&identifier, CameraParameters const &camParams,
        Point3Vector const &locations, Vec3Vector const &emissionDirection,
        std::vector<double> const &variance,
        BeaconIDPredicate const &autocalibrationFixedPredicate,
        size_t requiredInliers, size_t permittedOutliers,
        double beaconAutocalibErrorScale) {
        addSensor(std::move(identifier), camParams,
                  [&](BeaconBasedPoseEstimator &estimator) {
                      estimator.SetBeacons(locations, emissionDirection,
                                           variance,
                                           autocalibrationFixedPredicate,
                                           beaconAutocalibErrorScale);
                  },
                  requiredInliers, permittedOutliers);
    }

    void VideoBasedTracker::dumpKeypointDebugData(
        std::vector<cv::KeyPoint> const &keypoints) {
        {
            std::cout << "Dumping blob detection debug data, capture frame "
                      << m_debugFrame << std::endl;
            cv::imwrite("debug_rawimage" + std::to_string(m_debugFrame) +
                            ".png",
                        m_frame);
            cv::imwrite("debug_blobframe" + std::to_string(m_debugFrame) +
                            ".png",
                        m_imageWithBlobs);
            cv::imwrite("debug_thresholded" + std::to_string(m_debugFrame) +
                            ".png",
                        m_thresholdImage);
        }

        {
            auto filename = std::string{"debug_data" +
                                        std::to_string(m_debugFrame) + ".txt"};
            std::ofstream datafile{filename.c_str()};
            datafile << "MinThreshold: " << m_sbdParams.minThreshold
                     << std::endl;
            datafile << "MaxThreshold: " << m_sbdParams.maxThreshold
                     << std::endl;
            datafile << "ThresholdStep: " << m_sbdParams.thresholdStep
                     << std::endl;
            datafile << "Thresholds:" << std::endl;
            for (double thresh = m_sbdParams.minThreshold;
                 thresh < m_sbdParams.maxThreshold;
                 thresh += m_sbdParams.thresholdStep) {
                datafile << thresh << std::endl;
            }
        }
        {
            using namespace osvr::util;
            CSV kpcsv;
            for (auto &keypoint : keypoints) {
                kpcsv.row() << cell("x", keypoint.pt.x)
                            << cell("y", keypoint.pt.y)
                            << cell("size", keypoint.size);
            }
            auto filename = std::string{"debug_blobdetect" +
                                        std::to_string(m_debugFrame) + ".csv"};
            std::ofstream csvfile{filename.c_str()};
            kpcsv.output(csvfile);
            csvfile.close();
        }
        std::cout << "Data dump complete." << std::endl;
        m_debugFrame++;
    }

bool VideoBasedTracker::processImage(cv::Mat frame, cv::Mat grayImage,
                                     OSVR_TimeValue const &tv,
                                     PoseHandler handler) {
    bool done = false;
    m_frame = frame;
    m_imageGray = grayImage;
    auto foundLeds = m_blobExtractor.extractBlobs(grayImage); // blob 偵測

    auto undistortedLeds = undistortLeds(foundLeds, m_camParams); // 去除扭曲

    // We allow multiple sets of LEDs, each corresponding to a different
    // sensor, to be located in the same image.  We construct a new set
    // of LEDs for each and try to find them.  It is assumed that they all
    // have unique ID patterns across all sensors.
    for (size_t sensor = 0; sensor < m_identifiers.size(); sensor++) {

        osvrPose3SetIdentity(&m_pose);
        auto ledsMeasurements = undistortedLeds; // 複製一份

        // Locate the closest blob from this frame to each LED found
        // in the previous frame.  If it is close enough to the nearest
        // neighbor from last time, we assume that it is the same LED and
        // update it.  If not, we delete the LED from the list.  Once we
        // have matched a blob to an LED, we remove it from the list.  If
        // there are any blobs leftover, we create new LEDs from them.
        // @todo: Include motion estimate based on Kalman filter along with
        // model of the projection once we have one built.  Note that this
        // will require handling the lens distortion appropriately.
        
        // 第一次從 myLeds 取出來的 led 為空, while 就跳過不做, 所以全部的 ledsMeasurements 都
        // 
        {
            auto &myLeds = m_led_groups[sensor];
            auto led = begin(myLeds);
            while (led != end(myLeds)) {
                led->resetUsed();
                auto threshold = m_params.blobMoveThreshold *
                                 led->getMeasurement().diameter;
                auto nearest = led->nearest(ledsMeasurements, threshold); // 找最近的
                if (nearest == end(ledsMeasurements)) {
                    // We have no blob corresponding to this LED, so we need
                    // to delete this LED.
                    // 如果找到最後都沒有找到對應的 LED, 就把 led 從 myLeds 裡移除
                    led = myLeds.erase(led);
                } else {
                    // Update the values in this LED and then go on to the
                    // next one. Remove this blob from the list of potential matches.
                    // 因為已經這個 blob 是哪個 LED, 所以從 ledsMeasurements 裡移除這個 LED
                    led->addMeasurement(*nearest, m_params.blobsKeepIdentity);
                    ledsMeasurements.erase(nearest);
                    ++led;
                }
            } // while
            // If we have any blobs that have not been associated with an
            // LED, then we add a new LED for each of them.
            // std::cout << "Had " << Leds.size() << " LEDs, " <<
            // keyPoints.size() << " new ones available" << std::endl;
            for (auto &remainingLed : ledsMeasurements) {
                myLeds.emplace_back(m_identifiers[sensor].get(),
                                    remainingLed);
            }
        }
        //==================================================================
        // Compute the pose of the HMD w.r.t. the camera frame of
        // reference.
        bool gotPose = false;
        if (m_estimators[sensor]) {

            // Get an estimated pose, if we have enough data.
            OSVR_PoseState pose;
            if (m_estimators[sensor]->EstimatePoseFromLeds( // 估算 Pose
                    m_led_groups[sensor], tv, pose)) {
                m_pose = pose;
                handler(static_cast<unsigned>(sensor), pose); // 送 Pose 出去
                gotPose = true;
            }
        }
    } // for

    return done;
}

    void VideoBasedTracker::drawLedCircleOnStatusImage(Led const &led,
                                                       bool filled,
                                                       cv::Vec3b color) {
        cv::circle(m_statusImage, led.getLocation(),
                   led.getMeasurement().diameter / 2., cv::Scalar(color),
                   filled ? -1 : 1);
    }

    void VideoBasedTracker::drawRecognizedLedIdOnStatusImage(Led const &led) {

        auto label = std::to_string(led.getOneBasedID());
        cv::putText(m_statusImage, label, led.getLocation(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(127, 127, 127));
    }

} // namespace vbtracker
} // namespace osvr
