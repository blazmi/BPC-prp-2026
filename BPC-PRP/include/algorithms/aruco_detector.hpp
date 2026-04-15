#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

namespace algorithms {

    class ArucoDetector {
    public:
        // Represents one detected marker
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;
        };

        ArucoDetector() {
            // Inicializace slovníku s 4x4 markery (50 možných ID)
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            // Inicializace výchozích parametrů detekce
            params_ = cv::aruco::DetectorParameters::create();
        }

        ~ArucoDetector() = default;

        // Detect markers in the input image
        std::vector<Aruco> detect(cv::Mat frame) {
            std::vector<Aruco> arucos;

            if (frame.empty()) return arucos;

            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            std::vector<std::vector<cv::Point2f>> rejected_candidates;

            // Detekce markerů pomocí klasického OpenCV ArUco API
            cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, params_, rejected_candidates);

            if (!marker_ids.empty()) {
                //std::cout << "Arucos found: ";
                for (size_t i = 0; i < marker_ids.size(); i++) {
                  //  std::cout << marker_ids[i] << " ";

                    // Vytvoření struktury Aruco a přidání do výsledného vektoru
                    Aruco detected_marker;
                    detected_marker.id = marker_ids[i];
                    detected_marker.corners = marker_corners[i];

                    arucos.push_back(detected_marker);
                }
                //std::cout << std::endl;
            }

            return arucos;
        }

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        cv::Ptr<cv::aruco::DetectorParameters> params_;
    };

} // namespace algorithms

#endif // ARUCO_DETECTOR_HPP