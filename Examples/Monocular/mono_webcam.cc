/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
* José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
* University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms
* of the GNU General Public License as published by the Free Software Foundation,
* either version 3 of the License, or (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <System.h>

using namespace std;

static volatile std::sig_atomic_t g_should_exit = 0;
static void OnSigInt(int) { g_should_exit = 1; }

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        cerr << endl
             << "Usage: ./mono_webcam path_to_vocabulary path_to_settings [camera_id=0] [trajectory_file_prefix]"
             << endl;
        return 1;
    }

    // NOTE(중요):
    // - ORB-SLAM3는 "학습 데이터로 학습(Training)"을 수행하는 파이프라인이 아닙니다.
    // - 따라서 "학습 데이터를 활용할지 말지"를 결정하는 구간은 없고,
    //   예제마다 "입력 소스"를 무엇으로 할지만 결정합니다.
    //   * 데이터셋 재생 예제: cv::imread + 파일 타임스탬프를 TrackMonocular(image, t)로 전달
    //   * 웹캠 실시간 예제(현재 파일): cv::VideoCapture로 캡처한 프레임 + 실시간 타임스탬프를 전달
    //
    // (추가) "미리 만든 맵으로 localization-only"는 아래 2가지가 핵심입니다.
    // - Atlas(맵) 로드/저장:
    //   * settings YAML에 `System.LoadAtlasFromFile: <path>`를 넣으면 System 생성 시 Atlas를 파일에서 로드합니다.
    //   * settings YAML에 `System.SaveAtlasToFile: <path>`를 넣으면 Shutdown() 시 Atlas를 파일로 저장합니다.
    //     (구현: src/System.cc에서 Shutdown() 때 SaveAtlas() 호출)
    // - Localization-only(Tracking-only) 모드:
    //   * `SLAM.ActivateLocalizationMode()`를 호출하면 LocalMapping(맵 빌드)을 멈추고 추적만 수행합니다.
    //   * 뷰어를 켠 경우, GUI의 `menu.Localization Mode` 토글로도 동일하게 켜고 끌 수 있습니다. (src/Viewer.cc)

    int camera_id = 0;
    if (argc >= 4) camera_id = std::stoi(argv[3]);

    const bool bUseViewer = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, bUseViewer);
    const float imageScale = SLAM.GetImageScale();

    cv::VideoCapture cap(camera_id);
    if (!cap.isOpened())
    {
        cerr << endl << "Failed to open camera id: " << camera_id << endl;
        return 1;
    }

    // In some environments (notably USB/IP + WSL2), requesting FHD without forcing a
    // compatible pixel format can lead to V4L2 timeouts. This camera exposes FHD via MJPG.
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FPS, 30);

    std::signal(SIGINT, OnSigInt);

    cv::Mat frame;
    const auto t_start = std::chrono::steady_clock::now();

    while (!g_should_exit)
    {
        if (!cap.read(frame) || frame.empty())
        {
            cerr << endl << "Failed to capture frame." << endl;
            break;
        }

        if (imageScale != 1.f)
        {
            const int width = static_cast<int>(frame.cols * imageScale);
            const int height = static_cast<int>(frame.rows * imageScale);
            cv::resize(frame, frame, cv::Size(width, height));
        }

        const double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(
                                  std::chrono::steady_clock::now() - t_start)
                                  .count();

        // 웹캠/실시간 입력에서 "재생 속도"는 데이터셋처럼 usleep으로 맞추지 않고,
        // 캡처 드라이버가 제공하는 프레임레이트(요청값: 30fps)에 맡기는 형태입니다.
        SLAM.TrackMonocular(frame, tframe);

        // Optional: allow quitting from this window if OpenCV highgui is available.
        // (Viewer is handled by Pangolin inside ORB-SLAM3 when enabled.)
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
            break;
    }

    SLAM.Shutdown();

    const bool has_prefix = (argc >= 5);
    if (has_prefix)
    {
        const string prefix = string(argv[4]);
        SLAM.SaveTrajectoryEuRoC("f_" + prefix + ".txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("kf_" + prefix + ".txt");
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

