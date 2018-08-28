#pragma once

#include "BaseDetector.h"

#include "Ctracker.h"
#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include "nms.h"

typedef std::unordered_map<std::string, std::string> params_config;
///
/// \brief The VideoExample class
///
class AbstractTargetTracker
{
public:
  //construcotrs
  AbstractTargetTracker(const params_config& params_parser):
      m_showLogs(true),
      m_useLocalTracking(false),
      m_isTrackerInitialized(false),
      m_finishDelay(0),
      m_currFrame(0)
    {
      m_showLogs = std::stoi(params_parser.find("show_logs")->second);

      m_colors.push_back(cv::Scalar(255, 0, 0));
      m_colors.push_back(cv::Scalar(0, 255, 0));
      m_colors.push_back(cv::Scalar(0, 0, 255));
      m_colors.push_back(cv::Scalar(255, 255, 0));
      m_colors.push_back(cv::Scalar(0, 255, 255));
      m_colors.push_back(cv::Scalar(255, 0, 255));
      m_colors.push_back(cv::Scalar(255, 127, 255));
      m_colors.push_back(cv::Scalar(127, 0, 255));
      m_colors.push_back(cv::Scalar(127, 0, 127));
    }

    virtual ~AbstractTargetTracker()
    {

    }

    ///
    /// \brief Process
    ///
    void Process()
    {
        m_currFrame = 1;
        bool stopCapture = false;
        std::mutex frameLock;
        std::condition_variable frameCond;

        std::mutex trackLock;
        std::condition_variable trackCond;
        //zhanghm: build a image capture and detect thread
        std::thread thCapDet(CaptureAndDetect, this, &stopCapture, &frameLock, &frameCond, &trackLock, &trackCond);
        thCapDet.detach();

        const int captureTimeOut = 5000;
        {
            //std::cout << "Process -1: frameLock" << std::endl;
            std::unique_lock<std::mutex> lock(frameLock);
            //std::cout << "Process -1: frameCond.wait_until" << std::endl;
            auto now = std::chrono::system_clock::now();
            if (frameCond.wait_until(lock, now + std::chrono::milliseconds(captureTimeOut)) == std::cv_status::timeout)
            {
                std::cerr << "Process: Init capture timeout" << std::endl;
                stopCapture = true;

                if (thCapDet.joinable())
                {
                    thCapDet.join();
                }
                return;
            }
        }


        cv::namedWindow("Video");

        int k = 0;

        double freq = cv::getTickFrequency();

        int64 allTime = 0;

        bool manualMode = false;
        int framesCounter = 1;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        trackCond.notify_all();
        //std::cout << "Process 0: trackCond.notify_all(); " << std::endl;

        int currFrame = 0;
        for (; !stopCapture && k != 27; )//zhanghm:no error or no press ESC, just loop run
        {
            //std::cout << "Process: m_currFrame = " << m_currFrame << ", currFrame = " << currFrame << std::endl;

            //if (currFrame == m_currFrame)
            {
                //std::cout << "Process: frameLock" << std::endl;
                std::unique_lock<std::mutex> lock(frameLock);
                //std::cout << "Process: frameCond.wait_until" << std::endl;
                auto now = std::chrono::system_clock::now();
                if (frameCond.wait_until(lock, now + std::chrono::milliseconds(captureTimeOut)) == std::cv_status::timeout)
                {
                    std::cerr << "Process: Frame capture timeout" << std::endl;
                    break;
                }
            }
            if (stopCapture)
            {
                break;
            }

            frameLock.lock();
            currFrame = m_currFrame;//zhanghm: m_currFrame value can be changed in CaptureAndDetect function
            //std::cout << "Process: currFrame = " << currFrame << std::endl;
            frameLock.unlock();

            int64 t1 = cv::getTickCount();
            /* ********************
             *  Tracking
             * ********************/
            Tracking(m_frameInfo[currFrame].m_frame, m_frameInfo[currFrame].m_gray, m_frameInfo[currFrame].m_regions);

            int64 t2 = cv::getTickCount();
            //std::cout << "Process: tracking" << std::endl;

            allTime += t2 - t1 + m_frameInfo[currFrame].m_dt;
            int currTime = cvRound(1000 * (t2 - t1 + m_frameInfo[currFrame].m_dt) / freq);
            /* ********************
             *  Draw final results
             * ********************/
            DrawData(m_frameInfo[currFrame].m_frame, framesCounter, currTime);

            cv::imshow("Video", m_frameInfo[currFrame].m_frame);

            int waitTime = manualMode ? 0 : std::max<int>(1, cvRound(1000 / m_fps - currTime));
            k = cv::waitKey(waitTime);
            //std::cout << "Process: waitkey" << std::endl;

            if (k == 'm' || k == 'M')
            {
                manualMode = !manualMode;
            }


            trackCond.notify_all();
            //std::cout << "Process 1: trackCond.notify_all(); " << std::endl;

            ++framesCounter;
        }//end for (; !stopCapture && k != 27; )
        stopCapture = true;

        if (thCapDet.joinable())
        {
            thCapDet.join();
        }

        std::cout << "work time = " << (allTime / freq) << std::endl;
        cv::waitKey(m_finishDelay);
    }//end Process()

protected:
    ///
    /// \brief CaptureAndDetect
    /// \param stopCapture
    /// \param frameLock
    /// \param frameCond
    ///
    static void CaptureAndDetect(AbstractTargetTracker* thisPtr,
                                 bool* stopCapture,
                                 std::mutex* frameLock,
                                 std::condition_variable* frameCond,
                                 std::mutex* trackLock,
                                 std::condition_variable* trackCond)
    {
        cv::Mat image_in = thisPtr->image_raw_;

        if (image_in.empty())
        {
            throw std::logic_error("Invalid image input, maybe no data!");
            return;
        }

        const int trackingTimeOut = 5000;

        frameCond->notify_all();
        //std::cout << "CaptureAndDetect: init capture frameCond->notify_all();" << std::endl;

        int currFrame = 0;
        for (; !(*stopCapture);)//not stop, just loop
        {
            //std::cout << "CaptureAndDetect: m_currFrame = " << thisPtr->m_currFrame << ", currFrame = " << currFrame << std::endl;

            {
                //std::cout << "CaptureAndDetect: trackLock" << std::endl;
                std::unique_lock<std::mutex> lock(*trackLock);
                //std::cout << "CaptureAndDetect: trackCond->wait_until" << std::endl;
                auto now = std::chrono::system_clock::now();
                if (trackCond->wait_until(lock, now + std::chrono::milliseconds(trackingTimeOut)) == std::cv_status::timeout)
                {
                    std::cerr << "CaptureAndDetect: Tracking timeout!" << std::endl;
                    break;
                }
            }
            frameLock->lock();
            currFrame = thisPtr->m_currFrame ? 0 : 1;
            //std::cout << "CaptureAndDetect: currFrame = " << currFrame << std::endl;
            frameLock->unlock();

//            capture >> thisPtr->m_frameInfo[currFrame].m_frame;//zhanghm: get image
            thisPtr->m_frameInfo[currFrame].m_frame = image_in;//get image
            if (thisPtr->m_frameInfo[currFrame].m_frame.empty())
            {
                std::cerr << "CaptureAndDetect: frame is empty!" << std::endl;
                break;
            }
            cv::cvtColor(thisPtr->m_frameInfo[currFrame].m_frame, thisPtr->m_frameInfo[currFrame].m_gray, cv::COLOR_BGR2GRAY);

            //std::cout << "CaptureAndDetect: capture" << std::endl;

            if (!thisPtr->m_isTrackerInitialized)//zhanghm:use gray image to initialize tracker
            {

                thisPtr->m_isTrackerInitialized = thisPtr->InitTracker(thisPtr->m_frameInfo[currFrame].m_gray);
                if (!thisPtr->m_isTrackerInitialized)
                {
                    //std::cerr << "CaptureAndDetect: Tracker initilize error!!!" << std::endl;
                    break;
                }
            }

            /* ********************
             *       Detection
             * ********************/
            int64 t1 = cv::getTickCount();
            thisPtr->Detection(thisPtr->m_frameInfo[currFrame].m_frame, thisPtr->m_frameInfo[currFrame].m_gray, thisPtr->m_frameInfo[currFrame].m_regions);
            int64 t2 = cv::getTickCount();
            thisPtr->m_frameInfo[currFrame].m_dt = t2 - t1;

            //std::cout << "CaptureAndDetect: detection" << std::endl;

            frameLock->lock();
            thisPtr->m_currFrame = thisPtr->m_currFrame ? 0 : 1;
            std::cout << "CaptureAndDetect: thisPtr->m_currFrame = " << thisPtr->m_currFrame << std::endl;
            frameLock->unlock();
            frameCond->notify_all();
            //std::cout << "CaptureAndDetect 0: frameCond->notify_all();" << std::endl;
        }//end for(; !(*stopCapture);)

        *stopCapture = true;
        frameCond->notify_all();
        //std::cout << "CaptureAndDetect 1: frameCond->notify_all();" << std::endl;
    }

    ///
    /// \brief GrayProcessing
    /// \return
    ///
    virtual bool GrayProcessing() const
    {
        return true;
    }

    ///
    /// \brief InitTracker
    /// \param frame
    /// \return
    ///
    virtual bool InitTracker(cv::UMat frame) = 0; //pure virtual function

    ///
    /// \brief Detection
    /// \param frame
    /// \param grayFrame
    ///
    void Detection(cv::Mat frame, cv::UMat grayFrame, regions_t& regions)
    {
        cv::UMat clFrame;
        if (!GrayProcessing() || !m_tracker->GrayFrameToTrack())//zhanghm: not enter this line by default
        {
            clFrame = frame.getUMat(cv::ACCESS_READ);
        }

        m_detector->Detect(GrayProcessing() ? grayFrame : clFrame);

        const regions_t& regs = m_detector->GetDetects();

        regions.assign(std::begin(regs), std::end(regs));
    }

    ///
    /// \brief Tracking
    /// \param frame
    /// \param grayFrame
    ///
    void Tracking(cv::Mat frame, cv::UMat grayFrame, const regions_t& regions)
    {
        cv::UMat clFrame;
        //zhanghm: m_tracker->GrayFrameToTrack() always true
        if (!GrayProcessing() || !m_tracker->GrayFrameToTrack())
        {
            clFrame = frame.getUMat(cv::ACCESS_READ);
        }

        m_tracker->Update(regions, m_tracker->GrayFrameToTrack() ? grayFrame : clFrame, m_fps);
    }

    ///
    /// \brief DrawData
    /// \param frame
    /// \param framesCounter
    /// \param currTime
    ///
    virtual void DrawData(cv::Mat frame, int framesCounter, int currTime) = 0;

    ///
    /// \brief DrawTrack
    /// \param frame
    /// \param resizeCoeff
    /// \param track
    ///
    void DrawTrack(cv::Mat frame,
                   int resizeCoeff,
                   const CTrack& track,
                   bool drawTrajectory = true
                   )
    {
        auto ResizeRect = [&](const cv::Rect& r) -> cv::Rect
        {
            return cv::Rect(resizeCoeff * r.x, resizeCoeff * r.y, resizeCoeff * r.width, resizeCoeff * r.height);
        };
        auto ResizePoint = [&](const cv::Point& pt) -> cv::Point
        {
            return cv::Point(resizeCoeff * pt.x, resizeCoeff * pt.y);
        };

        cv::rectangle(frame, ResizeRect(track.GetLastRect()), cv::Scalar(0, 255, 0), 1, CV_AA);

        if (drawTrajectory)
        {
            cv::Scalar cl = m_colors[track.m_trackID % m_colors.size()];

            for (size_t j = 0; j < track.m_trace.size() - 1; ++j)
            {
                const TrajectoryPoint& pt1 = track.m_trace.at(j);
                const TrajectoryPoint& pt2 = track.m_trace.at(j + 1);

                cv::line(frame, ResizePoint(pt1.m_prediction), ResizePoint(pt2.m_prediction), cl, 1, CV_AA);
                if (!pt2.m_hasRaw)
                {
                    cv::circle(frame, ResizePoint(pt2.m_prediction), 4, cl, 1, CV_AA);
                }
            }
        }

        if (m_useLocalTracking)
        {
            cv::Scalar cl = m_colors[track.m_trackID % m_colors.size()];

            for (auto pt : track.m_lastRegion.m_points)
            {
                cv::circle(frame, cv::Point(cvRound(pt.x), cvRound(pt.y)), 1, cl, -1, CV_AA);
            }
        }
    }

protected:
    std::unique_ptr<BaseDetector> m_detector;
    std::unique_ptr<CTracker> m_tracker;

    bool m_showLogs;
    bool m_useLocalTracking;

private:
    bool m_isTrackerInitialized;
    cv::Mat image_raw_;
    int m_finishDelay;
    std::vector<cv::Scalar> m_colors;

    struct FrameInfo
    {
        cv::Mat m_frame;
        cv::UMat m_gray;
        regions_t m_regions;
        int64 m_dt;//zhanghm:detection time?

        FrameInfo(): m_dt(0)
        {

        }
    };
    FrameInfo m_frameInfo[2];

    int m_currFrame;
};



///
/// \brief The SSDMobileNetTracker class
///
class SSDMobileNetTracker : public AbstractTargetTracker
{
public:

  SSDMobileNetTracker(const params_config& params_parser):
      AbstractTargetTracker(params_parser)
    {

    }



protected:
    ///
    /// \brief InitTracker
    /// \param grayFrame
    ///
    bool InitTracker(cv::UMat frame)
    {
        BaseDetector::config_t config;
        config["modelConfiguration"] = "../data/MobileNetSSD_deploy.prototxt";
        config["modelBinary"] = "../data/MobileNetSSD_deploy.caffemodel";
        config["confidenceThreshold"] = "0.5";
        config["maxCropRatio"] = "3.0";
        m_detector = std::unique_ptr<BaseDetector>(CreateDetector(tracking::Detectors::SSD_MobileNet, config, m_useLocalTracking, frame));
        if (!m_detector.get())
        {
            return false;
        }
        m_detector->SetMinObjectSize(cv::Size(frame.cols / 20, frame.rows / 20));

        m_tracker = std::make_unique<CTracker>(m_useLocalTracking,
                                               tracking::DistRects,
                                               tracking::KalmanLinear,
                                               tracking::FilterRect,
                                               tracking::TrackKCF,      // Use KCF tracker for collisions resolving
                                               tracking::MatchHungrian,
                                               0.3f,                     // Delta time for Kalman filter
                                               0.1f,                     // Accel noise magnitude for Kalman filter
                                               frame.rows / 10,          // Distance threshold between region and object on two frames
                                               2 * m_fps,                // Maximum allowed skipped frames
                                               5 * m_fps                 // Maximum trace length
                                               );

        return true;
    }

    ///
    /// \brief DrawData
    /// \param frame
    ///
    void DrawData(cv::Mat frame, int framesCounter, int currTime)
    {
        if (m_showLogs)
        {
            std::cout << "Frame " << framesCounter << ": tracks = " << m_tracker->tracks.size() << ", time = " << currTime << std::endl;
        }

        for (const auto& track : m_tracker->tracks)
        {
            if (track->IsRobust(5,                           // Minimal trajectory size
                                0.2f,                        // Minimal ratio raw_trajectory_points / trajectory_lenght
                                cv::Size2f(0.1f, 8.0f))      // Min and max ratio: width / height
                    )
            {
                DrawTrack(frame, 1, *track);

                std::string label = track->m_lastRegion.m_type + ": " + std::to_string(track->m_lastRegion.m_confidence);
                int baseLine = 0;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                auto rect(track->GetLastRect());
                cv::rectangle(frame, cv::Rect(cv::Point(rect.x, rect.y - labelSize.height), cv::Size(labelSize.width, labelSize.height + baseLine)), cv::Scalar(255, 255, 255), CV_FILLED);
                cv::putText(frame, label, cv::Point(rect.x, rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
        }

        m_detector->CalcMotionMap(frame);
    }

    ///
    /// \brief GrayProcessing
    /// \return
    ///
    bool GrayProcessing() const
    {
        return false;
    }
};

