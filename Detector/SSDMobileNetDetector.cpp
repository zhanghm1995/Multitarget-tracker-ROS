#include "SSDMobileNetDetector.h"
#include <opencv2/opencv.hpp>
#include "nms.h"

///
/// \brief SSDMobileNetDetector::SSDMobileNetDetector
/// \param collectPoints
/// \param gray
///
SSDMobileNetDetector::SSDMobileNetDetector(
	bool collectPoints,
    cv::UMat& colorFrame):
      BaseDetector(collectPoints, colorFrame),
      m_WHRatio(InWidth / (float)InHeight),
      m_inScaleFactor(0.007843f),
      m_meanVal(127.5),
      m_confidenceThreshold(0.5f),
      m_maxCropRatio(2.0f)
{
    m_classNames = { "background",
                     "aeroplane", "bicycle", "bird", "boat",
                     "bottle", "bus", "car", "cat", "chair",
                     "cow", "diningtable", "dog", "horse",
                     "motorbike", "person", "pottedplant",
                     "sheep", "sofa", "train", "tvmonitor" };
}

///
/// \brief SSDMobileNetDetector::~SSDMobileNetDetector
///
SSDMobileNetDetector::~SSDMobileNetDetector(void)
{
}

///
/// \brief SSDMobileNetDetector::Init
/// \return
///
bool SSDMobileNetDetector::Init(const config_t& config)
{
    auto modelConfiguration = config.find("modelConfiguration");
    auto modelBinary = config.find("modelBinary");
    if (modelConfiguration != config.end() && modelBinary != config.end())
    {
        m_net = cv::dnn::readNetFromCaffe(modelConfiguration->second, modelBinary->second);
    }

    auto confidenceThreshold = config.find("confidenceThreshold");
    if (confidenceThreshold != config.end())
    {
        m_confidenceThreshold = std::stof(confidenceThreshold->second);
    }

    auto maxCropRatio = config.find("maxCropRatio");
    if (maxCropRatio != config.end())
    {
        m_maxCropRatio = std::stof(maxCropRatio->second);
        if (m_maxCropRatio < 1.f)
        {
            m_maxCropRatio = 1.f;
        }
    }

    return !m_net.empty();
}

///
/// \brief SSDMobileNetDetector::Detect
/// \param gray
///
void SSDMobileNetDetector::Detect(cv::UMat& colorFrame)
{
    m_regions.clear();

    regions_t tmpRegions;

    cv::Mat colorMat = colorFrame.getMat(cv::ACCESS_READ);

    int cropHeight = cvRound(m_maxCropRatio * InHeight);
    int cropWidth = cvRound(m_maxCropRatio * InWidth);

    if (colorFrame.cols / (float)colorFrame.rows > m_WHRatio)
    {
        if (m_maxCropRatio <= 0 || cropHeight >= colorFrame.rows)
        {
            cropHeight = colorFrame.rows;
        }
        cropWidth = cvRound(cropHeight * m_WHRatio);
    }
    else
    {
        if (m_maxCropRatio <= 0 || cropWidth >= colorFrame.cols)
        {
            cropWidth = colorFrame.cols;
        }
        cropHeight = cvRound(colorFrame.cols / m_WHRatio);
    }

    cv::Rect crop(0, 0, cropWidth, cropHeight);

    for (; crop.y < colorMat.rows; crop.y += crop.height / 2)
    {
        bool needBreakY = false;
        if (crop.y + crop.height >= colorMat.rows)
        {
            crop.y = colorMat.rows - crop.height;
            needBreakY = true;
        }
        for (crop.x = 0; crop.x < colorMat.cols; crop.x += crop.width / 2)
        {
            bool needBreakX = false;
            if (crop.x + crop.width >= colorMat.cols)
            {
                crop.x = colorMat.cols - crop.width;
                needBreakX = true;
            }

            DetectInCrop(colorMat, crop, tmpRegions);

            if (needBreakX)
            {
                break;
            }
        }
        if (needBreakY)
        {
            break;
        }
    }

    nms3<CRegion>(tmpRegions, m_regions, 0.4f,
         [](const CRegion& reg) -> cv::Rect { return reg.m_rect; },
    [](const CRegion& reg) -> float { return reg.m_confidence; },
    [](const CRegion& reg) -> std::string { return reg.m_type; },
    0, 0.f);

    if (m_collectPoints)
    {
        for (auto& region : m_regions)
        {
            CollectPoints(region);
        }
    }
}

///
/// \brief SSDMobileNetDetector::DetectInCrop
/// \param colorFrame
/// \param crop
/// \param tmpRegions
///
void SSDMobileNetDetector::DetectInCrop(cv::Mat colorFrame, const cv::Rect& crop, regions_t& tmpRegions)
{
    //Convert Mat to batch of images
    cv::Mat inputBlob = cv::dnn::blobFromImage(cv::Mat(colorFrame, crop), m_inScaleFactor, cv::Size(InWidth, InHeight), m_meanVal, false, true);

    m_net.setInput(inputBlob, "data"); //set the network input

    cv::Mat detection = m_net.forward("detection_out"); //compute output

    //std::vector<double> layersTimings;
    //double freq = cv::getTickFrequency() / 1000;
    //double time = m_net.getPerfProfile(layersTimings) / freq;

    cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    //cv::Mat frame = colorFrame(crop);

    //ss << "FPS: " << 1000/time << " ; time: " << time << " ms";
    //putText(frame, ss.str(), Point(20,20), 0, 0.5, Scalar(0,0,255));
    //std::cout << "Inference time, ms: " << time << endl;

    //cv::Point correctPoint((colorFrame.cols - crop.width) / 2, (colorFrame.rows - crop.height) / 2);

    for (int i = 0; i < detectionMat.rows; ++i)
    {
        float confidence = detectionMat.at<float>(i, 2);

        if (confidence > m_confidenceThreshold)
        {
            size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));

            int xLeftBottom = cvRound(detectionMat.at<float>(i, 3) * crop.width) + crop.x;
            int yLeftBottom = cvRound(detectionMat.at<float>(i, 4) * crop.height) + crop.y;
            int xRightTop = cvRound(detectionMat.at<float>(i, 5) * crop.width) + crop.x;
            int yRightTop = cvRound(detectionMat.at<float>(i, 6) * crop.height) + crop.y;

            cv::Rect object(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);

            tmpRegions.push_back(CRegion(object, m_classNames[objectClass], confidence));

            //cv::rectangle(frame, object, Scalar(0, 255, 0));
            //std::string label = classNames[objectClass] + ": " + std::to_string(confidence);
            //int baseLine = 0;
            //cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            //cv::rectangle(frame, cv::Rect(cv::Point(xLeftBottom, yLeftBottom - labelSize.height), cv::Size(labelSize.width, labelSize.height + baseLine)), cv::Scalar(255, 255, 255), CV_FILLED);
            //cv::putText(frame, label, Point(xLeftBottom, yLeftBottom), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
        }
    }
}
