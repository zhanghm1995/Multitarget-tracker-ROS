#pragma once
#include <iostream>
#include <vector>
#include <memory>
#include <array>
#include <deque>

#include "defines.h"
#include "track.h"
#include "LocalTracker.h"

// ----------------------------------------------------------------------
class CTracker
{
public:
    CTracker(bool useLocalTracking,
             tracking::DistType distType,
             tracking::KalmanType kalmanType,
             tracking::FilterGoal filterGoal,
             tracking::LostTrackType lostTrackType,
             tracking::MatchType matchType,
             track_t dt_,
             track_t accelNoiseMag_,
             track_t dist_thres_ = 60,
             size_t maximum_allowed_skipped_frames_ = 10,
             size_t max_trace_length_ = 10);
	~CTracker(void);

    tracks_t tracks;
    void Update(const regions_t& regions, cv::UMat grayFrame, float fps);

    bool GrayFrameToTrack() const
    {
        return m_lostTrackType != tracking::LostTrackType::TrackGOTURN;
    }

private:
    // Use local tracking for regions between two frames
    bool m_useLocalTracking;

    tracking::DistType m_distType;
    tracking::KalmanType m_kalmanType;
    tracking::FilterGoal m_filterGoal;
    tracking::LostTrackType m_lostTrackType;
    tracking::MatchType m_matchType;

	// Шаг времени опроса фильтра 过滤投票超时
    track_t m_dt;

    track_t m_accelNoiseMag;

	// Порог расстояния. Если точки находятся дуг от друга на расстоянии,
    //距离阈值
	// превышающем этот порог, то эта пара не рассматривается в задаче о назначениях.
    track_t m_distThres;
	// Максимальное количество кадров которое трек сохраняется не получая данных о измерений.
    size_t m_maximumAllowedSkippedFrames;
	// Максимальная длина следа
    size_t m_maxTraceLength;

    size_t m_nextTrackID;

    LocalTracker m_localTracker;

    cv::UMat m_prevFrame;

    void UpdateHungrian(const regions_t& regions, cv::UMat grayFrame, float fps);
};
