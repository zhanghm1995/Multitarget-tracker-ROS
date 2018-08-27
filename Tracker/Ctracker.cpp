#include "Ctracker.h"
#include "HungarianAlg.h"

#include <GTL/GTL.h>
#include "mygraph.h"
#include "mwbmatching.h"
#include "tokenise.h"

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(
        bool useLocalTracking,
        tracking::DistType distType,
        tracking::KalmanType kalmanType,
        tracking::FilterGoal filterGoal,
        tracking::LostTrackType lostTrackType,
        tracking::MatchType matchType,
        track_t dt_,
        track_t accelNoiseMag_,
        track_t dist_thres_,
        size_t maximum_allowed_skipped_frames_,
        size_t max_trace_length_
        )
    :
      m_useLocalTracking(useLocalTracking),
      m_distType(distType),
      m_kalmanType(kalmanType),
      m_filterGoal(filterGoal),
      m_lostTrackType(lostTrackType),
      m_matchType(matchType),
      m_dt(dt_),
      m_accelNoiseMag(accelNoiseMag_),
      m_distThres(dist_thres_),
      m_maximumAllowedSkippedFrames(maximum_allowed_skipped_frames_),
      m_maxTraceLength(max_trace_length_),
      m_nextTrackID(0)
{
}

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTracker::~CTracker(void)
{
}

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(
        const regions_t& regions,
        cv::UMat grayFrame,
        float fps
        )
{
    if (m_prevFrame.size() == grayFrame.size())
    {
        if (m_useLocalTracking)
        {
            m_localTracker.Update(tracks, m_prevFrame, grayFrame);
        }
    }

    UpdateHungrian(regions, grayFrame, fps);

    grayFrame.copyTo(m_prevFrame);
}

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::UpdateHungrian(
        const regions_t& regions,
        cv::UMat grayFrame,
        float /*fps*/
        )
{
    size_t N = tracks.size();		// треки
    size_t M = regions.size();	// детекты

    assignments_t assignment(N, -1); // define a N-dimensions vector with initial value -1
    std::cout<<"tracks info =========="<<N<<" "<<M<<std::endl;
    if (!tracks.empty())
    {
        // Матрица расстояний от N-ного трека до M-ного детекта.
    	// 当前跟踪的目标和检测到的目标距离矩阵
        distMatrix_t Cost(N * M);

        // -----------------------------------
        // Треки уже есть, составим матрицу расстояний
        // 填充距离矩阵
        // -----------------------------------
        const track_t maxPossibleCost = grayFrame.cols * grayFrame.rows;
        track_t maxCost = 0;
        switch (m_distType)
        {
        case tracking::DistCenters:
            for (size_t i = 0; i < tracks.size(); i++)//对于某一个跟踪目标，计算其与当前所有检测到的目标距离
            {
                for (size_t j = 0; j < regions.size(); j++)
                {
                	//计算track预测点和regions中矩形框中心点之间的距离
                    auto dist = tracks[i]->CheckType(regions[j].m_type) ? tracks[i]->CalcDist((regions[j].m_rect.tl() + regions[j].m_rect.br()) / 2) : maxPossibleCost;
                    Cost[i + j * N] = dist;
                    if (dist > maxCost)
                    {
                        maxCost = dist;
                    }
                }
            }
            break;

        case tracking::DistRects:
            for (size_t i = 0; i < tracks.size(); i++)
            {
                for (size_t j = 0; j < regions.size(); j++)
                {
                    auto dist = tracks[i]->CheckType(regions[j].m_type) ? tracks[i]->CalcDist(regions[j].m_rect) : maxPossibleCost;
                    Cost[i + j * N] = dist;
                    if (dist > maxCost)
                    {
                        maxCost = dist;
                    }
                }
            }
            break;

        case tracking::DistJaccard:
            for (size_t i = 0; i < tracks.size(); i++)
            {
                for (size_t j = 0; j < regions.size(); j++)
                {
                    auto dist = tracks[i]->CheckType(regions[j].m_type) ? tracks[i]->CalcDistJaccard(regions[j].m_rect) : 1;
                    Cost[i + j * N] = dist;
                    if (dist > maxCost)
                    {
                        maxCost = dist;
                    }
                }
            }
            break;

        //case tracking::DistLines:
            //break;
        }
        // -----------------------------------
        // Solving assignment problem (tracks and predictions of Kalman filter)
        // -----------------------------------
        if (m_matchType == tracking::MatchHungrian)//choose this one by default
        {
            AssignmentProblemSolver APS;
            APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);
        }
        else
        {
            MyGraph G;
            G.make_directed();

            std::vector<node> nodes(N + M);

            for (size_t i = 0; i < nodes.size(); ++i)
            {
                nodes[i] = G.new_node();
            }

            edge_map<int> weights(G, 100);
            for (size_t i = 0; i < tracks.size(); i++)
            {
                bool hasZeroEdge = false;

                for (size_t j = 0; j < regions.size(); j++)
                {
                    track_t currCost = Cost[i + j * N];

                    edge e = G.new_edge(nodes[i], nodes[N + j]);

                    if (currCost < m_distThres)
                    {
                        int weight = maxCost - currCost + 1;
                        G.set_edge_weight(e, weight);
                        weights[e] = weight;
                    }
                    else
                    {
                        if (!hasZeroEdge)
                        {
                            G.set_edge_weight(e, 0);
                            weights[e] = 0;
                        }
                        hasZeroEdge = true;
                    }
                }
            }

            edges_t L = MAX_WEIGHT_BIPARTITE_MATCHING(G, weights);
            for (edges_t::iterator it = L.begin(); it != L.end(); ++it)
            {
                node a = it->source();
                node b = it->target();
                assignment[b.id()] = a.id() - N;
            }
        }//end else

        // -----------------------------------
        // clean assignment from pairs with large distance
        // -----------------------------------
        for (size_t i = 0; i < assignment.size(); i++)
        {
        	std::cout<<std::endl;
        	std::cout<<assignment[i]<<std::endl;
        	std::cout<<std::endl;
            if (assignment[i] != -1)
            {
                if (Cost[i + assignment[i] * N] > m_distThres)
                {
                	std::cout<<"[INFO]----------enter m_distThres judgement"<<std::endl;
                    assignment[i] = -1;
                    tracks[i]->m_skippedFrames++;
                }
            }
            else
            {
                // If track have no assigned detect, then increment skipped frames counter.
                tracks[i]->m_skippedFrames++;
            }
        }

        // -----------------------------------
        // If track didn't get detects long time, remove it.
        // -----------------------------------
        for (int i = 0; i < static_cast<int>(tracks.size()); i++)
        {
            if (tracks[i]->m_skippedFrames > m_maximumAllowedSkippedFrames)
            {
                tracks.erase(tracks.begin() + i);
                assignment.erase(assignment.begin() + i);
                i--;
            }
        }
    }//end if (!tracks.empty())

    // -----------------------------------
    // Search for unassigned detects and start new tracks for them.
    // -----------------------------------
    for (size_t i = 0; i < regions.size(); ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
        	std::cout<<"[Cout]enter this one"<<std::endl;
            tracks.push_back(std::make_unique<CTrack>(regions[i],
                                                      m_kalmanType,
                                                      m_dt,
                                                      m_accelNoiseMag,
                                                      m_nextTrackID++,
                                                      m_filterGoal == tracking::FilterRect,
                                                      m_lostTrackType));
        }
    }

    // Update Kalman Filters state
    std::cout<<"[Cout---------------]assignment size "<<assignment.size()<<std::endl;
    for (size_t i = 0; i < assignment.size(); i++)
    {
        // If track updated less than one time, than filter state is not correct.

        if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
        {
            tracks[i]->m_skippedFrames = 0;
            tracks[i]->Update(regions[assignment[i]], true, m_maxTraceLength, m_prevFrame, grayFrame);
        }
        else				     // if not continue using predictions
        {
            tracks[i]->Update(CRegion(), false, m_maxTraceLength, m_prevFrame, grayFrame);
        }
    }
}
