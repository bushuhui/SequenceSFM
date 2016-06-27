/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#ifndef __SFM_DATA_H__
#define __SFM_DATA_H__


#include <vector>
#include <iostream>
#include <list>
#include <set>
#include <map>
#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class SfM_Frame;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct CloudPoint {
    cv::Point3d         pt;                     // 3D point
    cv::Vec3b           rgb;                    // color

    int                 match_id;               // match id

    std::map<int, int>  img_kp;                 // frame & keypoint map

    double              reprojection_error;
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef std::deque<SfM_Frame*>                      FrameQueue;
typedef std::vector<CloudPoint>                     PointArray;
typedef std::map<int, SfM_Frame*>                   KeyframeMap;
typedef std::map<int, cv::Matx34d>                  Keyframe2CamMap;
typedef std::map<int, std::vector<cv::KeyPoint> >   Keyframe2KPMap;

typedef std::vector<cv::DMatch>                     DMatchArray;
typedef std::map<int, DMatchArray>                  FrameMatch;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline int CloudPoint_GetMeasurementNum(PointArray &pta)
{
    int K = 0;

    for (unsigned int i=0; i<pta.size(); i++) {
        K += pta[i].img_kp.size();
    }

    return K;
}

#endif // end of __SFM_DATA_H__
