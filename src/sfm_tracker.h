/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#ifndef __SFM_TRACKER_H__
#define __SFM_TRACKER_H__

#include <vector>
#include <deque>

#include <opencv2/core/core.hpp>

#include "utils.h"
#include "utils_gui.h"

#include "sfm_data.h"
#include "sfm_frame.h"


using namespace std;
using namespace cv;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class SfM_Tracker
{
public:
    SfM_Tracker();
    SfM_Tracker(CParamArray *pa_);
    ~SfM_Tracker();

    int setPCDViewer(PCD_Viewer *pcdv);

    int appendFrame(Mat &imgIn, string fnImg="");
    SfM_Frame* getFrame(int frmIdx);

    int appendKeyframe(SfM_Frame *f);

    int getKeyframes(vector<int> &keyframes, int n=5);

    int getKeypointFromFrame(int frmIdx, vector<int> &kp);

    int findBaseline(void);
    int get2D3DCorrespondings(SfM_Frame *f1, SfM_Frame *f2,
                              vector<Point2f> &iptImg, vector<Point3f> &pt3D);
    int estimateNewPos(vector<Point2f> pt2D, vector<Point3f> pt3D,
                       cv::Mat_<double>& rvec,
                       cv::Mat_<double>& t,
                       cv::Mat_<double>& R);
    int triangulateTwoFrames(SfM_Frame *f1, SfM_Frame *f2,
                             vector<CloudPoint> &pcn, vector<int> &addFlag);
    int bundleAdjustFrames(void);

    int saveKeyframes(void);
    int savePointclouds(char *fname=NULL);

public:
    FrameQueue      frameQueue;                     // frame queue
    int             frmeQueueBegIndex;              // frame queue begin index

    int             frameCount;                     // frame count index
    int             keyframeLast;                   // last keyframe index

    int             foundBaseline;                  // found baseline or not

    PointArray      pointCloud;                     // point cloud

    KeyframeMap     keyframeMap;                    // keyframe map
    Keyframe2CamMap kf2CamMap;                      // keyframe->Camera map
    Keyframe2KPMap  kf2KPMap;                       // keyframe->vector<KeyPoint> map

    Mat_<double>    cam_k, cam_d, cam_kinv;         // camera instric & distortion

    int             baselineF1, baselineF2;         // baseline frame 1 & 2
    double          baselineEvaluationValue;        // base line evaluation value

    PCD_Viewer      *pcdViewer;

protected:
    CParamArray     *pa;

    int             frameQueue_maxSize;

    int             tracker_keyFrameMinInterval;
    int             tracker_keyFrameMaxInterval;
    double          tracker_dispThreshold;
    int             tracker_baselineMinImages;
    int             tracker_baselineMaxImages;

    int             tracker_keyFrameSearchRange;

    double          tracker_visScale;

protected:
    int loadConf(void);
    void free(void);
    void init(void);
};

#endif // end of __SFM_TRACKER_H__
