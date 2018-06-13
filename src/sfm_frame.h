/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#ifndef __SFM_FRAME_H__
#define __SFM_FRAME_H__

#include <vector>
#include <map>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "utils.h"
#include "sfm_data.h"

using namespace std;
using namespace cv;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class SfM_Frame
{
public:
    SfM_Frame();
    SfM_Frame(int idx_, Mat &imgIn);
    ~SfM_Frame();

    int setImage(Mat &imgIn);

    int detectKeypoints(void);
    int matchFrame(SfM_Frame &fp, int filtByFundMatrix=0);

    int addMatchRaw(int frameIdx, DMatchArray &match);
    int getMatchRaw(int frameIdx, DMatchArray &match);
    int addMatchFine(int frameIdx, DMatchArray &match);
    int getMatchFine(int frameIdx, DMatchArray &match);

    int getKeypointSet(set<int> &kpSet);

    Vec3b   getColor(int ix, int iy);
    Vec3b   getColor(float fx, float fy);
    Vec3b   getColor(int kpIdx);

    int saveData(string fnOut, PointArray &pta);


public:
    int                     idx;                // frame index
    int                     isKeyFrame;         // key-frame or not

    Mat                     imgRaw;             // raw image (color)
    Mat                     imgGray;            // CV_8UC1 image


    vector<KeyPoint>        kpRaw;              // keypoint (raw)
    Mat                     kpDesc;             // feature descriptor

    FrameMatch              matchRaw;           // match to previsou frames
    FrameMatch              matchFine;          // fine match (delete wrong matches)

    Matx34d                 camP;               // current camer position & orientation

    string                  fnImg;              // image file name

protected:
    void _free(void);
};


#endif // end of __SFM_FRAME_H__
