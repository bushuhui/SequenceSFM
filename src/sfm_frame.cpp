/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>
#include <opencv2/nonfree/gpu.hpp>


#include "utils.h"

#include "sfm_utils.h"
#include "sfm_frame.h"
#include "sfm_tracker.h"
#include "sfm_cameraMatrices.h"

using namespace cv;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

SfM_Frame::SfM_Frame()
{
    pa = NULL;
    idx = -1;
    isKeyFrame = 0;
}

SfM_Frame::SfM_Frame(int idx_, Mat &imgIn, CParamArray *pa_)
{
    idx = idx_;
    pa = pa_;
    isKeyFrame = 0;

    setImage(imgIn);
}

SfM_Frame::~SfM_Frame()
{
    _free();
}

void SfM_Frame::_free(void)
{
    matchRaw.clear();
    matchFine.clear();

    kpRaw.clear();
}

int SfM_Frame::setImage(Mat &imgIn)
{
    // copy image
    imgIn.copyTo(imgRaw);

    // convert to gray-scale
    if( imgRaw.channels() > 1 ) {
        cvtColor(imgRaw, imgGray, COLOR_BGR2GRAY);
    } else {
        imgGray = imgRaw;
    }

    return 0;
}

int SfM_Frame::detectKeypoints(void)
{
    string                  fd_name = "SURF";
    double                  SURF_minHessian = 50;

    FeatureDetector         *detector;
    SurfDescriptorExtractor *extractor;

    // load parameters
    if( pa != NULL ) {
        pa->s("kp_detector", fd_name);
        pa->d("kp_SURF_minHessian", SURF_minHessian);
    }

    /////////////////////////////////////////////////
    /// create feature detector
    /////////////////////////////////////////////////
    if( fd_name == "SURF" ) {
        detector = new SurfFeatureDetector( SURF_minHessian );
    } else if ( fd_name == "FAST" ) {
        detector = new FastFeatureDetector();
    } else if ( fd_name == "PyramidFAST" )  {
        detector = FeatureDetector::create("PyramidFAST");
    } else if ( fd_name == "SIFT" ) {
        detector = new SiftFeatureDetector;
    }

    extractor  = new SurfDescriptorExtractor(48, 12, true);


    detector->detect(imgGray, kpRaw);
    extractor->compute(imgGray, kpRaw, kpDesc);


    delete extractor;
    delete detector;

    return 0;
}

int SfM_Frame::matchFrame(SfM_Frame &fp, int filtByFundMatrix)
{
    string  match_method = "BFMatcher";

    if( pa != NULL ) {
        pa->s("match_method", match_method);
    }


    // if not detected keypoint
    if( kpDesc.empty() ) {
        detectKeypoints();
    }

    vector<DMatch> matches;

    if( match_method == "BFMatcher" ) {
        BFMatcher matcher(NORM_L2);
        matcher.match(fp.kpDesc, kpDesc, matches);
    } else if ( match_method == "FLANN" ) {
        FlannBasedMatcher matcher;
        matcher.match(fp.kpDesc, kpDesc, matches);
    }


    /////////////////////////////////////////////////
    /// filter wrong correspondences by histogram
    /////////////////////////////////////////////////
    if( 1 ) {
        int             match_filter_histn = 100;
        double          match_filter_sign = 0.7;
        double          tracker_dispThreshold = 60;

        double          d_mean, d_sig;

        // load parameters
        if( pa != NULL ) {
            pa->i("match_filter_histn", match_filter_histn);
            pa->d("match_filter_sign", match_filter_sign);
            pa->d("tracker_dispThreshold", tracker_dispThreshold);
        }

        int     i;
        Point2f p1, p2;
        double  d;

        int     nMatch = matches.size();

        vector<double>  arr_d;
        vector<int>     arr_f;
        vector<DMatch>  newMatch;

        arr_d.resize(nMatch);
        for(i=0; i<nMatch; i++) {
            p1 = fp.kpRaw[matches[i].queryIdx].pt;
            p2 = kpRaw[matches[i].trainIdx].pt;

            d = sqrt( sqr(p1.x-p2.x) + sqr(p1.y-p2.y) );
            arr_d[i] = d;
        }

        // filter data
        if( 1 ) {
            FilterData_NormDist(arr_d, arr_f,
                                match_filter_histn, match_filter_sign,
                                &d_mean, &d_sig);

            // if motion of pixel is smaller than a threshold then give up
            if( d_mean < tracker_dispThreshold )
                return -1;
        } else {
            double fd_mean, fd_sig;

            FilterData_Median(arr_d, arr_f, fd_mean, fd_sig, 1.0/8.0, 7.0/8.0, 0.7);

            printf("fd_mean, sig = %9f %9f, %5d/%5d = %9f\n", fd_mean, fd_sig,
                   newMatch.size(), matches.size(), 1.0*newMatch.size()/matches.size());
        }

        // update match
        for(i=0; i<nMatch; i++) {
            if( arr_f[i] == 1 ) {
                newMatch.push_back(matches[i]);
            }
        }

        matches = newMatch;
    }

    // insert to match map
    FrameMatch::iterator    it;

    it = matchRaw.find(fp.idx);
    if( it != matchRaw.end() ) {
        it->second = matches;
    } else {
        matchRaw.insert(make_pair(fp.idx, matches));
    }

    // filter by Fundamental matrix
    if( filtByFundMatrix ) {
        Mat                 fm;
        vector<KeyPoint>    pts_n1, pts_n2;

        fm = GetFundamentalMat(fp.kpRaw, kpRaw,
                          pts_n1, pts_n2,
                          matches);

        // insert to match map
        it = matchFine.find(fp.idx);
        if( it != matchFine.end() ) {
            it->second = matches;
        } else {
            matchFine.insert(make_pair(fp.idx, matches));
        }
    }

    return 0;
}

int SfM_Frame::addMatchRaw(int frameIdx, DMatchArray &match)
{
    FrameMatch::iterator        it;

    it = matchRaw.find(frameIdx);
    if( it != matchRaw.end() ) {
        it->second = match;
    } else {
        matchRaw.insert(make_pair(frameIdx, match));
    }

    return 0;
}

int SfM_Frame::getMatchRaw(int frameIdx, DMatchArray &match)
{
    FrameMatch::iterator        it;

    it = matchRaw.find(frameIdx);
    if( it != matchRaw.end() ) {
        match = it->second;
    } else {
        match.clear();
    }

    return 0;
}

int SfM_Frame::addMatchFine(int frameIdx, DMatchArray &match)
{
    FrameMatch::iterator        it;

    it = matchFine.find(frameIdx);
    if( it != matchFine.end() ) {
        it->second = match;
    } else {
        matchFine.insert(make_pair(frameIdx, match));
    }

    return 0;
}

int SfM_Frame::getMatchFine(int frameIdx, DMatchArray &match)
{
    FrameMatch::iterator        it;

    it = matchFine.find(frameIdx);
    if( it != matchFine.end() ) {
        match = it->second;
    } else {
        match.clear();
    }

    return 0;
}


int SfM_Frame::getKeypointSet(set<int> &kpSet)
{
    FrameMatch::iterator        it;
    DMatchArray::iterator       itm;

    DMatchArray                 *ma;

    kpSet.clear();

    for(it=matchFine.begin(); it!=matchFine.end(); it++) {
        ma = &( it->second );

        for(itm=ma->begin(); itm!=ma->end(); itm++) {
            kpSet.insert(itm->trainIdx);
        }
    }

    return 0;
}


Vec3b   SfM_Frame::getColor(int ix, int iy)
{
    ru8     *p2, gv;
    int     w, h, c;
    Vec3b   rgb;

    p2 = imgRaw.data;
    w  = imgRaw.cols;
    h  = imgRaw.rows;
    c  = imgRaw.channels();

    rgb = 0;

    if( ix < 0 || iy < 0 || ix >= w || iy >= h )
        return rgb;

    if( c == 3 ) {
        rgb(0) = p2[(iy*w+ix)*3+2];
        rgb(1) = p2[(iy*w+ix)*3+1];
        rgb(2) = p2[(iy*w+ix)*3+0];
    } else {
        gv = p2[iy*w+ix];

        rgb(0) = gv;
        rgb(1) = gv;
        rgb(2) = gv;
    }

    return rgb;
}

Vec3b   SfM_Frame::getColor(float fx, float fy)
{
    ru8     *p2, gv;
    int     ix, iy, w, h, c;
    Vec3b   rgb;

    p2 = imgRaw.data;
    w  = imgRaw.cols;
    h  = imgRaw.rows;
    c  = imgRaw.channels();

    ix = (int)( fx + 0.5 );
    iy = (int)( fy + 0.5 );

    rgb = 0;

    if( ix < 0 || iy < 0 || ix >= w || iy >= h )
        return rgb;

    if( c == 3 ) {
        rgb(0) = p2[(iy*w+ix)*3+2];
        rgb(1) = p2[(iy*w+ix)*3+1];
        rgb(2) = p2[(iy*w+ix)*3+0];
    } else {
        gv = p2[iy*w+ix];

        rgb(0) = gv;
        rgb(1) = gv;
        rgb(2) = gv;
    }

    return rgb;
}

Vec3b   SfM_Frame::getColor(int kpIdx)
{
    ru8     *p2, gv;
    int     ix, iy, w, h, c;
    Vec3b   rgb;
    float   fx, fy;

    if( kpIdx >= kpRaw.size() ) {
        dbg_pe("kpIdx exceed kpRaw range: %d, %d\n", kpIdx, kpRaw.size());
        return Vec3b(0, 0, 0);
    }

    fx = kpRaw[kpIdx].pt.x;
    fy = kpRaw[kpIdx].pt.y;

    p2 = imgRaw.data;
    w  = imgRaw.cols;
    h  = imgRaw.rows;
    c  = imgRaw.channels();

    ix = (int)( fx + 0.5 );
    iy = (int)( fy + 0.5 );

    rgb = 0;

    if( ix < 0 || iy < 0 || ix >= w || iy >= h )
        return rgb;

    if( c == 3 ) {
        rgb(0) = p2[(iy*w+ix)*3+2];
        rgb(1) = p2[(iy*w+ix)*3+1];
        rgb(2) = p2[(iy*w+ix)*3+0];
    } else {
        gv = p2[iy*w+ix];

        rgb(0) = gv;
        rgb(1) = gv;
        rgb(2) = gv;
    }

    return rgb;
}

/*******************************************************************************
 * Save current frame's feature point, 3Dpoints, camera position (3x4 SE)
 *  to file named as 'fnOut'
 *
 * File format:
 *      nf np
 *      cam (3x4 matrix)
 *      1 x y (feature index, x, y)
 *      2 x y
 *      ...
 *      20 X Y Z 2  (point index, x, y, z, feature index)
 *      10 X Y Z 1
 *      ...
 *
 ******************************************************************************/
int SfM_Frame::saveData(string fnOut, PointArray &pta)
{
    vector<Point3d>     pa;
    vector<int>         pai, kpi;

    FILE                *fp = NULL;
    int                 i, j;

    // extract points
    {
        PointArray::iterator    it;
        map<int, int>           *m;
        map<int, int>::iterator itmii;

        pa.reserve(pta.size());
        pai.reserve(pta.size());
        kpi.reserve(pta.size());

        i = 0;
        for(it=pta.begin(); it!=pta.end(); it++) {
            m = &( it->img_kp );

            itmii = m->find(idx);
            if( itmii != m->end() ) {
                kpi.push_back(itmii->second);
                pai.push_back(i);
                pa.push_back(it->pt);
            }

            i++;
        }
    }

    // open file
    fp = fopen(fnOut.c_str(), "wt");
    if( fp == NULL ) {
        dbg_pe("Failed to open file: %s\n", fnOut.c_str());
        return -1;
    }

    // save feature number, 3D point numnber
    fprintf(fp, "%d %d\n", kpRaw.size(), pa.size());

    // save cmaera SE3 (3x4 matrix) (row first)
    for(j=0; j<3; j++) for(i=0; i<4; i++)
        fprintf(fp, "%g ", camP(j, i));
    fprintf(fp, "\n");

    // save feature points
    for(i=0; i<kpRaw.size(); i++)
        fprintf(fp, "%d %g %g\n", i, kpRaw[i].pt.x, kpRaw[i].pt.y);

    // save 3D points
    for(i=0; i<pa.size(); i++) {
        fprintf(fp, "%d %g %g %g %d\n", pai[i], pa[i].x, pa[i].y, pa[i].z, kpi[i]);
    }


    fclose(fp);

    return 0;
}
