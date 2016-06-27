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
#include <string.h>

#include <vector>
#include <map>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include "sfm_utils.h"
#include "sfm_tracker.h"
#include "sfm_ba.h"
#include "sfm_cameraMatrices.h"
#include "sfm_triangulation.h"

#include "pba_util.h"



SfM_Tracker::SfM_Tracker()
{
    init();
}

SfM_Tracker::SfM_Tracker(CParamArray *pa_)
{
    init();

    pa = pa_;

    loadConf();
}

SfM_Tracker::~SfM_Tracker()
{
    free();
}

void SfM_Tracker::init(void)
{
    pa = NULL;
    pcdViewer = NULL;

    frmeQueueBegIndex = 0;

    frameCount = 0;

    foundBaseline = 0;
    baselineF1 = 0;
    baselineF2 = 1;
    baselineEvaluationValue = 9e99;

    // set parameters
    frameQueue_maxSize = 99999;

    tracker_keyFrameMinInterval = 5;
    tracker_keyFrameMaxInterval = 15;
    tracker_baselineMinImages = 10;
    tracker_baselineMaxImages = 20;

    tracker_dispThreshold = 60;

    tracker_keyFrameSearchRange = 4;

    tracker_visScale = 0.5;
}

void SfM_Tracker::free(void)
{
    FrameQueue::iterator it;

    for(it=frameQueue.begin(); it!=frameQueue.end(); it++) {
        delete *it;
    }

    frameQueue.clear();
}

int SfM_Tracker::setPCDViewer(PCD_Viewer *pcdv)
{
    pcdViewer = pcdv;

    return 0;
}

int SfM_Tracker::appendFrame(Mat &imgIn, string fnImg)
{
    SfM_Frame   *frame;
    SfM_Frame   *frame1, *frame2;


    vector<KeyPoint>        pts_n1, pts_n2;
    vector<DMatch>          matches, matches_f;
    vector<CloudPoint>      cloud, cloud_f;

    vector<int>             arrKF;

    int                     k;

    printf("\n\n");
    printf("===============================================\n");
    printf("Append image: %5d\n", frameCount);
    printf("===============================================\n");

    // append to frame queue
    frame = new SfM_Frame(frameCount++, imgIn, pa);
    frame->fnImg = fnImg;
    frameQueue.push_back(frame);


    if( foundBaseline ) {
        // triangulation new frame
        if( frameCount - keyframeLast >= tracker_keyFrameMinInterval ) {
            int                         maxKP, maxKP_frm;

            vector<Point2f>             _tmp2D, max2D;
            vector<Point3f>             _tmp3D, max3D;

            // detect keypoints
            frame->detectKeypoints();

            // get previous keyframes
            getKeyframes(arrKF, tracker_keyFrameSearchRange);

            maxKP = 0;
            maxKP_frm = 0;

            // for each previous keyframe
            for(k=0; k<arrKF.size(); k++) {
                frame1 = getFrame(arrKF[k]);

                printf("\nTRY: %5d - %5d\n", arrKF[k], frame->idx);

                // perform match
                if( 0 != frame->matchFrame(*frame1, 1) ) {
                    goto APPEND_FRAME_EXIT;
                    //return 0;
                }

                // get 2D - 3D corresponding
                get2D3DCorrespondings(frame1, frame, _tmp2D, _tmp3D);

                if( _tmp2D.size() > maxKP ) {
                    maxKP = _tmp2D.size();
                    maxKP_frm = arrKF[k];
                    max2D = _tmp2D;
                    max3D = _tmp3D;
                }
            }

            printf("\n\nmaxKP = %d, maxKP_frm = %d\n", maxKP, maxKP_frm);
            frame1 = getFrame(maxKP_frm);

            // estimate new frame's position
            cv::Matx34d P1 = getFrame(keyframeLast)->camP;
            cv::Mat_<double> t = (cv::Mat_<double>(1,3) << P1(0,3), P1(1,3), P1(2,3));
            cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P1(0,0), P1(0,1), P1(0,2),
                                                           P1(1,0), P1(1,1), P1(1,2),
                                                           P1(2,0), P1(2,1), P1(2,2));
            cv::Mat_<double> rvec(1,3); Rodrigues(R, rvec);

            if( estimateNewPos(max2D, max3D, rvec, t, R) ) {
                goto APPEND_FRAME_EXIT;
                //return -1;
            }

            Matx34d P_new = cv::Matx34d	(R(0,0),R(0,1),R(0,2),t(0),
                                         R(1,0),R(1,1),R(1,2),t(1),
                                         R(2,0),R(2,1),R(2,2),t(2));
            frame->camP = P_new;

            // try triangluate new frame
            vector<CloudPoint>  pcn;
            vector<int>         addFlags;

            if( triangulateTwoFrames(frame1, frame, pcn, addFlags) ) {
                goto APPEND_FRAME_EXIT;
                //return -2;
            }

            // insert to keyframe
            appendKeyframe(frame);

            // bundle adjust
            bundleAdjustFrames();

            if( pcdViewer != NULL ) {
                pcdViewer->setData(pointCloud);
                pcdViewer->setCamera(kf2CamMap);
                pcdViewer->redraw();

                // draw keypoints & motion lines
                Mat     img_2, img_kp2, img_kp_s2;

                frame->imgRaw.copyTo(img_2);
                frame->getMatchFine(frame1->idx, matches_f);

                drawKeypoints( img_2, frame->kpRaw, img_kp2,
                               Scalar(255, 0, 0), DrawMatchesFlags::DEFAULT );

                for(k=0; k<matches_f.size(); k++) {
                    Point2f p1, p2;
                    Point   p1i, p2i;
                    double  d;

                    p1 = frame1->kpRaw[matches_f[k].queryIdx].pt;
                    p2 = frame->kpRaw[matches_f[k].trainIdx].pt;

                    p1i.x = (int)( p1.x + 0.5 );    p1i.y = (int)( p1.y + 0.5 );
                    p2i.x = (int)( p2.x + 0.5 );    p2i.y = (int)( p2.y + 0.5 );

                    d = sqrt(sqr(p1.x-p2.x) + sqr(p1.y-p2.y));

                    cv::line(img_kp2, p1i, p2i, Scalar(0, 0, 255), 1);
                }

                resize(img_kp2, img_kp_s2, Size(), tracker_visScale, tracker_visScale);
                imshow_qt("kp2", img_kp_s2);
            }
        }
    }

    if( !foundBaseline ) {
        // detect keypoints
        frame->detectKeypoints();

        // find baseline triangulation
        if( frameCount >= tracker_baselineMinImages ) {
            findBaseline();
        }

        // if the current frame count > baseline max images
        if( frameCount >= tracker_baselineMaxImages ) {
            printf("\n\n>>> Found baseline: %5d - %5d (%9f) <<<\n",
                   baselineF1, baselineF2,
                   baselineEvaluationValue);

            foundBaseline = 1;

            frame1 = getFrame(baselineF1);
            frame2 = getFrame(baselineF2);
            frame2->getMatchRaw(baselineF1, matches);

            cv::Matx34d             P(  1,0,0,0,
                                        0,1,0,0,
                                        0,0,1,0),
                                    P1( 1,0,0,0,
                                        0,1,0,0,
                                        0,0,1,0);

            cloud.clear();
            cloud_f.clear();

            FindCameraMatrices(cam_k, cam_kinv, cam_d,
                               frame1->kpRaw, frame2->kpRaw,
                               pts_n1, pts_n2,
                               P, P1,
                               matches,
                               cloud);



            // determin point cloud good or not
            if (1) {
                double r_sigmean;

                CloudPoint_FilterZ(matches, matches_f, cloud, cloud_f, r_sigmean);
            }

            // process point cloud
            {
                int         pi1, pi2;
                CloudPoint  *cp;

                for(k=0; k<matches_f.size(); k++) {
                    pi1 = matches_f[k].queryIdx;
                    pi2 = matches_f[k].trainIdx;

                    cp = &( cloud_f[k] );

                    // set frame -> keypoint map
                    cloud_f[k].img_kp.clear();
                    cloud_f[k].img_kp.insert(make_pair(baselineF1, pi1));
                    cloud_f[k].img_kp.insert(make_pair(baselineF2, pi2));

                    // RGB
                    cloud_f[k].rgb = frame2->getColor(pi2);
                }
            }

            frame1->camP = P;
            frame2->camP = P1;
            frame2->addMatchFine(baselineF1, matches_f);

            // store keyframe data
            appendKeyframe(frame1);
            appendKeyframe(frame2);

            pointCloud = cloud_f;
            pointCloud.reserve(4096);

            if( pcdViewer != NULL ) {
                pcdViewer->setData(pointCloud);
                pcdViewer->setCamera(kf2CamMap);
                pcdViewer->redraw();
            }

            // bundle adjust
            bundleAdjustFrames();

            // show current point cloud
            if( pcdViewer != NULL ) {
                pcdViewer->setData(pointCloud);
                pcdViewer->setCamera(kf2CamMap);
                pcdViewer->redraw();

                // draw keypoints & motion lines
                Mat     img_2, img_kp2, img_kp_s2;

                frame2->imgRaw.copyTo(img_2);

                drawKeypoints( img_2, pts_n2, img_kp2,
                               Scalar(255, 0, 0), DrawMatchesFlags::DEFAULT );

                for(k=0; k<matches_f.size(); k++) {
                    Point2f p1, p2;
                    Point   p1i, p2i;
                    double  d;

                    p1 = frame1->kpRaw[matches_f[k].queryIdx].pt;
                    p2 = frame2->kpRaw[matches_f[k].trainIdx].pt;

                    p1i.x = (int)( p1.x + 0.5 );    p1i.y = (int)( p1.y + 0.5 );
                    p2i.x = (int)( p2.x + 0.5 );    p2i.y = (int)( p2.y + 0.5 );

                    d = sqrt(sqr(p1.x-p2.x) + sqr(p1.y-p2.y));

                    cv::line(img_kp2, p1i, p2i, Scalar(0, 0, 255), 1);
                }

                resize(img_kp2, img_kp_s2, Size(), tracker_visScale, tracker_visScale);
                imshow_qt("kp2", img_kp_s2);
            }

            //waitKey_qt(0);
        }
    }

APPEND_FRAME_EXIT:
    // save frame data
    {
        int key;

        key = waitKey_qt(20);

        // save frame data & pointCloud
        if( key == OSA_VK_P ) {
            saveKeyframes();
            savePointclouds();
        }
    }

    return 0;
}

SfM_Frame* SfM_Tracker::getFrame(int frmIdx)
{
    if( frmIdx < 0 || frmIdx >= frameCount)
        return NULL;

    int idx = frmIdx - frmeQueueBegIndex;

    if( idx < 0 || idx >= frameQueue.size() )
        return NULL;

    return frameQueue[idx];
}

int SfM_Tracker::appendKeyframe(SfM_Frame *f)
{
    f->isKeyFrame = 1;
    keyframeLast = f->idx;

    keyframeMap.insert(make_pair(f->idx, f));
    kf2CamMap.insert(make_pair(f->idx, f->camP));
    kf2KPMap.insert(make_pair(f->idx, f->kpRaw));

    return 0;
}

int SfM_Tracker::getKeyframes(vector<int> &keyframes, int n)
{
    KeyframeMap::reverse_iterator   rit;
    int                             k;

    keyframes.clear();
    k = 0;

    for(rit=keyframeMap.rbegin(); rit != keyframeMap.rend(); rit++) {
        keyframes.push_back(rit->first);

        k ++;
        if( k >= n ) break;
    }

    return 0;
}

int SfM_Tracker::getKeypointFromFrame(int frmIdx, vector<int> &kpSet)
{
    PointArray::iterator    it;
    map<int, int>           *m;
    map<int, int>::iterator itm;

    kpSet.clear();
    kpSet.reserve(pointCloud.size());

    for(it=pointCloud.begin(); it!=pointCloud.end(); it++) {
        m = &( it->img_kp );

        itm = m->find(frmIdx);
        if( itm != m->end() ) {
            kpSet.push_back(itm->second);
        }
    }

    // sort kp vector
    sort(kpSet.begin(), kpSet.end());

    return 0;
}

int SfM_Tracker::findBaseline(void)
{
    int         frameIdx1, frameIdx2;
    SfM_Frame   *frame1, *frame2;

    vector<KeyPoint>        pts_n1, pts_n2;
    vector<DMatch>          matches, matches_f;
    vector<CloudPoint>      cloud, cloud_f;

    double                  r_sigmean;
    int                     k;

    int                     frameSkip = 2;

    frameIdx2 = frameCount-1;
    frameIdx1 = frameIdx2 - tracker_keyFrameMinInterval;

    frame2 = getFrame(frameIdx2);

    while(1) {
        if( frameIdx2 - frameIdx1 > tracker_keyFrameMaxInterval )
            break;

        // get previous frame
        frame1 = getFrame(frameIdx1);
        if( frame1 == NULL ) break;

        printf("\nTRY: %5d - %5d\n", frameIdx1, frameIdx2);

        // perform match
        if( 0 != frame2->matchFrame(*frame1) ) {
            frameIdx1 -= frameSkip;
            continue;
        }

        frame2->getMatchRaw(frameIdx1, matches);

        // try trianglulation
        cv::Matx34d             P(  1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0),
                                P1( 1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0);

        cloud.clear();
        cloud_f.clear();

        FindCameraMatrices(cam_k, cam_kinv, cam_d,
                           frame1->kpRaw, frame2->kpRaw,
                           pts_n1, pts_n2,
                           P, P1,
                           matches,
                           cloud);

        if( cloud.size() < 10 ) {
            frameIdx1 -= frameSkip;
            continue;
        }

        // determin point cloud good or not
        //  FIXME: just for aerial images (imaging the ground)
        if (1) {
            CloudPoint_FilterZ(matches, matches_f, cloud, cloud_f, r_sigmean);

            if( r_sigmean < baselineEvaluationValue ) {
                baselineEvaluationValue = r_sigmean;
                baselineF1 = frameIdx1;
                baselineF2 = frameIdx2;
            }
        }

        // show current point cloud
        if( pcdViewer != NULL ) {
            vector<Matx34d> cams;

            cams.push_back(P);
            cams.push_back(P1);

            vector<CloudPoint>::iterator    it;
            for(it=cloud_f.begin(); it!=cloud_f.end(); it++) {
                it->rgb(0) = 255;
                it->rgb(1) = 255;
                it->rgb(2) = 255;
            }

            pcdViewer->setData(cloud_f);
            pcdViewer->setCamera(cams);
            pcdViewer->redraw();


            // draw keypoints & motion lines
            Mat     img_2, img_kp2, img_kp_s2;

            frame2->imgRaw.copyTo(img_2);

            drawKeypoints( img_2, pts_n2, img_kp2,
                           Scalar(255, 0, 0), DrawMatchesFlags::DEFAULT );

            for(k=0; k<matches_f.size(); k++) {
                Point2f p1, p2;
                Point   p1i, p2i;
                double  d;

                p1 = frame1->kpRaw[matches_f[k].queryIdx].pt;
                p2 = frame2->kpRaw[matches_f[k].trainIdx].pt;

                p1i.x = (int)( p1.x + 0.5 );    p1i.y = (int)( p1.y + 0.5 );
                p2i.x = (int)( p2.x + 0.5 );    p2i.y = (int)( p2.y + 0.5 );

                d = sqrt(sqr(p1.x-p2.x) + sqr(p1.y-p2.y));

                cv::line(img_kp2, p1i, p2i, Scalar(0, 0, 255), 1);
            }

            resize(img_kp2, img_kp_s2, Size(), tracker_visScale, tracker_visScale);
            imshow_qt("kp2", img_kp_s2);
        }

        frameIdx1 -= frameSkip;
    }

    return 0;
}

int SfM_Tracker::get2D3DCorrespondings(SfM_Frame *f1, SfM_Frame *f2,
                          vector<Point2f> &ptImg, vector<Point3f> &pt3D)
{
    vector<int>                 kpSet1, kpSet2, kpSetInt;
    vector<int>::iterator       itv;

    vector<DMatch>              matches;
    vector<DMatch>::iterator    itm;

    map<int, Point3d>           mapKeyPoint_3D;
    map<int, int>               mapKP1_KP2;

    int                         i, j;


    // get keypoint set frome frame 1
    {
        PointArray::iterator    it;
        map<int, int>           *m;
        map<int, int>::iterator itmii;

        kpSet1.clear();
        kpSet1.reserve(pointCloud.size());

        for(it=pointCloud.begin(); it!=pointCloud.end(); it++) {
            m = &( it->img_kp );

            itmii = m->find(f1->idx);
            if( itmii != m->end() ) {
                kpSet1.push_back(itmii->second);
                mapKeyPoint_3D.insert(make_pair(itmii->second, it->pt));
            }
        }

        // sort kp vector
        sort(kpSet1.begin(), kpSet1.end());
    }

    // get keypoint from frame 2
    f2->getMatchFine(f1->idx, matches);
    kpSet2.clear();
    kpSet2.reserve(matches.size());
    for(itm=matches.begin(); itm!=matches.end(); itm++) {
        kpSet2.push_back(itm->queryIdx);
        mapKP1_KP2.insert(make_pair(itm->queryIdx, itm->trainIdx));
    }
    sort(kpSet2.begin(), kpSet2.end());

    // get set intersection
    kpSetInt.resize(kpSet1.size()+kpSet2.size(), -1);
    itv = set_intersection(kpSet1.begin(), kpSet1.begin() + kpSet1.size(),
                           kpSet2.begin(), kpSet2.begin() + kpSet2.size(),
                           kpSetInt.begin());
    kpSetInt.resize(itv-kpSetInt.begin());

    printf("kpSet1 = %d, kpSet2 = %d, kpSetInt = %d\n",
           kpSet1.size(), kpSet2.size(), kpSetInt.size());

    // generate corresponding 2D - 3D data
    ptImg.clear();
    pt3D.clear();

    for(itv=kpSetInt.begin(); itv!=kpSetInt.end(); itv++) {
        i = *itv;

        j = mapKP1_KP2[i];
        ptImg.push_back(f2->kpRaw[j].pt);

        Point3d p3d = mapKeyPoint_3D[i];
        Point3f p3f = Point3f(p3d.x, p3d.y, p3d.z);
        pt3D.push_back(p3f);
    }


    return 0;
}

int SfM_Tracker::estimateNewPos(vector<Point2f> pt2D, vector<Point3f> pt3D,
                   cv::Mat_<double>& rvec,
                   cv::Mat_<double>& t,
                   cv::Mat_<double>& R)
{
    // check input data
    if(pt3D.size() <= 7 || pt2D.size() <= 7 || pt3D.size() != pt2D.size()) {
        // something went wrong aligning 3D to 2D points..
        dbg_pe("coundn't find enough corresponding cloud points %d", pt2D.size());
        return 1;
    }

    vector<int> inliers;
    double minVal,maxVal;

    cv::minMaxIdx(pt2D, &minVal, &maxVal);
    cv::solvePnPRansac(pt3D, pt2D, cam_k, cam_d,
                       rvec, t,
                       true, 1000, 0.006 * maxVal, 0.25 * (double)(pt2D.size()),
                       inliers,
                       CV_EPNP);

    vector<cv::Point2f> projected3D;
    cv::projectPoints(pt3D, rvec, t, cam_k, cam_d, projected3D);

    // get inliers
    if( inliers.size()==0 ) {
        for(int i=0; i<projected3D.size(); i++) {
            if( norm(projected3D[i]-pt2D[i]) < 10.0 )
                inliers.push_back(i);
        }
    }

    if( inliers.size() < (double)(pt2D.size())/5.0 ) {
        dbg_pe("Not enough inliers to estimate a good pose (%d/%d)\n",
               inliers.size(), pt2D.size());
        return 2;
    }

    cv::Rodrigues(rvec, R);
    if( !CheckCoherentRotation(R) ) {
        dbg_pe("Rotation is incoherent, it is necessary to try a different base view!\n");
        return 3;
    }

    return 0;
}

int SfM_Tracker::triangulateTwoFrames(SfM_Frame *f1, SfM_Frame *f2,
                                      vector<CloudPoint> &pcn, vector<int> &addFlag)
{
    vector<KeyPoint>    correspImg1Pt;

    // get camera matrix
    cv::Matx34d P  = f1->camP;
    cv::Matx34d P1 = f2->camP;

    // get keypoints & match
    std::vector<cv::KeyPoint> pt_set1, pt_set2;
    std::vector<cv::DMatch> matches;

    f2->getMatchFine(f1->idx, matches);
    GetAlignedPointsFromMatch(f1->kpRaw, f2->kpRaw,
                              matches,
                              pt_set1, pt_set2);

    // triangulation
    pcn.reserve(4096);
    double reproj_error = TriangulatePoints(pt_set1, pt_set2,
                                            cam_k, cam_kinv, cam_d,
                                            P, P1,
                                            pcn, correspImg1Pt);

    if( pcn.size() < 10 ) {
        dbg_pw("triangulated point is too few %d\n", pcn.size());
        return -1;
    }

    vector<uchar> trig_status;
    if( !TestTriangulation(pcn, P,  trig_status) ||
        !TestTriangulation(pcn, P1, trig_status) ) {
        dbg_pw("Triangulated points are not in front!\n");
        return -2;
    }

    //filter out outlier points with high reprojection
    vector<double> reprj_errors;
    for(int i=0;i<pcn.size();i++)
        reprj_errors.push_back(pcn[i].reprojection_error);
    std::sort(reprj_errors.begin(), reprj_errors.end());
    //get the 80% precentile
    //  threshold from Snavely07 4.2
    double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4;
    printf("reprj_err_cutoff: %f\n", reprj_err_cutoff);

    vector<CloudPoint> new_triangulated_filtered;
    std::vector<cv::DMatch> new_matches;

    new_triangulated_filtered.reserve(4096);
    new_matches.reserve(4096);

    /*
    for(int i=0;i<pcn.size();i++) {
        if(trig_status[i] == 0)
            continue; //point was not in front of camera
        if(pcn[i].reprojection_error > 16.0) {
            continue; //reject point
        }
        if(pcn[i].reprojection_error < 4.0 ||
            pcn[i].reprojection_error < reprj_err_cutoff) {
            new_triangulated_filtered.push_back(pcn[i]);
            new_matches.push_back(matches[i]);
        } else {
            continue;
        }
    }
    */
    for(int i=0;i<pcn.size();i++) {
        if(trig_status[i] == 0)
            continue; //point was not in front of camera

        if( pcn[i].reprojection_error < reprj_err_cutoff) {
            new_triangulated_filtered.push_back(pcn[i]);
            new_matches.push_back(matches[i]);
        } else {
            continue;
        }
    }

    // all points filtered out?
    if( new_triangulated_filtered.size() <= 10 ) {
        dbg_pw("Filted points are too few! %d\n", new_triangulated_filtered.size());
        return -3;
    }

    // use filtered points & matches
    pcn = new_triangulated_filtered;
    matches = new_matches;

    //now, determine which points should be added to the cloud
    addFlag.resize(pcn.size(), 0);

    int         f1idx = f1->idx;
    set<int>    set_pc_f1kp;

    vector<CloudPoint>::iterator    itcp;
    map<int, int>                   *m;
    map<int, int>::iterator         itm;

    map<int, map<int, int>*>        mapKP2CP;


    set_pc_f1kp.clear();
    mapKP2CP.clear();

    for(itcp=pointCloud.begin(); itcp!=pointCloud.end(); itcp++) {
        m = &( itcp->img_kp );

        itm = m->find(f1idx);
        if( itm != m->end() ) {
            set_pc_f1kp.insert(itm->second);
            mapKP2CP.insert(make_pair(itm->second, m));
        }
    }

    printf("set_pc_f1kp.size = %d, mapKP2CP.size = %d\n",
           set_pc_f1kp.size(), mapKP2CP.size());

    int                         pi, pi1, pi2;
    CloudPoint                  *pp;
    vector<CloudPoint>          cp_add;

    cp_add.clear();
    cp_add.reserve(1024);

    for(pi=0; pi<matches.size(); pi++) {

        pi1 = matches[pi].queryIdx;
        pi2 = matches[pi].trainIdx;

        // if found triangulated point
        if( mapKP2CP.count(pi1) > 0 ) {
            // add new frame index & keypoint index            
            mapKP2CP[pi1]->insert(make_pair(f2->idx, pi2));
        } else {
            // new added point
            pp = &( pcn[pi] );

            // frame -> keypoint map
            pp->img_kp.insert(make_pair(f1->idx, pi1));
            pp->img_kp.insert(make_pair(f2->idx, pi2));

            // set color
            pp->rgb = f2->getColor(pi2);

            // set flag
            addFlag[pi] = 1;

            // insert to global point cloud
            cp_add.push_back(pcn[pi]);
        }
    }

    for(int i=0; i<cp_add.size(); i++) {
        pointCloud.push_back(cp_add[i]);
    }

    printf("Triangluate frame %5d-%5d: %5d/%5d (%7d)\n",
           f1->idx, f2->idx, cp_add.size(), matches.size(), pointCloud.size());

    return 0;
}

int SfM_Tracker::bundleAdjustFrames(void)
{
    vector<Matx34d> cams;

    // bundle adjust
    if( kf2KPMap.size() < 5 )
        adjustBundle_ssba(pointCloud, cam_k, kf2KPMap, kf2CamMap);
    else
        adjustBundle_pba(pointCloud, cam_k, cam_d, kf2KPMap, kf2CamMap);

    // update frame's camera matrix
    map<int, Matx34d>::iterator it;
    for(it=kf2CamMap.begin(); it!=kf2CamMap.end(); it++) {
        keyframeMap[it->first]->camP = it->second;
        cams.push_back(it->second);
    }

    // plane fitting & filtering
    Vec4d       p_est;
    Matx34d     p_se3, axes_se3;
    vector<int> p_flt;
    PCD_Plane   gplane;
    PointArray  pc_new;

    vector<double>  arr_d;

    PointsPlaneFit_RANSAC(pointCloud,
                          p_est, p_se3,
                          p_flt);


    for(int i=0; i<pointCloud.size(); i++) {
        if( p_flt[i] ) {
            pc_new.push_back(pointCloud[i]);
        }
    }

    pointCloud = pc_new;


    // transform point cloud & cameras
    Matx34d &c1 = kf2CamMap.begin()->second;

    CalcCameraGround_axes(c1, p_est, p_se3, axes_se3);

    TransCloudPoint(axes_se3, pointCloud);
    TransCamera(axes_se3, kf2CamMap);

    for(it=kf2CamMap.begin(); it!=kf2CamMap.end(); it++) {
        keyframeMap[it->first]->camP = it->second;
    }

    // draw plane
#if 0
    if( pcdViewer != NULL ) {
        gplane.set(axes_se3, 10, 10);

        pcdViewer->setPlane(gplane);
        //pcdViewer->setData(pointCloud);
        //pcdViewer->redraw();
    }
#endif

    return 0;
}

int SfM_Tracker::loadConf(void)
{
    if( pa != NULL ) {
        pa->i("frameQueue_maxSize", frameQueue_maxSize);

        pa->i("tracker_keyFrameMinInterval", tracker_keyFrameMinInterval);
        pa->i("tracker_keyFrameMaxInterval", tracker_keyFrameMaxInterval);
        pa->d("tracker_dispThreshold", tracker_dispThreshold);
        pa->i("tracker_baselineMinImages", tracker_baselineMinImages);
        pa->i("tracker_baselineMaxImages", tracker_baselineMaxImages);
        pa->i("tracker_keyFrameSearchRange", tracker_keyFrameSearchRange);

        pa->d("tracker_visScale", tracker_visScale);

        // load camera calibiaration data
        {
#if 1
            double      fx, fy, cx, cy;
            double      d[5];
            string      camName = "cam_138s";

            pa->s("cam_name", camName);

            pa->d(camName + ".fx", fx);
            pa->d(camName + ".fy", fy);
            pa->d(camName + ".cx", cx);
            pa->d(camName + ".cy", cy);

            pa->d(camName + ".d0", d[0]);
            pa->d(camName + ".d1", d[1]);
            pa->d(camName + ".d2", d[2]);
            pa->d(camName + ".d3", d[3]);
            pa->d(camName + ".d4", d[4]);

            cam_k = cv::Mat::eye(3, 3, CV_64FC1);
            cam_k.at<double>(0, 0) = fx;
            cam_k.at<double>(1, 1) = fy;
            cam_k.at<double>(0, 2) = cx;
            cam_k.at<double>(1, 2) = cy;

            cam_d = cv::Mat::zeros(5, 1, CV_64FC1);
            cam_d.at<double>(0) = d[0];
            cam_d.at<double>(1) = d[1];
            cam_d.at<double>(2) = d[2];
            cam_d.at<double>(3) = d[3];
            cam_d.at<double>(4) = d[4];

            cam_kinv = cam_k.inv();

            cout << "cam_k: " << cam_k << "\n\n";
            cout << "cam_d: " << cam_d << "\n\n";
#else
            FileStorage     fs;
            string cam_calFN = "cam_138.yml";

            pa->s("cam_calFN", cam_calFN);

            fs.open(cam_calFN, FileStorage::READ);
            if( !fs.isOpened() ) {
                dbg_pe("Can not open file: %s\n", cam_calFN.c_str());
            } else {
                fs["camera_matrix"]           >> cam_k;
                fs["distortion_coefficients"] >> cam_d;

                cam_kinv = cam_k.inv();
            }
#endif
        }
    }
}

int SfM_Tracker::saveKeyframes(void)
{
    int             i;
    char            fn_buf[200];

    vector<int>     keyframes;
    SfM_Frame       *f;

    getKeyframes(keyframes, frameCount);
    for(i=0; i<keyframes.size(); i++) {
        f = getFrame(keyframes[i]);

        sprintf(fn_buf, "%s_info", f->fnImg.c_str());
        printf("save to file: %s\n", fn_buf);
        f->saveData(fn_buf, pointCloud);
    }

    return 0;
}

/*******************************************************************************
 * Save point cloud data to NVM format
 ******************************************************************************/
int SfM_Tracker::savePointclouds(char *fname)
{
    char            fn_buf[200];
    string          fn_path;
    int             i, j;

    // get output file name
    if( fname == NULL ) {
        pa->s("fn_in", fn_path);
        sprintf(fn_buf, "%s/pcd_all.nvm", fn_path.c_str());
    } else {
        strcpy(fn_buf, fname);
    }

    // PBA data struct
    vector<CameraT> camera_data;
    vector<Point3D> point_data;
    vector<Point2D> measurements;
    vector<int>     ptidx, camidx;
    vector<string>  cam_names;
    vector<int>     ptc;

    int             N, M, K;

    float           f, cx, cy,
                    d[2],
                    q[9], c[3];

    N = kf2CamMap.size();
    M = pointCloud.size();
    K = CloudPoint_GetMeasurementNum(pointCloud);

    f  = (cam_k.at<double>(0,0) + cam_k.at<double>(1,1))/2.0;
    cx = cam_k.at<double>(0,2);
    cy = cam_k.at<double>(1,2);

    d[0] = cam_d.at<double>(0);
    d[1] = cam_d.at<double>(1);

    //////////////////////////////////////////////////////////
    /// camera data
    //////////////////////////////////////////////////////////
    camera_data.resize(N);
    cam_names.reserve(N);


    map<int, Matx34d>::iterator itcam;
    map<int, int>               cam_inv_map;
    StringArray                 sa_filename;
    SfM_Frame                   *frame;

    for(i=0,itcam=kf2CamMap.begin(); itcam!=kf2CamMap.end(); i++,itcam++) {
        int camId  = itcam->first;
        Matx34d &P = itcam->second;

        q[0] = P(0, 0); q[1] = P(0, 1); q[2] = P(0, 2); c[0] = P(0, 3);
        q[3] = P(1, 0); q[4] = P(1, 1); q[5] = P(1, 2); c[1] = P(1, 3);
        q[6] = P(2, 0); q[7] = P(2, 1); q[8] = P(2, 2); c[2] = P(2, 3);

        camera_data[i].SetFocalLength(f);
        camera_data[i].SetMatrixRotation(q);
        camera_data[i].SetTranslation(c);

        cam_inv_map[camId] = i;

        // get image file name
        frame = getFrame(camId);
        sa_filename = path_split(frame->fnImg);
        if( sa_filename.size() > 1 )
            cam_names.push_back(sa_filename[1]);
        else
            cam_names.push_back("UNKNOWN");

    }

    //////////////////////////////////////////////////////////
    /// point data & measurements
    //////////////////////////////////////////////////////////
    float   pt[3];
    int     cidx, kpidx;
    float   imx, imy;

    map<int, int>::iterator itkp;

    point_data.resize(M);
    ptc.resize(M*3);

    for(j=0; j<M; j++) {
        // XYZ
        pt[0] = pointCloud[j].pt.x;
        pt[1] = pointCloud[j].pt.y;
        pt[2] = pointCloud[j].pt.z;
        point_data[j].SetPoint(pt);

        // RGB
        ptc[j*3+0] = pointCloud[j].rgb(0);
        ptc[j*3+1] = pointCloud[j].rgb(1);
        ptc[j*3+2] = pointCloud[j].rgb(2);


        // for each image point
        for(itkp=pointCloud[j].img_kp.begin(); itkp!=pointCloud[j].img_kp.end(); itkp++) {
            cidx  = cam_inv_map[itkp->first];
            kpidx = itkp->second;

            camidx.push_back(cidx);    //camera index
            ptidx.push_back(j);        //point index

            imx = (kf2KPMap[itkp->first][kpidx].pt.x - cx)/f;
            imx = (kf2KPMap[itkp->first][kpidx].pt.y - cy)/f;

            measurements.push_back(Point2D(imx, imy));
        }
    }

    // save to NVM file
    SaveNVM(fn_buf,
            camera_data, point_data, measurements,
            ptidx, camidx,
            cam_names,
            ptc);

    return 0;
}
