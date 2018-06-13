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


#ifdef USE_PBA
#include <pba.h>
#endif

#include "sfm_ba.h"


//count number of 2D measurements
int Count2DMeasurements_pba(const vector<CloudPoint>& pointcloud)
{
    int K = 0;

    for (unsigned int i=0; i<pointcloud.size(); i++) {
        K += pointcloud[i].img_kp.size();
    }

    return K;
}

int adjustBundle_pba(vector<CloudPoint> &pointcloud,
                        Mat &camIntrinsic,
                        Mat &camDistortion,
                        map<int, vector<KeyPoint> > &imgpts,
                        map<int, Matx34d>  &Pmats)
{
    int     i, j, k;
    int     N, M, K;

    N = Pmats.size();
    M = pointcloud.size();
    K = Count2DMeasurements_pba(pointcloud);

    printf("\nadjustBundle_pba: \n");
    printf("  N (cams) = %d, M (points) = %d, K (measurements) = %d\n", N, M, K);

#ifdef USE_PBA
    // CameraT, Point3D, Point2D are defined in pba/DataInterface.h
    vector<CameraT>         camera_data;    //camera (input/ouput)
    vector<Point3D>         point_data;     //3D point(iput/output)
    vector<Point2D>         measurements;   //measurment/projection vector
    vector<int>             camidx, ptidx;  //index of camera/point for each projection

    float   f, cx, cy,
            d[2],
            q[9], c[3];

    f  = (camIntrinsic.at<double>(0,0) + camIntrinsic.at<double>(1,1))/2.0;
    cx = camIntrinsic.at<double>(0,2);
    cy = camIntrinsic.at<double>(1,2);

    d[0] = camDistortion.at<double>(0);
    d[1] = camDistortion.at<double>(1);

    //////////////////////////////////////////////////////////
    /// camera data
    //////////////////////////////////////////////////////////
    camera_data.resize(N);

    map<int, Matx34d>::iterator itcam;
    map<int, int> cam_inv_map;

    for(i=0,itcam=Pmats.begin(); itcam!=Pmats.end(); i++,itcam++) {
        int camId  = itcam->first;
        Matx34d &P = itcam->second;

        q[0] = P(0, 0); q[1] = P(0, 1); q[2] = P(0, 2); c[0] = P(0, 3);
        q[3] = P(1, 0); q[4] = P(1, 1); q[5] = P(1, 2); c[1] = P(1, 3);
        q[6] = P(2, 0); q[7] = P(2, 1); q[8] = P(2, 2); c[2] = P(2, 3);

        camera_data[i].SetFocalLength(f);
        camera_data[i].SetMatrixRotation(q);
        camera_data[i].SetTranslation(c);
        camera_data[i].SetNormalizedMeasurementDistortion(d[0]);    // FIXME: why d[0]?

        //camera_data[i].SetFixedIntrinsic();

        cam_inv_map[camId] = i;
    }

    // fix first frame
    camera_data[0].SetConstantCamera();

    //////////////////////////////////////////////////////////
    /// point data & measurements
    //////////////////////////////////////////////////////////
    float   pt[3];
    int     cidx, kpidx;
    float   imx, imy;

    map<int, int>::iterator itkp;

    point_data.resize(M);

    for(j=0; j<M; j++) {
        pt[0] = pointcloud[j].pt.x;
        pt[1] = pointcloud[j].pt.y;
        pt[2] = pointcloud[j].pt.z;

        point_data[j].SetPoint(pt);

        // for each image point
        for(itkp=pointcloud[j].img_kp.begin(); itkp!=pointcloud[j].img_kp.end(); itkp++) {
            cidx  = cam_inv_map[itkp->first];
            kpidx = itkp->second;

            camidx.push_back(cidx);    //camera index
            ptidx.push_back(j);        //point index

            imx = (imgpts[itkp->first][kpidx].pt.x - cx);
            imy = (imgpts[itkp->first][kpidx].pt.y - cy);

            measurements.push_back(Point2D(imx, imy));
        }
    }

    //////////////////////////////////////////////////////////
    /// begin PBA
    //////////////////////////////////////////////////////////
    ParallelBA::DeviceT device = ParallelBA::PBA_CUDA_DEVICE_DEFAULT;

    ParallelBA pba(device);

    pba.SetFixedIntrinsics(true);

    pba.SetCameraData(camera_data.size(),  &camera_data[0]);    //set camera parameters
    pba.SetPointData(point_data.size(),    &point_data[0]);     //set 3D point data
    pba.SetProjection(measurements.size(), &measurements[0],    //set the projections
                        &ptidx[0], &camidx[0]);

    pba.RunBundleAdjustment();


    //////////////////////////////////////////////////////////
    /// copy data back
    //////////////////////////////////////////////////////////
    for(j=0; j<M; j++) {
        point_data[j].GetPoint(pt);

        pointcloud[j].pt.x = pt[0];
        pointcloud[j].pt.y = pt[1];
        pointcloud[j].pt.z = pt[2];
    }

    for(i=0,itcam=Pmats.begin(); itcam!=Pmats.end(); i++,itcam++) {
        Matx34d &P = itcam->second;

        camera_data[i].GetMatrixRotation(q);
        camera_data[i].GetTranslation(c);

        P(0, 0) = q[0]; P(0, 1) = q[1]; P(0, 2) = q[2]; P(0, 3) = c[0];
        P(1, 0) = q[3]; P(1, 1) = q[4]; P(1, 2) = q[5]; P(1, 3) = c[1];
        P(2, 0) = q[6]; P(2, 1) = q[7]; P(2, 2) = q[8]; P(2, 3) = c[2];
    }
#else
    return adjustBundle_ssba(pointcloud,
                      camIntrinsic,
                      imgpts, Pmats);
#endif

    return 0;
}
