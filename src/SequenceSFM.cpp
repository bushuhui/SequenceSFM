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
#include <opencv2/highgui/highgui.hpp>

#include "GSLAM/core/Svar.h"
#include "GImageIO/GImage_IO.h"

#include "utils.h"
#include "utils_gui.h"
#include "sfm_tracker.h"

using namespace std;
using namespace cv;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


void *test_sfm_tracker_threadfunc(void *args)
{
    string          fn_in, fn_img, fn_ext;
    StringArray     fl_img;
    vector<string>  img_list;


    // load parameters
    fn_in = svar.GetString("fn_in", "../data/img_sfm_1s");
    svar.ParseFile(fn_in+".ini");

    // get image file names
    path_lsdir(fn_in, fl_img);

    // create pcd viewer
    PCD_Viewer *viewer;
    pcv_qt("PointCloud Viewer", &viewer);

    viewer->setSceneCenter(qglviewer::Vec(0, 0, 0));
    viewer->setSceneRadius(1000.0);

    SfM_Tracker *sfmTracker = new SfM_Tracker();
    sfmTracker->setPCDViewer(viewer);

    // list images
    int i = 0;
    while(1) {
        if( i >= fl_img.size() ) break;

        fn_ext = path_extname(fl_img[i]);
        fn_ext = str_tolower(fn_ext);

        if( fn_ext == ".png" || fn_ext == ".jpg" ) {
            img_list.push_back(fl_img[i]);
        }

        i ++;
    }

    printf("Images: %d\n\n", img_list.size());

    for(i=0; i<img_list.size(); i++) {
        fn_img = path_join(fn_in, img_list[i]);
        Mat img = GSLAM::imread(fn_img.c_str());

        sfmTracker->appendFrame(img, fn_img);
    }

    delete sfmTracker;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    svar.ParseMain(argc, argv);

    qt_start(argc, argv,
             test_sfm_tracker_threadfunc,
             NULL);
}

