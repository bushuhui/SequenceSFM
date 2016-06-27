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

    CParamArray     *pa = pa_get();

    SfM_Tracker     sfmTracker(pa);

    int             i;

    // load parameters
    fn_in = "./data/img_sfm_1s";
    pa->s("fn_in", fn_in);

    // get image file names
    path_lsdir(fn_in, fl_img);

    // create pcd viewer
    PCD_Viewer *viewer;
    pcv_qt("PointCloud Viewer", &viewer);

    viewer->setSceneCenter(qglviewer::Vec(0, 0, 0));
    viewer->setSceneRadius(1000.0);

    sfmTracker.setPCDViewer(viewer);

    // list images
    i = 0;
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
        Mat img = imread(fn_img);

        sfmTracker.appendFrame(img, fn_img);
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int         g_argc;
char        **g_argv;

int test_sfm_tracker(CParamArray *pa)
{
    qt_start(g_argc, g_argv,
             test_sfm_tracker_threadfunc,
             NULL);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct RTK_TestFunctionArray g_fa[] =
{
    RTK_FUNC_TEST_DEF(test_sfm_tracker,         "Detect keypoints"),

    {NULL,  "NULL",  "NULL"},
};


int main(int argc, char *argv[])
{
    CParamArray     *pa = pa_create();

    g_argc = argc;
    g_argv = argv;

    return rtk_test_main(argc, argv,
                         g_fa, *pa);
}

