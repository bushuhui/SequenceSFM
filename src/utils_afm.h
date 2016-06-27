/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#ifndef __UTILS_AFM_H__
#define __UTILS_AFM_H__

#include <vector>
#include <map>

#include "utils.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct AFM_FrameInfo
{
    int         long_m, lat_m;          // minute
    double      long_deg, lat_deg;      // degree

    int         height;                 // height (GPS)
    int         speed;                  // speed (GPS)

    int         tm_h, tm_m, tm_s;       // GPS time
    int         pos_resolution;         // GPS position resolution

    void print(void);
    int  time_stamp(void);
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class AFM_Reader
{
public:
    AFM_Reader() {
        _init();
    }

    ~AFM_Reader() {
        _release();
    }

    int open(const char *fname);
    int close(void);
    int parse(void);

    int get_frame(int frm_idx, AFM_FrameInfo &fi);
    int get_frame(AFM_FrameInfo &fi);

    int get_frame_by_ts(int ts, AFM_FrameInfo &fi);

    int get_frame_idx(void);
    int get_frame_len(int &len);

    int next(int n=1);
    int prev(int n=1);
    int first(void);
    int last(void);

    int frames(void);

private:
    void _init(void) {
        frame_buf = NULL;
        frame_buf_len = 0;

        frame_n = 0;
        frame_i = 0;
        frame_pos.clear();

        frame_bytes = 72;               // one frame size (in bytes)
        local_time = 8;                 // UTC+8 (China)

    }

    void _release(void) {
        frame_pos.clear();
        frame_time.clear();
    }


    int get_frame_at_pos(int pos, AFM_FrameInfo &fi);
    int seek_frame_begin(int pos, int &pos_beg, int seek_len=145);

private:

    ru8                 *frame_buf;
    ru32                frame_buf_len;

    int                 frame_n;        // total frames
    int                 frame_i;        // current frames
    std::vector<int>    frame_pos;      // frame positions
    std::vector<int>    frame_time;     // frame time


    ru32                frame_bytes;    // one frame size (in bytes)
    int                 local_time;     // local time
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

double calc_earth_dis(double long_1, double lat_1, double long_2, double lat_2);
double calc_longitude_unit(double lat);
int calc_earth_offset(double lng1, double lat1, double lng2, double lat2,
                      double &dx, double &dy);


#endif // end of __UTILS_AFM_H__
