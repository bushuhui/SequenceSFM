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
#include <math.h>

#include "utils_afm.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void AFM_FrameInfo::print(void)
{
    printf("deg: %d - %d (%f - %f), time: %2d:%2d:%2d, spd: %3d, height: %3d, pos_res: %3d",
           long_m, lat_m,
           long_deg, lat_deg,
           tm_h, tm_m, tm_s,
           speed, height,
           pos_resolution);
}

int AFM_FrameInfo::time_stamp(void)
{
    return tm_h*60*60 + tm_m*60 + tm_s;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int AFM_Reader::open(const char *fname)
{
    FILE            *fp = NULL;

    fp = fopen(fname, "rb");
    if( fp == NULL ) {
        printf("ERR: failed to open file (%s)\n", fname);
        return -1;
    }

    frame_buf_len = filelength(fp);
    frame_buf = new ru8[frame_buf_len];

    fread(frame_buf, frame_buf_len, 1, fp);

    fclose(fp);

    return 0;
}

int AFM_Reader::close(void)
{
    if( frame_buf != NULL ) {
        delete frame_buf;
        frame_buf_len = 0;

        frame_n = 0;
        frame_i = 0;
        frame_pos.clear();
    }

    return 0;
}

int AFM_Reader::parse(void)
{
    int             i, i1, i2, ts;
    AFM_FrameInfo   fi;

    int             s = 0;

    // check file pointer is NULL
    if( frame_buf_len <= 0 ) {
        printf("ERR: file not open!\n");
        return -1;
    }

    frame_n = 0;
    frame_i = 0;
    i = 0;
    i1 = 0;
    i2 = 0;

    // seek frame begin position
    i = 0;
    while(1) {
        if( 0 == seek_frame_begin(i, i1) )
            break;

        i += frame_bytes;
    }

    // get first frame
    i = i1;
    get_frame_at_pos(i1, fi);
    frame_pos.push_back(i1);
    ts = fi.time_stamp();
    frame_time.push_back(ts);
    //printf("[%5d] %6d %6d\n", frame_n, i1, ts);
    frame_n ++;


    while(1) {
        i1 = i  + frame_bytes;
        i2 = i1 + frame_bytes;
        if( i1 >= frame_buf_len ) break;

        if( frame_buf[i1] != 0xF1 ) {
            printf("Not Sync\n");

            s = 0;
            while(1) {
                if( 0 != seek_frame_begin(i1, i) ) {
                    i1 = i1 + frame_bytes;
                    if( i1 >= frame_buf_len ) {
                        s = 1;
                        break;
                    }
                } else {
                    s = 2;
                    break;
                }
            }

            if( s == 0 || s == 1 ) break;

            i1 = i;
            i2 = i1 + frame_bytes;
        }

        if( i2 > frame_buf_len ) break;

        // get frame info
        get_frame_at_pos(i1, fi);

        frame_pos.push_back(i1);
        ts = fi.time_stamp();
        frame_time.push_back(ts);
        //printf("[%5d] %6d %6d\n", frame_n, i1, ts);

        frame_n ++;

        i += frame_bytes;
    }

    return 0;
}

int AFM_Reader::seek_frame_begin(int pos, int &pos_beg, int seek_len)
{
    int     i = 0, i1 = 0, i2 = 0;

    pos_beg = -1;

    // check file pointer is NULL
    if( frame_buf_len <= 0 ) {
        printf("ERR: file not open!\n");
        return -1;
    }

    // seek frame begin position
    for(i=0; i<seek_len; i++) {
        i1 = pos + i;
        i2 = i1  + frame_bytes;

        if( i1 >= frame_buf_len || i2 > frame_buf_len )
            return -2;

        if( frame_buf[i1] == 0xF1 && frame_buf[i2] == 0xF1 ) {
            pos_beg = i1;
            break;
        }
    }

    if( pos_beg < 0 ) {
        printf("ERR: can not get begining frame\n");
        return -3;
    }

    return 0;
}

int AFM_Reader::get_frame_at_pos(int pos, AFM_FrameInfo &fi)
{
    double      d, m, s;

    // 0xF1 [long_0] [long_1] [long_2] [long_3] [lat_0] [lat_1] [lat_2] [lat_3]
    fi.long_m = frame_buf[pos+1]       | frame_buf[pos+2] << 8 |
                frame_buf[pos+3] << 16 | frame_buf[pos+4] << 24;
    fi.lat_m  = frame_buf[pos+5]       | frame_buf[pos+6] << 8 |
                frame_buf[pos+7] << 16 | frame_buf[pos+8] << 24;


    m  = 1.0 * fi.long_m / 10000.0;
    d = (int)( m / 60.0 );
    m  = m - d*60.0;
    s  = m - (int)(m);
    fi.long_deg = d + m/60.0 + s/3600.0;

    m  = 1.0 * fi.lat_m / 10000.0;
    d = (int)( m / 60.0 );
    m  = m - d*60.0;
    s  = m - (int)(m);
    fi.lat_deg = d + m/60.0 + s/3600.0;


    // 0xF6 [speed] [hight low] [hight high]
    // 45   46      47          48
    fi.speed  = frame_buf[pos+46];
    fi.height = frame_buf[pos+47] | frame_buf[pos+48] << 8;

    // 0xF7 [hour] [minute] [second]
    // 54   55     56       57
    fi.tm_h = (frame_buf[pos+55] + local_time) % 24;
    fi.tm_m = frame_buf[pos+56];
    fi.tm_s = frame_buf[pos+57];

    fi.pos_resolution = frame_buf[pos+62];

    return 0;
}

int AFM_Reader::get_frame(int frm_idx, AFM_FrameInfo &fi)
{
    int         pos;

    if( frm_idx < 0 || frm_idx >= frame_n ) {
        printf("ERR: frame index out of range!\n");
        return -1;
    }

    pos = frame_pos[frm_idx];
    return get_frame_at_pos(pos, fi);
}

int AFM_Reader::get_frame(AFM_FrameInfo &fi)
{
    return get_frame(frame_i, fi);
}

int AFM_Reader::get_frame_by_ts(int ts, AFM_FrameInfo &fi)
{
    vector<int>::iterator       it;
    int                         i, ti, pos;


    ti = -1;
    for(it=frame_time.begin(),i=0; it!=frame_time.end(); it++, i++) {
        if( *it >= ts ) {
            ti = i;
            break;
        }
    }

    if( ti >= 0 ) {
        pos = frame_pos[ti];
        return get_frame_at_pos(pos, fi);
    }

    return -1;
}


int AFM_Reader::get_frame_idx(void)
{
    return frame_i;
}


int AFM_Reader::next(int n)
{
    frame_i+=n;

    if( frame_i >= frame_n ) {
        frame_i = frame_n -1;
        return -1;
    }

    return 0;
}

int AFM_Reader::prev(int n)
{
    frame_i-=n;

    if( frame_i < 0 ) {
        frame_i = 0;
        return -1;
    }

    return 0;
}

int AFM_Reader::first(void)
{
    frame_i = 0;
    return 0;
}

int AFM_Reader::last(void)
{
    frame_i = frame_n - 1;
    return 0;
}

int AFM_Reader::frames(void)
{
    return frame_n;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

double calc_earth_dis(double long_1, double lat_1, double long_2, double lat_2)
{
    const double    EARTH_RADIUS = 6378137.0;
    double          radLat1, radLat2;
    double          a, b, s;

    radLat1 = lat_1*M_PI/180.0;
    radLat2 = lat_2*M_PI/180.0;

    a = radLat1 - radLat2;
    b = (long_1 - long_2)*M_PI/180.0;

    s = 2*asin( sqrt(sqr(sin(a/2)) + cos(radLat1)*cos(radLat2)*sqr(b/2)) );
    s = s*EARTH_RADIUS;
    //s = round(s*10000)/10000.0;

    return s;
}


double calc_longitude_unit(double lat)
{
    double      a, f, e_2, l, l2;
    double      phi, phi_rad;
    int         i;

    a = 6378137.0;
    f = 1.0/298.257222101;
    e_2 = 2*f - f*f;

    phi_rad = lat;

    l  = M_PI/180.0 * a * cos(phi_rad) / sqrt(1 - e_2*sin(phi_rad)*sin(phi_rad));
    l2 = a*cos(phi_rad) * M_PI/180.0;

    return l;

}

int calc_earth_offset(double lng1, double lat1, double lng2, double lat2,
                      double &dx, double &dy)
{
    double      lng_unit;
    double      a;

    a = 6378137.0;

    lng_unit = calc_longitude_unit(lat1);
    dx = (lng2 - lng1) * lng_unit;
    dy = a*(lat2 - lat1)*M_PI/180.0;

    return 0;
}
