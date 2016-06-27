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

#include "utils.h"
#include "utils_event.h"

/******************************************************************************
 *	Key map
 *****************************************************************************/
/* Number & char key map */
static ru32 g_arrKeyTrans_Qt_0[OSA_VK_KEYTAB_LEN] =
{
    /* 000 - 004 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 005 - 009 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_BACK_SPACE, OSA_VK_TAB,
    /* 010 - 014 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_ENTER, OSA_VK_NULL,
    /* 015 - 019 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 020 - 024 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,

    /* 025 - 029 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_ESCAPE, OSA_VK_NULL, OSA_VK_NULL,
    /* 030 - 034 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_SPACE, OSA_VK_NULL, OSA_VK_NULL,
    /* 035 - 039 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_COMMA,
    /* 040 - 044 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 045 - 049 */	OSA_VK_MINUS, OSA_VK_PERIOD, OSA_VK_SLASH, OSA_VK_0, OSA_VK_1,

    /* 050 - 054 */	OSA_VK_2, OSA_VK_3, OSA_VK_4, OSA_VK_5, OSA_VK_6,
    /* 055 - 059 */	OSA_VK_7, OSA_VK_8, OSA_VK_9, OSA_VK_NULL, OSA_VK_SEMICOLON,
    /* 060 - 064 */	OSA_VK_NULL, OSA_VK_EQUALS, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 065 - 069 */	OSA_VK_A, OSA_VK_B, OSA_VK_C, OSA_VK_D, OSA_VK_E,
    /* 070 - 074 */	OSA_VK_F, OSA_VK_G, OSA_VK_H, OSA_VK_I, OSA_VK_J,

    /* 075 - 079 */	OSA_VK_K, OSA_VK_L, OSA_VK_M, OSA_VK_N, OSA_VK_O,
    /* 080 - 084 */	OSA_VK_P, OSA_VK_Q, OSA_VK_R, OSA_VK_S, OSA_VK_T,
    /* 085 - 089 */	OSA_VK_U, OSA_VK_V, OSA_VK_W, OSA_VK_X, OSA_VK_Y,
    /* 090 - 094 */	OSA_VK_Z, OSA_VK_OPEN_BRACKET, OSA_VK_BACK_SLASH, OSA_VK_CLOSE_BRACKET, OSA_VK_NULL,
    /* 095 - 099 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_a, OSA_VK_b, OSA_VK_c,

    /* 100 - 104 */	OSA_VK_d, OSA_VK_e, OSA_VK_f, OSA_VK_g, OSA_VK_h,
    /* 105 - 109 */	OSA_VK_i, OSA_VK_j, OSA_VK_k, OSA_VK_l, OSA_VK_m,
    /* 110 - 114 */	OSA_VK_n, OSA_VK_o, OSA_VK_p, OSA_VK_q, OSA_VK_r,
    /* 115 - 119 */	OSA_VK_s, OSA_VK_t, OSA_VK_u, OSA_VK_v, OSA_VK_w,
    /* 120 - 124 */	OSA_VK_x, OSA_VK_y, OSA_VK_z, OSA_VK_NULL, OSA_VK_NULL,

    /* 125 - 129 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 130 - 134 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 135 - 139 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 140 - 144 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 145 - 149 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL
};

/* key map for Qt 2 */
static ru32 g_arrKeyTrans_Qt_1[OSA_VK_KEYTAB_LEN] =
{
    /* 000 - 004 */	OSA_VK_ESCAPE, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_BACK_SPACE, OSA_VK_ENTER,
    /* 005 - 009 */	OSA_VK_NULL, OSA_VK_INSERT, OSA_VK_DELETE, OSA_VK_NULL, OSA_VK_NULL,
    /* 010 - 014 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 015 - 019 */	OSA_VK_NULL, OSA_VK_HOME, OSA_VK_END, OSA_VK_LEFT, OSA_VK_UP,
    /* 020 - 024 */	OSA_VK_RIGHT, OSA_VK_DOWN, OSA_VK_PAGE_UP, OSA_VK_PAGE_DOWN, OSA_VK_NULL,

    /* 025 - 029 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 030 - 034 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_SHIFT_L, OSA_VK_CONTROL_L, OSA_VK_NULL,
    /* 035 - 039 */	OSA_VK_ALT_L, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 040 - 044 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 045 - 049 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_F1, OSA_VK_F2,

    /* 050 - 054 */	OSA_VK_F3, OSA_VK_F4, OSA_VK_F5, OSA_VK_F6, OSA_VK_F7,
    /* 055 - 059 */	OSA_VK_F8, OSA_VK_F9, OSA_VK_F10, OSA_VK_F11, OSA_VK_F12,
    /* 060 - 064 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 065 - 069 */	OSA_VK_UP, OSA_VK_DOWN, OSA_VK_RIGHT, OSA_VK_LEFT, OSA_VK_NULL,
    /* 070 - 074 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,

    /* 075 - 079 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 080 - 084 */	OSA_VK_COLOR_KEY_0, OSA_VK_COLOR_KEY_1, OSA_VK_COLOR_KEY_2, OSA_VK_COLOR_KEY_3, OSA_VK_NULL,
    /* 085 - 089 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 090 - 094 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 095 - 099 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,

    /* 100 - 104 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 105 - 109 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 110 - 114 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 115 - 119 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 120 - 124 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,

    /* 125 - 129 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 130 - 134 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 135 - 139 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 140 - 144 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL,
    /* 145 - 149 */	OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL, OSA_VK_NULL
};


int input_event_qt_code_trans(ru32 cin, ru32 &cout)
{
    int     i;

    if( cin  &  0xFF000000 ) {
        i = cin & 0xFF;
        if( i > 149 ) return -1;

        cout = g_arrKeyTrans_Qt_1[i];
        return 0;
    } else {
        i = cin & 0xFF;
        if( i > 149 ) return -1;

        cout = g_arrKeyTrans_Qt_0[i];
        return 0;
    }
}
