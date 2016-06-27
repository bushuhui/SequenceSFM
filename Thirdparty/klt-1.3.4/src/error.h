/*********************************************************************
 * error.h
 *********************************************************************/

#ifndef _ERROR_H_
#define _ERROR_H_

#include <stdio.h>
#include <stdarg.h>

namespace cv_klt {

void KLTError(const char *fmt, ...);
void KLTWarning(const char *fmt, ...);

}

#endif

