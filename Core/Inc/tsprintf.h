/*
  Tiny sprintf module
   for Embedded microcontrollers

   (Ver 1.0)
*/

#ifndef _TSPRINTF_H_
#define _TSPRINTF_H_

#include <stdarg.h>

extern int tsprintf(char* ,char* , ...);
extern int vtsprintf(char* buff,char* fmt,va_list arg);

#endif /* _TSPRINTF_H_ */
