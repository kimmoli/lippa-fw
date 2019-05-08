#ifndef HELPERS_H
#define HELPERS_H

#include "chprintf.h"

extern BaseSequentialStream* consoleStream;

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define PRINT(f_, ...) chprintf(consoleStream, (f_), ##__VA_ARGS__)
#define DEBUG(f_, ...) do { PRINT("%s(%d) : ", __func__,__LINE__); PRINT((f_), ##__VA_ARGS__); } while (0)

#define ISPRINT(c) (char)(((((char)c)>=((char)32))&&(((char)c)<=((char)126)))?((char)c):((char)'.'))

extern void dump(const char* data, int len);

#endif

