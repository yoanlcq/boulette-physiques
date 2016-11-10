#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/glut.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

#include "Vector.h"
#include "Physics.h"

#define MIN( A, B ) ( (A) < (B) ? (A) : (B) ) //Just two macros to determine the minimum/maximum of two values
#define MAX( A, B ) ( (A) > (B) ? (A) : (B) )
#define SGN( A ) ( (A) < 0 ? (-1) : (1) ) //Sign of A

extern int GWidth; //We're gonna use those in the Physics.cpp, so we have to do this
extern int GHeight;

extern Physics World;

#endif
