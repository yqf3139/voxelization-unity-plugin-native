#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#include <emmintrin.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>
#include <math.h>

#if !defined(max)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#if !defined(min)
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#if !defined(PI)
#define PI 3.14159
#endif

#include <basetsd.h>

#define VSID 1024   //hard-coded VXL x&y dimensions
#define MAXZDIM 256 //hard-coded VXL z dimension

#define STATICSPACE true

#define LMAXDIM 12
#define LMAYDIM 12
#define LMAZDIM 12
#define MAXDIM (1<<LMAXDIM)
#define MAYDIM (1<<LMAYDIM)
#define MAZDIM (1<<LMAZDIM)
#define KMOD32(a) ((a)&31)

typedef struct { int x, y; } lpoint2d;
typedef struct { int x, y, z; } ipoint3d;
typedef struct { unsigned short x, y, z, dum; int prev, itri; } hashdat_t;
typedef struct { union { struct { float x, y, z; }; float a[3]; }; } point3d;
typedef struct { int x, y, z, i; } trityp;
typedef struct { union { struct { float x, y, z; }; float a[3]; }; float u, v; } vertyp;

#define _DLLExport __declspec (dllexport) //标记为导出函数;

//定义函数指针;
typedef void(__stdcall *CPPCallback)(int tick);

extern "C" _DLLExport void SetCallback(CPPCallback callback);

extern "C" _DLLExport long long dlltest();

extern "C" _DLLExport int* getResultBuffer();

extern "C" _DLLExport int* getTriBuffer();

extern "C" _DLLExport float* getVertexBuffer();

extern "C" _DLLExport int* getColourBuffer();

extern "C" _DLLExport int* getTriMatBuffer();

extern "C" _DLLExport float* getBondBuffer();

extern "C" _DLLExport void voxel(int, int, int, int);

extern "C" _DLLExport void construct(int, int, int);

extern "C" _DLLExport void constructCylinder(
	int, int, float, float, float, float);

extern "C" _DLLExport void deconstruct();

extern "C" _DLLExport void setGradientColor(
	bool, int, int, int, int, int, int, int);

extern "C" _DLLExport void setSolidFill(
	bool);

extern "C" _DLLExport void setFillColor(
	int);