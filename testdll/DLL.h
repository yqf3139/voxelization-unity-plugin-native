#if 0

Voxelization Unity Plugin Native Module by yqf3139

Modified from POLY2VOX.C by Ken Silverman(http://advsys.net/ken)

License for this code :
	* No commercial exploitation please
	* Do not remove my name or credit
	* You may distribute modified code / executables but please make it clear that it is modified
#endif

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
#include <basetsd.h>
#include <winsock2.h>

#pragma comment(lib, "Ws2_32.lib")

#if !defined(max)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#if !defined(min)
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#if !defined(PI)
#define PI 3.14159
#endif

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

#define _DLLExport __declspec (dllexport)

typedef void(__stdcall *CPPCallback)(int tick);

typedef struct { int x, y; } lpoint2d;
typedef struct { int x, y, z; } ipoint3d;
typedef struct { unsigned short x, y, z, dum; int prev, itri; } hashdat_t;
typedef struct { union { struct { float x, y, z; }; float a[3]; }; } point3d;
typedef struct { int x, y, z, i; } trityp;
typedef struct { union { struct { float x, y, z; }; float a[3]; }; float u, v; } vertyp;

// =================== export below funcions to Unity 3D =================== //

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

extern "C" _DLLExport void setSolidFill(bool);

extern "C" _DLLExport void setFillColor(int);