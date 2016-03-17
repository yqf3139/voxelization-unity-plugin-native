#include "DLL.h"

//Starting maximums: bytes per unit:
static int MAXMAT = 512;		// 8 (    4KB)
static int MAXTRIS = 32768;		//16 (  512KB)
static int MAXVERTS = 32768;	//20 (  640KB)
static int MAXHASH = 1048576;	//12 (12288KB)

//Dynamic arrays:
static vertyp *vert = 0;
static trityp *tri = 0;
static int *colrgb = 0;
static hashdat_t *hashdat = 0;

#define LHASHEAD 20
static int hashead[1 << LHASHEAD], hashcnt = 0;

static int xsiz, ysiz, zsiz;
static float lastx[MAYDIM], xpiv, ypiv, zpiv;

static int SAMLEMODE = 0;
static int LEDX, LEDY, LEDZ, LEDXYZMAX;

static int frameidx = 0;

static int numobjects = 0, numverts = 0, numtris = 0, matnum;

static int output[200][200][200];
static int ledpal[257];
static float vertbuffer[32768 << 2];
static int tribuffer[32768 << 2];
static int trimatbuffer[32768];

static float step = 7.0f;
static float distance = 7.0f;
static float height = 5.0f;
static float pillar = 7.0f;

static int floorCounter = 80;
static int roundsCounter = 18;
static float diameter, radius, perimeter;

static point3d positions[20][200];
static int numPerRound[20];

static void vox_free();

static void quitout(char *str)
{
	if (str) puts(str);

	if (hashdat) free(hashdat);
	if (colrgb) free(colrgb);
	if (tri) free(tri);
	if (vert) free(vert);
	vox_free();

	exit(0);
}

#define cvtss2si(f) _mm_cvt_ss2si(_mm_set_ss(f))
#define cvttss2si(f) _mm_cvtt_ss2si(_mm_set_ss(f))

// Hashmap x,y,z => tri idx begins -----------------------------------------------------
#define GETHASH(x,y,z) (((x+y)*(x-y)+z) & ((1<<LHASHEAD)-1))
static void hash_init(void) { memset(hashead, -1, sizeof(hashead)); hashcnt = 0; }
static int hash_get(int x, int y, int z)
{
	__int64 q;
	int i, hashval;

	hashval = GETHASH(x, y, z);
	q = (__int64)((y << 16) + x) + (((_int64)z) << 32);
	for (i = hashead[hashval];i >= 0;i = hashdat[i].prev)
		if (*(__int64 *)&hashdat[i].x == q) return(hashdat[i].itri);
	return(-1);
}
static void hash_set(int x, int y, int z, int v)
{
	__int64 q;
	int i, hashval;

	hashval = GETHASH(x, y, z);
	q = (__int64)((y << 16) + x) + (((_int64)z) << 32);
	for (i = hashead[hashval];i >= 0;i = hashdat[i].prev)
		if (*(__int64 *)&hashdat[i].x == q) 
		{ 
			hashdat[i].itri = v; return; 
		}
	if (hashcnt >= MAXHASH)
	{
		MAXHASH += (MAXHASH >> 3);
		if (!(hashdat = (hashdat_t *)realloc(hashdat, sizeof(hashdat_t)*MAXHASH))) quitout("realloc failed: hashdat");
	}
	hashdat[hashcnt].prev = hashead[hashval]; hashead[hashval] = hashcnt;
	*(__int64 *)&hashdat[hashcnt].x = q;
	hashdat[hashcnt].itri = v;
	hashcnt++;
}
static int getpix32(int x, int y, int z)
{
	int i;

	i = hash_get(x, y, z);
	if (i < 0)
	{
		// printf("hash_get 0");
		return(0x808080);
	}

	return(tri[i].i);
}
// Hashmap x,y,z => tri idx ends   -----------------------------------------------------

//FLOODFILL2D/3D begins -----------------------------------------------------

#define FILLBUFSIZ 65536 //Use realloc instead!
typedef struct { unsigned short x, y, z0, z1; } cpoint4d; //512K
static cpoint4d fbuf[FILLBUFSIZ];

#ifdef _MSC_VER
#ifndef _WIN64
static __forceinline unsigned int bsf(unsigned int a) { _asm bsf eax, a }
static __forceinline unsigned int bsr(unsigned int a) { _asm bsr eax, a }
#else
static __forceinline unsigned int bsf(unsigned int a) { unsigned long r; _BitScanForward(&r, a); return(r); }
static __forceinline unsigned int bsr(unsigned int a) { unsigned long r; _BitScanReverse(&r, a); return(r); }
#endif
#else
#define bsf(r) ({ long __r=(r); __asm__ __volatile__ ("bsf %0, %0;": "=a" (__r): "a" (__r): "cc"); __r; })
#define bsr(r) ({ long __r=(r); __asm__ __volatile__ ("bsr %0, %0;": "=a" (__r): "a" (__r): "cc"); __r; })
#endif

static __forceinline int dntil0(int *iptr, int z, int zmax)
{
	//   //This line does the same thing (but slow & brute force)
	//while ((z < zmax) && (iptr[z>>5]&(1<<KMOD32(z)))) z++; return(z);
	int i;
	//WARNING: zmax must be multiple of 32!
	i = (iptr[z >> 5] | ((1 << KMOD32(z)) - 1)); z &= ~31;
	while (i == -1)
	{
		z += 32; if (z >= zmax) return(zmax);
		i = iptr[z >> 5];
	}
	return(bsf(~i) + z);
}

static __forceinline int dntil1(int *iptr, int z, int zmax)
{
	//   //This line does the same thing (but slow & brute force)
	//while ((z < zmax) && (!(iptr[z>>5]&(1<<KMOD32(z))))) z++; return(z);
	int i;
	//WARNING: zmax must be multiple of 32!
	i = (iptr[z >> 5] & (-1 << KMOD32(z))); z &= ~31;
	while (!i)
	{
		z += 32; if (z >= zmax) return(zmax);
		i = iptr[z >> 5];
	}
	return(bsf(i) + z);
}

//--------------------------------------------------------------------------------------------------

//Set all bits in iptr from (z0) to (z1-1) to 0's
static __forceinline void bit_setzrange0(int *iptr, int z0, int z1)
{
	int z, ze;
	if (!((z0^z1)&~31)) { iptr[z0 >> 5] &= ((~(-1 << KMOD32(z0))) | (-1 << KMOD32(z1))); return; }
	z = (z0 >> 5); ze = (z1 >> 5);
	iptr[z] &= ~(-1 << KMOD32(z0)); for (z++;z<ze;z++) iptr[z] = 0;
	iptr[z] &= (-1 << KMOD32(z1));
}

//Set all bits in iptr from (z0) to (z1-1) to 1's
static __forceinline void bit_setzrange1(int *iptr, int z0, int z1)
{
	int z, ze;
	if (!((z0^z1)&~31)) { iptr[z0 >> 5] |= ((~(-1 << KMOD32(z1)))&(-1 << KMOD32(z0))); return; }
	z = (z0 >> 5); ze = (z1 >> 5);
	iptr[z] |= (-1 << KMOD32(z0)); for (z++;z<ze;z++) iptr[z] = -1;
	iptr[z] |= ~(-1 << KMOD32(z1));
}

//rle_t: unsigned short n, z0, z1, z0, z1, ..
static unsigned short *rle = 0;
static int *rlebit = 0, rlemal = 0;
static int rlehead[MAXDIM*MAYDIM];

static unsigned short zlst[MAXZDIM]; //temp array

//      n: # structs to alloc
//returns: bit index or crash if realloc fails
static int vox_bitalloc(int n)
{
	static int gind = 0;
	int i, oi, ie, i0, i1, cnt;

	i = gind; oi = i; ie = rlemal - n;
	for (cnt = 1;1;cnt--)
	{
		for (;i<ie;i = i1 + 1)
		{
			i0 = dntil0(rlebit, i, ie); if (i0 >= ie) break;
			i1 = dntil1(rlebit, i0 + 1, i0 + n); if (i1 - i0 < n) continue;
			bit_setzrange1(rlebit, i0, i0 + n); gind = i0 + n; return(i0);
		}
		cnt--; if (cnt < 0) break;
		i = 0; ie = min(oi, rlemal - n);
	}

	i = rlemal;
	rlemal = max((rlemal >> 2) + rlemal, rlemal + n); //grow space by ~25%
	rle = (unsigned short *)realloc(rle, rlemal*sizeof(rle[0])); if (!rle) { quitout("realloc(rle) failed"); }
	rlebit = (int *)realloc(rlebit, ((((int)rlemal + 31) >> 5) << 2) + 16); if (!rlebit) { quitout("realloc(rlebit) failed"); }
	bit_setzrange1(rlebit, i, i + n);
	bit_setzrange0(rlebit, i + n, rlemal);
	gind = i + n;
	return(i);
}
static void vox_bitdealloc(int i, int n) { bit_setzrange0(rlebit, i, i + n); }

static int rle_setzrange0(unsigned short *b0, int n0, unsigned short *b1, int z0, int z1)
{
	int i, n1 = 0;
	for (i = 0;i<n0;i += 2)
	{
		if ((b0[i] < z0) || (b0[i] > z1)) { b1[n1] = b0[i]; n1++; }
		if ((z0 >  b0[i]) && (z0 <= b0[i + 1])) { b1[n1] = z0; n1++; }
		if ((z1 >= b0[i]) && (z1 <  b0[i + 1])) { b1[n1] = z1; n1++; }
		if ((b0[i + 1] < z0) || (b0[i + 1] > z1)) { b1[n1] = b0[i + 1]; n1++; }
	}
	return(n1);
}
static int rle_setzrange1(unsigned short *b0, int n0, unsigned short *b1, int z0, int z1)
{
	int i, n1 = 0;
	if (!n0) { b1[0] = z0; b1[1] = z1; n1 = 2; }
	else
	{
		if (z0 <= b0[0]) { b1[n1] = z0; n1++; }
		if (z1 <  b0[0]) { b1[n1] = z1; n1++; }
		for (i = 0;i<n0;i += 2)
		{
			if ((b0[i] < z0) || (b0[i] > z1)) { b1[n1] = b0[i]; n1++; }
			if ((b0[i + 1] < z0) || (b0[i + 1] > z1)) { b1[n1] = b0[i + 1]; n1++; }
			if (i < n0 - 2)
			{
				if ((z0 >  b0[i + 1]) && (z0 <= b0[i + 2])) { b1[n1] = z0; n1++; }
				if ((z1 >= b0[i + 1]) && (z1 <  b0[i + 2])) { b1[n1] = z1; n1++; }
			}
		}
		if (z0 >  b0[n0 - 1]) { b1[n1] = z0; n1++; }
		if (z1 >= b0[n0 - 1]) { b1[n1] = z1; n1++; }
	}
	return(n1);
}
static int rle_xorzrange(unsigned short *b0, int n0, unsigned short *b1, int z0, int z1)
{
	int i, n1 = 0;
	if (!n0) { b1[0] = z0; b1[1] = z1; n1 = 2; }
	else
	{
		if (z0 < b0[0]) { b1[n1] = z0; n1++; }
		if (z1 < b0[0]) { b1[n1] = z1; n1++; }
		for (i = 0;i<n0;i++)
		{
			if ((b0[i] != z0) && (b0[i] != z1)) { b1[n1] = b0[i]; n1++; }
			if (i < n0 - 1)
			{
				if ((z0 > b0[i]) && (z0 < b0[i + 1])) { b1[n1] = z0; n1++; }
				if ((z1 > b0[i]) && (z1 < b0[i + 1])) { b1[n1] = z1; n1++; }
			}
		}
		if (z0 > b0[n0 - 1]) { b1[n1] = z0; n1++; }
		if (z1 > b0[n0 - 1]) { b1[n1] = z1; n1++; }
	}
	return(n1);
}

// Render geometry to vox buffer and colors to sparse hash ---------------------

static int vox_init(void)
{
	rlemal = 1048576;
	rle = (unsigned short *)malloc(rlemal*sizeof(rle[0])); if (!rle) return(0);
	rlebit = (int *)malloc((rlemal + 31) >> 3); if (!rlebit) return(0);
	memset(rlehead, -1, MAXDIM*MAYDIM*sizeof(rlehead[0]));
	return(1);
}

static void vox_free(void)
{
	if (rlebit) { free(rlebit); rlebit = 0; }
	if (rle) { free(rle); rle = 0; }
}

static int vox_test(int x, int y, int z)
{
	int i, ind, n;

	ind = rlehead[x*MAYDIM + y]; 
	if (ind < 0) return(0);
	n = rle[ind];
	for (i = ind + 1;i <= ind + n;i += 2) 
		if (z < rle[i + 1]) 
			return(z >= rle[i]);
	return(0);
}

static void vox_clear(int x, int y, int z)
{
	int i, n;
	i = rlehead[x*MAYDIM + y]; if (i < 0) return;
	vox_bitdealloc(i, rle[i] + 1);
	n = rle_setzrange0(&rle[i + 1], rle[i], zlst, z, z + 1); if (!n) { rlehead[x*MAYDIM + y] = -1; return; }
	i = vox_bitalloc(n + 1); rlehead[x*MAYDIM + y] = i; rle[i] = n;
	memcpy(&rle[i + 1], zlst, n*sizeof(rle[0]));
}
static void vox_set(int x, int y, int z)
{
	int i, n;
	i = rlehead[x*MAYDIM + y];
	if (i < 0) { zlst[0] = z; zlst[1] = z + 1; n = 2; }
	else { vox_bitdealloc(i, rle[i] + 1); n = rle_setzrange1(&rle[i + 1], rle[i], zlst, z, z + 1); }

	i = vox_bitalloc(n + 1); rlehead[x*MAYDIM + y] = i; rle[i] = n;
	memcpy(&rle[i + 1], zlst, n*sizeof(rle[0]));
}
static void vox_xor(int x, int y, int z)
{
	int i, n;
	i = rlehead[x*MAYDIM + y];
	if (i < 0) { zlst[0] = z; zlst[1] = z + 1; n = 2; }
	else { vox_bitdealloc(i, rle[i] + 1); n = rle_xorzrange(&rle[i + 1], rle[i], zlst, z, z + 1); if (!n) { rlehead[x*MAYDIM + y] = -1; return; } }
	i = vox_bitalloc(n + 1); rlehead[x*MAYDIM + y] = i; rle[i] = n;
	memcpy(&rle[i + 1], zlst, n*sizeof(rle[0]));
}

static void vox_setzrange0(int x, int y, int z0, int z1)
{
	int i, n;
	i = rlehead[x*MAYDIM + y]; if (i < 0) return;
	vox_bitdealloc(i, rle[i] + 1);
	n = rle_setzrange0(&rle[i + 1], rle[i], zlst, z0, z1); if (!n) { rlehead[x*MAYDIM + y] = -1; return; }
	i = vox_bitalloc(n + 1); rlehead[x*MAYDIM + y] = i; rle[i] = n;
	memcpy(&rle[i + 1], zlst, n*sizeof(rle[0]));
}
static void vox_setzrange1(int x, int y, int z0, int z1)
{
	int i, n;
	i = rlehead[x*MAYDIM + y];
	if (i < 0) { zlst[0] = z0; zlst[1] = z1; n = 2; }
	else { vox_bitdealloc(i, rle[i] + 1); n = rle_setzrange1(&rle[i + 1], rle[i], zlst, z0, z1); }
	i = vox_bitalloc(n + 1); rlehead[x*MAYDIM + y] = i; rle[i] = n;
	memcpy(&rle[i + 1], zlst, n*sizeof(rle[0]));

}
static void vox_xorzrange(int x, int y, int z0, int z1)
{
	int i, n;
	i = rlehead[x*MAYDIM + y];
	if (i < 0) { zlst[0] = z0; zlst[1] = z1; n = 2; }
	else { vox_bitdealloc(i, rle[i] + 1); n = rle_xorzrange(&rle[i + 1], rle[i], zlst, z0, z1); if (!n) { rlehead[x*MAYDIM + y] = -1; return; } }
	i = vox_bitalloc(n + 1); rlehead[x*MAYDIM + y] = i; rle[i] = n;
	memcpy(&rle[i + 1], zlst, n*sizeof(rle[0]));
}

//rle[] = {5,13,18,23};
//          11111111112222222222
//012345678901234567890123456789
//-----11111111-----11111-------
static int vox_dntil0(int x, int y, int z)
{
	int i, ind, n;

	//while ((z < zsiz) && (vox_test(x,y,z))) z++;

	ind = rlehead[x*MAYDIM + y];
	if (ind >= 0)
	{
		n = rle[ind];
		for (i = ind + 1;i <= ind + n;i += 2)
		{
			if (z < rle[i]) break;
			if (z < rle[i + 1]) { z = rle[i + 1]; break; }
		}
	}
	return(z);
}

static int vox_dntil1(int x, int y, int z)
{
	int i, ind, n;

	//while ((z < zsiz) && (!vox_test(x,y,z))) z++;

	ind = rlehead[x*MAYDIM + y]; if (ind < 0) { z = zsiz; }
	else
	{
		n = rle[ind];
		for (i = ind + 1;i <= ind + n;i += 2)
		{
			if (z < rle[i]) { z = rle[i]; break; }
			if (z < rle[i + 1]) break;
		}
		if (i > ind + n) z = zsiz;
	}
	return(z);
}

static int vox_uptil1(int x, int y, int z)
{
	int i, ind, n;

	//while ((z > 0) && (!vox_test(x,y,z-1))) z--;

	ind = rlehead[x*MAYDIM + y]; if (ind < 0) { z = 0; }
	else
	{
		n = rle[ind];
		for (i = ind + n - 1;i>ind;i -= 2)
		{
			if (z > rle[i + 1]) { z = rle[i + 1]; break; }
			if (z > rle[i]) break;
		}
		if (i <= ind) z = 0;
	}
	return(z);
}

//static void vox_copyslabxy_shiftupz(int xd, int yd, int xs, int ys, int zs)
//{
//	int i, ind;
//	ind = rlehead[xs*MAYDIM + ys];
//	if ((xs != xd) || (ys != yd)) { rlehead[xd*MAYDIM + yd] = ind; rlehead[xs*MAYDIM + ys] = -1; }
//	for (i = rle[ind] + ind;i>ind;i--) rle[i] -= zs;
//}

//--------------------------------------------------------------------------------------------------

static void floodfill3dbits(int x, int y, int z) //Conducts on 0's and sets to 1's
{
	int j, z0, z1, i0, i1;
	cpoint4d a;

	if (vox_test(x, y, z)) return;
	a.x = x; a.z0 = vox_uptil1(x, y, z);
	a.y = y; a.z1 = vox_dntil1(x, y, z + 1);
	vox_setzrange1(x, y, a.z0, a.z1);
	i0 = 0; i1 = 0; goto floodfill3dskip;
	do
	{
		a = fbuf[i0]; i0 = ((i0 + 1)&(FILLBUFSIZ - 1));
	floodfill3dskip:;
		for (j = 3;j >= 0;j--)
		{
			if (j & 1) { x = a.x + (j & 2) - 1; if ((unsigned int)x >= xsiz) continue; y = a.y; }
			else { y = a.y + (j & 2) - 1; if ((unsigned int)y >= ysiz) continue; x = a.x; }

			if (vox_test(x, y, a.z0)) { z0 = vox_dntil0(x, y, a.z0); z1 = z0; }
			else { z0 = vox_uptil1(x, y, a.z0); z1 = a.z0; }
			while (z1 < a.z1)
			{
				z1 = vox_dntil1(x, y, z1);
				fbuf[i1].x = x; fbuf[i1].y = y;
				fbuf[i1].z0 = z0; fbuf[i1].z1 = z1;
				i1 = ((i1 + 1)&(FILLBUFSIZ - 1));
				if (i0 == i1) puts("WARNING: floodfill3d stack overflow!");
				vox_setzrange1(x, y, z0, z1);
				z0 = vox_dntil0(x, y, z1); z1 = z0;
			}
		}
	} while (i0 != i1);
}

//FLOODFILL2D/3D ends -------------------------------------------------------

static void renderface0tri(int faceind)
{
	// TODO : skip outrange tris
	float f, u, fx, fy, fz, fx1, fy1, fz1, fx2, fy2, fz2;
	int i, j, k, ind[3], sy[3], sx0, sx1, x, y, z, i0, i1, i2;

	ind[0] = tri[faceind].x; ind[1] = tri[faceind].y; ind[2] = tri[faceind].z;

	for (k = 0;k<3;k++)
	{
		i0 = k; i2 = k - 1; if (i2 < 0) i2 = 2; i1 = 3 - i0 - i2;
		fx = vert[ind[0]].a[i0]; fx1 = vert[ind[1]].a[i0] - fx; fx2 = vert[ind[2]].a[i0] - fx;
		fy = vert[ind[0]].a[i1]; fy1 = vert[ind[1]].a[i1] - fy; fy2 = vert[ind[2]].a[i1] - fy;
		fz = vert[ind[0]].a[i2]; fz1 = vert[ind[1]].a[i2] - fz; fz2 = vert[ind[2]].a[i2] - fz;
		f = 1.0f / (fx1*fy2 - fy1*fx2);
		fx1 = (fx1*fz2 - fz1*fx2)*f;
		fy1 = (fz1*fy2 - fy1*fz2)*f;
		fz -= (fy1*fx + fx1*fy);

		for (i = 2;i >= 0;i--) 
			sy[i] = (int)vert[ind[i]].a[i1];

		for (i = 2, j = 0;i >= 0;j = i--)
		{
			if (sy[j] >= sy[i]) continue;
			f = (vert[ind[i]].a[i0] - vert[ind[j]].a[i0]) / (vert[ind[i]].a[i1] - vert[ind[j]].a[i1]);
			for (y = sy[j] + 1;y <= sy[i];y++)
				lastx[y] = ((float)y - vert[ind[j]].a[i1])*f + vert[ind[j]].a[i0];
		}
		for (i = 2, j = 0;i >= 0;j = i--)
		{
			if (sy[j] <= sy[i]) continue;
			f = (vert[ind[j]].a[i0] - vert[ind[i]].a[i0]) / (vert[ind[j]].a[i1] - vert[ind[i]].a[i1]);
			for (y = sy[i] + 1;y <= sy[j];y++)
			{
				u = ((float)y - vert[ind[i]].a[i1])*f + vert[ind[i]].a[i0];
				sx0 = (int)min(lastx[y], u);
				sx1 = (int)max(lastx[y], u);
				for (x = sx0 + 1;x <= sx1;x++)
				{
					z = (int)(x*fy1 + y*fx1 + fz);
					switch (k)
					{
					case 0: vox_set(x, y, z); hash_set(x, y, z, faceind); break;
					case 1: vox_set(z, x, y); hash_set(z, x, y, faceind); break;
					case 2: vox_set(y, z, x); hash_set(y, z, x, faceind); break;
					}
				}
			}
		}
	}
}

static int gclipx0, gclipy0, gclipz0, gclipx1, gclipy1, gclipz1;

// fill the inner of the mode as white
static void hollowfix(int xlen, int ylen, int zlen)
{
	gclipx0 = 0; gclipy0 = 0; gclipz0 = 0; gclipx1 = xsiz; gclipy1 = ysiz; gclipz1 = zsiz;

	int i, x, y, z;

	//Set all seeable 0's to 1's
	for (x = 0;x<xlen;x++)
		for (y = 0;y<ylen;y++)
		{
			floodfill3dbits(x, y, 0); floodfill3dbits(x, y, zlen - 1);
		}
	for (x = 0;x<xlen;x++)
		for (z = 0;z<zlen;z++)
		{
			floodfill3dbits(x, 0, z); floodfill3dbits(x, ylen - 1, z);
		}
	for (y = 0;y<ylen;y++)
		for (z = 0;z<zlen;z++)
		{
			floodfill3dbits(0, y, z); floodfill3dbits(xlen - 1, y, z);
		}

	//Invert all solid & air
	for (x = 0;x<xlen;x++)
		for (y = 0;y<ylen;y++)
			vox_xorzrange(x, y, 0, zlen);

	//Make surface voxels solid again
	for (i = hashcnt - 1;i >= 0;i--)
	{
		x = hashdat[i].x; if ((x < gclipx0) || (x >= gclipx1)) continue;
		y = hashdat[i].y; if ((y < gclipy0) || (y >= gclipy1)) continue;
		z = hashdat[i].z; if ((z < gclipz0) || (z >= gclipz1)) continue;
		vox_set(x, y, z);
	}
}

//Calculate scale and scale the verts -------------------------------------------------------

static float sf, sfx, sfy, sfz;

static void getscale(int voxdim)
{
	float g, h, fx0, fy0, fz0, fx1, fy1, fz1;
	int x, y, z, i, j;
	// Scale data to specified resolution ---------------------------------------

	fx0 = 1e32f; fx1 = -1e32f;
	fy0 = 1e32f; fy1 = -1e32f;
	fz0 = 1e32f; fz1 = -1e32f;
	for (z = 0;z<numverts;z++)
	{
		if (vert[z].x < fx0) fx0 = vert[z].x;
		if (vert[z].y < fy0) fy0 = vert[z].y;
		if (vert[z].z < fz0) fz0 = vert[z].z;
		if (vert[z].x > fx1) fx1 = vert[z].x;
		if (vert[z].y > fy1) fy1 = vert[z].y;
		if (vert[z].z > fz1) fz1 = vert[z].z;
	}

	printf("%9.6f,%9.6f,%9.6f\n", fx0, fy0, fz0);
	printf("%9.6f,%9.6f,%9.6f\n", fx1, fy1, fz1);

	h = (MAYDIM - .1f) / (fx1 - fx0); //Find maximum scale factor (g)
	sf = (MAXDIM - .1f) / (fy1 - fy0);
	if (sf < h) h = sf;
	sf = (MAZDIM - .1f) / (fz1 - fz0);
	if (sf < h) h = sf;

	g = fx1 - fx0; i = 'x';
	if (fy1 - fy0 > g) { g = fy1 - fy0; i = 'y'; }
	if (fz1 - fz0 > g) { g = fz1 - fz0; i = 'z'; }
	sf = ((float)voxdim - .1f) / g;
	if (sf > h)
	{
		sf = h; voxdim = cvtss2si(h*g);
		printf(" NOTE: %cdim limited to %d\n", i, voxdim);
	}
	if (voxdim <= 0) quitout("error: voxdim must be > 0");

	xsiz = cvtss2si((fy1 - fy0)*sf + .5f); if (xsiz > 256) printf(" WARNING: xsiz > 256!");
	ysiz = cvtss2si((fx1 - fx0)*sf + .5f); if (ysiz > 256) printf(" WARNING: ysiz > 256!");
	zsiz = cvtss2si((fz1 - fz0)*sf + .5f); if (zsiz > 256) printf(" WARNING: zsiz > 256!");

	sfy = fy0; sfx = fx0; sfz = fz1;

	printf("Scale factor used (voxel/polygon units):%9.6f\n", sf);

	xpiv = (0.f - sfy)*sf + .05f;
	ypiv = (0.f - sfx)*sf + .05f;
	zpiv = (sfz - 0.f)*sf + .05f;

	printf("Scale output: xsiz %d, ysiz %d, zsiz %d\n", xsiz, ysiz, zsiz);
	printf("Scale output: xpiv %9.6f, ypiv %9.6f, zpiv %9.6f\n", xpiv, ypiv, zpiv);

}

static void scale()
{
	point3d oldp;

	int z;
	for (z = 0;z<numverts;z++)
	{
		oldp = *(point3d *)&vert[z];
		vert[z].x = (oldp.y - sfy)*sf + .05f;
		vert[z].y = (oldp.x - sfx)*sf + .05f;
		vert[z].z = (sfz - oldp.z)*sf + .05f;
	}
	oldp.x = 0.f; oldp.y = 0.f; oldp.z = 0.f;
}

//Calculate scale and scale the verts ends -------------------------------------------------------

static void clearvox()
{
	hash_init();
	memset(rlehead, -1, MAXDIM*MAYDIM*sizeof(rlehead[0]));
	memset(output, 0, 200 * 200 * 200 * sizeof(int));
}

static void savevox()
{
	int i, j, x, y, z;

	if (257 < matnum)quitout("Error: 257 < matnum");
	for (i = 0;i<matnum;i++)
	{
		ledpal[i] = colrgb[i] << 8;
	}

	int realx, realy, realz, tmpx, tmpy, tmpz, angleShareCount;
	float fx, fz, distanceTo0, oneShare, angle;
	for (x = 0;x<xsiz;x++)
		for (y = 0;y<ysiz;y++)
			for (z = 0;z<zsiz;z++)
			{
				if (SAMLEMODE)
				{
					tmpx = y;
					tmpy = x;
					tmpz = zsiz - 1 - z;

					tmpx = 2 * tmpx - ysiz;
					tmpz = 2 * tmpz - zsiz;

					fx = (int)(((float)tmpx / ysiz) * radius);
					fz = (int)(((float)tmpz / zsiz) * radius);
					if (fz > radius || fx > radius)
						continue;

					distanceTo0 = sqrtf(fx * fx + fz * fz);
					if (distanceTo0 < 1.0f || distanceTo0 > radius)
						continue;

					realx = (int)(((float)tmpy / xsiz) * floorCounter);

					realy = (int)(distanceTo0 / step - 1.5f);
					realy = realy < 0 ? 0 : realy;

					angleShareCount = numPerRound[realy];
					oneShare = 2.0f * PI / angleShareCount;
					angle = acosf(fx / distanceTo0);
					angle = fz >= 0 ? angle : 2 * PI - angle;
					realz = (int)(angle / oneShare);
					realz = realz < 0 || realz == angleShareCount ? 0 : realz;
				}
				else 
				{
					realx = y;
					realy = x;
					realz = zsiz - 1 - z;
				}

				if (!vox_test(x, y, z))
				{
					output[realx][realy][realz] = 0; // blank, transparent
					continue;
				}

				// find the color surround by colors and mark as white(inner)
				j = 0;
				if ((!vox_test(x - 1, y, z))) j |= 1;
				if ((!vox_test(x + 1, y, z))) j |= 2;
				if ((!vox_test(x, y - 1, z))) j |= 4;
				if ((!vox_test(x, y + 1, z))) j |= 8;
				if ((!vox_test(x, y, z - 1))) j |= 16;
				if ((!vox_test(x, y, z + 1))) j |= 32;

				if (!j)
				{
					output[realx][realy][realz] = 0xffffff00; // full white
					continue;
				}

				output[realx][realy][realz] = ledpal[getpix32(x, y, z)];
			}

}

// =================== bridge to Unity 3D =================== //

long long dlltest()
{
	return 0;
}

void SetCallback(CPPCallback callback)
{
	int tick = 1223;
	callback(tick);
}

int* getResultBuffer() { return (int *)output; }

int* getTriBuffer() { return (int *)tribuffer; }

float* getVertexBuffer() { return (float *)vertbuffer; }

int* getColourBuffer() { return colrgb; }

int* getTriMatBuffer() { return (int *)trimatbuffer; }

void voxel(int _numobjects, int _numverts, int _numtris, int _matnum)
{
	numobjects = _numobjects;
	numverts = _numverts;
	numtris = _numtris;
	matnum = _matnum+1;

	// copy input data from buffers to local structs arrays
	for (int i = 0; i < _numverts; i++)
	{
		vert[i].x = vertbuffer[3 * i + 0];
		vert[i].y = vertbuffer[3 * i + 1];
		vert[i].z = vertbuffer[3 * i + 2];
	}
	for (int i = 0; i < _numtris; i++)
	{
		tri[i].x = tribuffer[i * 3 + 0];
		tri[i].y = tribuffer[i * 3 + 1];
		tri[i].z = tribuffer[i * 3 + 2];
		tri[i].i = trimatbuffer[i] + 1;
	}

	frameidx++;

	// calculate current scale
	// if the space dimensions are static
	// only calculate it on the first time
	if (!STATICSPACE || frameidx == 1)
	{
		// TODO : input vox space box
		getscale(LEDXYZMAX);
	}
	scale();

	// render the vox matrix
	clearvox();
	for (int i = numtris - 1;i >= 0;i--)
	{
		renderface0tri(i); //Minimal face fill (best!)
	}

	// fill the inside of the models
	hollowfix(xsiz, ysiz, zsiz);

	// output to the color matrix
	savevox();
}

FILE * pConsole;

void construct(int ledx, int ledy, int ledz)
{
	frameidx = 0;

	AllocConsole();
	freopen_s(&pConsole, "CONOUT$", "wb", stdout);

	if (!vox_init()) quitout("vox_init() failed");
	colrgb = (int *)malloc(sizeof(int)*MAXMAT); if (!colrgb) quitout("malloc failed: colrgb");
	tri = (trityp *)malloc(sizeof(trityp)*MAXTRIS); if (!tri) quitout("malloc failed: tri");
	vert = (vertyp *)malloc(sizeof(vertyp)*MAXVERTS); if (!vert) quitout("malloc failed: vert");
	hashdat = (hashdat_t *)malloc(sizeof(hashdat_t)*MAXHASH); if (!hashdat) quitout("malloc failed: hashdat");

	LEDX = ledx; LEDY = ledy; LEDZ = ledz;
	LEDXYZMAX = max(LEDX, max(LEDY, LEDZ));

	// Other mock options
	// Mock material 1 to red, argb
	colrgb[0] = 0x00ff0000;
	matnum = 1;
	numobjects = 1;
	printf("construct done\n");
}

void constructCylinder(
	int _floorCounter, int _roundsCounter, float _step, float _distance, float _height, float _pillar)
{
	SAMLEMODE = 1;

	floorCounter = _floorCounter;
	roundsCounter = _roundsCounter;
	step = _step;
	distance = _distance;
	height = _height;
	pillar = _pillar;

	int counter;
	for (int i = 0; i < roundsCounter; i++)
	{
		diameter = (pillar + (i + 1) * step * 2);
		radius = diameter / 2;
		perimeter = diameter * 3.14f;
		counter = (int)(perimeter / distance + 1.5f);
		float angle = 2.0f * PI / counter;

		numPerRound[i] = counter;
		for (int j = 0; j < counter; j++)
		{
			positions[i][j].x = radius * cos(angle * j);
			positions[i][j].y = 0.0f;
			positions[i][j].z = radius * sin(angle * j);
		}
	}

	construct((int)(counter / 3.0f), (int)(floorCounter), (int)(counter / 3.0f));
	printf("r %f, LEDX %d\n", radius, (int)(counter / 3.0f));
	printf("constructCylinder done\n");
}

void deconstruct()
{
	printf("deconstruct done\n");

	SAMLEMODE = 0;
	if (hashdat) free(hashdat);
	if (colrgb) free(colrgb);
	if (tri) free(tri);
	if (vert) free(vert);
	vox_free();
	fclose(pConsole);
}