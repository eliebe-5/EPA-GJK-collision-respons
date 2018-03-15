#include <math.h>
#include <xmmintrin.h>

#define PI 3.14159265

struct m4x4
{
  float m00, m01, m02, m03;
  float m10, m11, m12, m13;
  float m20, m21, m22, m23;
  float m30, m31, m32, m33;
};

struct _m128x4
{
  __m128 xmm0, xmm1, xmm2, xmm3;
};

//Using this union you can use all the matrix functions by using the sse part, and easily extract the data as floats.

union xmm_m4x4
{
  struct m4x4 m;
  struct _m128x4 sse;
};

static inline struct _m128x4 identity_m4x4()
{
  struct _m128x4 res;

  res.xmm0 = _mm_setr_ps(1.f, 0.f, 0.f, 0.f);
  res.xmm1 = _mm_setr_ps(0.f, 1.f, 0.f, 0.f);
  res.xmm2 = _mm_setr_ps(0.f, 0.f, 1.f, 0.f);
  res.xmm3 = _mm_setr_ps(0.f, 0.f, 0.f, 1.f);
	
  return res;
}

static inline __m128 lincomb_m128(__m128 a, struct _m128x4 B)
{
    __m128 result;
    result = _mm_mul_ps(_mm_shuffle_ps(a, a, 0x00), B.xmm0);
    result = _mm_add_ps(result, _mm_mul_ps(_mm_shuffle_ps(a, a, 0x55), B.xmm1));
    result = _mm_add_ps(result, _mm_mul_ps(_mm_shuffle_ps(a, a, 0xaa), B.xmm2));
    result = _mm_add_ps(result, _mm_mul_ps(_mm_shuffle_ps(a, a, 0xff), B.xmm3));
    return result;
}

static inline struct _m128x4 mult_m4x4(struct _m128x4 one, struct _m128x4 two)
{
  struct _m128x4 res;

  res.xmm0 = lincomb_m128(one.xmm0, two);
  res.xmm1 = lincomb_m128(one.xmm1, two);
  res.xmm2 = lincomb_m128(one.xmm2, two);
  res.xmm3 = lincomb_m128(one.xmm3, two);
  
  return res;
}

/*
	This could probably be done nicer and faster with sse, but it is not really math, so it's probably not too slow.
*/

static inline struct _m128x4 transpose_m4x4(struct _m128x4 one)
{
  union xmm_m4x4 res;
  union xmm_m4x4 arg;

  arg.sse = one;
  
  res.m.m00 = arg.m.m00; res.m.m01 = arg.m.m10; res.m.m02 = arg.m.m20; res.m.m03 = arg.m.m30;
  res.m.m10 = arg.m.m01; res.m.m11 = arg.m.m11; res.m.m12 = arg.m.m21; res.m.m13 = arg.m.m31;
  res.m.m20 = arg.m.m02; res.m.m21 = arg.m.m12; res.m.m22 = arg.m.m22; res.m.m23 = arg.m.m32;
  res.m.m30 = arg.m.m03; res.m.m31 = arg.m.m13; res.m.m32 = arg.m.m23; res.m.m33 = arg.m.m33;

  return res.sse;
}

static inline struct _m128x4 rot_m4x4(float x, float y, float z)
{
  struct _m128x4 res;

  union xmm_m4x4 mx;
  union xmm_m4x4 my;
  union xmm_m4x4 mz;
	
  mx.sse = identity_m4x4();
  my.sse = identity_m4x4();
  mz.sse = identity_m4x4();

  float val = PI / 180.0f;
  
  float cx = cos(x*val);
  float cy = cos(y*val);
  float cz = cos(z*val);

  float sx = sin(x*val);
  float sy = sin(y*val);
  float sz = sin(z*val);
  
  mx.m.m11 = cx; mx.m.m12 = -sx; mx.m.m21 = sx; mx.m.m22 = cx;
  my.m.m00 = cy; my.m.m02 = sy; my.m.m20 = -sy; my.m.m22 = cy;
  mz.m.m00 = cz; mz.m.m01 = -sz; mz.m.m10 = sz; mx.m.m11 = cz;
	
  res = mult_m4x4(mx.sse, mult_m4x4(my.sse, mz.sse));

  return res;
}

static inline struct _m128x4 translation_m4x4(float x, float y, float z)
{
  union xmm_m4x4 res;

  res.sse = identity_m4x4();
  res.m.m03 = x;
  res.m.m13 = y;
  res.m.m23 = z;

  return res.sse;
}
