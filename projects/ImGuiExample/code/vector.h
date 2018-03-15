#include <math.h>
#include <xmmintrin.h>
#include <smmintrin.h>


struct v4
{
  float x;
  float y;
  float z;
  float w;
};

//Using this union you can use all the vector functions by using the sse part, and easily extract the data as floats. Unions are magic.

union xmm_v4
{
  struct v4 v;
  __m128 sse;
};

static inline __m128 set_v4(float x, float y, float z, float w) { return _mm_setr_ps(x, y, z, w); };

static inline __m128 add_v4(__m128 one, __m128 two) { return _mm_add_ps(one, two); }

static inline __m128 sub_v4(__m128 one, __m128 two) { return _mm_sub_ps(one, two); }

static inline __m128 mult_v4_f(__m128 one, const float two) { return _mm_mul_ps(one, _mm_load_ps1(&two)); }

static inline float dot_v4(__m128 one, __m128 two) { return _mm_cvtss_f32(_mm_dp_ps(one, two, 0xff)); }

static inline __m128 div_v4_f(__m128 one, const float two) { const float div = 1 / two; return _mm_mul_ps(one, _mm_load_ps1(&div)); }

static inline float mag_v4(__m128 one) { return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(one, one, 0x71))); }
