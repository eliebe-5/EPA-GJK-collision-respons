#include "vector.h"
#include "matrix.h"

static inline __m128 transform_v4(struct _m128x4 one, __m128 two)
{
  return _mm_setr_ps(dot_v4(one.xmm0, two), dot_v4(one.xmm1, two), dot_v4(one.xmm2, two), dot_v4(one.xmm3, two));
}
