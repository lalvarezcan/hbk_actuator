
#include "FastMath.h"

namespace Transforms{
    void Park(float alpha, float beta, float theta, float *d, float *q);
    void InvPark(float d, float q, float theta, float *alpha, float *beta);
    void Clarke(float a, float b, float *alpha, float *beta);
    void InvClarke(float alpha, float beta, float *a, float *b, float *c);
    
    };
    
