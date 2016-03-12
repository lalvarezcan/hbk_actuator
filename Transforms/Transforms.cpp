#include "mbed.h"
#include "Transforms.h"
#include "FastMath.h"

using namespace FastMath;

void Transforms::Park(float alpha, float beta, float theta, float *d, float *q){
    float cosine = cos(theta);
    float sine = sin(theta);
    *d = alpha*cosine - beta*sine;
    *q = -beta*cosine - alpha*sine;
    //*d = alpha*cosine + beta*sine;
    //*q = beta*cosine - alpha*sine;
    //DAC->DHR12R1 = (int) (*q*49.648f) + 2048;
    //DAC->DHR12R1 = (int) (*q*2048.0f) + 2048;
    }

void Transforms::InvPark(float d, float q, float theta, float *alpha, float *beta){
    float cosine = cos(theta);
    float sine = sin(theta);
    *alpha = d*cosine - q*sine;
    *beta = q*cosine + d*sine;
    }

void Transforms::Clarke(float a, float b, float *alpha, float *beta){
    *alpha = a;
    *beta = 0.57735026919f*(a + 2.0f*b);
    }

void Transforms::InvClarke(float alpha, float beta, float *a, float *b, float *c){
    *a = alpha;
    *b = 0.5f*(-alpha + 1.73205080757f*beta);
    *c = 0.5f*(-alpha - 1.73205080757f*beta);
    } 

