
#include "math_ops.h"


float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }
    
void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }
