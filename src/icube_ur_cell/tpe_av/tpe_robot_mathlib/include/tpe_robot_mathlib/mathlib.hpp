#ifndef TPE_ROBOT_MATHLIB__MATHLIB_HPP_
#define TPE_ROBOT_MATHLIB__MATHLIB_HPP_

//#include "tpe_robot_mathlib/visibility_control.h"

namespace tpe_robot_mathlib
{

double  **dmatrice( int  p , int q );
double *dvect( int  p );
void  Detruitdvect( double *v ) ;
void  Detruitdmatrice( double **m, int p );
void dcopy( double **m , double **m2 , int p , int q );
void mxv( double **m , double *v1 , double *v2 , int p , int q );
void mxm( double  **m1, double  **m2 , double  **m3 , int p , int q ,int r );
int pinvGreville(double **A,int m, int n,double **B);

}  // namespace tpe_robot_mathlib

#endif  // TPE_ROBOT_MATHLIB__MATHLIB_HPP_
