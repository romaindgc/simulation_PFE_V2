#include "tpe_robot_mathlib/mathlib.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <malloc.h>
#include <string.h>
#include <math.h>

namespace tpe_robot_mathlib
{



/**********************  void  nerreur() ************************/

void  nerreur( char *erreur_text )


{

 fprintf( stderr , "*** ERREUR ...\n");
 fprintf( stderr , "%s\n",erreur_text);

 return;
}

/******************** double **dmatrice() **********************/

double  **dmatrice( int  p , int q )


{
 double     **m;
 int          i,t;

 if ( !p || !q )
    return NULL;

 t = q*sizeof(double);

 m = (double**) malloc((unsigned) (p)*sizeof(double*) );

 if ( !m ){
     //nerreur(" echec pour l'allocation 1 dmatrice() ");
    return m;
    }

 for ( i = 0 ; i < p ; i++ )
     {
      m[i] = (double *) malloc(t);

      if ( !m[i] ){
            //nerreur(" echec pour l'allocation 2 dmatrice() ");
            m = NULL;
            return m;
        }
     }

 return m;
}

/*********************  DOUBLE *dvect() *************************/

double *dvect( int  p )


{
 double *v;

 if ( !p )
    return NULL;

 v = (double*) malloc((unsigned) (p)*sizeof(double));

 if ( !v ) 	{
    //nerreur(" echec pour l'allocation dvect() ");
    }


 return v;
}

/********************  void  Detruitdvect() **********************/

void  Detruitdvect( double *v )


{
 free((char*) (v) );

 return;
}

/******************** void  Detruitdmatrice() *******************/

void  Detruitdmatrice( double **m, int p )


{
    int  i;



    if(m==NULL)
    {
        return;
    }

    for ( i = p-1 ; i >= 0 ; i-- )  free( m[i] );

    free( m );

    return;
}
/******************** void dcopy() *************************/
void dcopy( double **m , double **m2 , int p , int q ) {
    int  i , j;
    for ( i = 0 ; i < p ; i++ ) {
       for ( j = 0 ; j < q ; j++ )
           m2[i][j] = m[i][j];
    }

}

/******************** void  mxm() *************************/

void mxm( double  **m1, double  **m2 , double  **m3 , int p , int q ,int r )


{
 int  i , j , k;


 for ( i = 0 ; i < p ; i++ )
     {
      for ( j = 0 ; j < r ; j++ )
      {

        m3[i][j] = 0.0;

       for ( k = 0 ; k < q ; k++ )
           m3[i][j] += m1[i][k]*m2[k][j];
      }
     }

 return;
}

/******************** void mxv() ****************************/

void mxv( double **m , double *v1 , double *v2 , int p , int q )


{
 int   i , j;


 for ( i = 0 ; i < p ; i++ )
     {
      v2[i] = 0.0;

      for ( j = 0 ; j < q ; j++ )  v2[i] += m[i][j]*v1[j];
     }

 return;
}

/******************** void  mxs() *************************/

void mxs( double  **m1, double  m2 , double **m3 , int p , int q )


{
 int  i , j;


 for ( i = 0 ; i < p ; i++ )
     {
      for ( j = 0 ; j < q ; j++ )
      {
       m3[i][j] = m2*m1[i][j];
      }
     }
 return;
}
/******************** void  dmxs() *************************/

void dmxs( double  **m1, double  m2 , double **m3 , int p , int q )


{
 int  i , j;


 for ( i = 0 ; i < p ; i++ )
     {
      for ( j = 0 ; j < q ; j++ )
      {
       m3[i][j] = m2*m1[i][j];
      }
     }
 return;
}
/********************* void  addm() *************************/

void  addm( double  **m1 , double  **m2 , double  **m3 , int p , int q )


{
 int  i , j;


 for ( i = 0 ; i < p ; i++ )
     {
      for ( j = 0 ; j < q ; j++ )
      m3[i][j] =  m1[i][j]+m2[i][j];
     }

 return;
}

/********************* void  subm() *************************/

void  subm( double  **m1 , double  **m2 , double  **m3 , int p , int q )


{
 int  i , j;


 for ( i = 0 ; i < p ; i++ )
     {
      for ( j = 0 ; j < q ; j++ )
      m3[i][j] =  m1[i][j]-m2[i][j];
     }

 return;
}
/********************* void  addv() *************************/

void  addv( double  *v1 , double  *v2 , double  *v3 , int p )


{
 int  i;

 for ( i = 0 ; i < p ; i++ ) v3[i] = v1[i]+v2[i];

 return;
}

/********************* void  subv() *************************/

void  subv( double  *v1 , double  *v2 , double  *v3 , int p )


{
 int  i;

 for ( i = 0 ; i < p ; i++ ) v3[i] = v1[i]-v2[i];

 return;
}

/********************** void  tr() **************************/

void tr( double **m, double **tm , int p ,int q )


{
 int  i , j;


 for ( i = 0 ; i < p ; i++ )
     {
      for ( j = 0 ; j < q ; j++ )  tm[j][i] = m[i][j];
     }

 return;
}

/********************** void  norm2() **************************/
double norm2(double *v, int m)	{
    int i;
    double nm;
    nm =0.0;

    for (i=0; i < m; i++)
        nm += v[i]*v[i];
    return(nm);

    }

void vxs(double *v,double s,double *w,int m)	{

    int i;

    for (i=0;i<m;i++)	{
        w[i] = s*v[i];
        }

    }


void vxm(double *v, double **W, double *b, int m, int n)	{

    int i,j;
    for (i=0;i<n;i++)	{
        b[i] = 0.0;
        for (j = 0; j <m; j++)
            b[i] +=v[j]*W[j][i];

        }

    }

int pinvGreville(double **A,int m, int n,double **B)	{

    double **At;
    double nck,ndk;
    int k;
    int i,j;
    double *dk,*ck,*tk/*,*bk*/;

    At = dmatrice(n,m);

    dk =dvect(n);
    ck =dvect(m);
    tk =dvect(m);
    //bk =dvect(m);

    tr(A,At,m,n);
    nck = norm2(At[0],m);


    if ((nck > 1e-10))
        dmxs(At,1.0/nck,B,1,m);
    else
        dmxs(At,0.0,B,1,m);

    for (k = 1; k < n; k++)	{

        mxv(B,At[k],dk,k,m);
        mxv(A,dk,tk,m,k);
        subv(At[k],tk,ck,m);
        nck = norm2(ck,m);

        if( (nck > 1e-10) )	{
            vxs(ck,1/nck,B[k],m);
            }

        else	{
            ndk = norm2(dk,k);
            vxm(dk,B,tk,k,m);
            vxs(tk,1/(1+ndk),B[k],m);
        }


        for ( i = 0; i< k; i++)
            for (j=0; j <m; j++)	{
                B[i][j] = B[i][j] - dk[i]*B[k][j];
            }

        }

    Detruitdmatrice(At,n);
    Detruitdvect(dk);
    Detruitdvect(ck);
    Detruitdvect(tk);
    return 0;

    }

}  // namespace tpe_robot_mathlib
