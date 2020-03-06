/* Name        : [t]=least_squares_cartesian(d1, d2)
   Description : Computes the transformation (x,y,o) that minimizes the sum
                 of squared distances between pair of points in d1 and d2.
   Input       : d1         - Reference points. Each column has the X,Y
                              coordinates.
                 d2         - Current points. Same format as S1. Each point
                              in S2 corresponds to the point in S1 in the
                              same index.
   Output      : t          - Motion (x,y,o)' from d1 to d2.
   Author      : Antoni Burguera Burguera
                 Based on the minimization proposed by Lu and Milios for the
                 ICP algorithm.
                 antoni.burguera@uib.es */

#include "mex.h"
#include <math.h>
#define NROWS   2
double mean(double *d, int nItems) {
    double s=0;
    int i;
    for (i=0;i<nItems*2;i+=2) {
        s+=d[i];
    }
    return s/(double)nItems;
}

double computes(double *d2, double *d1, double mx, double mx2, int nItems) {
    int i;
    double s=0;
    for (i=0;i<nItems*2;i+=2) {
        s+=((d2[i]-mx)*(d1[i]-mx2));
    }
    return s;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    /* Parameter check is not performed to increase speed. */
    double *d1;
    double *d2;
    double mx,my,mx2,my2,Sxx,Syy,Sxy,Syx,tx,ty,to,si,co;
    int numPoints, r, c;
    mxArray *b_out;
    double *b;
    d1=mxGetPr(prhs[0]);
    d2=mxGetPr(prhs[1]);
    numPoints=mxGetN(prhs[0]); 
    mx=mean(d2,numPoints);
    my=mean(&(d2[1]),numPoints);
    mx2=mean(d1,numPoints);
    my2=mean(&(d1[1]),numPoints);
    Sxx=computes(d2,d1,mx,mx2,numPoints);
    Syy=computes(&(d2[1]),&(d1[1]),my,my2,numPoints);
    Sxy=computes(d2,&(d1[1]),mx,my2,numPoints);
    Syx=computes(&(d2[1]),d1,my,mx2,numPoints);
    to=atan2(Sxy-Syx,Sxx+Syy);
    co=cos(to);
    si=sin(to);
    tx=mx2-(mx*co-my*si);
    ty=my2-(mx*si+my*co);
    b_out=plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    b=mxGetPr(b_out);
    b[0]=tx;
    b[1]=ty;
    b[2]=to;   
    return;
}





