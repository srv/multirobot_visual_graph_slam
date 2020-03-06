% Name        : [t]=least_squares_cartesian(d1, d2)
% Description : Computes the transformation (x,y,o) that minimizes the sum
%               of squared distances between pair of points in d1 and d2.
% Input       : d1         - Reference points. Each column has the X,Y
%                            coordinates.
%               d2         - Current points. Same format as S1. Each point
%                            in S2 corresponds to the point in S1 in the
%                            same index.
% Output      : t          - Motion (x,y,o)' from d1 to d2.
% Author      : Antoni Burguera Burguera
%               Based on the minimization proposed by Lu and Milios for the
%               ICP algorithm.
%               antoni.burguera@uib.es
function [t]=least_squares_cartesian(d1, d2)
    mx=mean(d2(1,:));
    my=mean(d2(2,:));
    mx2=mean(d1(1, :));
    my2=mean(d1(2, :));
    Sxx=sum((d2(1, :)-mx).*(d1(1, :)-mx2));
    Syy=sum((d2(2, :)-my).*(d1(2, :)-my2));
    Sxy=sum((d2(1, :)-mx).*(d1(2, :)-my2));
    Syx=sum((d2(2, :)-my).*(d1(1, :)-mx2));  
    t(3,1)=atan2(Sxy-Syx,Sxx+Syy);
    t(1,1)=mx2-(mx*cos(t(3,1))-my*sin(t(3,1)));
    t(2,1)=my2-(mx*sin(t(3,1))+my*cos(t(3,1)));
return;