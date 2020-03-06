% Name        : [X3]=compose_point(X1, X2)
% Description : Applies a 2D transformation to a set of 2D points.
% Input       : X1 - 2D transformation (x,y,o)'
%               X2 - Set of points. Each column is a point (x,y)'
% Output      : X3 - The set of points in X2 transformed by X1.
function [X3]=compose_point(X1, X2)
  s=sin(X1(3));
  c=cos(X1(3));
  X3=[X1(1)+[c -s]*X2;
      X1(2)+ [s c]*X2];
return;