% Name        : [X3 P3]=compose_references(X1, X2, P1, P2)
% Description : Composes two poses. If provided, it also computes the
%               covariance.
% Input       : X1 - Transformation from A to B
%               X2 - Transformation from B to C
%               P1 - Covariance of X1
%               P2 - Covariance of X2
% Output      : X3 - Transformation from A to C
%               P3 - Covariance of X3
function [X3,P3]=compose_references(X1, X2, P1, P2)
  P3=[];
  X3=[X1(1)+X2(1)*cos(X1(3))-X2(2)*sin(X1(3));
      X1(2)+X2(1)*sin(X1(3))+X2(2)*cos(X1(3));
      X1(3)+X2(3)]; % vector de estados de X1 a X3
  if size(P1, 1)~=0, 
      % si la dimensión de la primer columna de la matriz de covarianza es
      % diferente de cero 
    J1=[1, 0, -X2(1)*sin(X1(3))-X2(2)*cos(X1(3));
        0, 1, X2(1)*cos(X1(3))-X2(2)*sin(X1(3));
        0, 0, 1];
    J2=[cos(X1(3)), -sin(X1(3)), 0;
        sin(X1(3)), cos(X1(3)), 0;
        0, 0, 1];      
    P3=J1*P1*J1'+J2*P2*J2'; % matriz de covarianza de x3
  end;  
return;