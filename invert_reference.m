function [Xnew Pnew]=invert_reference(X1, P1)
  Xnew=[-X1(1)*cos(X1(3))-X1(2)*sin(X1(3));
        X1(1)*sin(X1(3))-X1(2)*cos(X1(3));
        -X1(3)];
  if size(P1, 1)~=0,
    Jinv=[-cos(X1(3)), -sin(X1(3)), -X1(1)*sin(X1(3))-X1(2)*cos(X1(3));
          sin(X1(3)), -cos(X1(3)), X1(1)*cos(X1(3))+X1(2)*sin(X1(3));
          0, 0, -1];
    Pnew=Jinv*P1*Jinv';
  else
    Pnew=[];
  end;
return;