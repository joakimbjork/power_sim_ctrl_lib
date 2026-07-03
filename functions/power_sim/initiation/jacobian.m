%---JACOBIAN.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   Computation of Jacobian in  NEWTON-RAPHSON  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [JACOB]=jacobian(nbus,nsl,npq,npv,P,Q,Pinj,Qinj);
%

%%%%%%%%%%%%%%%%%
%Jacobian Matrix:
%%%%%%%%%%%%%%%%%
H= Q-diag(Qinj);
N= P+diag(Pinj);
J=-P+diag(Pinj);
L= Q+diag(Qinj);

H(:,nsl)=[];H(nsl,:)=[];
N(:,[nsl;npv])=[];N(nsl,:)=[];
J(:,nsl)=[];J([nsl;npv],:)=[];
L(:,[nsl;npv])=[];L([nsl;npv],:)=[];
JACOB=[H N;J L];
  
return
