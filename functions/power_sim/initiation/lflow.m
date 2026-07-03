%---LFLOW.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This function comutes loadflow solution 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function [V,ANG,LINFLW,P_LOSS,YBUS,P,Q,Pinj,Qinj,iter]=lflow(BUSDATA,LINEDATA,flag_init,tole,maxiter);
%
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Intitialize 
%
 [nbus,V,ANG,Ps,Qs,nsl,npv,npq] = initialize(BUSDATA,flag_init);
 
% 
% form Ybus
 [YBUS] = buildybus(BUSDATA,LINEDATA);  
 [my ny ybus]=find(YBUS);
 absy=abs(ybus);
 angy=angle(ybus);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Start iteration
%%%%%%%%%%%%%%%%%%%%%%%%%
converged=0;
iter=0;
LANG=length(npv)+length(npq);
LV=length(npq);
%%%%%%%%%%%%%%%%%%%%%%%%%%
%calculate mismatch powers
%%%%%%%%%%%%%%%%%%%%%%%%%%
while ~converged & iter < maxiter
Pmn=V(my).*V(ny).*absy.*cos(ANG(my)-ANG(ny)-angy);
Qmn=V(my).*V(ny).*absy.*sin(ANG(my)-ANG(ny)-angy);
P=sparse(my,ny,Pmn);
Q=sparse(my,ny,Qmn);
Pinj=sum(P');
Qinj=sum(Q');
DP=Ps-Pinj';             
DQ=Qs-Qinj';     
DELP=DP(sort([npv;npq]));
DELQ=DQ(npq);
DF=[DELP;DELQ];

  
if all(abs(DF)< tole),
   converged=1;
   [LINFLW,P_LOSS]=lineflow(V,ANG,LINEDATA);
   iter=iter;
   return
else
   [JACOB]=jacobian(nbus,nsl,npq,npv,P,Q,Pinj,Qinj);
         
   DX=JACOB\DF;
   DANG=DX(1:LANG);
   if ~isempty(npq)
      DV=1+DX(LANG+1:length(DX));
      V(npq)=V(npq).*DV;
   end,
   
   ANG(sort([npv;npq]))=ANG(sort([npv;npq]))+DANG;
   
   iter=iter+1;
  end, %if
end, %while

          
         
 
