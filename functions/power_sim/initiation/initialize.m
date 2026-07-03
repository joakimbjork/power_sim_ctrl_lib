
%---INITIALI.M
function [nbus,V,ANG,Ps,Qs,nsl,npv,npq] = initialize(BUSDATA,flag_init); 

nbus = length(BUSDATA(:,1));
nsl=find(BUSDATA(:,2) == 1);
npv=find(BUSDATA(:,2) == 2);   
npq=find(BUSDATA(:,2) == 3);
%
% specified powers
   
Ps=BUSDATA(:,3)-BUSDATA(:,5);
Qs=BUSDATA(:,4)-BUSDATA(:,6);

%
% initial values

if flag_init==1
    V=BUSDATA(:,9);
    ANG=BUSDATA(:,10);
elseif flag_init==2
    BUSDATA(npq,9)=BUSDATA(nsl,9);
    BUSDATA([npv;npq],10)=BUSDATA(nsl,10);
    V=BUSDATA(:,9);
    ANG=BUSDATA(:,10);
elseif flag_init==3
    BUSDATA(npq,9)=1;
    BUSDATA([npv;npq],10)=0;
    V=BUSDATA(:,9);
    ANG=BUSDATA(:,10);
else
    disp('Initial values are not specified')
    return,
end,
