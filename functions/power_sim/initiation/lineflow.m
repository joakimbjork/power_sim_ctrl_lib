

   function [LINFLW,P_LOSS]=lineflow(V,ANG,LINEDATA);
   
   yy=1./(LINEDATA(:,4)+j*LINEDATA(:,5));
   g=real(yy);b=imag(yy);
   k=LINEDATA(:,2);
   m=LINEDATA(:,3);
   b0=LINEDATA(:,6)/2;
   dkm=ANG(k)-ANG(m);
%---transmitted power according to LINEDATA
   

P_pos=V(k).*[(V(k).*g)-V(m).*(g.*cos(dkm)+b.*sin( dkm))];
P_neg=V(m).*[(V(m).*g)-V(k).*(g.*cos(dkm)+b.*sin(-dkm))];

Q_pos=V(k).*[V(k).*(-b0-b)-V(m).*(g.*sin( dkm)-b.*cos(dkm))];
Q_neg=V(m).*[V(m).*(-b0-b)-V(k).*(g.*sin(-dkm)-b.*cos(dkm))];

LINFLW=[P_pos P_neg  Q_pos Q_neg];

P_LOSS=P_pos+P_neg;