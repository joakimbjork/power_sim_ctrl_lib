% BUILDYBUS.M
function [YBUS] = buildybus(BUSDATA,LINEDATA);

p=LINEDATA(:,2:7);
tap=p(:,6);
yy=1./(p(:,3)+j*p(:,4));
y_y=yy;
yy=tap.*yy;
b=j*p(:,5)/2;
YBUS=sparse([p(:,1);p(:,2);p(:,1);p(:,2);p(:,1);p(:,2);  BUSDATA(:,1);   BUSDATA(:,1)],...
            [p(:,2);p(:,1);p(:,1);p(:,2);p(:,1);p(:,2);  BUSDATA(:,1);   BUSDATA(:,1)],...
            [  -yy ; -yy  ; yy   ;  yy  ;  b   ;  b   ;  BUSDATA(:,7); j*BUSDATA(:,8)]);
         
          
mm=find(tap ~= 1);

if ~isempty(mm),
   
ytap=[p(mm,1) p(mm,2) tap(mm) y_y(mm)];
         
for i=1:length(ytap(:,1));
   
    YBUS(ytap(i,1),ytap(i,1)) = YBUS(ytap(i,1),ytap(i,1)) + ...
                                (ytap(i,3)^2- ytap(i,3))*ytap(i,4);
     
    YBUS(ytap(i,2),ytap(i,2)) = YBUS(ytap(i,2),ytap(i,2)) + ...
                                (1- ytap(i,3))*ytap(i,4);
 end,
            
end,
