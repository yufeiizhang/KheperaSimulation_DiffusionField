function [ fArc , bArc ] = fun_fakeObsSensor( obsImg , locAgent , senRange , senAcc , senNum )
% input is the location of obstacle shown in obsImg
% obsImg should be an image of an x-y plane, and the obstacles should be
% draw in black, and white(255,255,255) is the backgroud
% fieldgrid should be like a matrix which size is, for example, 100,100
% senRange and senAcc is about the Fake sensor.
% senRange is the detect range of the sensor (m, should be an integer)
% senAcc, is the accuracy of the sensor (m)
% senNum is the number of sensor on ONE SIDE

senRange = senRange * 100;% from meter to centimeter
senAcc = senAcc * 100;% same

% maximum distance
dMax = senRange * 2;

% define field grid instep load from outside
fieldgrid=zeros(41,41);% accuracy as 1 centimeter
% map the agents
%locAgent = floor(locAgent .*100);% only keep centimeter here is enough

% resize the image
rObsImg = imresize( obsImg , size( fieldgrid ) );
% binaryzation, obs as 1
bObsImg = rgb2gray( rObsImg );
bObsImg( bObsImg<255 ) = 1;
bObsImg( bObsImg>1 ) = 0;
bObsImg = im2double( bObsImg );
bObsImg( bObsImg>0 ) = 1;

% prepare for sensitivity, generate a group of step
senStep = 0:senAcc:senRange;
senStep = senStep ./ 100;
[~,len] = size(senStep);

% prepare a map
xg = -2:0.1:2;% also 1 cm
yg = -2:0.1:2;
[ xm , ym ] = meshgrid( xg , yg );
xm0 = xm; 
ym0 = ym;
xm = xm - locAgent(1);
%ym = -ym;
ym = ym - locAgent(2);
tm = atan(xm./ym);
dm = sqrt(xm.^2+ym.^2);

% distant map
fieldgridDist = fieldgrid;
for index = 1 : len-1
    fieldgridDist( and( dm>senStep(index) , dm<=senStep(index+1) ) ) = senStep(index+1);
end
% mark every obstacles
obsDist = fieldgridDist .* bObsImg;
% set 0 as maximum distance
obsDist( ~obsDist ) = dMax;
% calculate sensor angular range
dAng = pi / ( senNum );
mAng = -pi/2:dAng:pi/2; % record all angles
% mark every sensor
fieldgridAng = fieldgrid;
for index = 1 : senNum
    fieldgridAng( and( tm>mAng(index) , tm<=mAng(index+1)) ) = index;
end
% generate the result
fArc = zeros( 1 , senNum );
bArc = zeros( 1 , senNum );
% for forward case, y>0 direction
for index = 1 : senNum
    fArc( index ) = min(min( obsDist(  and( ym>=0 , fieldgridAng==index ) ) ));
end
% for backward case, y<0 direction
for index =1 : senNum
    bArc( index ) = min(min(  obsDist(  and( ym<=0 , fieldgridAng==index ) ) ));
end

end

