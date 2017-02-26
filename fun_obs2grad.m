function [ grad ] = fun_obs2grad( fArc , bArc , effectlen , para )
% surrounding obstracle distance to gradient direction
% obs can be seen as a circle describe by 12 arcs (15degree/arc)
% 12 arc is only a demo, in practice, we need more fractions
% two vectors: front and back
% fArc = [ -90~-60 , -60~-30 , -30~0 , 0~30 , 30~60 , 60~90 ]
% from -x to x via y direction
% bArc = [ -90 ~ 60 , ... same , 60 ~ 90 ]
% from x to -x via -y direction
% and the number in this vector is the nearest point in the arc
% effectlen is the distance where repulse force exist
% para is a parameter
[ ~ , len ] = size(fArc);% should be 6
%force = para .* ( 1 ./ fArc - 1 ./ detect ) .* ( - 1 ) ./ ( fArc .^ 2 );
% calculate repulse force for each arc
fforce  = -1 .* log( fArc ./ effectlen );
fforce( fforce < 0 ) = 0;
bforce = -1 .* log( bArc ./ effectlen );
bforce( bforce < 0 ) = 0;
% normalize the force by input number
fforce = fforce ./ len;
bforce = bforce ./ len;
% calculate central angle for each arc
dAng = pi / ( 2 * len );
mAng = []; % record all angles
for index = 1 : len
    mAng( index ) = -pi/2 + ( 2*index-1 ) * dAng;
end
% generate a vector in x-y plane from gradient
fVec( 1 , : ) = -sin( mAng );% x axis
fVec( 2 , : ) = -cos( mAng );% y axis
% calculate gradient of the front
fgrad( 1 ) = sum( fVec( 1 , : ) .* fforce );
fgrad( 2 ) = sum( fVec( 2 , : ) .* fforce );
% for backward
bVec( 1 , : ) = sin( mAng );
bVec( 2 , : ) = cos( mAng );
bgrad( 1 ) = sum( bVec( 1 , : ) .* bforce );
bgrad( 2 ) = sum( bVec( 2 , : ) .* bforce );
% sum
grad = [ fgrad( 1 ) + bgrad( 1 ) ; fgrad( 2 ) + bgrad( 2 ) ];
grad = grad .* para;
end

