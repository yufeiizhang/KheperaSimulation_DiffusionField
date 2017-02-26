function [ bObsImg ] = fun_img2obs( obsImg )
% generate obs matrix from an image.
% if=1 obs exist in the grid point, if =0 okay for robot system

fieldgrid=zeros(41,41);
% resize the image
rObsImg = imresize( obsImg , size( fieldgrid ) );
% binaryzation, obs as 1
bObsImg = rgb2gray( rObsImg );
bObsImg( bObsImg<255 ) = 1;
bObsImg( bObsImg>1 ) = 0;
bObsImg = im2double( bObsImg );
bObsImg( bObsImg>0 ) = 1;

end

