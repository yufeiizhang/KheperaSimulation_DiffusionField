function [ rot ] = fun_rotRegulator( rot )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


while rot > 180
    rot = rot - 360;
end
while rot <= -180
    rot = rot + 360;
end


end

