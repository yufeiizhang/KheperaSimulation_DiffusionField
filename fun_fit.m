function [re] = fun_fit( frame , FieldValue , MeanValue )
% FieldValue = [values_new_1;values_new_2;values_new_3;values_new_4;values_new_5;values_new_6;values_new_7;values_new_8];
% MeanValue = [bg_values_1;bg_values_2;bg_values_3;bg_values_4;bg_values_5;bg_values_6;bg_values_7;bg_values_8];
MeanValue = mean(MeanValue,2);
TrueValue = FieldValue(:,frame)-MeanValue;% true value of the frame
TrueValue(TrueValue<0)=0;
% sensor loc
X = [ -1.7830 , -1.3675 , -0.8213 , -1.3155 , -0.3937 , -0.6332 ,  0.7754 ,  0.4264 ,  0.0907 , -0.0148 , -0.0155 , -0.1612 ,  1.0675 ,  0.6497 ,  0.1942 ,  1.2309 ,  0.7974 ,  0.3256 , -1.6218 , -1.1848 , -0.7314 , -0.4053 , -0.3243 , -0.2984 ]';
Y = [ -1.1141 , -0.7923 , -0.3959 ,  1.6759 ,   1.0858 ,  0.5902 , -1.5882 , -1.0466 , -0.5119 ,  1.2643 ,  0.8393 ,  0.4642 , -0.2736 , -0.1809 , -0.1060 ,  1.1787 ,  0.8284 ,  0.4384 ,  0.2628 ,  0.1705 ,  0.0792 , -1.3225 , -0.8878 , -0.4536 ]';
ft = 'thinplateinterp';
sample = TrueValue;
[xData, yData, zData] = prepareSurfaceData( X, Y, sample );
[re, ] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );
end