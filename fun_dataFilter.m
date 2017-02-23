function [ re ] = fun_dataFilter( ffactor , preSeries , reading )
% A simple filter for unstable sensor and timer 
% reading in the real experiments
% input: ffactor - define in Scr_para, a global parameter
% input: preSeries - input reading series at time t-1 and t-2 (or more)
% input: reading - sensor reading now
% output: re - output after filtering
% input should include background information and initial offset!

% Get length of preSeries
seriesSize = size(preSeries);
seriesLen = seriesSize(2);
% Get sensor readings
reading_1 = preSeries( seriesLen );
reading_2 = preSeries( seriesLen-1 );
reading_0 = reading;
% Calculate
re = ...
	ffactor(1) * ( reading_2 ) + ...
	ffactor(2) * ( reading_1 ) + ...
	ffactor(3) * ( reading_0 ) + ...
		0;