function [ resultNum ] = fun_SensorReader( mbed, prev_sensorCal )

flushPause = 0.05;
counter = 1;
flg = 0;
counterMax = 10;

if mbed.BytesAvailable
    fread(mbed, mbed.BytesAvailable);
end
readasync(mbed,32);
fgetl(mbed);
feed = fgetl(mbed);
if isempty(feed)
    resultNum = prev_sensorCal;
else
    if flg == 1
        feed = '0';
    end
    resultNum = str2num(feed);
end
% resultNum = feed;
if resultNum > 10000
    resultNum = prev_sensorCal;
end

end