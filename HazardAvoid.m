% rigid without any rotation here.
% same structure as new.m
% change obstacle avoidance as hazard avoidance
% we assume the field is harzard to our robots, and set a threhold value
% as a obstacle for robot system

% clear workspace
clear

%% Load Data
% Load Data of previous CO2 field
load('fieldData.mat');
% Load trajectory
% Load Background Obstacle
load('obs00.mat');
% flip
obs00f = flip(obs00,1);% nothing happened :D
% to obs
obs00b = fun_img2obs( obs00f );
% if with obs
load('obs01.mat');
obs01f = flip(obs01,1);
obs01b = fun_img2obs( obs01f );
obs00b = or( obs00b, obs01b );

%% Claim Parameters
% Number of Agent in Simulation
AgentNumber = 4;
% Field concentration offset for simulation
concOffset =100;
% DataSet length for one agent
dataLen = 12;
% Initial dt setting for simulation for field estimation
dt = 0.325;
% Estimation threshold (minimum value)
estThreshold = 0.05;
% Factor for filter to deal with data from sensor
ffactor =[0.1, 0.3, 0.6];
% filterOut length for one iteration
filOLen = 7;
% gradient coefficient for formation center movenment
gradCoe = 0.5;
% hazard limit
hazardLim = 3000;
% Initial Background concentraion for CO2 field in Simulation
iniBG = 0;
% Initial Estimation Value
iniEst = 0.6;
% Initial Location for Simulation
initLocC = ones(AgentNumber,1)*[ 0 , -0.5 , 0 ];
initLoc = [ 0 , 0.2 , 0 ; 0.2 , 0 , 0 ;  0 , -0.2 , 0 ; -0.2 , 0 , 0 ] + initLocC;
% maximum loop number
loopNumMax = 300;
% obs repulse force parameter
obsRepulse = 0.03;
% fake obs sensor range
SensingRange = 1.5;%(1.5m)
% fake obs sensor accuracy
SensorAcc = 0.1;
% Set Background Concentraion of Simulation
sensorBG = [0,0,0,0];
% the frame that CO2 field become stable in fieldData, skip in simulation
skipInitField = 120;
% speed limit for robot in simulation
splim = 0.02;


%% Initialize the Simulation
% Calculate the formation for robot system, generate initDist
initCenter = mean( initLoc( : , 1:2 ) );
for agent = 1 : AgentNumber
    initDist( agent , : ) = initLoc( agent , 1:2 ) - initCenter;
end

%% Initialize Counter 1
counter = 1;
% Fit sensor grid
FieldValue = [values_new_1;values_new_2;values_new_3;values_new_4;values_new_5;values_new_6;values_new_7;values_new_8];
MeanValue = [bg_values_1;bg_values_2;bg_values_3;bg_values_4;bg_values_5;bg_values_6;bg_values_7;bg_values_8];
frame = skipInitField-2;
field = fun_fit( frame , FieldValue , MeanValue );
% Init DataSet.
DataSet( counter , 1 : dataLen*agent ) = ...
    zeros( 1 ,  AgentNumber * dataLen );
% Feed data into DataSet
for agent = 1 : AgentNumber
    % Feed Location
    DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+3 )  = initLoc( agent , : );
    % Feed Sensor Reading
    DataSet( counter , dataLen*(agent-1)+4 ) = ...
        field(DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 ))+concOffset;
    % Feed Filter Result
    DataSet( counter , dataLen*(agent-1)+5 ) = ...
        DataSet( counter , dataLen*(agent-1)+4 );% No filter here
    % Feed Target Location
    DataSet( counter , dataLen*(agent-1)+6:dataLen*(agent-1)+7 ) = ...
        DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 );
end
%% Initialize Counter 2
counter = 2;
frame = skipInitField-1;
field = fun_fit( frame , FieldValue , MeanValue );
% Init DataSet.
DataSet( counter , 1 : dataLen*agent ) = ...
    zeros( 1 ,  AgentNumber * dataLen );
for agent = 1 : AgentNumber
    % Feed Location
    DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+3 )  = initLoc( agent , : );
    % Feed Sensor Reading
    DataSet( counter , dataLen*(agent-1)+4 ) = ...
        field(DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 ))+concOffset;
    % Feed Filter Result
    DataSet( counter , dataLen*(agent-1)+5 ) = ...
        DataSet( counter , dataLen*(agent-1)+4 );% No filter here
    % Feed Target Location
    DataSet( counter , dataLen*(agent-1)+6:dataLen*(agent-1)+7 ) = ...
        DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 );
end

%% Init filterOut for Estimation
filterOut = zeros( 2 , filOLen );
for agent = 1 : AgentNumber
    meanx(agent) = DataSet( counter , dataLen*(agent-1)+1 );
    meany(agent) = DataSet( counter , dataLen*(agent-1)+2 );
end
cTarx = mean(meanx);
cTary = mean(meany);
filterOut(1,:) = [ cTarx , cTary , 0 , 0 , 0 , iniEst , 0 ];
filterOut(2,:) = [ cTarx , cTary , 0 , 0 , 0 , iniEst , 0 ];
clear cTarx cTary meanx meany

%% Init states(s,r) and kalState
% no idea of how to rewrite this part...
for agent = 1 : AgentNumber
    centerreading1(agent) = DataSet( 1 , dataLen*(agent-1)+5 );
    centerreading2(agent) = DataSet( 2 , dataLen*(agent-1)+5 );
end
R = mean( centerreading2 );
Rk = mean( centerreading1 );
px = DataSet( 2 , dataLen*(1-1)+5 ) - DataSet( 2 , dataLen*(3-1)+5 );%px = 1-3
py = DataSet( 2 , dataLen*(2-1)+5 ) - DataSet( 2 , dataLen*(4-1)+5 );%py = 2-4
s.x = [ R ; px ; py ; iniEst ; Rk ; px ; py ];
s.P = 0;
s.H=0.01*[1 1 1 1 1 1 1 1]';
clear R Rk px py centerreading1 centerreading2
kalState(1).s = s;
kalState(2).s = s;
r.x = iniEst;
r.P = 4;
kalState(1).r = r;
kalState(2).r = r;

%% Figure
% mesh for field
xmesh = [-2:0.1:2];
ymesh = [-2:0.1:2];
[xm,ym] = meshgrid(xmesh,ymesh);
fm = field(xm,ym);
contour(xm,ym,fm)
hold on
grid on
% scatter robot location
for agent = 1 : AgentNumber
    % khepera position
    x0(agent) = DataSet( counter , dataLen*(agent-1)+1 );
    y0(agent) = DataSet( counter , dataLen*(agent-1)+2 );
    % sensor reading
    co2 = DataSet( counter , dataLen*(agent-1)+5 );
    %text( x0 , y0 , num2str(co2) );
end
x0(AgentNumber+1) = x0( 1 );
y0(AgentNumber+1) = y0( 1 );
plot( x0 , y0 , '-' );
drawnow;


%% Begin Iteration
while(1)
    % update counter
    counter = counter + 1;
    %% prepare
    % prepare an empty DataSet
    DataSet( counter, : ) = zeros(1, dataLen*AgentNumber);
    % fit CO2 field
    frame = skipInitField + counter - 3 ;
    field = fun_fit( frame , FieldValue , MeanValue );
    % feed new location of robot system (movement in simulation)
    for agent  = 1 : AgentNumber
        DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 ) = ...
            DataSet( counter-1 , dataLen*(agent-1)+6:dataLen*(agent-1)+7 );
        % no more rotation in simulation
        DataSet( counter , dataLen*(agent-1)+3 ) = 0;
    end
    % feed concentration to DataSet
    for agent = 1 : AgentNumber
        % Prepare sensor noise
        sensorNoise = 0;
        % Get result from fitting results
        DataSet( counter , dataLen*(agent-1)+4 ) = ...
            field( DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )) + concOffset + sensorNoise;
    end
    % feed the filter result
    for agent = 1 : AgentNumber
        % read previous concentration
        reading_0 = DataSet( counter-0 , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG;
        reading_1 = DataSet( counter-1 , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG;
        reading_2 = DataSet( counter-2 , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG;
        % feed to DataSet
        DataSet( counter , dataLen*(agent-1)+5 ) = ...
            fun_dataFilter( ffactor , [ reading_2 , reading_1 ], reading_0 );
    end
    % generate hazard region
    xmesh = [-2:0.1:2];
    ymesh = [-2:0.1:2];
    [xh,yh] = meshgrid(xmesh,ymesh);
    zh = field( xh , yh );
    zh( zh<hazardLim ) = 0;
    zh( zh>=hazardLim ) =1;
    % merge with obs
    obs02b = or( obs00b, zh );
    clear zh
    %% run algorithm
    % prepare data (empty vector)
    values = zeros( 2 , AgentNumber );
    rn = zeros( 2 , AgentNumber );
    rk = zeros( 2 , AgentNumber );
    % load data from DataSet
    for agent = 1 : AgentNumber
        % sensor reading for this frame
        values( 1 , agent ) = DataSet( counter-0 , dataLen*(agent-1)+5 );
        % sensor reading for last frame
        values( 2 , agent ) = DataSet( counter-1 , dataLen*(agent-1)+5 );
        % location of agent this frame
        rn( : , agent ) = DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
        % location of agent last frame
        rk( : , agent ) = DataSet( counter-1 , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
    end
    rc = mean( rn , 2 );
    rck = mean( rk , 2 );
    % measure average distance between agent and center
    drc = zeros(2,4);
    for agent = 1 : AgentNumber
        drc(:,agent) = rn(:,agent) - rc;
        drc(1,agent) = drc(1,agent)^2;
        drc(2,agent) = drc(2,agent)^2;
    end
    aveDist = mean(sum(drc));
    aveDist = sqrt(aveDist);
    % Estimate Laplacian ( Last Frame )
    %l1 = ( 1 / (16*aveDist ) ) *...
    l1 = ( 1 / (sqrt(2)*aveDist ) ) *...
        ( ( sum( values(2,:) ) ) - AgentNumber * s.x( 5 , 1 ) );
    % Call cooperative Kalman filter
    values2 = [ values( 1 , : )' ; values( 2 , : )' ];
    s = fun_kalmanf3...
        ( s , values2 , rn(:,1) , rn(:,2) , rn(:,3) , rn(:,4) , ...
        rc , rk(:,1) , rk(:,2) , rk(:,3) , rk(:,4) , rck , l1 , dt);
    % Estimate Laplacian Again ( This Frame )
    l2 = ( 1 / (aveDist ) ) *...
        ( ( sum( values(1,:) ) ) - AgentNumber * s.x( 5 , 1 ) );
    % Estimate Error in the Center of Agent Group
    error = s.x(1,1) - s.x(5,1);
    errort = error / dt;
    % Center Gradient
    grad = [ s.x(2,1) ; s.x(3,1) ] / norm([ s.x(2,1) , s.x(3,1) ]);
    % Move the center of robot formation
    %rc = rc + gradCoe * dt*grad;
    % speed control here
    dc = gradCoe * dt * grad;
    v = norm(dc);
    vcoe = 1;% coefient for velocity
    if v > splim
        vcoe = splim / v;
    end
    dc = dc .* vcoe;% update speed
    rc = rc + dc;% back to rc
    %% simulate the obstacles
    RobotCenter = mean( rn , 2 );
    %SensingRange = 1.5;%(1.5m)
    %SensorAcc = 0.1;
    % generate obs sensor result
    [fArc,bArc]  = fun_fakeObsSensor( obs02b , RobotCenter , SensingRange , SensorAcc , 6 );
    % calculate gradient for group of robot
    obst = fun_obs2grad(fArc,bArc,SensingRange , obsRepulse );
    % update rc
    rc = rc + obst;
    %grad = fun_potential( grad , obst );
    %% Estimation and save the result
    % Parameter Estimation
    r = fun_RLS1 (r,errort,l2);
    est = r.x;
    % record Estimation
    s.x(4,1) = est;
    % Collect Data to DataSet
    filterOut( counter , : ) = zeros( 1 , filOLen );
    filterOut( counter , : ) = [ rc' , grad' , error , est , l2 ];
    clear rc rck rn rk  values grad error l est
    %% For individual robot (skip the instruction and monocycle model part)
    % Formation Control for Khepera Robot
    [r1,r2,r3,r4] = fun_jacobi( initDist(1,:)' , initDist(2,:)' , ...
        initDist(3,:)' , initDist(4,:)' , filterOut( counter , 1:2 )' );
    % record robot position
    DataSet( counter , dataLen*(1-1)+6:dataLen*(1-1)+7 ) = r1';
    DataSet( counter , dataLen*(2-1)+6:dataLen*(2-1)+7 ) = r2';
    DataSet( counter , dataLen*(3-1)+6:dataLen*(3-1)+7 ) = r3';
    DataSet( counter , dataLen*(4-1)+6:dataLen*(4-1)+7 ) = r4';
    clear r1 r2 r3 r4
    %% Figure
    clf
    % mesh for field
    xmesh = [-2:0.1:2];
    ymesh = [-2:0.1:2];
    [xm,ym] = meshgrid(xmesh,ymesh);
    fm = field(xm,ym);
    contour(xm,ym,fm)
    hold on
    grid on
    % scatter robot location
    for agent = 1 : AgentNumber
        % khepera position
        x0(agent) = DataSet( counter , dataLen*(agent-1)+1 );
        y0(agent) = DataSet( counter , dataLen*(agent-1)+2 );
        % sensor reading
        co2 = DataSet( counter , dataLen*(agent-1)+5 );
        %text( x0 , y0 , num2str(co2) );
    end
    x0(AgentNumber+1) = x0( 1 );
    y0(AgentNumber+1) = y0( 1 );
    plot( x0 , y0 , '-' );
    % draw obstacles
    contour(xm,ym,obs02b);
    drawnow;
    %% Break Condition
    if counter >= ( loopNumMax + 2 )
        break;
    end
end
hold off;
%% Clean
% nothing here


