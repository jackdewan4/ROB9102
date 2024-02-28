    %% Load and display robot
clear
clc

clear
clc
addpath(genpath(pwd));

% Import the robot description from the URDF file
try
    robot = importrobot('robotisOpenManipulator.urdf');
    
    % Add gravity
    gravityVec = [0 0 -9.80665];
    robot.Gravity = gravityVec;
        
    % Add another massless coordinate frame for the end effector
    eeOffset = 0.12;
    eeBody = robotics.RigidBody('end_effector');
    eeBody.Mass = 0;
    eeBody.Inertia = [0 0 0 0 0 0];
    setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
    addBody(robot,eeBody,'link5');
    
% If this fails, load the preloaded rigid body tree
catch
    warning('Error importing URDF file. Loading presaved object.');
    load openManipulatorDescription robot
end

% Return its home configuration
homeConfig = robot.homeConfiguration;

axes = show(robot, homeConfig, 'Frames', 'off', 'PreservePlot', false);
xlim([-0.4 0.4]), ylim([-0.4 0.4]), zlim([0 0.5])
hold on
axes.CameraPositionMode = 'auto';

% create a set of start, end, and intermediate via points for the trajectory to follow

% Positions (Z X Y)


waypoints = [0 0.2 0.2; 0.2 -0.2 0.02; 0.2 0 0.15; 0.2 0.1 0.4; 0.2 0.2 0.2; 0.2 0.2 0.1]';
% Orientations Euler Angles(Z Y X)

orientations = [0 0 0; 0 pi/2 0; 0 0 0; 0 -pi/2 0; 0 0 0; 0 0 0]';                




% define inverse kinematics for robot
eeName = 'end_effector';
numJoints = numel(robot.homeConfiguration);


ik = robotics.InverseKinematics('RigidBodyTree',robot);
ikWeights = [0.1 0.1 0 1 1 1];
ikInitGuess = robot.homeConfiguration;

plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

% second method
 % Array of waypoint times
 waypointTimes = 0:4:20;

 % Trajectory sample time
 ts = 0.2;


 % Acceleration times 
 waypointAccelTimes = diff(waypointTimes)/4;

% generate trajectory

numTotalPoints = size(waypoints,2);
waypointTime = 16;
for w = 1 : numTotalPoints-1

    R0 = eul2quat(orientations(:,w)');
    Rf = eul2quat(orientations(:,w+1)');
    timeInterval = waypointTimes(w:w+1);
    trajTimes = timeInterval(1):ts:timeInterval(2);


    noPoints = size(waypoints, 2);

    [q, qd, qdd] = trapveltraj(waypoints(:,w:w+1),numel(trajTimes), ...
        'AccelTime',waypointAccelTimes(w), ... 
        'EndTime',diff(waypointTimes(w:w+1)));

    
    
   
    [R, omega, alpha] = rottraj(R0, Rf, [0, waypointTimes(end)], trajTimes); 
    hold on
    plot3(q(1,:),q(2,:),q(3,:),'r-','LineWidth',2);

    % solve for every end effector position

    for idx = 1:numel(trajTimes)
        tform = trvec2tform(q(:,idx)') * quat2tform(R(:,idx)');
        configSoln(idx,:) = ik('end_effector',tform,ikWeights,ikInitGuess);
        ikInitGuess = configSoln(idx,:);
    
        show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
       
        title(['Trajectory at t = ' num2str(trajTimes(idx))])
        drawnow
    end
    hold off

    

    
end


plotTrajectory(trajTimes, q, qd, qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)


