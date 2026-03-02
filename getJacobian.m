clear; clc
R = (139.105+19.583)/1000; % wheel hub distance in meters
rw = 48/1000; % Wheel radius in meters

J_class = 1/rw * [0 1 R; sqrt(3)/2 1/2 -R; sqrt(3)/2 -1/2 R];
J = J_class; J(:, 1) = -J(:, 1); 
J(2:3, :) = -J(2:3, :);
J; % J maps from units of meters per second or rad/s to rad/s of motors

% need to map motors from rad/s to -400 to 400
% will figure out ideal ratio later, right now just using a prescaling
% factor of 10
J  = J * 10;

%rotation speeds need to increase relative to linear speeds
J(:, 3) = J(:,3)*10;


% -1.586 x is the amount to move center of rotation to back motor