clear; clc
R = (139.105+19.583)/1000;
rw = 48/1000;

J_class = 1/rw * [0 1 R; sqrt(3)/2 1/2 -R; sqrt(3)/2 -1/2 R];
J = J_class; J(:, 1) = -J(:, 1); 
J(2:3, :) = -J(2:3, :);
J

% log(v) = a*log(d)+b;
a = -0.9912; b = 7.813;
dist = @(v) exp((log(v)-b)/a);