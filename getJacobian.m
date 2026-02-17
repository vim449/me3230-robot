clear; clc
R = (139.105+19.583)/1000;
rw = 48/1000;

J_class = 1/rw * [0 1 R; sqrt(3)/2 1/2 -R; sqrt(3)/2 -1/2 R];
J = J_class; J(:, 1) = -J(:, 1); 
J(2:3, :) = -J(2:3, :);
J