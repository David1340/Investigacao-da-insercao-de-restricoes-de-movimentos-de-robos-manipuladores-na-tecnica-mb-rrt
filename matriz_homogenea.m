function [ T ] = matriz_homogenea( d,a,alpha,theta)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
T = zeros(4,4);
T(1,:) = [cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)];
T(2,:) = [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)];
T(3,:) = [0,sin(alpha),cos(alpha),d];
T(4,:) = [0,0,0,1];

end

