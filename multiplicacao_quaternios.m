function [resultado] = multiplicacao_quaternios(q,q2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
resultado = zeros(4,1);
resultado(1) = q(1)*q2(1) - q(2)*q2(2) - q(3)*q2(3) - q(4)*q2(4);
resultado(2) = q(1)*q2(2) + q(2)*q2(1) + q(3)*q2(4) - q(4)*q2(3);
resultado(3) = q(1)*q2(3) - q(2)*q2(4) + q(3)*q2(1) + q(4)*q2(2);
resultado(4) = q(1)*q2(4) + q(2)*q2(3) - q(3)*q2(2) + q(4)*q2(1);
end

