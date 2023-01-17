function [ d] = distancias(A,B)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
d = zeros(1,size(B,2));
for i = 1:length(d)
  d(i) = sqrt(sum((A - B(:,i)).^2));
end

end

