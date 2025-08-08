function [q_mult] = quat_mult(q1,q2)
%QUAT_MULT Multiply two quaternions q1 and q2
%   Detailed explanation goes here
q_mult = [q2(1) * q1(1) - q2(2) * q1(2) - q2(3) * q1(3) - q2(4) * q1(4);
          q2(1) * q1(2) + q2(2) * q1(1) - q2(3) * q1(4) + q2(4) * q1(3);
          q2(1) * q1(3) + q2(3) * q1(1) + q2(2) * q1(4) - q2(4) * q1(2);
          q2(1) * q1(4) - q2(2) * q1(3) + q2(3) * q1(2) + q2(4) * q1(1)];
end
