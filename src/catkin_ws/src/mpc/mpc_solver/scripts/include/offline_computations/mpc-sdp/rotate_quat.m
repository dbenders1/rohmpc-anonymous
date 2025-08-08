function [v_rot] = rotate_quat(q1,v1)
%ROTATE_QUAT Rotate vector v1 by quaternion q1
mult = quat_mult(quat_mult(q1, [0;v1]), [q1(1); -q1(2); -q1(3); -q1(4)]);
v_rot = mult(2:end);
end
