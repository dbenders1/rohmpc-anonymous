function R = get_rot_matrix_rates(phi,theta)
  R = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
       0, cos(phi),            -sin(phi);
       0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
end
