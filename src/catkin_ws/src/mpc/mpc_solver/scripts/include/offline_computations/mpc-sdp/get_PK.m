function [P, K] = get_PK(X,Y)
  P = inv(X);
  K = Y/X;
end
