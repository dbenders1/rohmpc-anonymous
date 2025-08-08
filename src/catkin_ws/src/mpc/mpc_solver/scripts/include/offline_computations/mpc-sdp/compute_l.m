function [l] = compute_l(L)
  l = max(eig(L));
end