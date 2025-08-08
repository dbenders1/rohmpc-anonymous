function [beta] = compute_beta(do_print,Q,R,rho_c,P_delta,K_delta,ugrid,xgrid,n_uxABXYgrid)
  n_bytes = 0;

  for i = 1:n_uxABXYgrid
    u = ugrid(:,i);
    x = xgrid(:,i);
    K_delta_xu = full(K_delta(x,u));
    P_delta_xu = full(P_delta(x,u));
    % beta = max(eig(Q+K_delta_xu'*R*K_delta_xu))/((2*rho)*min(eig(P_delta_xu)))
    beta = max(eig(Q+K_delta_xu'*R*K_delta_xu,P_delta_xu))/(2*rho_c);
    if do_print && rem(i,10) == 0
      fprintf(repmat('\b',1,n_bytes));
      n_bytes = fprintf("[compute_beta] Iteration %i/%i\n",i,n_uxABXYgrid);
    end
  end

  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_beta] Iteration %i/%i\n",i,n_uxABXYgrid);
  end
end
