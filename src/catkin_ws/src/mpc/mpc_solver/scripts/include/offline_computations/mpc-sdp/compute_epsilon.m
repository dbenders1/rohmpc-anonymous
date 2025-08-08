function [data,timing] = compute_epsilon(do_print,model,P_delta,L)
  timer = tic;
  n_bytes = 0;

  % Obtain grid points
  [n_uxABXYgrid, ugrid, xgrid] = model.get_uxABXYgrid(do_print,true,0);

  % Obtain wgrid
  [wgrid, n_wgrid] = model.get_wgrid();
  E = model.get_E();

  % Obtain etagrid
  [etagrid, n_etagrid] = model.get_etagrid();

  % Obtain F
  F = model.get_F();

  % Compute epsilon
  n_uxABXYgrid = 1;
  n_iter = n_uxABXYgrid*n_wgrid*n_etagrid;
  epsilon_sq = 0;
  i = 0;
  for j = 1:n_uxABXYgrid
    u = ugrid(:,j);
    x = xgrid(:,j);
    P_delta_xu = full(P_delta(x,u));
    for k=1:n_wgrid
      w = wgrid(:,k);
      for l=1:n_etagrid
        eta = etagrid(:,l);
        i = i + 1;
        epsilon_sq = max(epsilon_sq,(E*w+L*F*eta)'*P_delta_xu*(E*w+L*F*eta));
        if do_print && rem(i,200) == 0
          fprintf(repmat('\b',1,n_bytes));
            n_bytes = fprintf("[compute_epsilon] Iteration %i/%i\n",i,n_iter);
        end
      end
    end
    if j >= 1
      break;
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_epsilon] Iteration %i/%i\n",i,n_iter);
  end
  epsilon = sqrt(epsilon_sq);

  % Save relevant data to struct
  data.epsilon = epsilon;

  % Stop timing
  timing.t_epsilon_total = toc(timer);
  if do_print
    fprintf("[compute_epsilon] Total time: %f\n\n",timing.t_epsilon_total);
  end
end
