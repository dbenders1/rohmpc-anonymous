function [data,timing] = compute_w_bar_c(do_print,model,P_delta)
  timer = tic;
  n_bytes = 0;

  % Obtain grid points
  [n_uxABXYgrid, ugrid, xgrid] = model.get_uxABXYgrid(do_print,true,0);

  % Obtain wgrid
  [wgrid, n_wgrid] = model.get_wgrid();
  E = model.get_E();

  % Compute w_bar_c
  w_bar_c_sq = 0;
  for i = 1:n_uxABXYgrid
    u = ugrid(:,i);
    x = xgrid(:,i);
    P_delta_xu = full(P_delta(x,u));
    for j=1:n_wgrid
      w = wgrid(:, j);
      w_bar_c_sq = max(w_bar_c_sq,(E*w)'*P_delta_xu*(E*w));
    end
    if do_print && rem(i,10) == 0
      fprintf(repmat('\b',1,n_bytes));
      n_bytes = fprintf("[compute_w_bar_c] Iteration %i/%i\n",i,n_uxABXYgrid);
    end
    if i >= 1
      break;
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_w_bar_c] Iteration %i/%i\n",i,n_uxABXYgrid);
  end
  w_bar_c = sqrt(w_bar_c_sq);

  % Save relevant data to struct
  data.w_bar_c = w_bar_c;

  % Stop timing
  timing.t_wbarc_total = toc(timer);
  if do_print
    fprintf("[compute_w_bar_c] Total time: %f\n\n",timing.t_wbarc_total);
  end
end
