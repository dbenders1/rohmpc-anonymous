function [data,timing] = compute_w_o_bar_c(do_print,model,L,epsilon,P_delta)
  timer = tic;
  n_bytes = 0;

  % Obtain model variables
  [n_uxABXYgrid, ugrid, xgrid] = model.get_uxABXYgrid(do_print,true,0);
  C = model.get_C();
  [etagrid, n_etagrid] = model.get_etagrid();
  F = model.get_F();

  % Compute w_bar_c
  w_bar_c = 0;
  for i = 1:n_uxABXYgrid
    u = ugrid(:,i);
    x = xgrid(:,i);
    P_delta_xu = full(P_delta(x,u));
    LC_P = norm(sqrtm(P_delta_xu)*L*C/sqrtm(P_delta_xu));
    for j=1:n_etagrid
      eta = etagrid(:, j);
      w_bar_c = max(w_bar_c,sqrt((L*F*eta)'*P_delta_xu*(L*F*eta))+LC_P*epsilon);
    end
    if do_print && rem(i,10) == 0
      fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_w_o_bar_c] Iteration %i/%i\n",i,n_uxABXYgrid);
    end
    if i >= 1
      break;
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_w_o_bar_c] Iteration %i/%i\n",i,n_uxABXYgrid);
  end

  % Save relevant data to struct
  data.w_bar_c = w_bar_c;

  % Stop timing
  timing.t_wbarc_total = toc(timer);
  if do_print
    fprintf("[compute_w_o_bar_c] Total time: %f\n\n",timing.t_wbarc_total);
  end
end
