function [W_bar,W_top] = compute_W_bar(do_print,model,w,tol)
  n_bytes = 0;

  % Construct W_bar
  nx = model.nx;
  nw = model.nw;
  [w_min,w_max] = model.get_wminmax();
  w_min_max = [w_min,w_max];
  w_abs_max = max(abs(w_min_max),[],2);
  % w = sqrt(sum(w_abs_max.^2)*nw);
  % w = 0.1;
  W_diag = zeros(nw,1);
  for i=1:nw
    W_diag(i) = w_abs_max(i)^2*nw/w;
  end
  W_top = blkdiag(tol*eye(nx-nw,nx-nw),diag(W_diag));
  W_bar = blkdiag(W_top,w);
  if do_print
    fprintf("[compute_W_bar] Constructed W_bar\n");
  end

  % Check validity of W_bar
  [wgrid, n_wgrid] = model.get_wgrid();
  E = model.get_E();
  n_lmi = n_wgrid;
  i = 0;
  for j=1:n_wgrid
    i = i + 1;
    w = wgrid(:, j);
    ineq = [zeros(nx,nx),E*w;
            (E*w)',zeros(1,1)] - W_bar;

    max_eig(i) = max(eig(ineq));
    if (max_eig(i) > tol)
      result = [result;[i,max_eig(i)]];
      if do_print
        fprintf("[compute_W_bar] Maximum eigenvalue of W_bar-epsilon-RPI LMI %i: %f > tol: %f!\n",i,max_eig(i),tol);
      end
    end
    if do_print && rem(i,200) == 0
      fprintf(repmat('\b',1,n_bytes));
      n_bytes = fprintf("[compute_W_bar] Checked W_bar-epsilon-RPI LMI %i/%i\n",i,n_lmi);
    end
  end
  if do_print
    fprintf(repmat('\b',1,n_bytes));
    fprintf("[compute_W_bar] Checked W_bar-epsilon-RPI LMI %i/%i\n",i,n_lmi);
  end
end
