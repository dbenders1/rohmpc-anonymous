function result = compute_check_observer_minnorm_LMIs(do_print,do_check,model,P_delta_xu,L,L_norm_sq,tol)
  result = zeros(0,2);
  n_bytes = 0;
  n_lmi = 1;

  % Get model variables
  C = model.get_C();

  % Initialize result with no LMIs being violated, fill below if violated
  if do_check
    result = zeros(0,2);
  end

  i = 0;
  for j = 1:n_lmi
    i = i + 1;
    ineq = [P_delta_xu,P_delta_xu*L*C;(P_delta_xu*L*C)',P_delta_xu*L_norm_sq];
  
    if ~do_check
      result = [result;ineq>=0];
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_observer_minnorm_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
      end
    else
      min_eig(i) = min(eig(ineq));
      if (min_eig(i) < -tol)
        result = [result;[i,min_eig(i)]];
        if do_print
          fprintf("[compute_check_observer_minnorm_LMIs] Minimum eigenvalue of LMI %i: %f < -tol: %f!\n",i,min_eig(i),tol);
        end
      end
      if do_print && rem(i,200) == 0
        fprintf(repmat('\b',1,n_bytes));
        n_bytes = fprintf("[compute_check_observer_minnorm_LMIs] Checked LMI %i/%i\n",i,n_lmi);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_observer_minnorm_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_observer_minnorm_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
