function result = compute_check_P_LMIs(do_print,do_check,P,tol)
  result = zeros(0,2);
  n_bytes = 0;
  nx = size(P,1);
  n_lmi = 1;

  i = 1;
  ineq = P - tol*eye(nx);

  if ~do_check
    result = [result;ineq>=0];
  else
    min_eig(i) = min(eig(ineq));
    if (min_eig(i) < -tol)
      result = [result;[i,min_eig(i)]];
      if do_print
        fprintf("[compute_check_P_LMIs] Minimum eigenvalue of LMI %i: %f < -tol: %f!\n",i,min_eig(i),tol);
      end
    end
  end

  if ~do_check
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_P_LMIs] Constructed LMI %i/%i\n",i,n_lmi);
    end
  else
    if do_print
      fprintf(repmat('\b',1,n_bytes));
      fprintf("[compute_check_P_LMIs] Checked LMI %i/%i\n",i,n_lmi);
    end
  end
end
