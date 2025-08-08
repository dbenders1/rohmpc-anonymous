function [diagnostics,t_opt] = optimize_wrapper(do_print,con,obj,ops)
  timer_opt = tic;
  if do_print
      fprintf("[optimize_wrapper] Optimize");
  end
  diagnostics = optimize(con,obj,ops);
  t_opt = toc(timer_opt);
  if (ops.solver ~= "sdpt3")
    obj = value(obj);
  end
  if do_print
    print_diagnostics(diagnostics);
    if (ops.solver ~= "sdpt3")
      fprintf("[optimize_wrapper] Objective value: %f\n",obj);
    end
    fprintf("[optimize_wrapper] Optimization time: %f\n\n",t_opt);
  end
end
