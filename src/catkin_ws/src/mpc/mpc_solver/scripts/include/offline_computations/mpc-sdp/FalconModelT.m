classdef FalconModelT
  % SYSTEM Nonlinear system class
  %
  %   Args:
  %     None
  %
  %   Inputs:
  %     t0c  : commanded thrust of motor 0
  %     t1c  : commanded thrust of motor 1
  %     t2c  : commanded thrust of motor 2
  %     t3c  : commanded thrust of motor 3
  %
  %   States:
  %     px    : position along x-axis (world frame)
  %     py    : position along y-axis (world frame)
  %     pz    : position along z-axis (world frame)
  %     phi   : roll angle (using ZYX Euler angles)
  %     theta : pitch angle (using ZYX Euler angles)
  %     psi   : yaw angle (using ZYX Euler angles)
  %     vx    : velocity along x-axis (world frame)
  %     vy    : velocity along y-axis (world frame)
  %     vz    : velocity along z-axis (world frame)
  %     wbx   : angular velocity around x-axis (body frame)
  %     wby   : angular velocity around y-axis (body frame)
  %     wbz   : angular velocity around z-axis (body frame)
  %
  %   System constraints:
  %     L_u : Input constraint weights (L_u * u <= l_u)
  %     l_u : Input constraint bounds (L_u * u <= l_u)
  %     L_x : State constraint weights (L_x * x <= l_x)
  %     l_x : State constraint bounds (L_x * x <= l_x)
  %     L   : Constraint weights (L = [L_u;L_x]
  %     l   : Constraint bounds (l = [l_u;l_x])
  %

  properties (Constant)
    % Dimensions used for obstacle avoidance constraints
    npos = 2;

    % Grid settings
    % n_grid : default number of grid points
    % n_grid_vert : number of grid points for taking vertices
    % To create LMIs
    n_grid_construct = 2;
    n_grid_vertices_construct = 2;
    % To check whether LMIs hold at a finer grid in the input-state space
    n_grid_check = 2;
    n_grid_vertices_check = 2;
    % To grid w and eta values
    n_grid_w = 2;
    n_grid_eta = 2;

    % Define state indices that appear in parameter vector
    % p_xidc = [];
    p_xidc = [4,5,6];

    % Define number of parameterized X and Y matrices
    nX = 1;
    nY = 1;
    % nX = 1+length(FalconModelT.p_xidc);
    % nY = 1+length(FalconModelT.p_xidc);

    % Controller design
    % MPC tuning
    Q_mpc = diag([1,1,1,1,1,1,1,1,1,1,1,1]);
    R_mpc = diag([1,1,1,1]);
  end

  properties
    g
    ts
    nx
    nu
    ny
    mass
    inertia
    B_allocation
    motor_tau
    thrust_map
    kd
    p_min
    p_max
    att_min
    att_max
    v_min
    v_max
    wb_min
    wb_max
    t_hover
    t_min
    t_max
    M
    u_lb
    u_ub
    x_lb
    x_ub
    L_u
    l_u
    L_x
    l_x
    nw
    E
    w_min
    w_max
    w_bias
    C
    neta
    F
    eta_min
    eta_max
  end

  methods(Static)
    function [names] = get_u_x_names()
      names = ["t0c";
               "t1c";
               "t2c";
               "t3c";
               "x";
               "y";
               "z";
               "phi";
               "theta";
               "psi";
               "vx";
               "vy";
               "vz";
               "wbx";
               "wby";
               "wbz"];
    end
  end

  methods
    function obj = FalconModelT(filename,use_w_rel)
      % Set variables
      fprintf("[FalconModelT] Loading model variables from %s.mat in class FalconModelT\n",filename);
      data = load(filename);
      obj.g = double(data.g);
      obj.ts = double(data.ts);
      obj.nx = double(data.nx);
      obj.nu = double(data.nu);
      obj.mass = double(data.mass);
      obj.inertia = double(data.inertia);
      obj.B_allocation = double(data.B_allocation);
      obj.motor_tau = double(data.motor_tau);
      obj.thrust_map = double(data.thrust_map);
      obj.kd = double(data.kd);
      obj.p_min = double(data.p_min);
      obj.p_max = double(data.p_max);
      obj.att_min = double(data.att_min);
      obj.att_max = double(data.att_max);
      obj.v_min = double(data.v_min);
      obj.v_max = double(data.v_max);
      obj.wb_min = double(data.wb_min);
      obj.wb_max = double(data.wb_max);
      obj.t_hover = double(data.t_hover);
      obj.t_min = double(data.t_min);
      obj.t_max = double(data.t_max);
      obj.E = double(data.E);
      obj.nw = size(obj.E,2);
      if use_w_rel
        obj.w_min = double(data.w_min_rel');
        obj.w_max = double(data.w_max_rel');
        obj.w_bias = double(data.w_bias');
      else
        obj.w_min = double(data.w_min_abs');
        obj.w_max = double(data.w_max_abs');
        obj.w_bias = zeros(obj.nw,1);
      end
      obj.C = double(data.C);
      obj.F = double(data.F);
      obj.eta_min = double(data.eta_min_abs_gt');
      obj.eta_max = double(data.eta_max_abs_gt');
      obj.neta = size(obj.F,2);

      % Determine output dimension
      obj.ny = size(obj.C,1);

      % Set position selection matrix
      obj.M = [eye(FalconModelT.npos),zeros(FalconModelT.npos,obj.nx-FalconModelT.npos)];

      % Set system constraints
      obj.u_lb = kron(ones(obj.nu,1),obj.t_min);
      obj.u_ub = kron(ones(obj.nu,1),obj.t_max);
      obj.x_lb = [obj.p_min;
                  obj.p_min;
                  0;
                  obj.att_min;
                  obj.att_min;
                  obj.att_min;
                  obj.v_min;
                  obj.v_min;
                  obj.v_min;
                  obj.wb_min;
                  obj.wb_min;
                  obj.wb_min];
      obj.x_ub = [obj.p_max;
                  obj.p_max;
                  obj.p_max;
                  obj.att_max;
                  obj.att_max;
                  obj.att_max;
                  obj.v_max;
                  obj.v_max;
                  obj.v_max;
                  obj.wb_max;
                  obj.wb_max;
                  obj.wb_max];
      obj.L_u = kron(eye(obj.nu),[-1;1]);
      obj.l_u = [];
      for u_idx = 1:obj.nu
        obj.l_u = [obj.l_u;
                   -obj.u_lb(u_idx);
                   obj.u_ub(u_idx)];
      end
      obj.L_x = kron(eye(obj.nx),[-1;1]);
      obj.l_x = [];
      for x_idx = 1:obj.nx
        obj.l_x = [obj.l_x;
                   -obj.x_lb(x_idx);
                   obj.x_ub(x_idx)];
      end
    end

    % Continous-time model
    function f = fun(obj, x, u)
      % Assign current states
      phi   = x(4);
      theta = x(5);
      psi   = x(6);
      v     = x(7:9);
      wb    = x(10:12);

      % Compute forces and torques
      t = obj.B_allocation(1,:)*u;
      tau = obj.B_allocation(2:4,:)*u;

      % Rotation matrix to convert coordinates from body to inertial frame
      Rc = get_rot_matrix_coordinates(phi,theta,psi);

      % Rotation matrix to convert body rates to Euler angle rates
      Rr = get_rot_matrix_rates(phi,theta);

      % Calculate system update
      f = [v;...
           Rr*wb;
           [0;0;-obj.g]+Rc*([0;0;t/obj.mass]-obj.kd*Rc'*v);...
           obj.inertia\(tau-cross(wb,obj.inertia*wb))] +...
           obj.E * obj.w_bias;
    end

    function C = get_C(obj)
      C = obj.C;
    end

    % Get system constraints in matrix form
    function [L_u,l_u,L_x,l_x,L,l] = get_constraints(obj)
      L_u = obj.L_u;
      l_u = obj.l_u;
      L_x = obj.L_x;
      l_x = obj.l_x;

      % Combined state and input constraints
      L = [L_u,zeros(2*obj.nu,obj.nx);
           zeros(2*obj.nx,obj.nu),L_x];
      l = [l_u;l_x];
    end

    function [E] = get_E(obj)
      E = obj.E;
    end

    function [etagrid, n] = get_etagrid(obj)
      % Generate grid points for each dimension
      eta = cell(1,obj.neta);
      for i = 1:obj.neta
          eta{i} = linspace(obj.eta_min(i),obj.eta_max(i),obj.n_grid_eta); % Adjust the number of points as needed
      end

      % Create the grid
      [eta_grid{1:obj.neta}] = ndgrid(eta{:});

      % Combine the grid into a matrix
      etagrid = cell2mat(cellfun(@(x) x(:),eta_grid,'UniformOutput',false))';

      % Determine number of grid points
      n = size(etagrid,2);
    end

    function [eta_min,eta_max] = get_etaminmax(obj)
      eta_min = obj.eta_min;
      eta_max = obj.eta_max;
    end

    function [F] = get_F(obj)
      F = obj.F;
    end

    % Get casadi derivative functions
    % Note: use full(A_fun(x,u)) to obtain a normally-sized A matrix
    function [f_fun,A_fun,B_fun] = get_fABfun(obj)
      [x_sym,u_sym] = obj.get_uxsim();
      f     = obj.fun(x_sym,u_sym);
      f_fun = casadi.Function('f_fun',{x_sym,u_sym},{f});
      A     = jacobian(f,x_sym);
      B     = jacobian(f,u_sym);
      A_fun = casadi.Function('A_fun',{x_sym,u_sym},{A});
      B_fun = casadi.Function('B_fun',{x_sym,u_sym},{B});
    end

    function [L] = get_L(obj,rho_c)
      % Compute diagonal observer gain based on rho_c
      L = 5*rho_c*eye(obj.nx);
    end

    function [lb] = get_lower_bounds(obj)
      lb = [obj.u_lb;
            obj.x_lb];
    end

    function M = get_M(obj)
      M = obj.M;
    end

    function [np] = get_np(obj)
      p = obj.get_p(zeros(obj.nx,1),zeros(obj.nu,1));
      np = size(p,1);
    end

    function [nu] = get_nu(obj)
      nu = obj.nu;
    end

    function [nx] = get_nx(obj)
      nx = obj.nx;
    end

    function [nX, nY] = get_nXnY(obj)
      nX = obj.nX;
      nY = obj.nY;
    end

    % Use this function to obtain a real value for p
    function [p] = get_p(obj,x,u)
      p = x(obj.p_xidc);
    end
  
    % Use this function to obtain a symbolic value for p and its
    % derivatives
    function [p_fun,p_fun_x,p_fun_u] = get_pfuns(obj)
      [x_sym,u_sym] = obj.get_uxsim();
      p_sym = obj.get_p(x_sym,u_sym);
      if isempty(p_sym)
        p_fun = [];
        p_fun_x = [];
        p_fun_u = [];
      else
        p_fun   = casadi.Function('p_fun',{x_sym,u_sym},{p_sym});
        p_x_sym = jacobian(p_sym,x_sym);
        p_u_sym = jacobian(p_sym,u_sym);
        p_fun_x = casadi.Function('p_fun_x',{x_sym,u_sym},{p_x_sym});
        p_fun_u = casadi.Function('p_fun_u',{x_sym,u_sym},{p_u_sym});
      end
    end

    function [P_fun, K_fun] = get_PKfun(obj,X_arr,Y_arr)
      [x_sym,u_sym] = obj.get_uxsim();
      [X, Y] = obj.get_XY(X_arr, Y_arr, x_sym, u_sym);
      [P, K] = get_PK(X, Y);
      P_fun = casadi.Function('P_fun',{x_sym,u_sym},{P});
      K_fun = casadi.Function('K_fun',{x_sym,u_sym},{K});
    end

    function [p_xidc] = get_pxidc(obj)
      p_xidc = obj.p_xidc;
    end

    function [Q_mpc,R_mpc] = get_QRmpc(obj)
      Q_mpc = obj.Q_mpc;
      R_mpc = obj.R_mpc;
    end

    function [ts] = get_ts(obj)
      ts = obj.ts;
    end

    function [ub] = get_upper_bounds(obj)
      ub = [obj.u_ub;
            obj.x_ub];
    end

    function varargout = get_uxABXYgrid(obj,do_print,do_check,options,X_arr,Y_arr)
      % Options:
      % 0: outputs n, u, x
      % 1: outputs n, u, x, A, B
      % 2: outputs n, u, x, A, B, X, Y, dotXgrid
      if (options < 0 || options > 2)
        cprintf('red',"[FalconModelT.get_uxABXYgrid] Unrecognized value for options! Returning.\n");
        return;
      end
      n_bytes = 0;
      % t0c_vals = 0;
      % t1c_vals = 0;
      % t2c_vals = 0;
      % t3c_vals = 0;
      % phi_vals = 0;
      % theta_vals = 0;
      % psi_vals = 0;
      % Note: set velocity values to zero, unless drag is included
      vx_vals = 0;
      vy_vals = 0;
      vz_vals = 0;
      % wbx_vals = 0;
      % wby_vals = 0;
      % wbz_vals = 0;
      % Gridding choices:
      % - Individual rotor thrust commands appear linearly in A
      % - Euler angles appear nonlinearly in A and B
      % - Linear velocities appear linearly in A
      % - Angular velocities appear linearly in A
      % - Checking with denser grid since it is computationally cheap
      n_grid = obj.n_grid_construct;
      n_grid_vertices = obj.n_grid_vertices_construct;
      if do_check
        n_grid = obj.n_grid_check;
        n_grid_vertices = obj.n_grid_vertices_check;
      end
      t0c_vals = linspace(obj.t_min,obj.t_max,n_grid_vertices);
      t1c_vals = linspace(obj.t_min,obj.t_max,n_grid_vertices);
      t2c_vals = linspace(obj.t_min,obj.t_max,n_grid_vertices);
      t3c_vals = linspace(obj.t_min,obj.t_max,n_grid_vertices);
      phi_vals = linspace(obj.att_min,obj.att_max,n_grid);
      theta_vals = linspace(obj.att_min,obj.att_max,n_grid);
      psi_vals = linspace(obj.att_min,obj.att_max,n_grid);
      % vx_vals = linspace(obj.v_min,obj.v_max,n_grid_vertices);
      % vy_vals = linspace(obj.v_min,obj.v_max,n_grid_vertices);
      % vz_vals = linspace(obj.v_min,obj.v_max,n_grid_vertices);
      wbx_vals = linspace(obj.wb_min,obj.wb_max,n_grid_vertices);
      wby_vals = linspace(obj.wb_min,obj.wb_max,n_grid_vertices);
      wbz_vals = linspace(obj.wb_min,obj.wb_max,n_grid_vertices);
      n = length(t0c_vals)*length(t1c_vals)*length(t2c_vals)*length(t3c_vals)*length(phi_vals)*length(theta_vals)*length(psi_vals)*length(vx_vals)*length(vy_vals)*length(vz_vals)*length(wbx_vals)*length(wby_vals)*length(wbz_vals);
      ugrid = zeros(obj.nu, n);
      xgrid = zeros(obj.nx, n);
      if (options > 0)
        Agrid = zeros(obj.nx, obj.nx, n);
        Bgrid = zeros(obj.nx, obj.nu, n);
        [~, A_fun, B_fun] = obj.get_fABfun();
        if (options > 1)
          if ~do_check
            Xgrid = sdpvar(obj.nx, obj.nx, n);
            Ygrid = sdpvar(obj.nu, obj.nx, n);
            Xgrid = Xgrid * 0;
            Ygrid = Ygrid * 0;
            if obj.nX > 1
              dotXgrid = sdpvar(obj.nx, obj.nx, n);
              dotXgrid = dotXgrid * 0;
            else
              dotXgrid = zeros(obj.nx, obj.nx, n);
            end
          else
            Xgrid = zeros(obj.nx, obj.nx, n);
            Ygrid = zeros(obj.nu, obj.nx, n);
            dotXgrid = zeros(obj.nx, obj.nx, n);
          end
        end
      end
      i = 0;
      for t0c = t0c_vals
        for t1c = t1c_vals
          for t2c = t2c_vals
            for t3c = t3c_vals
              for phi = phi_vals
                for theta = theta_vals
                  for psi = psi_vals
                    for vx = vx_vals
                      for vy = vy_vals
                        for vz = vz_vals
                          for wbx = wbx_vals
                            for wby = wby_vals
                              for wbz = wbz_vals
                                i = i + 1;
                                u = [t0c;
                                     t1c;
                                     t2c;
                                     t3c];
                                x = [zeros(3, 1);
                                     phi;
                                     theta;
                                     psi;
                                     vx;
                                     vy;
                                     vz;
                                     wbx;
                                     wby;
                                     wbz];
                                ugrid(:,i) = u;
                                xgrid(:,i) = x;
                                if (options > 0)
                                  Agrid(:, :, i) = full(A_fun(x,u));
                                  Bgrid(:, :, i) = full(B_fun(x,u));
                                  if (options > 1)
                                    if n == 1
                                      if obj.nX == 1
                                        Xgrid = X_arr;
                                      else
                                        p = obj.get_p(x,u);
                                        [~,p_fun_x,~] = obj.get_pfuns();
                                        dp_x = full(p_fun_x(x,u));
                                        dp = zeros(obj.nX-1,1);
                                        Xgrid = X_arr(:, :, 1); % X_0
                                        for j = 2:obj.nX
                                          Xgrid = Xgrid + X_arr(:, :, j) * p(j-1);
                                          dp(j-1) = dp_x(j-1,:) * obj.fun(x,u);
                                          dotXgrid = dotXgrid + X_arr(:, :, j) * dp(j-1);
                                        end
                                      end
                                      if obj.nY == 1
                                        Ygrid = Y_arr;
                                      else
                                        p = obj.get_p(x,u);
                                        Ygrid = Y_arr(:, :, 1); % Y_0
                                        for j = 2:obj.nY
                                          Ygrid = Ygrid + Y_arr(:, :, j) * p(j-1);
                                        end
                                      end
                                    else
                                      if obj.nX == 1
                                        Xgrid(:, :, i) = X_arr;
                                      else
                                        p = obj.get_p(x,u);
                                        [~,p_fun_x,~] = obj.get_pfuns();
                                        dp_x = full(p_fun_x(x,u));
                                        dp = zeros(obj.nX-1,1);
                                        Xgrid(:, :, i) = X_arr(:, :, 1); % X_0
                                        for j = 2:obj.nX
                                          Xgrid(:, :, i) = Xgrid(:, :, i) + X_arr(:, :, j) * p(j-1);
                                          dp(j-1) = dp_x(j-1,:) * obj.fun(x,u);
                                          dotXgrid(:, :, i) = dotXgrid(:, :, i) + X_arr(:, :, j) * dp(j-1);
                                        end
                                      end
                                      if obj.nY == 1
                                        Ygrid(:, :, i) = Y_arr;
                                      else
                                        p = obj.get_p(x,u);
                                        Ygrid(:, :, i) = Y_arr(:, :, 1); % Y_0
                                        for j = 2:obj.nY
                                          Ygrid(:, :, i) = Ygrid(:, :, i) + Y_arr(:, :, j) * p(j-1);
                                        end
                                      end
                                    end
                                  end
                                end
                                if do_print && rem(i,10) == 0
                                  fprintf(repmat('\b',1,n_bytes));
                                  n_bytes = fprintf("[FalconModelT.get_uxABXYgrid] Constructed grid point %i/%i\n",i,n);
                                end
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
      if do_print
        fprintf(repmat('\b',1,n_bytes));
        fprintf("[FalconModelT.get_uxABXYgrid] Constructed grid point %i/%i\n",i,n);
      end
      varargout{1} = n;
      varargout{2} = ugrid;
      varargout{3} = xgrid;
      if (options > 0)
        varargout{4} = Agrid;
        varargout{5} = Bgrid;
        if (options > 1)
          varargout{6} = Xgrid;
          varargout{7} = Ygrid;
          varargout{8} = dotXgrid;
        end
      end
    end

    function [u, x] = get_uxhover(obj)
      u = obj.t_hover * ones(obj.nu, 1);
      x = zeros(obj.nx, 1);
      x(2) = 1;
    end

    function [x_sym,u_sym] = get_uxsim(obj)
      import casadi.*;
      x_sym = MX.sym('x',obj.nx,1);
      u_sym = MX.sym('u',obj.nu,1);
    end

    function [wgrid, n] = get_wgrid(obj)
      % Generate grid points for each dimension
      w = cell(1,obj.nw);
      for i = 1:obj.nw
          w{i} = linspace(obj.w_min(i),obj.w_max(i),obj.n_grid_w); % Adjust the number of points as needed
      end

      % Create the grid
      [w_grid{1:obj.nw}] = ndgrid(w{:});

      % Combine the grid into a matrix
      wgrid = cell2mat(cellfun(@(x) x(:),w_grid,'UniformOutput',false))';

      % Determine number of grid points
      n = size(wgrid,2);
    end

    function [w_min,w_max] = get_wminmax(obj)
      w_min = obj.w_min;
      w_max = obj.w_max;
    end

    function [X, Y] = get_XY(obj, X_arr, Y_arr, x, u)
      if obj.nX == 1
        X = X_arr;
      else
        p = obj.get_p(x,u);
        X = X_arr(:, :, 1);
        for i = 2:obj.nX
          X = X + X_arr(:, :, i) * p(i-1);
        end
      end

      if obj.nY == 1
        Y = Y_arr;
      else
        p = obj.get_p(x,u);
        Y = Y_arr(:, :, 1);
        for i = 2:obj.nY
          Y = Y + Y_arr(:, :, i) * p(i-1);
        end
      end
    end
  end
end
