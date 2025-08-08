%% Check eigenvalues
% Initialize MATLAB interface
clear;
close all;
clc;

% Select model
model = FalconModelT('falcon_t');

% Get A and B casadi functions
[~,A_ct_fun,B_ct_fun] = model.get_fABfun();

% Get x and u at hovering condition
[u_hover,x_hover] = model.get_uxhover();

% Compute A and B
x_around_hover = [0.1,0.1,1,0.01,0.01,0.01,0.1,0.1,0.1,0.1,0.1,0.1];
A_ct = full(A_ct_fun(x_around_hover,u_hover));

% Compute eigenvalues of A
[A_eigvec,A_eigval] = eig(A_ct);


%% Check closed-loop eigenvalues - LQR
% Initialize MATLAB interface
clear;
close all;
clc;

% Select model
model = FalconModelT('falcon_t');

% Get A and B casadi functions
[~,A_ct_fun,B_ct_fun] = model.get_fABfun();

% Get x and u at hovering condition
[u_hover,x_hover] = model.get_uxhover();

% Get sampling time
ts = model.get_ts();
% ts = 0.01;

% Compute A and B
A_ct = full(A_ct_fun(x_hover,u_hover));
B_ct = full(B_ct_fun(x_hover,u_hover));

% Get state and input dimension
nx = model.get_nx();
nu = model.get_nu();

% Get Q and R weighting matrices
[Q_ct,R_ct] = model.get_QRmpc();

% Determine number of uncontrollable modes
Co = ctrb(A_ct,B_ct);
unco = length(A_ct) - rank(Co);

% Compute ct LQR feedback
[K_ct,P_ct,~] = lqr(A_ct,B_ct,Q_ct,R_ct);

% Compute eigenvalues of closed-loop system after adding ct LQR feedback
A_ct_cl = A_ct - B_ct * K_ct;
[A_ct_cl_eigvec,A_ct_cl_eigval] = eig(A_ct_cl);

% Discretize closed-loop ct system and compute eigenvalues
A_ct_cl_dt = expm(A_ct_cl * ts);
[A_ct_cl_dt_eigvec,A_ct_cl_dt_eigval] = eig(A_ct_cl_dt);

% Compute eigenvalues of dt closed-loop system after adding dt LQR feedback
ss_ct = ss(A_ct,B_ct,eye(nx),zeros(nx,nu));
ss_dt = c2d(ss_ct,ts);
A_dt = ss_dt.A;
B_dt = ss_dt.B;
Q_dt = Q_ct * ts;
R_dt = R_ct * ts;
[K_dt,P_dt,~] = dlqr(A_dt,B_dt,Q_dt,R_dt);
A_dt_cl = A_dt - B_dt * K_dt;
[A_dt_cl_eigvec,A_dt_cl_eigval] = eig(A_dt_cl);

% Discretize ct system, then add ct LQR feedback, then compute eigenvalues
A_dt_ct_cl = A_dt - B_dt * K_ct;
[A_dt_ct_cl_eigvec,A_dt_ct_cl_eigval] = eig(A_dt_ct_cl);

% save('K_lqr_ct.mat','K_ct');
% save('K_lqr_dt.mat','K_dt');
% save('P_lqr_ct.mat','P_ct');

% Check norm of matrix differences
norm(A_dt_cl-A_ct_cl_dt);


%% Check closed-loop eigenvalues - RMPC
% Initialize MATLAB interface
clear;
close all;

% Load data
load offline_design_rmpc.mat;

% Select model
model = FalconModelT('falcon_t');

% Get A and B casadi functions
[~,A_ct_fun,B_ct_fun] = model.get_fABfun();

% Get x and u at hovering condition
[u_hover,x_hover] = model.get_uxhover();

% Get sampling time
ts = model.get_ts();

% Compute K_delta
K_delta_fun = K_delta;
K_delta = full(K_delta_fun(x_hover,u_hover));

% Compute A and B
A_ct = full(A_ct_fun(x_hover,u_hover));
B_ct = full(B_ct_fun(x_hover,u_hover));

% Get state and input dimension
nx = model.get_nx();
nu = model.get_nu();

% Compute eigenvalues of closed-loop system after adding feedback to discretized open-loop system
ss_ct = ss(A_ct,B_ct,eye(nx),zeros(nx,nu));
ss_dt = c2d(ss_ct,ts);
A_dt = ss_dt.A;
B_dt = ss_dt.B;
A_dt_cl = A_dt + B_dt * K_delta;
[A_dt_cl_eigvec,A_dt_cl_eigval] = eig(A_dt_cl);

% Compute eigenvalues of discretized closed-loop system
A_ct_cl = A_ct + B_ct * K_delta;
A_ct_cl_dt = expm(A_ct_cl * ts);
[A_ct_cl_dt_eigvec,A_ct_cl_dt_eigval] = eig(A_ct_cl_dt);

% Check norm of matrix differences
norm(A_dt_cl-A_ct_cl_dt);


%% Check closed-loop eigenvalues - ROMPC
% Initialize MATLAB interface
clear;
close all;

% Load data
load offline_design_rompc.mat;

% Select model
model = FalconModelT('falcon_t');

% Get A and B casadi functions
[~,A_ct_fun,B_ct_fun] = model.get_fABfun();

% Get x and u at hovering condition
[u_hover,x_hover] = model.get_uxhover();

% Get sampling time
ts = model.get_ts();

% Compute K_delta
K_delta_fun = K_delta;
K_delta = full(K_delta_fun(x_hover,u_hover));

% Compute A and B
A_ct = full(A_ct_fun(x_hover,u_hover));
B_ct = full(B_ct_fun(x_hover,u_hover));

% Get C
C = model.get_C();

% Get L
L = model.get_L(rho_c);

% Get state and input dimension
nx = model.get_nx();
nu = model.get_nu();

% Compute eigenvalues of continuous-time closed-loop system
A_ct_cl = [A_ct,B_ct*K_delta;
           L*C,A_ct+B_ct*K_delta-L*C];
[A_ct_cl_eigvec,A_ct_cl_eigval] = eig(A_ct_cl);

% Compute eigenvalues of exact discretized closed-loop system
A_ct_cl_dt = expm(A_ct_cl*ts);
[A_ct_cl_dt_eigvec,A_ct_cl_dt_eigval] = eig(A_ct_cl_dt);

% Compute eigenvalues of Euler discretized closed-loop system
A_ct_cl_dt_eul = eye(2*nx)+ts*A_ct_cl;
[A_ct_cl_dt_eul_eigvec,A_ct_cl_dt_eul_eigval] = eig(A_ct_cl_dt_eul);

% Check norm of matrix differences
norm(A_ct_cl_dt-A_ct_cl_dt_eul);

