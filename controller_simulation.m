%% Adaptive Project - Quadcopter

%% Drone Parameter Values
%close all; clear all;
global Ka Km m Ix Iy Iz g l;
Ct = 0.0107;
Cq = Ct*sqrt(Ct/2);
Rr = 33/1000;    % rotor radius
RA = pi*Rr^2;     % rotor radius
rho = 1.184;    % density of air
Ka = Ct*rho*RA*Rr^2;
Km = Cq*rho*RA*Rr^3;
g = 9.81;
Ix = 0.0686e-3;
Iy = 0.092e-3;
Iz = 0.1366e-3;
l = 0.0624;        %Distance from rotor to the center of Drone
m = 0.068;

%% Linearized System arround the equilibrium point (Hover)
global K A B;

O6 = zeros(6);
I6 = eye(6);
Psi = 0;    % Phi angule chosen as Eq point at the hover.

Phi = [0, 0, 0, -g*sin(Psi), -g*cos(Psi), 0;
       0, 0, 0, g*cos(Psi), -g*sin(Psi), 0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0;
       0, 0, 0, 0,           0,          0];
   
A = [O6,  I6;       % Jacobian Matrix A
     Phi, O6];
 
Ac = [Phi, O6];     % Part of A that is Controllable

O84 = zeros(8,4);

Delta = [Ka/m,      Ka/m,           Ka/m,       Ka/m;
         0,         -Ka*l/Ix,       0,          Ka*l/Ix;
         Ka*l/Iy,   0,              -Ka*l/Iy,   0;
         Km/Iz,     -Km/Iz,         Km/Iz,      -Km/Iz];
       
B = [O84;       % Jacobian Matrix B
     Delta];
 
%% Model Mismatch (the actual plant)
global A_act B_act
A_act = A;
m_act = 1.5 * m;
l_act = 1.5 * l;
Delta_act = [   Ka/m_act,     Ka/m_act,     Ka/m_act,    Ka/m_act;
               0, -Ka*l_act/Ix,        0, Ka*l_act/Ix;
         Ka*l_act/Iy,        0, -Ka*l_act/Iy,       0;
           Km/Iz,   -Km/Iz,    Km/Iz,  -Km/Iz];

B_act = [O84;
         Delta_act];
 
%% Pole Placement
global Kr_lin_ctr;

desired_poles = -linspace(1,12,12);
K = place(A,B,desired_poles);
Kr_lin_ctr = B\(A-B*K);

% Our calculated Gain K to stabilize the system.
array2table(K)

% Eigenvalues of the closed loop system.
EigenValues = array2table(eig(A - B*K))

%% Set the desired State Values (Reference Signal that the 12 states should track)
global r;
r = [1,1,1,0,0,0,0,0,0,0,0,0]';     % the desired state values

%% Linear Controller & Ideal Linearizd Plant
tspan = [0, 5];
X0 = [0;0;0;0;0;0;0;0;0;0;0;0]; % [x, y, z, phi, theta, psi, xdot, ydot, zdot, p, q, r] initial conditions.

[t, X] = ode45(@LC_LinearModel, tspan, X0); % Here X is actually Xtilde, where Xtilde = X - Xeq. 

%Graph of the states of Xtilde, where Xtilde = X - Xeq..
titlelegend = ["Positions in x, y, z", "Euler angules: \phi, \theta, \psi.", "Velocities in x, y, z.", "Body angular rates: p, q, r."];
i = 1;
for j = 1:4
    figure; hold on;
    plot(t,X(:,i)); plot(t,X(:,i+1)); plot(t,X(:,i+2));
    legend(['x_{' num2str(i) '}(t)'],['x_{' num2str(i+1) '}(t)'],['x_{' num2str(i+2) '}(t)']);
    xlabel('t');
    ylabel('Magnitude of the state $\tilde{X}$', 'Interpreter','latex')
    title(titlelegend(j));
    grid on;
    i = i + 3;
end

% Xeq = [0,0,3,0,0,Psi,0,0,0,0,0,0]; % [x, y, z, phi, theta, psi, xdot, ydot, zdot, p, q, r] at eq point.
% 
% %Graph of the states of X, where X = Xtilde + Xeq.
% Xreal = zeros(length(X(:,1)),length(X(1,:)));
% for i = 1:length(X(:,1))
%     Xreal(i,:) = X(i,:) + Xeq;
% end
% 
% i = 1;
% for j = 1:4
%     figure; hold on;
%     plot(t,Xreal(:,i)); plot(t,Xreal(:,i+1)); plot(t,Xreal(:,i+2));
%     legend(['x_{' num2str(i) '}(t)'],['x_{' num2str(i+1) '}(t)'],['x_{' num2str(i+2) '}(t)']);
%     xlabel('t');
%     ylabel('Magnitud of the actual state X', 'Interpreter','latex')
%     title(titlelegend(j));
%     grid on;
%     i = i + 3;
% end

%% Linear Controller & Linearized Plant (Mismatched Parameters)
tspan = [0,5];
x0 = zeros(1,12);
[t_lin_act, x_lin_act] = ode45(@LC_LinearMismatchedModel, tspan, x0);

%Graph of the states of Xtilde, where Xtilde = X - Xeq..
% titlelegend = ["Positions in x, y, z", "Euler angules: \phi, \theta, \psi.", "Velocities in x, y, z.", "Body angular rates: p, q, r."];
% i = 1;
% for j = 1:4
%     figure; hold on;
%     plot(t,X(:,i)); plot(t,X(:,i+1)); plot(t,X(:,i+2));
%     legend(['x_{' num2str(i) '}(t)'],['x_{' num2str(i+1) '}(t)'],['x_{' num2str(i+2) '}(t)']);
%     xlabel('t');
%     ylabel('Magnitude of the state $\tilde{X}$', 'Interpreter','latex')
%     title(titlelegend(j));
%     grid on;
%     i = i + 3;
% end

%% Linear Adaptive Controller & Linearized Plant (Mismatched Parameters)
tspan = [0,5];
x0 = zeros(88,1);
% initial conditions: x(0) = xm(0) = 0
% use the ideal linearized plant to obtain kx(0) and kr(0)
temp_K = -K';
x0(25:72) = [temp_K(1,:) temp_K(2,:) temp_K(3,:) temp_K(4,:) temp_K(5,:) temp_K(6,:) temp_K(7,:) temp_K(8,:) temp_K(9,:) temp_K(10,:) temp_K(11,:) temp_K(12,:)]';
x0(73) = 1;
x0(78) = 1;
x0(83) = 1;
x0(88) = 1;
[t, x] = ode45(@AC_LinearMismatchedModel, tspan, x0);
x_adap_act = x(:,1:12);
xm_adap_act = x(:,13:24);
%% Plots
for i = 1:12
    figure;
    hold on
    plot(t,x_adap_act(:,i));
    plot(t,xm_adap_act(:,i));
    plot(t_lin_act,x_lin_act(:,i));
    legend(['x-adaptive_{' num2str(i) '}(t)'],['xm_{' num2str(i) '}(t)'],['x-linear_{' num2str(i) '}(t)']);
    xlabel('t');
    title('Adaptive & Linear Controller with Imperfect Plant: x-adaptive(t), x-linear(t), xm(t)');
    grid on;
    hold off
end

%% Linear Controller & Nonlinear Plant
tspan = [0,5];
x0 = zeros(1,12);
[t_lc_nlp, x_lc_nlp] = ode45(@LC_NonlinearModel, tspan, x0);

%% Plots
figure;
subplot(4,1,1);
hold on
plot(t_lc_nlp, x_lc_nlp(:,1));
plot(t_lc_nlp, x_lc_nlp(:,2));
plot(t_lc_nlp, x_lc_nlp(:,3));
title('State Trajectories of Fixed-gain Linear Controller with Nonlinear Plant');
legend('x_{1}(t)','x_{2}(t)','x_{3}(t)');
xlabel('t');
ylabel('Magnitude');
grid on

subplot(4,1,2);
hold on
plot(t_lc_nlp, x_lc_nlp(:,4));
plot(t_lc_nlp, x_lc_nlp(:,5));
plot(t_lc_nlp, x_lc_nlp(:,6));
legend('x_{4}(t)','x_{5}(t)','x_{6}(t)');
xlabel('t');
ylabel('Magnitude');
grid on

subplot(4,1,3);
hold on
plot(t_lc_nlp, x_lc_nlp(:,7));
plot(t_lc_nlp, x_lc_nlp(:,8));
plot(t_lc_nlp, x_lc_nlp(:,9));
legend('x_{7}(t)','x_{8}(t)','x_{9}(t)');
xlabel('t');
ylabel('Magnitude');
grid on

subplot(4,1,4);
hold on
plot(t_lc_nlp, x_lc_nlp(:,10));
plot(t_lc_nlp, x_lc_nlp(:,11));
plot(t_lc_nlp, x_lc_nlp(:,12));
legend('x_{10}(t)','x_{11}(t)','x_{12}(t)');
xlabel('t');
ylabel('Magnitude');
grid on

hold off

%% Linear Adaptive Controller & Nonlinear Plant
tspan = [0,0.3];
x0 = zeros(88,1);
% initial conditions: x(0) = xm(0) = 0
% use the ideal linearized plant to obtain kx(0) and kr(0)
temp_K = -K';
x0(25:72) = [temp_K(1,:) temp_K(2,:) temp_K(3,:) temp_K(4,:) temp_K(5,:) temp_K(6,:) temp_K(7,:) temp_K(8,:) temp_K(9,:) temp_K(10,:) temp_K(11,:) temp_K(12,:)]';
x0(73) = 1;
x0(78) = 1;
x0(83) = 1;
x0(88) = 1;
[t, x] = ode45(@LAC_NonlinearModel, tspan, x0);
x_adap_act = x(:,1:12);
xm_adap_act = x(:,13:24);
%% Plots
figure(1);
hold on
plot(t,x_adap_act(:,1));
plot(t,x_adap_act(:,2));
plot(t,x_adap_act(:,3));
legend('x_{1}(t)','x_{2}(t)','x_{3}(t)');
xlabel('t');
title('Position in x,y,z | Linear MRAC');
grid on;
hold off

figure(2);
subplot(3,1,1);
hold on
plot(t,x_adap_act(:,7));
title('Velocities in x,y,z | Linear MRAC');
legend('x_{7}(t)');
xlabel('t');
ylabel('Magnitude');
grid on

subplot(3,1,2);
hold on
plot(t,x_adap_act(:,8));
legend('x_{8}(t)');
xlabel('t');
ylabel('Magnitude');
grid on

subplot(3,1,3);
hold on
plot(t,x_adap_act(:,9));
legend('x_{9}(t)');
xlabel('t');
ylabel('Magnitude');
grid on

hold off

%% Nonlinear Adaptive Controller & Nonlinear Plant
global lambda gamma_x gamma_r gamma_alpha;
lambda = 0.01; % the parameter in the coord transform

% Some Preparation
global A_ref B_ref K_ref Kr_nonlin_ctr r_pos Am Bm P;
A_ref = [zeros(3,3) , eye(3) ; zeros(3,3) , zeros(3,3)];
B_ref = zeros(6,4);
B_ref(4,1) = -lambda*Ka*l/Iy;
B_ref(4,3) = lambda*Ka*l/Iy;
B_ref(5,2) = -lambda*Ka*l/Ix;
B_ref(5,4) = lambda*Ka*l/Ix;
B_ref(6,:) = [-Ka/m , -Ka/m, -Ka/m, -Ka/m];
desired_poles = [-1,-2,-3,-4,-5,-6];       %linspace(5,10,6);
K_ref = place(A_ref,B_ref,desired_poles);
Kr_nonlin_ctr = B_ref\(A_ref-B_ref*K_ref);

% the reference model
Am = A_ref - B_ref*K_ref;
Bm = B_ref;
% matrix Q , P
Q = 300*eye(6);
P = lyap(Am',Q);
% adaptation rates
gamma_x = 50*eye(6);
gamma_r = 50*eye(4);
gamma_alpha = 50*eye(6);

% The desired values for the 6 position states
% x1,x2,x3,x7,x8,x9 (in the original coords)
r_pos = [0;0;1;0;0;0];

% start the simulation
tspan = [0,5];
x0 = zeros(88,1);
% initial conditions: x(0) = xm(0) = 0
% convert xm(0) to the new transformed coordinates
x0(15) = -lambda;
x0(85) = -lambda;
% use the position state dynamics in the new coords to obtain kx(0) and kr(0)
temp_K = -K_ref';
x0(19:42) = [temp_K(1,:) temp_K(2,:) temp_K(3,:) temp_K(4,:) temp_K(5,:) temp_K(6,:)]';
x0(43) = 1;
x0(48) = 1;
x0(53) = 1;
x0(58) = 1;

% calculate alpha_hat(0)
temp1 = [-Ka/m , -Ka/m , -Ka/m, -Ka/m;
         0 , -lambda*Ka*l/Ix , 0 , lambda*Ka*l/Ix;
         -lambda*Ka*l/Iy , 0 , lambda*Ka*l/Iy , 0];
temp2 = [lambda , g , 0 , 0 , 0 , 0;
         0 , 0 , lambda*(-Ix+Iy-Iz)/Ix , g , 0 , 0;
         0 , 0 , 0 , 0 , -lambda*(-Ix+Iy+Iz)/Iy , -g];
alpha_star = (temp1\temp2)';
x0(59:82) = [alpha_star(1,:) , alpha_star(2,:) , alpha_star(3,:) , alpha_star(4,:) , alpha_star(5,:) , alpha_star(6,:)]';
%%
[t, x] = ode45(@AC_NonlinearModel, tspan, x0);

%% Plots
xt = x(:,1:12);
xm = x(:,13:18);
x_prime = x(:,83:88);

for i = 1:6
    figure;
    hold on
    plot(t,x_prime(:,i));
    plot(t,xm(:,i));
    xlabel('t');
    legend(['x_{' num2str(i) '}(t)'],['xm_{' num2str(i) '}(t)']);
    title('x(t) and xm(t) for the 6 position states (in the transformed coordinates)');
    grid on
    hold off
end

for i = 1:12
    figure;
    plot(t,xt(:,i));
    xlabel('t');
    legend(['x_{' num2str(i) '}(t)']);
    title('x(t) in the original coordinates');
    grid on;
end

%% Linear Controller & Ideal Linearized Plant
function xdot = LC_LinearModel(t,X)
    global K A B Kr_lin_ctr r;
    ut = -K*X - Kr_lin_ctr*r;
    xdot = A*X + B*ut;
end

%% Linear Controller & Linearized Plant (mismatched parameters)
function xdot = LC_LinearMismatchedModel(t,x)
    global K A_act B_act Kr_lin_ctr r;
    ut = -K*x - Kr_lin_ctr*r;
    xdot = A_act*x + B_act*ut;
end

%% Linear Adaptive Controller(MRAC MIMO) & Linearized Plant (mismatched parameters)
function xdot = AC_LinearMismatchedModel(t,x)
global K Kr_lin_ctr A B A_act B_act r;       % x: [x xm kx(1-4,:) km(1-4,:)]

disp(t);
Am = A - B*K;
Bm = B;
Q = 600*eye(12);          % 600
P = lyap(Am',Q);
gamma_x = 0.005*eye(12);    % 0.005
gamma_r = 0.005*eye(4);     % 0.005
rt = -Kr_lin_ctr * r;

x = reshape(x,[1,88]);
xt = x(1:12)';
xm = x(13:24)';
Kx = [x(25:28);x(29:32);x(33:36);x(37:40);x(41:44);x(45:48);x(49:52);x(53:56);x(57:60);x(61:64);x(65:68);x(69:72)];
Kr = [x(73:76) ; x(77:80) ; x(81:84) ; x(85:88)];
ut = Kx' * xt + Kr' * rt;

e = xt - xm;

Kx_dot = -gamma_x * xt * e' * P * B_act;
Kr_dot = -gamma_r * rt * e' * P * B_act;

xdot = zeros(88,1);
xdot(1:12) = A_act*xt + B_act*ut;
xdot(13:24) = Am*xm + Bm*rt;
xdot(25:72) = [Kx_dot(1,:) Kx_dot(2,:) Kx_dot(3,:) Kx_dot(4,:) Kx_dot(5,:) Kx_dot(6,:) Kx_dot(7,:) Kx_dot(8,:) Kx_dot(9,:) Kx_dot(10,:) Kx_dot(11,:) Kx_dot(12,:)]';
xdot(73:88) = [Kr_dot(1,:) Kr_dot(2,:) Kr_dot(3,:) Kr_dot(4,:)]';
end

%% Linear Adaptive Controller(MRAC MIMO) & Nonlinear Plant
function xdot = LAC_NonlinearModel(t,x)
global K Ka Km Kr_lin_ctr A B A_act B_act r l m g Ix Iy Iz;       % x: [x xm kx(1-4,:) km(1-4,:)]
disp(t);
Am = A - B*K;
Bm = B;
Q = 600*eye(12);          % 600
P = lyap(Am',Q);
gamma_x = 0.005*eye(12);    % 0.005
gamma_r = 0.005*eye(4);     % 0.005
rt = -Kr_lin_ctr * r;

x = reshape(x,[1,88]);
xt = x(1:12)';
xm = x(13:24)';
Kx = [x(25:28);x(29:32);x(33:36);x(37:40);x(41:44);x(45:48);x(49:52);x(53:56);x(57:60);x(61:64);x(65:68);x(69:72)];
Kr = [x(73:76) ; x(77:80) ; x(81:84) ; x(85:88)];
ut = Kx' * xt + Kr' * rt;

e = xt - xm;

Kx_dot = -gamma_x * xt * e' * P * B_act;
Kr_dot = -gamma_r * rt * e' * P * B_act;

xdot = zeros(88,1);
x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);

T = Ka*sum(ut);
tau_x = Ka*l * (ut(4) - ut(2));
tau_y = Ka*l * (ut(1) - ut(3));
tau_z = Km * (ut(1) - ut(2) + ut(3) - ut(4));

xdot(1) = x7;
xdot(2) = x8;
xdot(3) = x9;
xdot(4) = x10 + sin(x4)*tan(x5)*x11 + cos(x4)*tan(x5)*x12;
xdot(5) = cos(x4)*x11 - sin(x4)*x12;
xdot(6) = sin(x4)*x11/cos(x5) + cos(x4)*x12/cos(x5);
xdot(7) = -T * (cos(x4)*sin(x5)*cos(x6) + sin(x4)*sin(x6)) / m;
xdot(8) = -T * (cos(x4)*sin(x5)*sin(x6) - sin(x4)*cos(x6)) / m;
xdot(9) = -T * cos(x4) * cos(x5) / m + g;
xdot(10) = (Iy - Iz)*x11*x12/Ix + tau_x/Ix;
xdot(11) = (-Ix + Iz)*x10*x12/Iy + tau_y/Iy;
xdot(12) = (Ix - Iy)*x10*x11/Iz + tau_z/Iz;

xdot(13:24) = Am*xm + Bm*rt;
xdot(25:72) = [Kx_dot(1,:) Kx_dot(2,:) Kx_dot(3,:) Kx_dot(4,:) Kx_dot(5,:) Kx_dot(6,:) Kx_dot(7,:) Kx_dot(8,:) Kx_dot(9,:) Kx_dot(10,:) Kx_dot(11,:) Kx_dot(12,:)]';
xdot(73:88) = [Kr_dot(1,:) Kr_dot(2,:) Kr_dot(3,:) Kr_dot(4,:)]';
end

%% Linear Controller with Nonlinear Plant
function xdot = LC_NonlinearModel(t,x)
    global K Ka Km Kr_lin_ctr r l m g Ix Iy Iz;
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
    x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);
    
    ut = -K*x - Kr_lin_ctr*r;
    T = Ka*sum(ut);
    tau_x = Ka*l * (ut(4) - ut(2));
    tau_y = Ka*l * (ut(1) - ut(3));
    tau_z = Km * (ut(1) - ut(2) + ut(3) - ut(4));
    
    xdot = zeros(12,1);
    xdot(1) = x7;
    xdot(2) = x8;
    xdot(3) = x9;
    xdot(4) = x10 + sin(x4)*tan(x5)*x11 + cos(x4)*tan(x5)*x12;
    xdot(5) = cos(x4)*x11 - sin(x4)*x12;
    xdot(6) = sin(x4)*x11/cos(x5) + cos(x4)*x12/cos(x5);
    xdot(7) = -T * (cos(x4)*sin(x5)*cos(x6) + sin(x4)*sin(x6)) / m;
    xdot(8) = -T * (cos(x4)*sin(x5)*sin(x6) - sin(x4)*cos(x6)) / m;
    xdot(9) = -T * cos(x4) * cos(x5) / m + g;
    xdot(10) = (Iy - Iz)*x11*x12/Ix + tau_x/Ix;
    xdot(11) = (-Ix + Iz)*x10*x12/Iy + tau_y/Iy;
    xdot(12) = (Ix - Iy)*x10*x11/Iz + tau_z/Iz;
end

%% Nonlinear Adaptive Controller (MRAC MIMO, nonlinear plant)
function xdot = AC_NonlinearModel(t,x)
disp(t);
% x: [x:12*1 xm:6*1 kx:6*4 kr:4*4 alpha:6*4 x_prime:6*1]
global lambda Am Bm P Kr_nonlin_ctr gamma_x gamma_r gamma_alpha r_pos Ka Km m Ix Iy Iz g l;       

x = reshape(x,[1,88]);
x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
x7 = x(7); x8 = x(8); x9 = x(9); x10 = x(10); x11 = x(11); x12 = x(12);

% transform the position states x1,x2,x3,x7,x8,x9 from the original
% coordinates to the new transformed coordinates (prime)
x1_prime = x1 + lambda*(-cos(x4)*sin(x5)*cos(x6) - sin(x4)*sin(x6));
x2_prime = x2 + lambda*(-cos(x4)*sin(x5)*sin(x6) + sin(x4)*cos(x6));
x3_prime = x3 - lambda*cos(x4)*cos(x5);
x7_prime = x7 + lambda*(sin(x4)*sin(x5)*cos(x6)*x10 - cos(x4)*sin(x6)*x10 - cos(x5)*cos(x6)*x11);
x8_prime = x8 + lambda*(sin(x4)*sin(x5)*sin(x6)*x10 + cos(x4)*cos(x6)*x10 - cos(x5)*sin(x6)*x11);
x9_prime = x9 + lambda*(sin(x4)*cos(x5)*x10 + sin(x5)*x11);

x_prime = [x1_prime;x2_prime;x3_prime;x7_prime;x8_prime;x9_prime];
%x_prime = reshape(x(83:88),[6,1]);

% transform the desired ref states r from the original
% coordinates to the new transformed coordinates (prime)
r1 = r_pos(1); r2 = r_pos(2); r3 = r_pos(3);
r7 = r_pos(4); r8 = r_pos(5); r9 = r_pos(6);
r1_prime = r1 + lambda*(-cos(x4)*sin(x5)*cos(x6) - sin(x4)*sin(x6));
r2_prime = r2 + lambda*(-cos(x4)*sin(x5)*sin(x6) + sin(x4)*cos(x6));
r3_prime = r3 - lambda*cos(x4)*cos(x5);
r7_prime = r7 + lambda*(sin(x4)*sin(x5)*cos(x6)*x10 - cos(x4)*sin(x6)*x10 - cos(x5)*cos(x6)*x11);
r8_prime = r8 + lambda*(sin(x4)*sin(x5)*sin(x6)*x10 + cos(x4)*cos(x6)*x10 - cos(x5)*sin(x6)*x11);
r9_prime = r9 + lambda*(sin(x4)*cos(x5)*x10 + sin(x5)*x11);
%r3_prime = r3 - lambda;
r_prime = [r1_prime;r2_prime;r3_prime;r7_prime;r8_prime;r9_prime];

rt_prime = -Kr_nonlin_ctr * r_prime;
%rt = -Kr_lin_ctr * r;

% The Phi(x) matrix that parametrizes the nonlinearity
phi_x = [x10*x10 + x11*x11;
         cos(x4) * cos(x5);
         x11 * x12;
         sin(x4) * cos(x5);
         x10 * x12;
         sin(x5)];

xt = reshape(x(1:12),[12,1]);
xm = reshape(x(13:18),[6,1]);
Kx = [x(19:22); x(23:26); x(27:30); x(31:34); x(35:38); x(39:42)];
Kr = [x(43:46); x(47:50); x(51:54); x(55:58)];
alpha_hat = [x(59:62); x(63:66); x(67:70); x(71:74) ; x(75:78) ; x(79:82)];


% The error signal for the position states (in the new transformed coords)
% xm1_prime = xm(1) + lambda*(-cos(x4)*sin(x5)*cos(x6) - sin(x4)*sin(x6));
% xm2_prime = xm(2) + lambda*(-cos(x4)*sin(x5)*sin(x6) + sin(x4)*cos(x6));
% xm3_prime = xm(3) - lambda*cos(x4)*cos(x5);
% xm4_prime = xm(4) + lambda*(sin(x4)*sin(x5)*cos(x6)*x10 - cos(x4)*sin(x6)*x10 - cos(x5)*cos(x6)*x11);
% xm5_prime = xm(5) + lambda*(sin(x4)*sin(x5)*sin(x6)*x10 + cos(x4)*cos(x6)*x10 - cos(x5)*sin(x6)*x11);
% xm6_prime = xm(6) + lambda*(sin(x4)*cos(x5)*x10 + sin(x5)*x11);
% xm_prime = [xm1_prime; xm2_prime; xm3_prime; xm4_prime; xm5_prime; xm6_prime];
e_prime = x_prime - xm;

% The B_prime matrix for the plant in the transformed coords
R = [cos(x4)*sin(x5)*cos(x6)+sin(x4)*sin(x6) , sin(x4)*sin(x5)*cos(x6)-cos(x4)*sin(x6) , cos(x5)*cos(x6);
     cos(x4)*sin(x5)*sin(x6)-sin(x4)*cos(x6) , sin(x4)*sin(x5)*sin(x6)+cos(x4)*cos(x6) , cos(x5)*sin(x6);
     cos(x4)*cos(x5) , sin(x4)*cos(x5) , -sin(x5)];
Ra = R(1,1); Rb = R(1,2); Rc = R(1,3); Rd = R(2,1); Re = R(2,2); Rf = R(2,3); Rg = R(3,1); Rh = R(3,2); Ri = R(3,3); 
Rstar = [-Ka*Ra/m - lambda*Ka*l*Rc/Iy , -Ka*Ra/m - lambda*Ka*l*Rb/Ix , -Ka*Ra/m + lambda*Ka*l*Rc/Iy , -Ka*Ra/m + lambda*Ka*l*Rb/Ix;
         -Ka*Rd/m - lambda*Ka*l*Rf/Iy , -Ka*Rd/m - lambda*Ka*l*Re/Ix , -Ka*Rd/m + lambda*Ka*l*Rf/Iy , -Ka*Rd/m + lambda*Ka*l*Re/Ix;
         -Ka*Rg/m - lambda*Ka*l*Ri/Iy , -Ka*Rg/m - lambda*Ka*l*Rh/Ix , -Ka*Rg/m + lambda*Ka*l*Ri/Iy , -Ka*Rg/m + lambda*Ka*l*Rh/Ix];
B_prime = [zeros(3,4) ; Rstar];

% The control output u(t)
ut = Kx' * x_prime + Kr' * rt_prime - alpha_hat' * phi_x;
% The adaptation laws
Kx_dot = -gamma_x * x_prime * e_prime' * P * B_prime;
Kr_dot = -gamma_r * rt_prime * e_prime' * P * B_prime;
Kalpha_dot = gamma_alpha * phi_x * e_prime' * P * B_prime;

xdot = zeros(88,1);
% state update for xm (in the transformed coords)
xdot(13:18) = Am*xm + Bm*rt_prime;
% state update for Kx_prime
xdot(19:42) = [Kx_dot(1,:) , Kx_dot(2,:) , Kx_dot(3,:) , Kx_dot(4,:) , Kx_dot(5,:) , Kx_dot(6,:)]';
xdot(43:58) = [Kr_dot(1,:) , Kr_dot(2,:) , Kr_dot(3,:) , Kr_dot(4,:)]';
xdot(59:82) = [Kalpha_dot(1,:) , Kalpha_dot(2,:) , Kalpha_dot(3,:) , Kalpha_dot(4,:) , Kalpha_dot(5,:) , Kalpha_dot(6,:)]';

% The actual nonlinear dynamics for updating x(t) (in the original coords)
T = Ka * sum(ut);
tau_x = Ka*l * (ut(4) - ut(2));
tau_y = Ka*l * (ut(1) - ut(3));
tau_z = Km * (ut(1) - ut(2) + ut(3) - ut(4));
xdot(1) = x7;
xdot(2) = x8;
xdot(3) = x9;
xdot(4) = x10 + sin(x4)*tan(x5)*x11 + cos(x4)*tan(x5)*x12;
xdot(5) = cos(x4)*x11 - sin(x4)*x12;
xdot(6) = sin(x4)*x11/cos(x5) + cos(x4)*x12/cos(x5);
xdot(7) = -T * (cos(x4)*sin(x5)*cos(x6) + sin(x4)*sin(x6)) / m;
xdot(8) = -T * (cos(x4)*sin(x5)*sin(x6) - sin(x4)*cos(x6)) / m;
xdot(9) = -T * cos(x4) * cos(x5) / m + g;
xdot(10) = (Iy - Iz)*x11*x12/Ix + tau_x/Ix;
xdot(11) = (-Ix + Iz)*x10*x12/Iy + tau_y/Iy;
xdot(12) = (Ix - Iy)*x10*x11/Iz + tau_z/Iz;

% The dynamics for x_prime
A = [zeros(3,3) , eye(3) ; zeros(3,3) , zeros(3,3)];
B = [zeros(3,3) ; R];
f = [-T/m + lambda*(x10*x10+x11*x11) + cos(x4)*cos(x5)*g;
     lambda*((-Ix+Iy-Iz)*x11*x12/Ix + tau_x/Ix) + sin(x4)*cos(x5)*g;
     -lambda*((-Ix+Iy+Iz)*x10*x12/Iy + tau_y/Iy) + sin(x5)*g];
x_prime_dot = A*x(83:88)' + B*f;
xdot(83:88) = x_prime_dot;

end