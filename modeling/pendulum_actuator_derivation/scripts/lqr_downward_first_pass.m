%% First-pass LQR model for the downward pendulum on a linear actuator
%
% This script mirrors lqr_first_pass.m, but linearizes around the stable
% hanging-down equilibrium instead of the upright equilibrium.
%
% The actuator input is idealized as commanded cart acceleration:
%
%   u = x_ddot
%
% State vector:
%
%   X = [x; x_dot; theta; theta_dot]
%
% where theta = 0 is the downward hanging equilibrium.
%
% This controller is useful for early bench tests where the pendulum starts
% hanging down and small perturbations should be damped out. It is not the
% inverted-pendulum balance controller.

clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, '..', 'results', 'downward_first_pass');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Physical parameters

g = 9.81;      % gravity [m/s^2]
l = 0.510;    % pivot to rough bob center of mass [m]

%% Linearized open-loop model about the downward equilibrium
%
% Compared with the upright linearization, the gravity stiffness changes
% sign. With theta measured from downward, gravity is restoring:
%
%   theta_ddot = -(g/l) theta - (1/l) u

A = [0 1    0    0;
     0 0    0    0;
     0 0    0    1;
     0 0 -g/l    0];

B = [0;
     1;
     0;
    -1/l];

%% LQR weights
%
% Since the downward equilibrium is already stable, these weights are
% deliberately gentler than the upright first pass. Increase q_theta or
% q_theta_dot if the controller should damp pendulum motion more strongly.
% Increase q_x or q_x_dot to keep the actuator closer to center. Increase R
% to reduce commanded actuator acceleration.

q_x = 1;
q_x_dot = 0.1;
q_theta = 5;
q_theta_dot = 1;

Q = diag([q_x, q_x_dot, q_theta, q_theta_dot]);
R = 10;

%% Compute LQR gain and closed-loop dynamics

K = lqr(A, B, Q, R);
Acl = A - B*K;

disp('Downward LQR gain K:');
disp(K);

disp('Closed-loop eigenvalues:');
disp(eig(Acl));

%% Simulate response from an initial downward-angle perturbation

theta0_deg = 5;
X0 = [0;                   % initial cart position [m]
      0;                   % initial cart velocity [m/s]
      deg2rad(theta0_deg); % initial pendulum angle from downward [rad]
      0];                  % initial pendulum angular velocity [rad/s]

t_final = 12;
t = linspace(0, t_final, 1200);

sys_cl = ss(Acl, zeros(4, 1), eye(4), zeros(4, 1));
[~, t_out, X] = initial(sys_cl, X0, t);

u = -(K * X.').';

%% Plot states and input

make_plots = true;

if make_plots
figure('Name', 'First-pass downward pendulum LQR response');

subplot(5, 1, 1);
plot(t_out, X(:, 1), 'LineWidth', 1.5);
grid on;
ylabel('x [m]');
title('Downward-equilibrium LQR response from initial pendulum angle perturbation');

subplot(5, 1, 2);
plot(t_out, X(:, 2), 'LineWidth', 1.5);
grid on;
ylabel('x_dot [m/s]');

subplot(5, 1, 3);
plot(t_out, rad2deg(X(:, 3)), 'LineWidth', 1.5);
grid on;
ylabel('theta [deg]');

subplot(5, 1, 4);
plot(t_out, rad2deg(X(:, 4)), 'LineWidth', 1.5);
grid on;
ylabel('theta_dot [deg/s]');

subplot(5, 1, 5);
plot(t_out, u, 'LineWidth', 1.5);
grid on;
ylabel('u [m/s^2]');
xlabel('time [s]');

savefig(fullfile(results_dir, 'LQR_downward_closed_loop_response.fig'));
exportgraphics(gcf, fullfile(results_dir, 'LQR_downward_closed_loop_response.png'), 'Resolution', 150);
end

fprintf('Peak commanded acceleration: %.3f m/s^2\n', max(abs(u)));
fprintf('Peak cart displacement: %.3f m\n', max(abs(X(:, 1))));
