%% First-pass LQR model for the inverted pendulum on a linear actuator
%
% This script uses the simplified point-mass model documented in
% parameters.md. The actuator input is idealized as commanded cart
% acceleration:
%
%   u = x_ddot
%
% State vector:
%
%   X = [x; x_dot; theta; theta_dot]
%
% where theta = 0 is the upright equilibrium.

clear; clc; close all;

%% Physical parameters

g = 9.81;      % gravity [m/s^2]
l = 0.510;     % pivot to rough bob center of mass [m]

%% Linearized open-loop model

A = [0 1   0   0;
     0 0   0   0;
     0 0   0   1;
     0 0 g/l   0];

B = [0;
     1;
     0;
    -1/l];

%% LQR weights
%
% Increase q_theta to prioritize upright balance.
% Increase q_x to keep the actuator near the center of travel.
% Increase R to reduce commanded actuator acceleration.

q_x = 1;
q_x_dot = 0.1;
q_theta = 50;
q_theta_dot = 1;

Q = diag([q_x, q_x_dot, q_theta, q_theta_dot]);
R = 1;

%% Compute LQR gain and closed-loop dynamics

K = lqr(A, B, Q, R);
Acl = A - B*K;

disp('LQR gain K:');
disp(K);

disp('Closed-loop eigenvalues:');
disp(eig(Acl));

%% Simulate response from an initial perturbation

theta0_deg = 5;
X0 = [0;                  % initial cart position [m]
      0;                  % initial cart velocity [m/s]
      deg2rad(theta0_deg);% initial pendulum angle [rad]
      0];                 % initial pendulum angular velocity [rad/s]

t_final = 8;
t = linspace(0, t_final, 1000);

sys_cl = ss(Acl, zeros(4, 1), eye(4), zeros(4, 1));
[~, t_out, X] = initial(sys_cl, X0, t);

u = -(K * X.').';

%% Plot states and input

make_plots = true;

if make_plots
figure('Name', 'First-pass inverted pendulum LQR response');

subplot(5, 1, 1);
plot(t_out, X(:, 1), 'LineWidth', 1.5);
grid on;
ylabel('x [m]');
title('Closed-loop response from initial pendulum angle perturbation');

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
end

fprintf('Peak commanded acceleration: %.3f m/s^2\n', max(abs(u)));
fprintf('Peak cart displacement: %.3f m\n', max(abs(X(:, 1))));
