%% Upright LQR sweep across pendulum length and Q/R choices
%
% This script compares the linearized upright-equilibrium LQR response for
% a range of pendulum lengths and several hand-picked Q/R weight cases.
%
% It produces:
%   1. Overlaid theta responses by length for each Q/R case
%   2. Overlaid cart-position responses by length for each Q/R case
%   3. A summary scatter of peak cart displacement vs. peak acceleration
%   4. A CSV table of the computed metrics for every case/length pair
%
% The model matches the first-pass upright script:
%
%   X = [x; x_dot; theta; theta_dot]
%   u = x_ddot
%
% where theta = 0 is the upright equilibrium.

clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, '..', 'results', 'upright_length_qr_sweep');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Shared simulation settings

g = 9.81;                      % gravity [m/s^2]
lengths_m = linspace(0.500, 1.000, 10);
theta0_deg = 5;
X0 = [0; 0; deg2rad(theta0_deg); 0];
t_final = 10;
t = linspace(0, t_final, 1200);
settle_theta_deg = 1.0;

%% Candidate Q/R cases
%
% These are deliberately simple hand-picked cases around the original
% first-pass choice from lqr_first_pass.m so the tradeoffs are easy to
% inspect in overlaid plots.

weight_cases = [
    struct( ...
        "name", "baseline", ...
        "Q", diag([1, 0.1, 50, 1]), ...
        "R", 1, ...
        "description", "Original first-pass upright LQR weights" ...
    )
    struct( ...
        "name", "angle_aggressive", ...
        "Q", diag([1, 0.1, 100, 2]), ...
        "R", 1, ...
        "description", "Heavier theta and theta_dot penalty" ...
    )
    struct( ...
        "name", "cart_conservative", ...
        "Q", diag([10, 1, 50, 1]), ...
        "R", 1, ...
        "description", "Heavier x and x_dot penalty to limit travel" ...
    )
    struct( ...
        "name", "input_conservative", ...
        "Q", diag([1, 0.1, 50, 1]), ...
        "R", 10, ...
        "description", "Same state weights as baseline, larger input penalty" ...
    )
    struct( ...
        "name", "angle_and_input_conservative", ...
        "Q", diag([1, 0.1, 100, 2]), ...
        "R", 10, ...
        "description", "More angle authority requested, but with larger input penalty" ...
    )
];

case_count = numel(weight_cases);
length_count = numel(lengths_m);
results(case_count, length_count) = struct();

%% Sweep

for case_idx = 1:case_count
    weight_case = weight_cases(case_idx);

    for length_idx = 1:length_count
        l = lengths_m(length_idx);

        A = [0 1   0   0;
             0 0   0   0;
             0 0   0   1;
             0 0 g/l   0];

        B = [0;
             1;
             0;
            -1/l];

        K = lqr(A, B, weight_case.Q, weight_case.R);
        Acl = A - B * K;

        sys_cl = ss(Acl, zeros(4, 1), eye(4), zeros(4, 1));
        [~, t_out, X] = initial(sys_cl, X0, t);
        u = -(K * X.').';

        theta_deg = rad2deg(X(:, 3));
        settle_time_s = estimate_settle_time(t_out, theta_deg, settle_theta_deg);

        results(case_idx, length_idx).case_name = weight_case.name;
        results(case_idx, length_idx).case_description = weight_case.description;
        results(case_idx, length_idx).length_m = l;
        results(case_idx, length_idx).Q = weight_case.Q;
        results(case_idx, length_idx).R = weight_case.R;
        results(case_idx, length_idx).K = K;
        results(case_idx, length_idx).Acl_eigs = eig(Acl);
        results(case_idx, length_idx).t = t_out;
        results(case_idx, length_idx).X = X;
        results(case_idx, length_idx).u = u;
        results(case_idx, length_idx).peak_x_m = max(abs(X(:, 1)));
        results(case_idx, length_idx).peak_x_dot_m_s = max(abs(X(:, 2)));
        results(case_idx, length_idx).peak_theta_deg = max(abs(theta_deg));
        results(case_idx, length_idx).peak_theta_dot_deg_s = max(abs(rad2deg(X(:, 4))));
        results(case_idx, length_idx).peak_u_m_s2 = max(abs(u));
        results(case_idx, length_idx).settle_time_s = settle_time_s;
    end
end

%% Write summary table

summary_rows = table();

for case_idx = 1:case_count
    for length_idx = 1:length_count
        result = results(case_idx, length_idx);
        summary_rows = [summary_rows; struct2table(struct( ...
            "case_name", string(result.case_name), ...
            "case_description", string(result.case_description), ...
            "length_m", result.length_m, ...
            "q_x", result.Q(1, 1), ...
            "q_x_dot", result.Q(2, 2), ...
            "q_theta", result.Q(3, 3), ...
            "q_theta_dot", result.Q(4, 4), ...
            "R", result.R, ...
            "K_x", result.K(1), ...
            "K_x_dot", result.K(2), ...
            "K_theta", result.K(3), ...
            "K_theta_dot", result.K(4), ...
            "peak_x_m", result.peak_x_m, ...
            "peak_x_dot_m_s", result.peak_x_dot_m_s, ...
            "peak_theta_deg", result.peak_theta_deg, ...
            "peak_theta_dot_deg_s", result.peak_theta_dot_deg_s, ...
            "peak_u_m_s2", result.peak_u_m_s2, ...
            "settle_time_s", result.settle_time_s ...
        ))];
    end
end

writetable(summary_rows, fullfile(results_dir, "lqr_upright_length_qr_sweep_summary.csv"));

%% Figure 1: theta response overlays by length, grouped by weight case

colors = turbo(length_count);

figure("Name", "Upright LQR theta sweep by length and Q/R", "Position", [50 50 1400 900]);
tiledlayout(case_count, 1, "TileSpacing", "compact", "Padding", "compact");

for case_idx = 1:case_count
    nexttile;
    hold on;

    for length_idx = 1:length_count
        result = results(case_idx, length_idx);
        plot(result.t, rad2deg(result.X(:, 3)), "LineWidth", 1.2, "Color", colors(length_idx, :));
    end

    grid on;
    ylabel("\theta [deg]");
    title(sprintf("%s: Q = diag([%.3g %.3g %.3g %.3g]), R = %.3g", ...
        weight_cases(case_idx).name, ...
        weight_cases(case_idx).Q(1, 1), ...
        weight_cases(case_idx).Q(2, 2), ...
        weight_cases(case_idx).Q(3, 3), ...
        weight_cases(case_idx).Q(4, 4), ...
        weight_cases(case_idx).R));

    if case_idx == case_count
        xlabel("time [s]");
    end
end

lgd = legend(compose("l = %.3f m", lengths_m), "Location", "eastoutside");
lgd.Layout.Tile = "east";

savefig(fullfile(results_dir, "lqr_upright_length_qr_sweep_theta.fig"));
exportgraphics(gcf, fullfile(results_dir, "lqr_upright_length_qr_sweep_theta.png"), "Resolution", 150);

%% Figure 2: cart-position response overlays by length, grouped by weight case

figure("Name", "Upright LQR cart-position sweep by length and Q/R", "Position", [80 80 1400 900]);
tiledlayout(case_count, 1, "TileSpacing", "compact", "Padding", "compact");

for case_idx = 1:case_count
    nexttile;
    hold on;

    for length_idx = 1:length_count
        result = results(case_idx, length_idx);
        plot(result.t, result.X(:, 1), "LineWidth", 1.2, "Color", colors(length_idx, :));
    end

    grid on;
    ylabel("x [m]");
    title(sprintf("%s: cart displacement", weight_cases(case_idx).name));

    if case_idx == case_count
        xlabel("time [s]");
    end
end

lgd = legend(compose("l = %.3f m", lengths_m), "Location", "eastoutside");
lgd.Layout.Tile = "east";

savefig(fullfile(results_dir, "lqr_upright_length_qr_sweep_x.fig"));
exportgraphics(gcf, fullfile(results_dir, "lqr_upright_length_qr_sweep_x.png"), "Resolution", 150);

%% Figure 3: summary tradeoff scatter

figure("Name", "Upright LQR sweep tradeoff summary", "Position", [100 100 1200 800]);
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");
markers = ["o", "square", "diamond", "^", "v", "pentagram", "hexagram"];

nexttile;
hold on;
for case_idx = 1:case_count
    peak_x = arrayfun(@(r) r.peak_x_m, results(case_idx, :));
    peak_u = arrayfun(@(r) r.peak_u_m_s2, results(case_idx, :));
    scatter(peak_x, peak_u, 70, lengths_m, "filled", ...
        "Marker", markers(mod(case_idx - 1, numel(markers)) + 1), ...
        "DisplayName", weight_cases(case_idx).name);
end
grid on;
xlabel("peak |x| [m]");
ylabel("peak |u| [m/s^2]");
title("Travel vs. actuator effort");
cb = colorbar;
cb.Label.String = "pendulum length [m]";
legend("Location", "best");

nexttile;
hold on;
for case_idx = 1:case_count
    settle_times = arrayfun(@(r) r.settle_time_s, results(case_idx, :));
    peak_u = arrayfun(@(r) r.peak_u_m_s2, results(case_idx, :));
    scatter(settle_times, peak_u, 70, lengths_m, "filled", ...
        "Marker", markers(mod(case_idx - 1, numel(markers)) + 1), ...
        "DisplayName", weight_cases(case_idx).name);
end
grid on;
xlabel(sprintf("settle time to |theta| < %.1f deg [s]", settle_theta_deg));
ylabel("peak |u| [m/s^2]");
title("Settling vs. actuator effort");
cb = colorbar;
cb.Label.String = "pendulum length [m]";
legend("Location", "best");

nexttile;
hold on;
for case_idx = 1:case_count
    peak_theta = arrayfun(@(r) r.peak_theta_deg, results(case_idx, :));
    plot(lengths_m, peak_theta, "-o", "LineWidth", 1.5, "DisplayName", weight_cases(case_idx).name);
end
grid on;
xlabel("pendulum length [m]");
ylabel("peak |\theta| [deg]");
title("Peak angle vs. length");
legend("Location", "best");

nexttile;
hold on;
for case_idx = 1:case_count
    peak_x = arrayfun(@(r) r.peak_x_m, results(case_idx, :));
    plot(lengths_m, peak_x, "-o", "LineWidth", 1.5, "DisplayName", weight_cases(case_idx).name);
end
grid on;
xlabel("pendulum length [m]");
ylabel("peak |x| [m]");
title("Peak cart displacement vs. length");
legend("Location", "best");

savefig(fullfile(results_dir, "lqr_upright_length_qr_sweep_tradeoffs.fig"));
exportgraphics(gcf, fullfile(results_dir, "lqr_upright_length_qr_sweep_tradeoffs.png"), "Resolution", 150);

%% Console summary

disp("Wrote lqr_upright_length_qr_sweep_summary.csv");
disp("Saved overlay figures:");
disp("  lqr_upright_length_qr_sweep_theta.fig/.png");
disp("  lqr_upright_length_qr_sweep_x.fig/.png");
disp("  lqr_upright_length_qr_sweep_tradeoffs.fig/.png");

for case_idx = 1:case_count
    peak_x = arrayfun(@(r) r.peak_x_m, results(case_idx, :));
    peak_u = arrayfun(@(r) r.peak_u_m_s2, results(case_idx, :));
    settle_times = arrayfun(@(r) r.settle_time_s, results(case_idx, :));
    fprintf("%s:\n", weight_cases(case_idx).name);
    fprintf("  peak |x| range: %.3f to %.3f m\n", min(peak_x), max(peak_x));
    fprintf("  peak |u| range: %.3f to %.3f m/s^2\n", min(peak_u), max(peak_u));
    fprintf("  settle-time range: %.3f to %.3f s\n", min(settle_times), max(settle_times));
end

%% Local helper

function settle_time_s = estimate_settle_time(t, theta_deg, settle_band_deg)
    settle_time_s = NaN;
    within_band = abs(theta_deg) <= settle_band_deg;

    for idx = 1:numel(t)
        if all(within_band(idx:end))
            settle_time_s = t(idx);
            return;
        end
    end
end
