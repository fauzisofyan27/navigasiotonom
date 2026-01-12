% Simulasi Robot Trajectory di MATLAB
clear; clc;

%% 1. Konfigurasi Parameter
ARENA_SIZE = 1.0;
GRID_SIZE = 5;
CELL_SIZE = ARENA_SIZE / GRID_SIZE;

% Daftar Obstacle (Koordinat Grid)
OBSTACLES = [
    0, 2;
    1, 1;
    2, 3;
    3, 2;
    4, 1
];

%% 2. Membaca Data Trajectory (CSV)
data = readtable('trajectory_map3.csv');
x = data.x;
y = data.y;

%% 3. Visualisasi
figure('Color', 'w', 'Name', 'Robot Trajectory Simulation');
hold on; grid on;

% Mengatur Tampilan Grid
ticks = (0:GRID_SIZE) * CELL_SIZE - ARENA_SIZE/2;
xticks(ticks);
yticks(ticks);
set(gca, 'GridLineStyle', '--', 'LineWidth', 0.5);

% Plot Obstacles (Kotak Merah)
for i = 1:size(OBSTACLES, 1)
    ox = OBSTACLES(i, 1);
    oy = OBSTACLES(i, 2);
    
    % Menghitung posisi koordinat dunia
    wx = ox * CELL_SIZE - ARENA_SIZE/2;
    wy = oy * CELL_SIZE - ARENA_SIZE/2;
    
    % Menggambar rectangle: [x_start, y_start, width, height]
    rectangle('Position', [wx, wy, CELL_SIZE, CELL_SIZE], ...
              'FaceColor', [0.75, 0, 0], 'EdgeColor', [0.75, 0, 0]);
end

% Plot Jalur Robot
plot(x, y, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot Trajectory');

% Plot Titik Start dan Goal
scatter(x(1), y(1), 60, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g', 'DisplayName', 'Start');
scatter(x(end), y(end), 60, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'DisplayName', 'Goal');

%% 4. Finishing (Anotasi)
axis equal;
xlim([-ARENA_SIZE/2, ARENA_SIZE/2]);
ylim([-ARENA_SIZE/2, ARENA_SIZE/2]);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Map 1 Trajectory Simulation');
legend('Location', 'northeastoutside');

hold off;