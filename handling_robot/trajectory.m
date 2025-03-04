% Figure of the end effector trajectory (Figure 21a in the paper)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_xaxis = 'samples'; % Optionen: time, normalized
%% Initialisierung
imescolors = imes_plot_template_colors();

repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir'));
datadir = fullfile(fullfile(repo_dir,'data'));
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');

importdir = HandlingRobot_dimsynth_data_dir();

%% Lade Trajektorie
OptName = 'ARK_3T1R_20230730_full_rep2';
setfile = dir(fullfile(importdir, OptName, '*settings.mat'));
if isempty(setfile)
  error('Simulationsdaten liegen nicht im Ordner');
end
d1 = load(fullfile(importdir, OptName, setfile(1).name));
d1.Traj.X(:,6) = NaN; % z-Rotation deaktivieren

fighdl = change_current_figure(1);clf;
axhdl = subplot(1,1,1); hold on;

linhdl1 = stairs(1:length(d1.Traj.t), 1e3*d1.Traj.X(:,1:3));
ylh = ylabel('End-effector position in mm');
grid on;
lw = 0.5; % line width
format = {'r',  '', '-', 0, lw; ...
          imescolors.gruen, '', '--', 2, lw; ...
          'b', 's', '-', 5, lw*2};
leglinhdl = line_format_publication(linhdl1, format);

if strcmp(usr_xaxis, 'time')
  xlabel('Trajectory time $t$ in s', 'interpreter', 'latex');
  error('Noch nicht vollständig implementiert');
elseif strcmp(usr_xaxis, 'samples')
  xlabel('Trajectory sample', 'interpreter', 'latex');
else
  xlabel('Trajectory progress $s$', 'interpreter', 'latex');
  error('Noch nicht vollständig implementiert');
end

if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
if exist('boxhdl', 'var'), delete(boxhdl); end
[zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [900, 1100; -100, 100]);
set(zoomhdl1, 'position', [0.3623 0.1630 0.6130 0.5330]);
if exist('arrhdl1', 'var'), delete(arrhdl1); end
arrhdl1 = annotation(fighdl,'arrow');
set(arrhdl1, 'position', [0.4151 0.2599 -0.1358 -0.0441]);

set(boxhdl, 'LineWidth', 2);

% Kompakter machen
ylhp = get(ylh, 'position');
[X_off, X_slope] = get_relative_position_in_axes(axhdl, 'x');
set(ylh, 'position', [X_off+X_slope*(-1.16), ylhp(2), 0]);

figure_format_publication(fighdl);
% MDPI verlangt, dass die Minus-Zeichen En-Dash.
set(axhdl, 'TickLabelInterpreter', 'latex');
ytickformat('$%g$')

legstr = {'$p_x$', '$p_y$', '$p_z$'};

% Höhe und sonstige Einstellungen konsistent mit Bild "detail" aus
% handlingpkm_perfmaps.m (beide Bilder stehen im Dokument nebeneinander)
set_size_plot_subplot(fighdl, ...
  9,6,axhdl,... 
  0.11,0.04, 0.12,0.13, 0,0);

if exist('leghdl', 'var'), delete(leghdl); end
leghdl = legendflex(leglinhdl(1:3), legstr(1:3), 'anchor', {'n','n'}, ...
  'ref', fighdl, ...
  'buffer', [0 -1], ...
  'ncol', 0, 'nrow', 1, ...
  'fontsize', 9, ...
  'FontName', 'Times New Roman', ...
  'xscale', 0.9, ...
  'padding', [2,2,2], ...
  'box', 'on', ...
  'interpreter', 'latex');

exportgraphics(fighdl, fullfile(outputdir, 'handlingpkm_trajectory.pdf'), ...
  'ContentType','vector');
