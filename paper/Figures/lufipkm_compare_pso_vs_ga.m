% Erzeuge ein Auswertungsbild für den Vergleich von GA und PSO.
% 
% Vorher: Führe eval_figures_pareto.m und eval_figures_pareto_groups.m aus
% Dort Einstellungen motordiagram_ga_1, motordiagram_ga_2, ... wählen.
% Damit werden mat-Dateien erzeugt, die hier geladen werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2025-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

if isempty(which('lufi_dimsynth_data_dir'))
  error(['You have to create a file lufi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end

repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');
paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');

fighdl = change_current_figure(1);clf;
% TODO: Schleife über drei Roboter
% Dann dort die jeweiligen Pareto-Fronten (gruppiert und evtl. ausgedünnt)
% aus dem anderen groups-Skript laden). Dann neues Pareto-Diag. damit
% zeichnen. PSO und GA Fronten in ein Subplot. Idealerweise sieht man, dass
% PSO deutlich besser ist.
% Zusätzlich mit printf die Anzahl der gültigen Lösungen auf der
% Pareto-Front ausgeben.
% tmp = load(fullfile(datadir, 'robot_groups_motordiagram_prismatic_ga_2.mat'));
% tmp.RobotGroups
% Für diese Strukturen (gruppiert aus ...groups.m) gibt es Pareto-Fronten
GroupNames = {'P6RRPRRR14V6G', 'P6RRRRRR10V6G', 'P6PRRRRR6V4G'};
TitleStrings = {'Hexapod (6-UPS)', 'Hexa (6-RUS)', 'HexaSlide (6-PUS)'};
% Vergleich von zwei Methoden
Methods = {'ga', 'pso'};
colors = {'r', 'b'}; % für Methode
markers = {'s', 'x', 'v', '^', '*', '+', '<', '>', 'o', 'd', 'p', 'h', '|'}; % Für Wiederholung
pareto_settings = {'actforce', 'actvelo'};
axhdl = gobjects(1,3);
for i_groups = 1:3 % Schleife über Strukturen
  GroupName = GroupNames{i_groups};
  if contains(GroupName, 'RRRRRR')
    act_type = 'revolute';
    objscale = [1e-3, 180/pi]; % Moment in kNm; Gelenkgeschwindigkeit in deg/s
  else
    act_type = 'prismatic';
    objscale = [1e-3, 1]; % kN, m/s
  end
  % Einstellung zum Ausdünnen der Marker (sonst erkennt man nichts mehr)
  if strcmp(act_type, 'revolute')
    dx = 0.1;
    dy = 2;
  else
    dx = 0.3;
    dy = 0.05;
  end

  axhdl(i_groups) = subplot(1,3,i_groups);
  hold on;

  % Lade die Pareto-Front aus den vollständigen Versuchen aus dem Haupttext
  usr_figselection = sprintf('motordiagram'); % act_type
  pf_file = fullfile(datadir, sprintf('group_%s_paretofront_%s.mat', ...
    GroupName, usr_figselection));
  tmp = load(pf_file);
  pf_data = [tmp.pt_i.Crit1, tmp.pt_i.Crit2];
  Idom_ges = pareto_dominance(pf_data);
  if any(Idom_ges), warning('Partikel auf der Gesamt-Front %s werden dominiert', usr_figselection); end
  plot(objscale(1)*tmp.pt_i.Crit1, objscale(2)*tmp.pt_i.Crit2, 'g-');
  II = select_plot_indices_downsample_nonuniform( ...
    [objscale(1)*pf_data(:,1), objscale(2)*pf_data(:,2)], [], dx, dy);
  plot(objscale(1)*pf_data(II,1), objscale(2)*pf_data(II,2), 'go', 'MarkerSize', 5);

  for i_reps = 1:5 % Jede Wiederholung der Optimierung einzeln betrachten
    for i_meth = 1:2 % GA und PSO; s.o.
      % Daten laden und Pareto-Diagramm zeichnen
      meth = Methods{i_meth};
      usr_figselection = sprintf('motordiagram_%s_%s_%d', act_type, meth, i_reps);
      pf_file = fullfile(datadir, sprintf('group_%s_paretofront_%s.mat', ...
        GroupName, usr_figselection));
      if ~exist(pf_file, 'file')
        fprintf('Keine Daten %s. Vermutlich Opt. nicht erfolgreich.\n', pf_file);
        continue
      end
      fprintf('Lade %s\n', pf_file);
      tmp = load(pf_file);
      pf_data = [tmp.pt_i.Crit1, tmp.pt_i.Crit2];
      Idom_ges = pareto_dominance(pf_data);
      if any(Idom_ges), warning('Partikel auf der Front %s werden dominiert', usr_figselection); end
      II = select_plot_indices_downsample_nonuniform( ...
        [objscale(1)*pf_data(:,1), objscale(2)*pf_data(:,2)], [], dx, dy);
      if any(~II)
        fprintf('%d/%d Marker genommen. Der Rest ausgedünnt.\n', sum(II), length(II));
      end
      plot(objscale(1)*pf_data(II,1), objscale(2)*pf_data(II,2), ...
        [colors{i_meth}, markers{i_reps}], 'MarkerSize', 5);
    end
  end

  % Formatierung
  if strcmp(act_type, 'revolute')
    xlabel('Actuator torque in kNm');
    ylabel('Actuated-joint velocity in deg/s');
  else
    xlabel('Actuator force in kN');
    ylabel('Actuated-joint velocity in m/s');
  end
  title(TitleStrings{i_groups});
  grid on;
  % Zum Prüfen nach manueller Änderung der Grenzen:
  % get(axhdl(i_groups), 'xlim')
  % get(axhdl(i_groups), 'ylim')
  if i_groups == 1
    set(axhdl(i_groups), 'xlim', [1.3, 12.5]);
    set(axhdl(i_groups), 'ylim', [0.36, 0.56]);
  elseif i_groups == 2
    set(axhdl(i_groups), 'xlim', [0.62, 4.4]);
    set(axhdl(i_groups), 'ylim', [33.2, 108]);
  else
    set(axhdl(i_groups), 'xlim', [1.1, 12.6]);
    set(axhdl(i_groups), 'ylim', [0.4, 0.67]);
  end


end
% Formatieren und speichern
figure_format_publication(fighdl);

leglinhdl = NaN(3,1);
leglinhdl(1) = plot(NaN,NaN, 'r-');
leglinhdl(2) = plot(NaN,NaN, 'b-');
leglinhdl(3) = plot(NaN,NaN, 'g-');
legtxt = {'GA', 'PSO', 'Optimal'};
if exist('lf_colors_hdl', 'var'), delete(lf_colors_hdl); end
lf_colors_hdl = legendflex(leglinhdl, legtxt, 'anchor', {'ne','ne'}, ...
  'ref', axhdl(3), ... % an Figure ausrichten (mitten oben)
  'buffer', [-6 -6], ... % Kein Versatz notwendig, da mittig oben
  'ncol', 1, 'nrow', 3, ... % eine Zeile für Legende
  'fontsize', 10, ...
  'FontName', 'Times New Roman', ...
  'title', 'Color:', ...
  'xscale', 0.6, ...
  ... % Für matlab2tikz: [-12,-14,8] (kein zusätzlicher Abstand mehr. Latex macht ??.?? aus den PR-Nummern für die Bestimmung der Größe der Text-Box)
  'padding', [1,1,3], ...
  'box', 'on', ...
  'interpreter', 'none');


% return
set_size_plot_subplot(fighdl, ...
  18.3, 6.7, axhdl,...
  0.05,0.005,0.08,0.13,0.06,0); % lrud xy
% Schmalere Variante
% set_size_plot_subplot(fighdl, ...
%   18.3, 5, axhdl,...
%   0.05,0.005,0.08,0.16,0.06,0); % lrud xy

% Zoom-Fenster
axes(axhdl(1));
if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
if exist('boxhdl', 'var'), delete(boxhdl1); end
[zoomhdl1, boxhdl1] = add_zoom_axis(axhdl(1), [2, 5; 0.39, 0.45]);
set(zoomhdl1, 'position', [0.1457 0.5455 0.1635 0.3241]);
if exist('arrhdl1', 'var'), delete(arrhdl1); end
arrhdl1 = annotation(fighdl,'arrow');
set(arrhdl1, 'position', [0.2610 0.5134 -0.1006 -0.0787]);

axes(axhdl(2));
if exist('zoomhdl2', 'var'), delete(zoomhdl2); end
if exist('boxhdl', 'var'), delete(boxhdl2); end
[zoomhdl2, boxhdl2] = add_zoom_axis(axhdl(2), [0.8, 2; 40, 70]);
set(zoomhdl2, 'position', [0.5072 0.4387 0.1393 0.4626]);
if exist('arrhdl2', 'var'), delete(arrhdl2); end
arrhdl2 = annotation(fighdl,'arrow');
set(arrhdl2, 'position', [0.6156 0.4190 -0.1127 -0.0553]);

ZOrderSet(lf_colors_hdl, -100); % nach vorne bringen (nicht im Hintergrund)


name = 'lufipkm_compare_GA_vs_PSO';
filename = fullfile(outputdir, sprintf('%s.pdf', name));
return
exportgraphics(fighdl, filename,'ContentType','vector');
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));
