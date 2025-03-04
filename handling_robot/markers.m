% Zeichne die Marker in jeweils ein eigenes Bild zum Einfügen in Tabelle

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_figselection = 'default'; % Auswertung in der alle Marker konsistent zu den Diss-Bildern enthalten sind

%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir2 = fullfile(paper_dir, 'paper', 'Figures', 'robots_handlingpkm');
mkdirs(outputdir2);
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');
tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)));
RobotGroups = tmp.RobotGroups;

outputdir1 = fullfile(paper_dir, 'paper', 'Figures');
InfoTab = readtable(fullfile(outputdir1, ...
  sprintf('handlingpkm_groups_legend_%s.csv', usr_figselection)), 'Delimiter', ';');

for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  fighdl = change_current_figure(1000);clf;
  if RobotGroups.ResultsFound(i) == 0, continue; end
  fprintf('Gruppe %d (%s): Erstelle Marker\n', i, GroupName)
  plot(1,1,RobotGroups.PlotMarker{i}, 'Color', RobotGroups.PlotColor{i});
  figure_format_publication(gca);
  set(gcf, 'color', 'none'); % transparent. Funktioniert nicht in pdf.
  set(gca, 'Box', 'off');
  set(gca, 'XTICK', [], 'YTICK', []);
  set(get(gca, 'XAXIS'), 'visible', 'off');
  set(get(gca, 'YAXIS'), 'visible', 'off');
  set_size_plot_subplot(fighdl, ...
    1,1,gca,0,0,0,0,0,0)

  i_IT = strcmp(InfoTab.GroupName, GroupName);
  if ~any(i_IT)
    warning('Gruppe nicht in InfoTab gefunden');
    continue
  end
  if GroupName(2) == '3'
    xTyR_str = '3T0R';
  else
    xTyR_str = '3T1R';
  end
  filename = sprintf('RobotFig_%s_PR_%d_%d_Group_%s_marker', xTyR_str, ...
    InfoTab.NumberMain(i_IT), InfoTab.NumberVar(i_IT), GroupName);
  cd(outputdir2);
%   export_fig([filename, '.pdf']);
  exportgraphics(fighdl, [filename, '.png'],'Resolution', 600);
end

fprintf('Marker nach %s exportiert\n', outputdir2);
