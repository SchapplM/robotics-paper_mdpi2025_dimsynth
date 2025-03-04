% Zeichne die Marker in jeweils ein eigenes Bild zum Einfügen in Tabelle

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_figselection = 'power_vs_coll'; % siehe andere Skripte

paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir2 = fullfile(paper_dir, 'paper', 'Figures', 'robots_lufipkm');

repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');
tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)));
RobotGroups = tmp.RobotGroups;

outputdir1 = fullfile(paper_dir, 'paper', 'Figures');
InfoTab = readtable(fullfile(outputdir1, ...
  sprintf('lufipkm_groups_legend_%s_add.csv', usr_figselection)), 'Delimiter', ';');

for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  fighdl = change_current_figure(1000);clf;
  if RobotGroups.ResultsFound(i) == 0, continue; end
  plot(1,1,RobotGroups.PlotMarker{i});
  figure_format_publication(gca);
  set(gcf, 'color', 'none'); % transparent. Funktioniert nicht in pdf.
  set(gca, 'Box', 'off');
  set(gca, 'XTICK', [], 'YTICK', []);
  set(get(gca, 'XAXIS'), 'visible', 'off');
  set(get(gca, 'YAXIS'), 'visible', 'off');
  set_size_plot_subplot(fighdl, ...
    1,1,gca,0,0,0,0,0,0)

  i_IT = strcmp(InfoTab.GroupName, GroupName);
  assert(any(i_IT), 'Gruppe nicht in InfoTab gefunden');
  [tokens, match] = regexp(InfoTab.TextFix{i_IT}, ...
    'PR (\d+)\.(\d+) .*', 'tokens', 'match');
  filename = sprintf('RobotFig_PR_%s_%s_Group_%s_marker', tokens{1}{1}, ...
    tokens{1}{2}, GroupName);
  export_fig(fullfile(outputdir2, [filename, '.pdf']));
  cd(outputdir2);
  exportgraphics(fighdl,[filename, '.png'],'Resolution', 600);
end

fprintf('Marker nach %s exportiert\n', outputdir2);
