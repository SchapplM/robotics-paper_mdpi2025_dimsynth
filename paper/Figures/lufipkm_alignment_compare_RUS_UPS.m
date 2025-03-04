% Erzeuge ein gemeinsames Bild zum Vergleich der Leistung von RUS und PUS
% Bezüglich der verschiedenen Koppelgelenk-Varianten
% 
% Vorher ausführen: eval_figures_pareto.m (aus LuFI-Skriptumgebung)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

if isempty(which('lufi_dimsynth_data_dir'))
  error(['You have to create a file lufi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end

%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');

fighdl = change_current_figure(1);clf;
CellFigfiles = cell(1,3);
filenames = { ...
  'lufipkm_pareto_power_mass_allvar_power_vs_mass_P6RRPRRR14V6_nolegend.fig', ...
  'lufipkm_pareto_power_mass_allvar_power_vs_mass_P6RRRRRR10V6_nolegend.fig', ...
  'lufipkm_pareto_power_mass_allvar_power_vs_mass_P6PRRRRR6V4_nolegend.fig'};
for i = 1:length(filenames)
  CellFigfiles{i} = fullfile(outputdir, filenames{i});
end
axhdl = set_fig2subfig(fighdl, CellFigfiles);

figure_format_publication();
set_size_plot_subplot(fighdl, ...
  18.3, 6.7, axhdl,...
  0.04,0.007,0.28,0.13,0.04,0); % lrud xy

% Manuelle Anpassung
set(axhdl(1), 'xlim', [0.5, 5]);
set(axhdl(2), 'xlim', [0.8, 6]);
set(axhdl(1), 'title', text('String', sprintf('Hexapod (6-UPS)')));% (6-UPS, PR %d.%d)', ...
  % PKM_Table_Diss.NumberMain(RobIdx_UPS), PKM_Table_Diss.NumberVar(RobIdx_UPS)), 'FontName', 'Times'));
set(axhdl(2), 'title', text('String', sprintf('Hexa (6-RUS)')));%, PR %d.%d)', ...
  % PKM_Table_Diss.NumberMain(RobIdx_RUS), PKM_Table_Diss.NumberVar(RobIdx_RUS)), 'FontName', 'Times'));
set(axhdl(3), 'title', text('String', sprintf('HexaSlide (6-PUS)')));%, PR %d.%d)', ...
  % PKM_Table_Diss.NumberMain(RobIdx_PUS), PKM_Table_Diss.NumberVar(RobIdx_PUS)), 'FontName', 'Times'));

% Übertrage die Legende
uiopen(fullfile(outputdir, 'lufipkm_pareto_power_mass_allvar_power_vs_mass_P6RRPRRR14V6.fig'), 1);
fighdl_w_legends = gcf;
axhdl_with_legends = get(fighdl_w_legends, 'Children');
colorlegendhdl = copyobj(axhdl_with_legends(1), fighdl);
markerlegendhdl = copyobj(axhdl_with_legends(2), fighdl);
close(fighdl_w_legends);

set(markerlegendhdl, 'Units', 'normalized');
set(markerlegendhdl, 'Position', [0.0198 0.79 0.9690 0.209]);
set(colorlegendhdl, 'Units', 'normalized');
set(colorlegendhdl, 'Position', [0.4593 0.5423 0.2581 0.1527]);

% x-Achsbeschriftungen entfernen (nicht die Ticks)
for jth = 2:length(axhdl)
  ylabel(axhdl(jth), '');
end

% text with subfig number (a/b/c) (place via latex instead)
% thdl = NaN(length(axhdl),1);
% for jth = 1:length(axhdl)
%   axes(axhdl(jth)); %#ok<LAXES>
%   thdl(jth) = text(0,0,sprintf('(%s)', char(96+jth)));
% end
% for jth = 1:length(axhdl)
%   [~, ~, x_pos] = get_relative_position_in_axes(axhdl(jth), 'x', -1.23);
%   [~, ~, y_pos] = get_relative_position_in_axes(axhdl(jth), 'y', -1.29);
%   set(thdl(jth), 'Position', [x_pos, y_pos, 0]);
%   set(thdl(jth), 'FontWeight', 'bold');
%   set(thdl(jth), 'FontName', 'Times');
% end

drawnow(); pause(0.5); % Befehl funktioniert sonst nicht...
ZOrderSet(colorlegendhdl, -100); % nach vorne bringen (nicht im Hintergrund)

name = 'lufipkm_alignment_compare_RUS_UPS';
filename = fullfile(outputdir, sprintf('%s.pdf', name));
% export_fig(fighdl, filename); % Subplot-Boxen sind hiermit nicht überall durchgezeichnet
exportgraphics(fighdl, filename,'ContentType','vector');
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));
fprintf('Bild %s nach %s gespeichert\n', name, outputdir);
