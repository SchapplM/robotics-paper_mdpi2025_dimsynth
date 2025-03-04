% Kombiniere zwei Pareto-Bilder zu einem Bild (Achsen verknüpfen)
% 
% Vorher ausführen: eval_figures_pareto_groups.m (aus LuFI-Skriptumgebung)
% (Anpassung an MDPI-Paper)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

%% Initialisierung
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');

%% Daten laden
fighdl = change_current_figure(1);clf;
CellFigfiles = cell(2,1);
filenames = {'lufipkm_pareto_power_mass_groups_power_vs_mass.fig', ...
  'lufipkm_pareto_power_colldist_groups_power_vs_coll_nolegend.fig'};
for i = 1:length(filenames)
  CellFigfiles{i} = fullfile(outputdir, filenames{i});
end
[axhdl, other_hdl] = set_fig2subfig(fighdl, CellFigfiles);

%% Formatieren
figure_format_publication();
set_size_plot_subplot(fighdl, ...
  18.3, 15, axhdl,... 
  0.05,0.009,  0.25,0.06,  0.06,0.02); % lrud xy

linkaxes(axhdl, 'x' );
remove_inner_labels(axhdl, 1);
% Achsbeschriftungen bündig machen
yl1 = get(axhdl(1), 'ylabel');
yl1p = get(yl1, 'position');
yl2 = get(axhdl(2), 'ylabel');
yl2p = get(yl2, 'position');
set(yl2, 'position', [yl1p(1), yl2p(2), 0]);

% Korrigiere die Größen der Zoom-Fenster wieder
zoomhdl_1 = other_hdl(1);
zoomhdl_2 = other_hdl(3);

% Nach dem händischen Korrigieren der Position der Zoom-Fenster: get(zoomhdl_2, 'position')
set(zoomhdl_1, 'position', [0.4864    0.4666    0.4942    0.2399]);
set(zoomhdl_2, 'position', [0.5785    0.0655    0.3934    0.2733]);

% Korrigiere die Legende wieder
leghdl = other_hdl(2);
set(leghdl, 'units', 'normalized');
% get(leghdl, 'position')
set(leghdl, 'position',  [0.0697    0.7532    0.8895    0.2451]);
% Zeichne Pfeile neu ein (werden nicht mit kopiert)
arrhdl1 = annotation(fighdl, 'arrow');
set(arrhdl1, 'position', [0.4627 0.5415 -0.2183 -0.0335]);
arrhdl2 = annotation(fighdl,'arrow');
set(arrhdl2, 'position', [0.5668 0.2928 -0.3065 0]);

%% Speichern
name = 'lufipkm_pareto_3obj_combined';
exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)), 'ContentType','vector');
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));

fprintf('Bild gespeichert: %s\n', name);
