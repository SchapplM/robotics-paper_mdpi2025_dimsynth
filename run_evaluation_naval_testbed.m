% Führe alle Skripte zur Auswertung der LuFI-Maßsynthese aus.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
addpath(fullfile(fileparts(which('run_evaluation_naval_testbed.m')), ...
  'naval_testbed'));
run(fullfile(fileparts(which('lufi_dimsynth_data_dir.m')), ...
  'eval_figures_pareto.m'));
run(fullfile(fileparts(which('lufi_dimsynth_data_dir.m')), ...
  'eval_existing_design.m'));
run(fullfile(fileparts(which('lufi_dimsynth_data_dir.m')), ...
  'eval_figures_pareto_groups.m'));
run(fullfile(fileparts(which('lufi_dimsynth_data_dir.m')), ...
  'select_eval_robot_examples.m'));
run(fullfile(fileparts(which('lufi_dimsynth_data_dir.m')), ...
  'robot_images.m'));
run(fullfile(fileparts(which('lufi_dimsynth_data_dir.m')), ...
  'markers.m'));
run(fullfile(fileparts(which('lufi_dimsynth_data_dir.m')), ...
  'results_tables_latex.m'));

% Create figures for the paper
repo_dir = fileparts(which('run_evaluation_naval_testbed.m'));
run(fullfile(repo_dir, 'paper', 'Figures', ...
  'lufipkm_alignment_compare_RUS_UPS.m'));
repo_dir = fileparts(which('run_evaluation_naval_testbed.m'));
run(fullfile(repo_dir, 'paper', 'Figures', ...
  'lufipkm_pareto_3obj_combined.m'));
repo_dir = fileparts(which('run_evaluation_naval_testbed.m'));
run(fullfile(repo_dir, 'paper', 'Figures', ...
  'calctime_vs_fval.m'));