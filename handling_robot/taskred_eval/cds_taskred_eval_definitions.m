% Definitionen für die Auswertung der Aufgabenredundanz-Einstellungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function defstruct = cds_taskred_eval_definitions()

defstruct_cds = cds_definitions();
objective_ik_cases = defstruct_cds.objective_ik_names_all;
% Für Legende
% defstruct.objective_ik_cases_leg = {'Std.', 'IK-Jac.', 'Jac.', ...
%   'Coll.', 'InstSpc.', 'No Obj.', 'Const. Rot.'};
objective_ik_cases_leg = objective_ik_cases;
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'default', 'Heuristic');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'ikjac_cond', 'IK Jacobian condition');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'jac_cond', 'Jacobian condition');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'coll_par', 'Collision');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'instspc_par', 'Installation space');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'poserr_ee', 'Position error');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'none', 'None');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'constant', 'Constant');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'maxactvelo', 'Actuator velocity');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'maxactforce', 'Actuator force');

default_markersize = 6; % 6pt (Matlab-Standard)
default_linewidth = 1.0; % 0.5pt ist Matlab-Standard. Nehme dickere Linien für Diss
imescolors = imes_plot_template_colors();
format = {imescolors.imesblau,  '+', '-', 7, default_linewidth, default_markersize, 0.05; ...
          imescolors.blau, 'd', '-', 10, default_linewidth, default_markersize, 0.02; ...
          imescolors.rot, 's', '-', 5, default_linewidth, default_markersize, 0.03; ...
          'k', '^', '-', 15, default_linewidth, default_markersize, 0.04; ...
          'c', 'p',  ':', 25, default_linewidth, default_markersize, 0.0; ...
          'b', 'x', '--', 18, default_linewidth, default_markersize, 0.01; ... % Marker für Positionsfehler (sehr ähnlich zu Jacobi-Matrix, daher gestrichelt darüber)
          'g', 'v', ':', 12, default_linewidth, default_markersize, 0.06; ...
          imescolors.gruen, '<', '--', 9, default_linewidth, default_markersize, 0.07; ...
          imescolors.schwarz, '>', '--', 15, default_linewidth, default_markersize, 0.08; ...
          imescolors.cyan, 'p',  '-', 10, default_linewidth, default_markersize, 0.09; ...
          'b', 'o', '--', 18, default_linewidth, default_markersize, 0.10};
% colors = {'r+', 'bd', 'ms', 'k^', 'cp', 'bx', 'gv', 'r<', 'k>', 'rp', 'bo'};
% styles = {'r+', 'bd', 'ms', 'k^', 'cp', 'bx', 'gv', 'r<', 'k>', 'rp', 'bo'};
% lines =  {'-',   '-',  '-',  '-',  '-',  '-',  ':', '--', '--',  ':', '--'};

defstruct = struct( ...
  'objective_ik_cases_leg', {objective_ik_cases_leg}, ...
  'format', {format});
