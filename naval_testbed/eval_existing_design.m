% Auswertung des aktuellen Entwurfs zum Vergleich mit der Maßsynthese
% (LuFI-PKM)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));
%% Lade Roboterdefinition (aus anderem Versuch, zur Vereinfachung)
if isempty(repo_dir)
  error(['You have to create a file lufi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = lufi_dimsynth_data_dir();
% Hole aus finalem Experiment aus MA Fettin
% OptName = '20220908_lufipkm_[V16]_process+workspace';
% LfdNr = 1;
% RobName = 'P6RRPRRR14V4G7P4A1';
% Alternativ: Hole aus einem aktuellen Versuch, damit die Ergebnisse
% vergleichbar sind
OptName = 'lufipkm_20230330_Diss_processworkspace_allrob';
LfdNr = 2166;
RobName = 'P6RRPRRR14V6G7P4A1';
parroblib_update_template_functions({RobName});

setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
Set_i = cds_settings_update(d1.Set);
Set_i.structures.prismatic_cylinder_no_lever = true; % die Parameter sollen null gesetzt sein
Set_i.optimization.obj_power.symmetric_speed_torque_limits = 1;
resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', ...
  LfdNr, RobName));
tmp = load(resfile);
Structure = tmp.RobotOptRes.Structure;

% parroblib_update_template_functions({RobName}, 0);
% parroblib_addtopath({RobName});
[R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, Structure, true);

%% Vektor der Optimierungsvariablen
% Vektor der Optimierungsvariablen (Synthese Ergebnis)
pval_syntRes = [4.0403; % scale
                1.5708; % pkin 3: alpha3
                1.5278; % pkin 4: alpha4
               -0.3079; % pkin 7: theta3
                2.5000; % base z
                2.3462; % ee rot 3
                2.3405; % baserotation z
                0.6696; % base radius
                0.1854; % platform radius
                0.2324; % base_morph_pairdist
                1.4845]; % platform_morph_pairdist
% Vektor der Optimierungsvariablen (Realisierte Lösung)
pval_actualDesign = [4.0403; % scale
                    1.5708; % pkin 3: alpha3
                    1.5278; % pkin 4: alpha4
                   -0.3079; % pkin 7: theta3
                    2.5000; % base z
                    pi; % ee rot 3
                    pi; % baserotation z
                    0.6696; % base radius
                    0.1854; % platform radius
                    0.2324; % base_morph_pairdist
                    1.4845]; % platform_morph_pairdist
%% Ersetze die Parameter
vn = Structure.varnames;
pval = NaN(length(vn),1);
% Skalierung der Parameter ist hier egal, da keine Parameter damit beein-
% flusst werden (Radius ist absoluter Wert, da Grenzen gesetzt sind)
% pval(strcmp(vn, 'scale')) = 1;
pval(strcmp(vn, 'scale')) = 4.0403; % RobotOptRes.p_val_pareto(PNr,1)
% pval(strcmp(vn, 'pkin 1: a3')) = R_ref.Leg(1).MDH.a(3);
% pval(strcmp(vn, 'pkin 2: a4')) = R_ref.Leg(1).MDH.a(4);
% Kinematikparameter der Beinkette: Siehe MA Fettin, Tab. B.4 (S. 112)
pval(contains(vn, 'alpha3')) = pi/2;%R_ref.Leg(1).MDH.alpha(3);
pval(contains(vn, 'alpha4')) = 1.5278;%R_ref.Leg(1).MDH.alpha(4);
% pval(strcmp(vn, 'pkin 7: d3')) = R_ref.Leg(1).MDH.d(3);
% pval(strcmp(vn, 'pkin 8: d5')) = R_ref.Leg(1).MDH.d(5);
pval(contains(vn, 'theta3')) = -0.3079;%R_ref.Leg(1).MDH.theta(3);
% EE-Transformation: Siehe MA Fettin, Gl. B.40 (S. 113)
pval(strcmp(vn, 'ee rot 3')) = pi;%[0,0,1]*r2eulxyz(R_ref.T_P_E(1:3,1:3));
% Gestellhöhe: Siehe MA Fettin, Gl. C.1 (S. 116)
pval(strcmp(vn, 'base z')) = 2.5;%R_ref.T_W_0(3,4);
% Basis-Transformation: Siehe MA Fettin, Gl. B.26 (S. 110) (Dort noch anders gedreht)
% Richtig ist: Basis-Transformation: Siehe MA Fettin, Gl. C.1 (S. 116)
pval(strcmp(vn, 'baserotation z')) = pi;%[0,0,1]*r2eulxyz(R_ref.T_W_0(1:3,1:3));
% Gestellradius: Siehe MA Fettin, Gl. B.27 (S. 110) (TODO: Ist das wirklich der Radius? JF: Ja)
pval(strcmp(vn, 'base radius')) = 0.6696;%R_ref.DesPar.base_par(1);
% Plattformradius: Siehe MA Fettin, Gl. B.28 (S. 110) (TODO: Ist das wirklich der Radius? JF: Ja)
pval(strcmp(vn, 'platform radius')) = 0.1854;%R_ref.DesPar.platform_par(1);
% TODO: Ist das wirklich der finale Entwurf? JF: Ja
pval(strcmp(vn, 'base_morph_pairdist')) = 0.2324;%R_ref.DesPar.base_par(2)/R_ref.DesPar.base_par(1);
pval(strcmp(vn, 'platform_morph_pairdist')) = 1.4845;%R_ref.DesPar.platform_par(2)/R_ref.DesPar.platform_par(1);

assert(all(length(pval)==[length(Structure.vartypes),length(Structure.varnames)]), ...
  'Inkonsistente Variablenanzahl');

assert(all(~isnan(pval)), 'p darf nicht NaN sein');
pval_phys_test = cds_update_robot_parameters(R, Set_i, Structure, pval);

% Vergleiche mit Endergebnis aus MA Fettin:
% Fettin_Jannik_MA\lufi-pkm_simulink\Bibliotheken\lufi-pkm_model'
ref_file = fullfile(repo_dir, 'data', 'Roboterklasse_MA_Fettin.mat');
if exist(ref_file, 'file')
  tmp2=load(ref_file);
  R_ref = tmp2.R;
  assert(all(abs(R.T_W_0(:)-R_ref.T_W_0(:))<1e-6), 'Basis-Transformation ist anders');
  % assert(strcmp(R.mdlname,R_ref.mdlname), 'Falsches Roboter-Modell');
  assert(abs(R.DesPar.base_par(1)-R_ref.DesPar.base_par(1))<1e-4, 'base radius falsch eingetragen');
  assert(abs(R.DesPar.base_par(2)-R_ref.DesPar.base_par(2))<1e-4, 'base_morph_pairdist falsch eingetragen');
  assert(all(abs(R.r_P_B_all(:)-R_ref.r_P_B_all(:))<1e-4), 'Plattform-Koppelpunkte sind anders')
  assert(all(abs(R.phi_P_B_all(:)-R_ref.phi_P_B_all(:))<1e-4), 'Plattform-Koppelgelenk-Ausrichtungen sind anders')
  assert(all(abs(R.Leg(1).T_W_0(:)-R_ref.Leg(1).T_W_0(:))<1e-4), 'Gestell-Koppelgelenk-Ausrichtungen sind anders')
else
  fprintf('Keine Prüfung gegen Ergebnis aus MA Fettin. Wurde aber vorher schon gemacht.');
end
%% Berechne die Zielfunktion. Dadurch Detail-Kennzahlen zur Kinematik
Set = Set_i;
Set.general.plot_details_in_fitness = 1e11;
Set.general.plot_robot_in_fitness = 1e11;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig', 'png'};
% Ordner für Speicherung von tmp-Bildern erzeugen.
Set.optimization.resdir = fullfile(repo_dir, 'data', 'existing_design');
[~,~,~,resdir] = cds_get_new_figure_filenumber(Set, Structure, '');
mkdirs(resdir);
fprintf('Speichere Bilder für Auswertung in %s\n', resdir);

Set.optimization.pos_ik_abort_on_success = true;
Set.optimization.traj_ik_abort_on_success = true; % Sofort aufhören

% defstruct = cds_definitions();
Set.optimization.objective = {'power', 'mass', 'colldist', 'actforce', 'actvelo', 'condition'}; % Berechne alle Kriterien aus den Plots
Set.optimization.obj_limit = zeros(length(Set.optimization.objective),1);
Set.optimization.obj_limit_physval = zeros(length(Set.optimization.objective),1);
% Set.optimization.collision_bodies_size = 20e-3;
cds_log(0, '', 'init', Set);
cds_save_particle_details(); cds_fitness();
[fval, physval, Q] = cds_fitness(R, Set,d1.Traj, Structure, pval);
PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');

% Berücksichtige neue Implementierung des Motorleistungs-Kritieriums
% (die Ergebnisse wurden noch mit alter Implementierung generiert)
% ToDo: Muss geändert werden, wenn Maßsynthese neu ausgeführt wird.
physval(strcmp(Set.optimization.objective, 'power')) = physval(strcmp(Set.optimization.objective, 'power'))/R.NLEG;

%% Validiere die Ergebnisse
fprintf('Daten des Roboters:\n');
fprintf('Gestell-Radius: %1.3fmm\n', 1e3*R.DesPar.base_par(1));
fprintf('Plattform-Radius: %1.3fmm\n', 1e3*R.DesPar.platform_par(1));

%% Berechne zusätzliche Daten und speichere die Ergebnisse
% Siehe select_eval_robot_examples.m (dort das gleiche und kommentiert)
condJ = PSO_Detail_Data_tmp.constraint_obj_val(1, 4, 1);  
Traj_0 = cds_transform_traj(R, d1.Traj);
X = Traj_0.X;
objective_names = Set.optimization.objective;
save(fullfile(repo_dir, 'data', sprintf('detail_result_engineering_solution.mat')), ...
  'R', 'pval', 'fval', 'physval', 'condJ', 'Structure', 'Q', 'X', ... % Daten zum Ergebnis
  'objective_names', 'Set');
