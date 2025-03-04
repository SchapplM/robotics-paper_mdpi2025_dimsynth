% Wähle Roboter aus den Ergebnissen aus für die Detail-Untersuchung
% (Detail-Bilder und Tabelle).
% Beeinflusst robot_images.m
% Rechne die Fitness-Funktion nach und speichere alle relevanten Daten ab.
% Der Roboter wird aus der Pareto-Front genommen, entsprechend der
% Einstellung.
%
% Bearbeitet für Dissertation
% 
% This script is based on the same file eval_figures_pareto_groups.m from 
% https://github.com/SchapplM/robotics-paper_ark2022_3T1R/
% (ARK paper "Inverse Kinematics for Task Redundancy of Symmetric 3T1R
% Parallel Manipulators using Tait-Bryan-Angle Kinematic Constraints",
% Schappler 2022) 

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
usr_fig_all = {'default_revolute', 'default_prismatic'};
for i_usr_fig = 1:2
usr_figselection = usr_fig_all{i_usr_fig}; % siehe andere Skripte.
%% Benutzereingaben
usr_recalc_fitnessfcn = true; % Neuberechnung der Fitness-Funktion (optional)
% Compute redundancy map and save it next to the results.
usr_recreate_redundancy_map = true;
usr_objective_ik = 'maxactforce';
regenerate_templates = false; %#ok<*UNRCH> % nur bei erstem Aufruf notwendig.
%% Sonstige Initialisierung
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir.m'));
if isempty(repo_dir)
  error(['You have to create a file HandlingRobot_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
importdir = HandlingRobot_dimsynth_data_dir();
datadir = fullfile(repo_dir, 'data');
tmp = load(fullfile(datadir, sprintf('results_all_reps_pareto_default.mat')));
ResTab = tmp.ResTab_ges;
tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)));
RobotGroups = tmp.RobotGroups;
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');
% Legende aus Pareto-Bild laden und mit Zusatz-Info erweitern
InfoTab = readtable(fullfile(outputdir, sprintf('handlingpkm_groups_legend_%s.csv', ...
  usr_figselection)), 'Delimiter', ';');
for i = 1:2, InfoTab = addvars(InfoTab, repmat({'TODO'},size(InfoTab,1),1) ); end
InfoTab.Properties.VariableNames{end-1} = 'CouplingOnly';
InfoTab.Properties.VariableNames{end} = 'TextCoupling';

%% Suche besten Wert für beide Kriterien
bestphysvals = NaN(1,2);
clear data_all
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  if RobotGroups.ResultsFound(i) == 0, continue; end % keine Ergebnisse vorliegend
  if contains(GroupName(2:end), 'P') && contains(usr_figselection, 'revolute') || ...
    ~contains(GroupName(2:end), 'P') && contains(usr_figselection, 'prismatic')
    continue % Gruppe wird hier nicht betrachtet.
  end
  pf_file = fullfile(datadir, sprintf('group_%s_paretofront_%s.mat', GroupName, usr_figselection));
  if ~exist(pf_file, 'file')
    warning('Pareto-Daten existieren nicht: %s', pf_file);
    continue
  end
  data_i = load(pf_file);
  if ~exist('data_all', 'var')
    data_all = data_i.pt_i;
  else
    data_all = [data_all; data_i.pt_i]; %#ok<AGROW> 
  end
end
data_phys = [data_all.Crit1, data_all.Crit2];
for k = 1:2
  [bestphysvals(k), Imin] = min(data_phys(:,k));
end
if contains(usr_figselection, 'default')
  fprintf(['Beste physikalische Werte für %s: actforce: %1.3f, ' ...
    'installspace: %1.3f\n'], usr_figselection, bestphysvals(1), bestphysvals(2));
else
  error('Nicht definiert');
end

%% Alle Gruppen durchgehen
num_groups_calc = 0;
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
%   if strcmp(GroupName(2), '3'), continue; end % Keine 3T0R-PKM (für Reproduktion der Redundanzkarten)
%   if ~strcmp(GroupName,'P5PRRRR6V1G'), continue; end % Debug
  if RobotGroups.ResultsFound(i) == 0, continue; end % keine Ergebnisse vorliegend
  if contains(GroupName(2:end), 'P') && contains(usr_figselection, 'revolute') || ...
    ~contains(GroupName(2:end), 'P') && contains(usr_figselection, 'prismatic')
    continue % Gruppe wird hier nicht betrachtet.
  end
%   if ~strcmp(GroupName, 'P4RPRRR8V2G'), continue; end % Debug
  fprintf('Lade Daten für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  pf_file = fullfile(datadir, sprintf('group_%s_paretofront_%s.mat', GroupName, usr_figselection));
  if ~exist(pf_file, 'file')
    warning('Pareto-Daten existieren nicht: %s', pf_file);
    continue
  end
  data_i = load(pf_file);
  % Manuelle Anpassung der zu findenden Roboter
  II = (1:size(data_i.pt_i,1))';
  % Falls ein Wert negativ ist, benutze den besten Wert als Offset
  if bestphysvals(2)<0
    data2normbest = (data_i.pt_i.Crit2+2*abs(bestphysvals(2)))/abs(bestphysvals(2));
  else
    data2normbest = data_i.pt_i.Crit2/bestphysvals(2);
  end

  % Wähle das Partikel, dass nach einer gewichteten Summe aus beiden
  % Kriterien am besten ist (jeweils relativ zum bestmöglichen Wert)
  data_i_rel = [data_i.pt_i.Crit1/bestphysvals(1), data2normbest];
  assert(all(data_i_rel(:)>=1), 'normierte Daten müssen größer 1 sein');
  if strcmp(usr_figselection, 'default')
    % Gehe fast nur auf die Antriebskraft ein
    [~,I_sort] = sort(0.90*data_i_rel(:,1)+0.10*data_i_rel(:,2), 'ascend');
  else % gleiche Gewichtung
    [~,I_sort] = sort(0.5*data_i_rel(:,1)+0.5*data_i_rel(:,2), 'ascend');
  end
  group_i_success = false;
  for iinearest = I_sort' % Gehe alle Partikel durch und nehme das passendste, was reproduzierbar ist
  inearest = II(iinearest);
  % Lade Daten für diesen Roboter aus den Ergebnissen
  Ipar = data_i.pt_i.ParetoIndNr(inearest);
  OptName = data_i.pt_i.OptName{inearest};
  RobName = data_i.pt_i.RobName{inearest};
  LfdNr = data_i.pt_i.LfdNr(inearest);
  fprintf('Wähle Opt. %s, Rob. %d, %s, Partikel %d\n', OptName, LfdNr, RobName, Ipar);
  setfile = dir(fullfile(importdir, OptName, '*settings.mat'));
  d1 = load(fullfile(importdir, OptName, setfile(1).name));
  Set_i = cds_settings_update(d1.Set);
  resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  RobotOptRes_i = tmp.RobotOptRes;
  resfile2 = fullfile(importdir, OptName, sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
  if exist(resfile2,'file')
    % Lade auch die Detail-Ergebnisse. Geht nicht ohne, wenn die
    % Detail-Ergebnisse genutzt wurden, um die Pareto-Front anzupassen.
    % (z.B. durch nachträgliche Filterung nach zusätzlichen Kriterien
    tmp = load(resfile2);
    PSO_Detail_Data_i = tmp.PSO_Detail_Data;
    RobotOptDetails_i = tmp.RobotOptDetails;
  else
    PSO_Detail_Data_i = [];
    RobotOptDetails_i = [];
  end
  % Nummer der Zielfunktionen
  if contains(usr_figselection, 'default')
    kk1 = find(strcmp(Set_i.optimization.objective, 'actforce'));
    kk2 = find(strcmp(Set_i.optimization.objective, 'installspace'));
  else
    error('Fall nicht definiert');
  end
  if isempty(kk1) || isempty(kk2)
    error('Falsche Kriterien gewählt');
  end
  % Prüfe, ob Werte zueinander passen. Kann nicht mehr direkt die
  % Pareto-Fronten nehmen, da die Pareto-Fronten neu gebildet werden. Es
  % muss immer das passende Partikel in den Zwischenergebnissen gesucht
  % werden.
  if ~isnan(Ipar) && abs(RobotOptRes_i.physval_pareto(Ipar,kk1) - data_i.pt_i.Crit1(inearest)) < 1e-3
    % Der Gesuchte Wert liegt ganz normal auf der Pareto-Front aus dem
    % Optimierungsergebnis (mit viel Toleranz)
    % Lade weitere Daten aus der Ergebnis-Datei (aus Endergebnis-Variable)
    pval_in = RobotOptRes_i.p_val_pareto(Ipar,:)';
    physval_in = RobotOptRes_i.physval_pareto(Ipar,:)';
    fval_in = RobotOptRes_i.fval_pareto(Ipar,:)';
    pval_desopt = RobotOptRes_i.desopt_pval_pareto(Ipar,:)';
    q0 = RobotOptRes_i.q0_pareto(Ipar,:)';
  elseif ~isempty(PSO_Detail_Data_i)
    % Suche in den Detail-Daten. Nehme beide Kriterien, damit die Daten
    % eindeutig sind. Ansonsten können mehrere Parametersätze teilweise die
    % gleichen Zielkriterien erzeugen
    PosErrMatrix = reshape(PSO_Detail_Data_i.physval(:,kk1,:), ...
      size(PSO_Detail_Data_i.physval,1), size(PSO_Detail_Data_i.physval,3));
    ActForceMatrix = reshape(PSO_Detail_Data_i.physval(:,kk2,:), ...
      size(PSO_Detail_Data_i.physval,1), size(PSO_Detail_Data_i.physval,3));
    k = find( ...
      abs(PosErrMatrix(:)  -data_i.pt_i.PosAcc(inearest))<1e-10 & ...
      abs(ActForceMatrix(:)-data_i.pt_i.ActForce(inearest))<1e-10);
    if length(k) > 1 % Es gibt mehrere Partikel mit genau diesen Werten für die Zielkriterien
      % Prüfe, ob wenigstens die Parameter unterschiedlich sind.
      pval_all = NaN(length(k), size(PSO_Detail_Data_i.pval,2));
      for ii = 1:length(k)
        [ii_ind,ii_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k(ii));
        pval_all(ii,:) = PSO_Detail_Data_i.pval(ii_ind,:,ii_gen);
      end
      if size(unique(pval_all,'rows'), 1) == 1
        k = k(1); % sind unterschiedlich. Also identische Roboter.
      else % nicht unterschiedlich. Prinzipielles Problem (redundante Parameter)
        error('Suchkriterium in Daten ist nicht eindeutig');
      end
    end
    [k_ind,k_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k);
    physval_in = PSO_Detail_Data_i.physval(k_ind,:,k_gen)';
    if abs(physval_in(kk1)-data_i.pt_i.PosAcc(inearest))>1e-10
      error('Gesuchter Wert konnte nicht gefunden werden. Logik-Fehler');
    end
    fval_in = PSO_Detail_Data_i.fval(k_ind,:,k_gen)';
    pval_in = PSO_Detail_Data_i.pval(k_ind,:,k_gen)';
    pval_desopt = PSO_Detail_Data_i.desopt_pval(k_ind,:,k_gen)';
    q0 = PSO_Detail_Data_i.q0_ik(k_ind,:,k_gen)';
  else
    error(['Ergebnis-Partikel liegt nicht in der finalen Pareto-Front ', ...
      'der Optimierung. Detail-Auswertung notwendig. Datei aber nicht da: %s'], ...
      resfile2);
  end

  if ~isempty(PSO_Detail_Data_i)
    % Lese die IK-Anfangswerte aus den Ergebnissen aus (sind indirekt in ge-
    % speicherten Zwischenwerten enthalten)
    [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data_i, fval_in);
    test_pval = PSO_Detail_Data_i.pval(k_ind,:,k_gen)' - pval_in;
    if any(abs(test_pval))
      error('Indizes für Generation/Individuum stimmen nicht (Parameter nicht gleich)');
    end
    q0 = PSO_Detail_Data_i.q0_ik(k_ind,:,k_gen)';
  end
  %% Nachrechnen der Fitness-Funktion
  % um die Gelenkwinkel aus der IK zu erhalten. Annahme: Die gespeicherten 
  % Detail-Informationen stehen aus Speicherplatzgründen nicht zur Verfügung.
  parroblib_addtopath({RobName}); % Für Ausführung der Fitness-Fcn
  if regenerate_templates
    parroblib_create_template_functions({RobName}, false); % Für Erstellung fehlender Dateien
    R_test = parroblib_create_robot_class(RobName, 1, 1);
    R_test.fill_fcn_handles(true, true); % Zur Kompilierung fehlender Funktionen zum Nachrechnen der Fitness-Funktion
  else
    parroblib_update_template_functions({RobName});
  end
  Set_tmp = Set_i; Set_tmp.structures.whitelist = {d1.Structures{LfdNr}.Name};
  Set_tmp.general.verbosity = 0;
  % Durch die Aktualisierung der Roboter-Datenbank inmitten der
  % Simulationen haben einige noch die falschen alpha-/theta-Parameter
  Set_tmp.structures.repeatlist = {};
  Structures_tmp = cds_gen_robot_list(Set_tmp);
  [R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, Structures_tmp{1}, true);
  pval = cds_parameters_update(RobotOptRes_i.Structure, ...
    Structure, pval_in); % Für Aktualisierung des Programms mit neuen Parametern
  cds_update_robot_parameters(R, Set_i, Structure, pval);
  % Fitness-Funktion neu definieren (mit weniger Log-Ausgaben)
  Set = Set_i;
  Set.general.plot_details_in_fitness = 0; % debug: 1e10
  Set.general.save_robot_details_plot_fitness_file_extensions = {};
  Set.general.verbosity = 4; % debug: 3
  Set.optimization.objective = {'power', 'mass', 'colldist', 'actforce', ...
    'actvelo', 'condition', 'installspace', 'footprint'};
  Set.optimization.obj_limit = zeros(length(Set.optimization.objective), 1);
  Set.optimization.obj_limit_physval = Set.optimization.obj_limit;
  Set.optimization.constraint_obj(3) = 0; % Antriebskraft-Grenze deaktivieren
  % Einstellungen für DP-IK ändern (ist nicht ganz sauber, da es in der
  % Optimierung eigentlich anders war)
  Set.optimization.ee_rotation_limit_rel_traj = [-45, 45]*pi/180;
  Set.general.taskred_dynprog_numstates = [5 0];
  if ~strcmp(Set.optimization.objective_ik, usr_objective_ik)
    warning('In Maßsynthese wurde objective_ik=%d benutzt. Hier Bild mit %s', ...
      Set.optimization.objective_ik, usr_objective_ik);
  end
  Set.optimization.objective_ik = usr_objective_ik;
  optstr = sprintf('_opt_%s', Set.optimization.objective_ik);
  if GroupName(2) == '5', optstr = ''; end
  Set.optimization.resdir = fullfile(datadir, ...
    sprintf('select_eval_reproduce%s', optstr));
  if usr_recreate_redundancy_map
    resdir_tmp = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
      'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
    mkdirs(resdir_tmp);
    Set.general.debug_taskred_perfmap = 1;
  end
  Set.general.save_robot_details_plot_fitness_file_extensions = {'png', 'fig'};
  % Schreibe temporäre Daten dahin, wo auch die Ursprungsdaten liegen.
  % Überschreibt nichts.

  cds_log(0, '', 'init', Set);
  % IK-Anfangswerte für dieses Partikel setzen
  for iLeg = 1:R.NLEG
    R.Leg(iLeg).qref = q0(R.I1J_LEG(iLeg):R.I2J_LEG(iLeg));
  end
  fitnessrecalcfile = fullfile(datadir, ...
    sprintf('group_%s_%s_selection_fitness.mat', GroupName, usr_figselection));
  % Prüfe vollständigkeit der Datei, falls vorhanden
  file_valid = true;
  if exist(fitnessrecalcfile, 'file')
    tmp = load(fitnessrecalcfile);
    if ~isfield(tmp, 'Jinv_ges')
      fprintf('Geladene Datei hat altes Format (Jinv_ges).\n');
      file_valid = false;
    elseif ~isfield(tmp, 'OptName')
      fprintf('Geladene Datei hat altes Format (OptName).\n');
      file_valid = false;
    elseif ~strcmp(tmp.OptName, OptName) || tmp.LfdNr ~= LfdNr || ~strcmp(tmp.RobName, RobName) || tmp.Ipar ~= Ipar
      fprintf('Daten in geladener Datei sind anders.\n');
      file_valid = false;
    end
  end
  if usr_recalc_fitnessfcn || ~exist(fitnessrecalcfile, 'file') || ~file_valid
    % Funktionsaufruf siehe cds_check_results_reproducability.m
    Structure_tmp = Structure;
    Structure_tmp.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
    Structure_tmp.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
    Structure_tmp.calc_spring_reg = false;
    Structure_tmp.calc_dyn_reg = false;
  %   Set.optimization.joint_limits_symmetric_prismatic = false; % relaxieren, nicht so wichtig für Bild
  %   Set.optimization.max_range_passive_universal = 150*pi/180;
    Set.optimization.traj_ik_abort_on_success = true; % Sofort aufhören
    for kk = 1:3 % mehrere Versuche
      % Erzwinge Prüfung dieses Anfangswerts für Trajektorie (falls IK anderes
      % Ergebnis hat). Diese Option sollte nicht notwendig sein. Ist sie leider
      % teilweise.
      if kk == 2
        warning('Beim ersten Durchgang keine Lösung. Erhöhe Anzahl der Versuche');
        Set.optimization.pos_ik_tryhard_num = 200;
      elseif kk == 3
        warning('Beim zweiten Durchgang keine Lösung. Benutze gespeicherte Anfangswerte');
        Structure_tmp.q0_traj = q0;
        Set.optimization.pos_ik_abort_on_success = false;
        Set.optimization.traj_ik_abort_on_success = false;
      end
      cds_save_particle_details(); cds_fitness();
      [fval, physval, Q, QD, QDD, TAU, ~, Jinv_ges, X6Traj_out] = cds_fitness(R, Set, d1.Traj, Structure_tmp, pval, pval_desopt);
      PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output'); 
      if any(fval <=1e3)
        break
      end
    end
    if any(fval > 1e3)
      warning('Die Nebenbedingungen wurden bei erneuter Prüfung verletzt');
      continue
    end
    q0_neu = PSO_Detail_Data_tmp.q0_ik(1,:,1)';
    % Vergleich kann nur bezüglich der vorherigen Zielfunktionen gemacht
    % werden.
    [~,I_inold,I_innew] = intersect(Set.optimization.objective, Set_i.optimization.objective);
    test_fval = fval_in(I_innew) - fval(I_inold);
    test_physval = physval_in(I_innew) - physval(I_inold);
    if any(abs(test_fval) > 1e-5)
      warning('Eine Zielfunktion hat einen anderen Wert beim neu nachrechnen');
      for kk = 1:length(I_inold)
        fprintf('%s: %1.5f -> %1.5f\n', Set.optimization.objective{I_inold(kk)}, ...
          physval_in(I_innew(kk)), physval(I_inold(kk)));
      end
    end
    % Prüfe auch die Masse
    m_fitness = physval(strcmp(Set.optimization.objective, 'mass'));
    m_sum = R.NLEG*sum(R.Leg(1).DynPar.mges)+(R.DynPar.mges(end));
    p_fitness = physval(strcmp(Set.optimization.objective, 'power'));
    p_fitness2 = physval(strcmp(Set.optimization.objective, 'actforce')) * ...
                 physval(strcmp(Set.optimization.objective, 'actvelo'));
    if any(abs(m_fitness-m_sum) > 1e-5)
      error('Die Zielfunktion Masse hat einen anderen Wert beim neu nachrechnen');
    end
    p_colldist = physval_in(strcmp(Set.optimization.objective, 'colldist'));
    p_colldist2 = physval(strcmp(Set.optimization.objective, 'colldist'));
    if any(abs(p_colldist-p_colldist2) > 1e-5)
      warning(['Der neu berechnete Kollisionsabstand (%1.1fmm) unterscheidet ' ...
        'sich. Nehme den alten (%1.1fmm) aus der Optimierung, da sich die ' ...
        'Berechnungsmethode evtl. geändert hat.'], -1e3*p_colldist2, -1e3*p_colldist)
      physval(strcmp(Set.optimization.objective, 'colldist')) = p_colldist;
    end
    qlim = R.update_qlim();
    save(fitnessrecalcfile, 'fval', 'physval', 'Q', 'QD', 'TAU', 'Jinv_ges', 'qlim', ...
      'OptName', 'RobName', 'LfdNr', 'Ipar');
  else
    % Mit der Roboter-Klasse werden auch die Grenzen für Schubantriebe
    % geladen (wichtig für Plots)
    load(fitnessrecalcfile, 'fval', 'physval', 'Q', 'QD', 'TAU', 'Jinv_ges', 'qlim');
    R.update_qlim(qlim);
  end
  % Trage in Tabelle ein
  [~, LEG_Names, ~, Coupling_i, ~, ~, ~, PName_Kin] = parroblib_load_robot(RobName, 0);
  tmpstr = combine_alignment_names(Coupling_i);
  [tokens_cpl, match] = regexp(tmpstr, '\\mbox{(\w)-(\w)}', 'tokens', 'match');
  i_IT = strcmp(InfoTab.GroupName, GroupName);
  if ~any(i_IT), error('Fehler bei Zuordnung von Legende zu Gruppenname'); end
  InfoTab.CouplingOnly{i_IT} = sprintf('%s-%s', tokens_cpl{1}{1}, tokens_cpl{1}{2});
  InfoTab.TextCoupling{i_IT} = sprintf('%s, %s-%s', ...
    parroblib_format_robot_name(RobName, 4), tokens_cpl{1}{1}, tokens_cpl{1}{2});
  %% Abschließende Berechnungen und Abspeichern
  % Speichere die Trajektorie in der Variable X (für späteres Plotten)
  Traj_0 = cds_transform_traj(R, d1.Traj);
  X = Traj_0.X;
  X(:,6) = X6Traj_out(:,1); % EE-Rotation ändert sich
  % Berechnung der tatsächlichen Gestellgröße (siehe cds_constraints.m)
  Structure = RobotOptRes_i.Structure;
  [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
      cds_update_collbodies(R, Set_i, Structure, Q);
  I_guidance = Structure.collbodies_robot.type==13;
  pts = [Structure.collbodies_robot.params(I_guidance,1:3); ...
         Structure.collbodies_robot.params(I_guidance,4:6)];
  r_base_eff = max((pts(:,1).^2 + pts(:,2).^2).^0.5);
  % Ergebnis-Variable abspeichern
  objectives_test = Set.optimization.objective;
  optstr = ''; % sprintf('_opt_%s', Set.optimization.objective_ik);
%   if GroupName(2) == '3', optstr = ''; end % 3T0R
  save(fullfile(datadir, sprintf('detail_result_group_%s_%s%s.mat', ...
    GroupName, usr_figselection, optstr)), ...
    'R', 'pval', 'pval_desopt', 'fval', 'physval', 'objectives_test', 'Structure', ...
    'Q', 'QD', 'TAU', 'X', 'Jinv_ges', ... % Daten zum Ergebnis
    'OptName', 'RobName', 'LfdNr', 'Ipar', 'r_base_eff'); % Herkunft des Ergebnisses
  group_i_success = true;
  num_groups_calc = num_groups_calc + 1;
  break; % Erfolgreich reproduziert
  end % for iinearest
  if ~group_i_success
    warning('Fehler. Mache temporär weiter.');
  end
%   assert(group_i_success, 'Kein Erfolg bei Reproduktion');
end
fprintf(['Insgesamt %d Ergebnisse aus %d verschiedenen Gruppen ausgewählt ' ...
  'und gespeichert\n'], num_groups_calc, size(RobotGroups,1));
if num_groups_calc == 0
  continue % Gibt nichts zu speichern
end
% Im Debug-Fall soll nicht die vollständige Tabelle wieder überschrieben
% werden. Dauert lange sie zu erstellen.
if num_groups_calc == 1
  filenamesuffix = ['_', InfoTab.GroupName{~strcmp(InfoTab.CouplingOnly,'TODO')}];
else
  filenamesuffix = '';
end
infotabfilename = sprintf('handlingpkm_groups_legend_%s_add%s.csv', ...
  usr_figselection, filenamesuffix);
writetable(InfoTab, fullfile(outputdir, infotabfilename), 'Delimiter', ';');
fprintf('Erweiterte Tabelle: %s in %s\n', infotabfilename, outputdir);
end
