% Erzeuge hochaufgelöste Box-Plots der Leistungsmerkmale für jede PKM
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% * Zusammenstellen der Details mit select_eval_robot_examples.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

for i_collcheck = 0:1
% usr_figselection = 'default_revolute'; % siehe andere Skripte.
usr_figselection = 'PrauseChaCor2015';
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures', 'boxplots_handlingpkm');
mkdirs(outputdir);
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');
importdir = HandlingRobot_dimsynth_data_dir();
if strcmp(usr_figselection,'PrauseChaCor2015')
  tmp = load(fullfile(datadir, sprintf('engsol_groups_default_prismatic.mat')), 'TabEng');
  RobotGroups = tmp.TabEng;
  InfoTab = [];
else
  tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)));
  RobotGroups = tmp.RobotGroups;
  InfoTab = readtable(fullfile(outputdir, '..', ...
    sprintf('handlingpkm_groups_legend_%s_add.csv', usr_figselection)), 'Delimiter', ';');

end

% Extra-Einträge für 
%% Variablen initialisieren
tmp = load(fullfile(datadir, sprintf('results_all_reps_pareto_default.mat')));
ResTab = tmp.ResTab_ges;
tmp = load(fullfile(importdir, ResTab.OptName{1}, [ResTab.OptName{1},'_settings.mat']));
PE_ges = NaN(length(tmp.Traj.t), size(RobotGroups,1));
CJ_ges = PE_ges; AV_ges = PE_ges; AF_ges = PE_ges; AP_ges = PE_ges;

%% Alle Gruppen durchgehen und Kennzahlen berechnen
countrob = 0; % Nummern aus Legende
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  if RobotGroups.ResultsFound(i) == 0, continue; end % keine Ergebnisse vorliegend
%   if ~strcmp(GroupName, 'P3RRRRR10G'), continue; end
  fprintf('Datensammlung für Box-Plots für PKM-Gruppe %d/%d (%s)\n', i, ...
    size(RobotGroups,1), GroupName);
  if strcmp(usr_figselection,'PrauseChaCor2015')
    resfile = fullfile(datadir, sprintf(['detail_result_PrauseChaCor2015_' ...
      '%s_%s_collcheck%d_config1.mat'], GroupName, 'default', i_collcheck));
  else
    resfile = fullfile(datadir, sprintf('detail_result_group_%s_%s_collcheck%d.mat', ...
      GroupName, usr_figselection, i_collcheck));
  end
  if ~exist(resfile, 'file')
    if strcmp(usr_figselection,'PrauseChaCor2015')
      warning(['Ergebnis für Gruppe existiert nicht. eval_existing_design' ...
        ' ausführen!']);
    else
      warning(['Ergebnis für Gruppe existiert nicht. select_eval_robot_' ...
        'examples ausführen!']);
    end
    continue
  end
  erg = load(resfile);
  if strcmp(usr_figselection,'PrauseChaCor2015')
    erg.OptName = 'ARK_3T1R_20230730_full_rep1'; % Konsistent mit eval_existing_design (aber nicht so wichtig)
  end
  if isfield(erg, 'Jinv_ges')
    Jinv_ges = erg.Jinv_ges;
  else
    Jinv_ges = erg.Jinv;
    warning('Altes Speicherformat für Jinv')
  end
  % countrob = countrob + 1; % nur gültige PKM-Gruppen zählen
  countrob = i; % Geht nur, wenn alle Gruppen gültig sind.
%   if strcmp(usr_figselection, 'volume') && GroupName(2)=='6'
%     continue % nur 5FG-PKM für kompakten Bauraum zeichnen
%   end
%   if strcmp(usr_figselection, 'motordiagram') && GroupName(2)=='5'
%     continue % nur 6FG-PKM für Motordiagramm zeichnen
%   end
%   if ~strcmp(GroupName, 'P3PRRRR4V1G1'), continue; end
  setfile = dir(fullfile(importdir, erg.OptName, '*settings.mat'));
  assert(~isempty(setfile), sprintf('Für Optimierung %s fehlt die Einstellungsdatei', erg.OptName));
  d1 = load(fullfile(importdir, erg.OptName, setfile(1).name));
  Set_i = cds_settings_update(d1.Set);

  %% Roboter initialisieren
  R = erg.R;
  parroblib_addtopath({R.mdlname});

  %% Kennzahlen bestimmen
  % Positionsfehler und Kondition bestimmen (für jeden Punkt)
  PE = NaN(length(d1.Traj.t),1);
  CJ = PE;
  for k = 1:size(PE,1)
    [~, ~, ~, f_poserr] = cds_obj_positionerror(R, Set_i, Jinv_ges(k,:), erg.Q(k,:));
    PE(k) = f_poserr;
    [~, ~, ~, f_cond] = cds_obj_condition(R, Set_i, erg.Structure, Jinv_ges(k,:), d1.Traj, erg.Q(k,:));
    CJ(k) = f_cond;
  end
  TAUa_max = max(abs(erg.TAU),[],2);
  QDa = erg.QD(:,R.I_qa);
  QDa_max = max(abs(QDa),[],2);
  Pa_max = max(abs(QDa.*erg.TAU),[],2);

  % Kennzahlen in Gesamtlisten eintragen
  PE_ges(:,i) = PE;
  CJ_ges(:,i) = CJ;
  AV_ges(:,i) = QDa_max;
  AF_ges(:,i) = TAUa_max;
  AP_ges(:,i) = Pa_max;
end
%% Box-Plots zeichnen
I_bp = all(~isnan(AF_ges));
% PKM-Namen vorbereiten für Achsbeschriftung
AxStr = cell(1, size(RobotGroups,1));
AxStr2 = cell(1, size(RobotGroups,1));
for k = 1:size(RobotGroups,1)
  gn = RobotGroups.GroupName{k};
  if gn(2) == '3'
    dofstr = '3T0R';
  else
    dofstr = '3T1R';
  end
  
  if strcmp(usr_figselection,'PrauseChaCor2015')
    % Erster Name enthält Texterklärung für Legendentitel.
    % Abschneiden (alle Texte sind gleich lang, 17 Zeichen)
    AxStr{k} = sprintf('%s', RobotGroups.NameManual{k}(1:17));
  else
    I_it = strcmp(InfoTab.GroupName, gn);
    if any(I_it)
      [tokens,~] = regexp(InfoTab.TextFix{I_it}, '(.*) \((.*)\)', 'tokens', 'match');
      AxStr{k} = [tokens{1}{2}, ' (', tokens{1}{1}, ')'];
    else
      AxStr{k} = sprintf('%s Rob %d.%d', dofstr, RobotGroups.KinematicsNumber(k,1), ...
        RobotGroups.KinematicsNumber(k,2));
      continue % ist kein Fehler
      % fprintf('Gruppe %s nicht in InfoTab gefunden.\n', gn)
    end
  end
end

for k = 1:5
  % Box-Plot zur Antriebskraft, wie PrauseChaCor2015, Fig. 5
  fighdl = change_current_figure(k);clf;
  subplot(1,1,1);
  axhdl = get(fighdl, 'Children');
  switch k
    case 1
      name = 'actforce';
      boxplot(AF_ges(:,I_bp));
      if contains(usr_figselection, 'revolute')
        ylabel('Actuator torque in Nm');
      else
        ylabel('Actuator force in N');
      end
      if strcmp(usr_figselection,'PrauseChaCor2015')
        ylim([2,14]); % siehe PrauseChaCor2015, Fig. 5
        set(axhdl, 'ytick', 2:2:14);
      end
    case 2
      name = 'actvelo';
      if contains(usr_figselection, 'revolute')
        scale = 180/pi;
      else
        scale = 1;
      end
      boxplot(scale * AV_ges(:,I_bp));
      if contains(usr_figselection, 'revolute')
        ylabel('Actuator velocities in deg/s');
      else
        ylabel('Actuator velocities in m/s');
      end
    case 3
      name = 'poserr';
      boxplot(1e3*PE_ges(:,I_bp));
      ylabel('Position error in mm');
      if strcmp(usr_figselection,'PrauseChaCor2015')
        ylim([0.08,.3]); % siehe PrauseChaCor2015, Fig. 5
        set(axhdl, 'ytick', 0.1:0.05:0.3);
      end
    case 4
      name = 'actpower';
      boxplot(AP_ges(:,I_bp));
      ylabel('Actuator power in W');
    case 5
      name = 'dexterity';
      boxplot(1./CJ_ges(:,I_bp));
      ylabel('Inverse Jacobian condition number');
      if strcmp(usr_figselection,'PrauseChaCor2015')
        ylim([0.13,1.0]); % siehe PrauseChaCor2015, Fig. 5
        set(axhdl, 'ytick', 0.2:0.2:1.0);
      end
  end
%   set(get(axhdl, 'xaxis'), 'interpreter', 'latex')
  set(axhdl, 'TickLabelInterpreter', 'Latex');
  xticklabels(AxStr(I_bp));
  grid on;
  % Bild formatieren
  figure_format_publication(fighdl);
  if strcmp(usr_figselection,'PrauseChaCor2015')
    % Kleineres Bild, damit Original-Bild aus Paper daneben passt
    set_size_plot_subplot(fighdl, ...
      9,5,axhdl,...
      0.10,0.01,0.03,0.08,0,0);
  else
    set_size_plot_subplot(fighdl, ...
      16.2,6,axhdl,...
      0.10,0.01,0.01,0.10,0,0);
  end
  name_file = sprintf('handlingpkm_boxplot_%s_%s_collcheck%d', name, ...
    usr_figselection, i_collcheck);
  saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name_file)));
  exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', name_file)),'ContentType','vector');
  fprintf('Auswertung %s gespeichert\n', name_file);
end

end % i_collcheck
