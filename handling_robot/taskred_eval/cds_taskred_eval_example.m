% Untersuche Einfluss von IK-Zielkriterien auf das Ergebnis der Maßsynthese
% 
% Siehe auch: cds_check_results_reproducability.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
%% Benutzereinstellungen
% Wird das Skript auf dem Cluster ausgeführt?
usr_cluster = false;
% Bevorzugt vorhandene Daten laden
usr_load_data_preferably = true;
% Debug: Liste von Einstellungs-Nummern, die neu gerechnet werden müssen,
% auch wenn schon Daten vorliegen.
usr_recompute_settings = []; % [8, 10];
% Debug: Filtern eines Versuchs/Roboters. Weglassen: Alle nehmen; wird auf Cluster überschrieben
usr_debug_optname = 'ARK_3T1R_20230730_full_rep1';
usr_debug_optname_findstr = {};%{'amunpkm_20230605'}; % amunpkm_20220827
% Nur eine bestimmte Kinematik untersuchen (wird auf Cluster überschrieben)
usr_debug_robname = 'P4RPRRR8V2G3P1A1'; % 'P6RRRRRR8V2G6P1A1';
% Nummer des Roboters vorgeben
usr_debug_robnr = [];
% Pareto-Partikel-Nummer vorgeben.
% Leeres Array: Alle Partikel. Nummer: Nur diese(s)
usr_select_pnr = [];
% Debug: Tabelle nicht neu laden. Sinnvoll, falls Code geändert wurde und
% Daten neu erzeugt werden müssen
usr_no_StatsTab_Rob_fromfile = true;
% Schalter zum Erzeugen von Debug-Informationen (notwendig für weitere
% Skripte)
usr_plot_perfmap = 0; % 1=Redundanzkarte zeichnen (notwendig zum Speichern der Informationen für die Redundanzkarte; überflüssig für das Erzeugen der Pareto-Front für viele Partikel)

%% Cluster-Einstellungen. Überschreibt die obigen Benutzereinstellungen
% Wenn das Skript auf dem Cluster ausgeführt wird und durch ....cluster_start
% nachbearbeitet wurde. Obige Einstellungen werden dann überschrieben.
%#CLUSTER usr_debug_robname = '';
%#CLUSTER usr_debug_optname = '';
%% Initialisierung
defstruct_cds = cds_definitions();
objective_ik_cases = defstruct_cds.objective_ik_names_all;

% Ergebnis-Tabelle vorbereiten
vn = {'OptName', 'RobName', 'RobNr', 'ParetoNr', 'Fval_Repro', ...
  'physval_actforce_repro', 'physval_actvelo_repro'};
if ~usr_plot_perfmap
  warning(['Es werden keine Debug-Informationen zur Redundanzkarte ', ...
    'gespeichert. Dadurch keine Auswertung mit Bild wie in Diss ', ...
    '(Fig. 5.20) möglich']); %#ok<UNRCH>
end
obj_cases = defstruct_cds.obj_names_all;
% Hier nicht funktionierende Kriterien deaktivieren
obj_cases = obj_cases(~strcmp(obj_cases, 'jointlimit'));
row = cell(1,length(vn));
% Numerische Spalten müssen auf NaN gesetzt werden
for i = [3 4 5 6 7]
  row{i} = NaN;
end
for i = 1:length(obj_cases)
  for j = 1:length(objective_ik_cases)
    vn = [vn, sprintf('physval_%s_ik_%s', obj_cases{i}, objective_ik_cases{j})]; %#ok<AGROW>
    row{length(vn)} = NaN;
  end
end
StatsTab_template = cell2table(row, 'VariableNames', vn);
StatsTab_empty = cell2table(cell(0,length(vn)), 'VariableNames', vn);
StatsTab = StatsTab_empty;
%% Ergebnisse durchgehen und neu berechnen
[resdir, datadir] = cds_taskred_eval_path_config(usr_cluster);
assert(isfolder(resdir), 'resdir existiert nicht');
assert(isfolder(datadir), 'datadir existiert nicht');
dirres = dir(fullfile(datadir, 'amun*'));
optnames = {};
for i = 1:length(dirres)
  [tokens, match] = regexp(dirres(i).name, '_p(\d+)$', 'tokens', 'match');
  if ~isempty(tokens)
    continue % Teil-Ergebnisse nicht mit reinnehmen
  end
  optnames = [optnames, dirres(i).name]; %#ok<AGROW> 
end
if ~isempty(usr_debug_optname)
  optnames = {usr_debug_optname};
end
fprintf('Ergebnis-Ordner: %s\n', resdir);

for ii_opt = 1:length(optnames)
StatsTab_Opt = StatsTab_empty;
OptName = optnames{ii_opt};
if ~isempty(usr_debug_optname_findstr)
  if ~any(contains(OptName, usr_debug_optname_findstr))
    continue % Gesuchte Optimierungen nicht gefunden.
  end
end
fprintf('Untersuche Optimierung %d/%d (%s)\n', ii_opt, length(optnames), OptName);
% Daten laden
settingsfile = fullfile(datadir, OptName, [OptName,'_settings.mat']);
if ~exist(settingsfile, 'file')
  warning('Einstellungsdatei %s nicht gefunden.', settingsfile);
  continue;
end
d1 = load(settingsfile);
Set = cds_settings_update(d1.Set, 1);
if ~all(Set.task.DoF == [1 1 1 0 0 0])
  warning('Task DoF [%s] unpassend', disp_array(Set.task.DoF, '%d'));
  continue;
end
% Speicherort ändern (in lokalen Ordner, nicht Cluster)
Set.optimization.resdir = fullfile(resdir, OptName);
% Debuggen
Set.general.matfile_verbosity = 3;
% Set.general.plot_details_in_fitness = -1e3; % Bei Misserfolg zeichnen  
% Einstellungen zur Aufgabenredundanz-Auswertung überschreiben
Set.general.debug_taskred_fig = false;
% Set.general.taskred_dynprog = false;
% Set.general.taskred_dynprog_and_gradproj = false;
% Set.general.debug_dynprog_files = false;
Set.general.debug_taskred_perfmap = usr_plot_perfmap;
% Dateiendung für den Fall, dass Dynamische Programmierung für IK benutzt
% wird. Die Ergebnisse sind dann nicht direkt vergleichbar
dpsuffix = '';
if Set.general.taskred_dynprog
  dpsuffix = [dpsuffix, '_dp']; %#ok<AGROW>
end
if Set.general.taskred_dynprog_and_gradproj
  dpsuffix = [dpsuffix, '_gp']; %#ok<AGROW>
end 

% Weitere Optionen anpassen
Set.optimization.desopt_vars = {}; % Keine Entwurfsoptimierung

% Einzelpunkt-IK nicht neu versuchen
Set.optimization.pos_ik_tryhard_num = -200;
% Lade Ergebnis-Tabelle mit Roboter-Namen
tablepath = fullfile(datadir, OptName, sprintf('%s_results_table.csv', OptName));
if ~exist(tablepath, 'file'), continue; end % Ohne Tabelle nicht weitermachen
ResTab_i = readtable(tablepath, 'HeaderLines', 2);
if isempty(ResTab_i)
  warning('Tabelle zu %s ist leer: %s', OptName, tablepath);
  continue % Sonst hiernach Fehler
end
ResTab_i_headers = readtable(tablepath, 'ReadVariableNames', true);
ResTab_i.Properties.VariableNames = ResTab_i_headers.Properties.VariableNames;

%% Alle Roboter durchgehen
if ~isempty(usr_debug_robname)
  if ~any(strcmp(usr_debug_robname, ResTab_i.Name))
    warning('Gesuchter Roboter %s nicht in Ergebnissen enthalten', usr_debug_robname);
    continue;
  end
  if ~any(strcmp(usr_debug_robname, ResTab_i.Name(ResTab_i.Fval_Opt < 1e3)))
    warning('Gesuchter Roboter %s in Ergebnissen enthalten aber nicht erfolgreich', usr_debug_robname);
    continue;
  end
end

for ii_rob = find(ResTab_i.Fval_Opt < 1e3)'
LfdNr = ResTab_i.LfdNr(ii_rob); % Zeilen-Nummer in der Tabelle (ungleich, falls Lückenhaft)
RobName = ResTab_i.Name{ii_rob};
if ~isempty(usr_debug_robname) && ~strcmp(RobName, usr_debug_robname)
  continue;
end
if ~isempty(usr_debug_robnr) && LfdNr ~= usr_debug_robnr
  continue;
end
fprintf('Untersuche Roboter %d (%s)\n', LfdNr, RobName);
resfile = fullfile(datadir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
if ~exist(resfile, 'file')
  warning('Ergebnis-Datei existiert nicht: %s', resfile);
  continue
end
tmp = load(resfile);
RobotOptRes_j = tmp.RobotOptRes;
[~,LEG_Names]=parroblib_load_robot(RobName,0);
serroblib_update_template_functions(LEG_Names(1));
parroblib_update_template_functions({RobName});
StatsTab_Rob = StatsTab_empty;
restabfile_rob = fullfile(resdir, OptName, sprintf('taskred_eval_example_result_rob%d%s.csv', LfdNr, dpsuffix));
if usr_load_data_preferably && exist(restabfile_rob, 'file')
  StatsTab_Rob = readtable(restabfile_rob);
  if size(StatsTab_Rob,2) ~= size(StatsTab_empty,2)
    warning('Geladene Tabelle hat falsche Spaltenzahl. Baue Tabelle neu auf.');
    StatsTab_Rob = StatsTab_empty;
  elseif length(unique(StatsTab_Rob.ParetoNr)) ~= length(StatsTab_Rob.ParetoNr)
    warning('Geladene Tabelle hat doppelte Zeilen. Baue Tabelle neu auf.');
    StatsTab_Rob = StatsTab_empty;
  elseif size(StatsTab_Rob,1) ~= size(RobotOptRes_j.p_val_pareto,1)
    warning('Geladene Tabelle ist nicht vollständig. Baue Tabelle neu auf.');
    StatsTab_Rob = StatsTab_empty;
  elseif usr_no_StatsTab_Rob_fromfile
    % Tabelle soll nicht geladen werden
    StatsTab_Rob = StatsTab_empty;
  else
    fprintf('Tabelle für Rob %s geladen. Überspringe Berechnung.\n', RobName)
  end
end
StatsTab_Rob_changed = false;
%% Gehe die Partikel der Pareto-Front durch
if ~isempty(usr_select_pnr)
  II_pareto = usr_select_pnr;
else
  II_pareto = 1:size(RobotOptRes_j.p_val_pareto, 1);
end
for ii_pareto = II_pareto(:)'
if ~isempty(StatsTab_Rob) && ii_pareto == 1
  break; % Wenn beim ersten Mal schon Tabelle belegt, dann erfolgreich vorher geladen
end
fprintf('Untersuche Partikel %d/%d der Pareto-Front\n', ii_pareto, size(RobotOptRes_j.p_val_pareto, 1));
robot_init_done = false;
% Fülle Identifikationsspalten der Tabelle
StatsTab_ii = StatsTab_template;
StatsTab_ii.OptName{1} = OptName;
StatsTab_ii.RobName{1} = RobName;
StatsTab_ii.RobNr(1) = LfdNr;
StatsTab_ii.ParetoNr(1) = ii_pareto;
%% Gehe die möglichen IK-Ziele durch und berechne die Zielfunktion neu
fval_all = NaN(length(objective_ik_cases), length(obj_cases));
i_set_old = inf;
% for i_set = [8, 10] % find(strcmp(objective_ik_cases, 'maxactforce'))
for i_set = 0:length(objective_ik_cases) % 0=Reproduktion aus Optimierung, 1,2,... neue Einstellungen
  fprintf('Werte Versuch %s/%s/P.%d mit IK-Einstellung %d aus.\n', OptName, RobName, ii_pareto, i_set);
  % Ergebnis-Datei für diese Einstellungs-Variante
  resfile_iset = fullfile(resdir, OptName, sprintf(['taskred_eval_example_' ...
    'result_rob%d_par%d_set%d%s.mat'],LfdNr, ii_pareto, i_set, dpsuffix));
  resfile_iset2 = strrep(resfile_iset, '.mat', '_short.mat');
  % Speichere die kompakte Datei ab, falls nur die größere existiert
  if ~exist(resfile_iset2, 'file') && exist(resfile_iset, 'file')
    fprintf('Große Datei existiert, kleine nicht. Erzeuge die kleine Datei');
    % Der Code hier ist eine Kopie des Codes unten.
    tmp = load(resfile_iset);
    fval_i = tmp.fval_i;
    physval_i = tmp.physval_i;
    if i_set == 0
      TAU = tmp.TAU;
      QD = tmp.QD;
      R = tmp.R;
    end
    Phiz_traj = tmp.X6Traj(:,1);
    if i_set == 0
      save(resfile_iset2, 'fval_i', 'physval_i', 'Phiz_traj', 'R', 'QD', 'TAU');
    else
      save(resfile_iset2, 'fval_i', 'physval_i', 'Phiz_traj');
    end
  end
  data_load_complete = false;
  if usr_load_data_preferably && exist(resfile_iset2, 'file')
       % Debug: Nur diese Fälle laden. Den Rest neu berechnen
    if ~any(i_set==usr_recompute_settings)
    try % durch viele parallele Zugriffe auf dem Cluster Dateifehler möglich.
      tmp = load(resfile_iset2);
    catch e
      fprintf('Datei konnte nicht geladen werden: %s\n', e.message);
      tmp = [];
    end
    if ~isfield(tmp, 'physval_i')
      fprintf('Datei enthält nicht die Variable physval_i\n');
    elseif i_set ~= 0 && length(tmp.physval_i) ~= length(obj_cases)
      fprintf('Datei enthält nicht alle Zielkriterien\n');
    elseif i_set == 0 && (~isfield(tmp, 'TAU') || ~isfield(tmp, 'R'))
      fprintf('Datei für i_set=0 enthält nicht die Variable TAU oder R\n');
    elseif i_set ~= 0 && strcmp(objective_ik_cases{i_set}, 'constant') && ...
        (isfield(tmp, 'Phiz_traj') && ...
           any(abs(tmp.Phiz_traj(1,1)-tmp.Phiz_traj(:,1))>1e-6) ||...
         isfield(tmp, 'X6Traj') && ...
           any(abs(tmp.X6Traj(1,1)   -tmp.X6Traj(:,1))>1e-6) )
      fprintf('Geladene Trajektorie für Fall "constant" ist nicht konstant\n');
    else
      fval_i = tmp.fval_i;
      physval_i = tmp.physval_i;
      if i_set == 0
        TAU = tmp.TAU;
        QD = tmp.QD;
        R = tmp.R;
      end
      fprintf('Ergebnisse aus Datei %s geladen.\n', resfile_iset);
      data_load_complete = true;
    end
    else
    fprintf('Ergebnis-Datei verfügbar, soll aber nicht geladen werden: %s.\n', resfile_iset);
    end
  elseif usr_load_data_preferably
    fprintf('Ergebnis-Datei nicht verfügbar: %s.\n', resfile_iset);
  end
  if i_set ~= 0
    % Einstellungen variieren. Zielkriterien müssen schon hier über-
    % schrieben werden, damit die Initialisierung richtig ist.
    Set.optimization.objective = obj_cases;
    Set.optimization.objective_ik = objective_ik_cases(i_set);
  else
    Set.optimization.objective = d1.Set.optimization.objective;
  end
  % Prüfe, ob die Redundanzkarte gespeichert wurde. Wenn nicht, ist die
  % später gewünschte Auswertung nicht möglich.
  if usr_plot_perfmap && data_load_complete
    pmfile = fullfile(resdir, OptName, sprintf('%s_rob%d_par%d_set%d%s', ...
      OptName, LfdNr, ii_pareto, i_set, dpsuffix), ...
      'tmp', sprintf('%d_%s', LfdNr, RobName), 'Gen-1_Ind-1_Konfig1_TaskRedPerfMap_Data.mat');
    if ~exist(pmfile, 'file')
      fprintf('Datei existiert nicht: %s\n', pmfile);
      data_load_complete = false;
    end
  end

  if ~data_load_complete
%     % Roboter initialisieren (falls notwendig)
%     if ~robot_init_done || i_set_old == 0 % bei Reproduktions-Lauf (0) ist die Initialisierung anders (Vorbereitung für Schnittkraftberechnung als Ziel)
      % Parameter-Wert für dieses Partikel
      pval_orig = RobotOptRes_j.p_val_pareto(ii_pareto,:)';
      % Parameter in neues Format konvertieren, falls sehr alter Versuch
      % basierend auf alter Code-Version
      [R, Structure] = cds_dimsynth_robot(Set, d1.Traj, d1.Structures{LfdNr}, true);
      pval = cds_parameters_update(RobotOptRes_j.Structure, Structure, pval_orig);
      if i_set_old ~= 0 % Nicht die Repro-Initialisierung für die weiteren merken (Einstellungen nicht gleich)
        robot_init_done = true;
      end
%     end
    Structure.q0_traj = RobotOptRes_j.q0_pareto(ii_pareto,:)';
%     if i_set < 7
%       % Einzelpunkt-IK nicht neu versuchen
%       Set.optimization.pos_ik_tryhard_num = -200;
%     else
%       % Bei konstanter Orientierung müssen viele Anfangswerte probiert
%       % werden
%       Set.optimization.pos_ik_tryhard_num = 50;
%     end
    % Modifikation von Einstellungen, damit Ergebnis brauchbarer wird
    Set.optimization.ElectricCoupling = false; % Sonst Energie immer in Summe 0
    % Hiermit direkt Abbruch nach der Berechnung mit q0_traj
    Set.optimization.obj_limit = 1e3*ones(length(Set.optimization.objective), 1); % Falls Dimension geändert wurde
    Set.optimization.obj_limit_physval = zeros(length(Set.optimization.objective), 1);
    % Debug:
    % Set.general.plot_details_in_fitness = 4e7;
    Set.optimization.optname = sprintf('%s_rob%d_par%d_set%d%s', OptName, LfdNr, ii_pareto, i_set, dpsuffix);
    % Ergebnis-Ordner erstellen (für Fitness-Aufruf weiter unten)
    mkdirs(fullfile(Set.optimization.resdir, Set.optimization.optname, 'tmp', sprintf('%d_%s', LfdNr, RobName)));
    mkdirs(fullfile(Set.optimization.resdir, Set.optimization.optname, sprintf('Rob%d_%s', LfdNr, RobName)));
    % Fitness-Funktion neu auswerten
    R.update_qref(Structure.q0_traj);
    R.update_EE_FG(R.I_EE, Set.task.DoF); % für Debuggen; gestört bei Abbruch mittendrin
    clear cds_save_particle_details cds_fitness
    if ~Structure.calc_cut % TODO: Stand 2024/12 unklar, woher das kommt.
      dbgfile = fullfile(Set.optimization.resdir, ['debug_', ...
        Set.optimization.optname, '_calc_cut_error.mat']);
      % warning(['Marker für Berechnung der Schnittkräfte nicht gesetzt. ' ...
      %   'Logik-Fehler. Speichere nach %s'], dbgfile);
      % save(dbgfile); % auskommentiert 2024-12-10
    end
    % Log initialisieren. Setzt Umbenennung des Optimierungsnamens voraus,
    % damit alte Ergebnisse nicht überschrieben werden.
    cds_log(1, sprintf('[dimsynth] Neuberechnung der Maßsynthese %s/%s',  ...
      Set.optimization.optname, Structure.Name), 'init', Set, Structure);
    [fval_i, physval_i, Q, QD, QDD, TAU, ~, ~, X6Traj] = cds_fitness(R, Set, d1.Traj, ...
      Structure, pval);
    % Speichere Ergebnis
    mkdirs(fileparts(resfile_iset2));
%     save(resfile_iset, 'fval_i', 'physval_i', 'R', 'Q', 'QD', 'TAU', 'X6Traj', 'Set', 'Structure');
    Phiz_traj = X6Traj(:,1); % Nur Positionsprofil speichern, nicht Geschw.
    if i_set == 0
      save(resfile_iset2, 'fval_i', 'physval_i', 'Phiz_traj', 'R', 'QD', 'TAU');
    else
      save(resfile_iset2, 'fval_i', 'physval_i', 'Phiz_traj');
    end
  end
  % Trage Ergebnis in Tabelle ein
  if i_set == 0
    StatsTab_ii.Fval_Repro(1) = mean(fval_i);
    if any(fval_i>1e3)
      warning('Ergebnis aus Maßsynthese nicht reproduzierbar (jetzt nicht mehr i.O.)');
      % Berechne trotzdem die 
    elseif any(abs(fval_i - RobotOptRes_j.fval_pareto(ii_pareto,:)') > 1e-3)
      warning('Ergebnis aus Maßsynthese nicht reproduzierbar. Fval alt: [%s], neu: [%s]', ...
        disp_array(fval_i', '%1.3f'), disp_array(RobotOptRes_j.fval_pareto(ii_pareto,:), '%1.3f'));
    end
    if any(fval_i<1e3)
      [fval_actforce_r,~, ~, physval_actforce_r] = cds_obj_actforce(TAU);
      [fval_actvelo_r,~, ~, physval_actvelo_r] = cds_obj_actvelo(R, QD);
      StatsTab_ii.physval_actforce_repro = physval_actforce_r;
      StatsTab_ii.physval_actvelo_repro = physval_actvelo_r;
    end
  else
    fval_all(i_set,:) = fval_i;
    for kk = 1:length(obj_cases)
      % Vorher wurde obj_cases in Set.optimization.objective eingesetzt.
      % Bedingung muss erfüllt sein.
      if ~any(strcmp(obj_cases{kk}, Set.optimization.objective)), continue; end
      StatsTab_ii.(sprintf('physval_%s_ik_%s', Set.optimization.objective{kk}, ...
        objective_ik_cases{i_set})) = physval_i(kk);
    end
  end
  i_set_old = i_set;
end % i_set
if isempty(StatsTab_Rob) % ansonsten werden alle Spalten zu cell-Arrays
  StatsTab_Rob = StatsTab_ii;
else
  StatsTab_Rob = [StatsTab_Rob; StatsTab_ii]; %#ok<AGROW>
end
StatsTab_Rob_changed = true;
% restabfile_i = fullfile(Set.optimization.resdir, OptName, 'taskred_eval_example_result.csv');
% writetable(StatsTab_ii, restabfile_i, 'Delimiter', ';');
end % ii_pareto
if StatsTab_Rob_changed
  writetable(StatsTab_Rob, restabfile_rob, 'Delimiter', ';');
end

%% Pareto-Diagramm für einzelnen Roboter
% Zeichne Pareto-Diagramm für diesen Roboter mit den Ergebnissen der
% verschiedenen Durchläufe.
StatsTab_RobPlot = StatsTab_empty;
% Fehlende Zeilen in Tabelle anhängen, falls unvollständig
for ii_pareto = 1:size(RobotOptRes_j.p_val_pareto, 1)
  if ~any(StatsTab_Rob.ParetoNr == ii_pareto) % Dummy-Zeile einfügen
    StatsTab_ii = StatsTab_template;
    StatsTab_ii.OptName{1} = OptName;
    StatsTab_ii.RobName{1} = RobName;
    StatsTab_ii.RobNr(1) = LfdNr;
    StatsTab_ii.ParetoNr(1) = ii_pareto;
    StatsTab_RobPlot = [StatsTab_RobPlot; StatsTab_ii]; %#ok<AGROW>
  else
    StatsTab_RobPlot = [StatsTab_RobPlot; StatsTab_Rob(StatsTab_Rob.ParetoNr == ii_pareto,:)]; %#ok<AGROW>
  end
end
% Prüfe Integrität der Tabelle
assert(length(unique(StatsTab_RobPlot.ParetoNr)) == length(StatsTab_RobPlot.ParetoNr), 'Tabelle ist nicht eindeutig');
assert(all(sort(StatsTab_RobPlot.ParetoNr) == StatsTab_RobPlot.ParetoNr), 'Tabelle ist nicht fortlaufend');

% Zusammenhang zwischen Zielfunktion der Optimierung und der dieser
% Auswertung
obj_cases_plot = {'actforce', 'actvelo'};
kk1 = find(strcmp(obj_cases_plot{1}, d1.Set.optimization.objective));
kk2 = find(strcmp(obj_cases_plot{2}, d1.Set.optimization.objective));
% Bild vorbereiten
fighdl = change_current_figure(1); clf; hold on;
set(fighdl, 'name', sprintf('%s_Rob%d_eval', OptName, LfdNr), 'NumberTitle', 'off');
Set_plot = Set;
Set_plot.optimization.objective = obj_cases_plot;
[obj_units, objscale] = cds_objective_plotdetails(Set_plot, {RobotOptRes_j.Structure});
xlabel(sprintf('%s in %s', obj_cases_plot{1}, obj_units{1}));
ylabel(sprintf('%s in %s', obj_cases_plot{2}, obj_units{2}));
styles = {'r+', 'bd', 'ms', 'k^', 'cp', 'bx', 'gv', 'r<', 'k>', 'rp', 'bo'};
n_iO = NaN(1, length(objective_ik_cases));
% Daten zusammentragen
I = true(size(StatsTab_RobPlot,1),1);
pareto_data = NaN(sum(I), length(obj_cases_plot), length(objective_ik_cases)+1);
for i_set = 1:length(objective_ik_cases)+1
  if i_set <= length(objective_ik_cases)
    % Nehme die Daten aus der Neuberechnung
    for ii = 1:length(obj_cases_plot)
      Col_ii = StatsTab_RobPlot.(sprintf('physval_%s_ik_%s', obj_cases_plot{ii}, ...
          objective_ik_cases{i_set}));
      pareto_data(:,ii, i_set) = Col_ii(I);
    end
  else
    % Nehme die Daten der ursprünglichen Optimierung
    if isempty(kk1) || isempty(kk2), plot(NaN,NaN,styles{i_set}); continue; end
    pareto_data(:,:, i_set) = RobotOptRes_j.physval_pareto(:, [kk1,kk2]);
  end
end
linhdl = NaN(size(pareto_data,3),1); % Für Legende
% Erzeuge Pareto-Front mit dominierenden Partikeln
% für Ergebnisse der ursprünglichen Optimierung.
if ~isempty(kk1) && ~isempty(kk2)
  Idom = ~pareto_dominance(pareto_data(:,:, end));
  linhdl(end) = plot(pareto_data(Idom,1,end), pareto_data(Idom,2,end), [styles{end},'-']);
  % Auch normale Partikel plotten (nur die fehlenden, nicht dominierenden)
  plot(pareto_data(~Idom,1,end), pareto_data(~Idom,2,end), styles{end});
end
% Daten plotten (neu berechnete Partikel)
pareto_data_dom_all = NaN(0,2);
for i_set = 1:length(objective_ik_cases)
  pareto_data_ii = pareto_data(:,:,i_set);
%   pareto_data(pareto_data>1e3) = NaN; % NB verletzt
  n_iO(i_set) = sum(~isnan(pareto_data_ii(:,1)));
  % Pareto-Fronten einzeichnen (dominierend)
  Idom = ~pareto_dominance(pareto_data_ii);
  pareto_data_iidom = pareto_data_ii(Idom,:);
  [~,Isort] = sort(pareto_data_iidom(:,1));
  Isort = Isort(~isnan(pareto_data_iidom(Isort,1)));
  pareto_data_iidom = pareto_data_iidom(Isort,:);
  % Platzhalter, falls keine Daten vorhanden
  if isempty(pareto_data_iidom), pareto_data_iidom = NaN(1,2); end
  linhdl(i_set) = plot(pareto_data_iidom(:,1), pareto_data_iidom(:,2), [styles{i_set},'-']);
  % Alle berechneten Punkte einzeichnen (auch nicht-dominierend)
  plot(pareto_data_ii(~Idom,1), pareto_data_ii(~Idom,2), styles{i_set});
  % Zeichne eine Verbindung von dem zugehörigen Punkt der ursprünglichen
  % Pareto-Front (mit Wert "default" für die IK) zu dem jeweiligen
  % aktuellen Punkt
  pareto_data_ref = pareto_data(:,:,end); % Alte Ergebnisse
  if i_set > 1
    for j = find(~isnan(pareto_data_ii(:,1)))'
      plot([pareto_data_ref(j,1);pareto_data_ii(j,1)], [pareto_data_ref(j,2);pareto_data_ii(j,2)], 'k--');
    end
  end
  % Speichere die dominierenden Partikel ab
  pareto_data_dom_all = [pareto_data_dom_all; pareto_data_iidom]; %#ok<AGROW>
end

legnames = cell(1, length(objective_ik_cases)+1);
for i = 1:length(objective_ik_cases)
  legnames{i} = sprintf('ik obj.: %s (%d i.O.)', objective_ik_cases{i}, n_iO(i)); % Text "default" geht nicht
end
legnames{end} = sprintf('orig (%d i.O.)', size(pareto_data,1));
legend(linhdl(~isnan(linhdl)), legnames(~isnan(linhdl)), 'interpreter', 'none');
grid on;
sgtitle(sprintf('%s / Rob %d (%s)', OptName, LfdNr, RobName), 'interpreter', 'none');
figdir = fullfile(resdir, OptName);
figname = sprintf('Rob%d_%s_pareto_compare_ik_obj', LfdNr, RobName);
saveas(fighdl,     fullfile(figdir, [figname,'.fig']));
export_fig(fighdl, fullfile(figdir, [figname,'.png']));
fprintf('Bild %s gespeichert nach %s\n', figname, figdir);
%% Pareto-Diagramm für mehrere Roboter
% TODO!
end % ii_rob
%% Gesamt-Tabelle für die Optimierung abspeichern
% Speichern am Ende reicht, da die Roboter-Tabellen immer wieder neu
% geladen werden können
if isempty(usr_debug_robname) % nur, wenn mehr als ein Roboter zusammengefasst wird
  if isempty(StatsTab_Opt) % ansonsten werden alle Spalten zu cell-Arrays
    StatsTab_Opt = StatsTab_Rob;
  else
    StatsTab_Opt = [StatsTab_Opt; StatsTab_Rob]; %#ok<AGROW>
  end
  restabfile_i = fullfile(resdir, OptName, ['taskred_eval_example_result_', ...
    OptName, '_', datestr(now,'yyyymmdd_HHMM'), '.csv']);
  writetable(StatsTab_Opt, restabfile_i, 'Delimiter', ';');
end
end % ii_opt
%% Gesamt-Tabelle für alle Optimierungen abspeichern
% Speichern am Ende reicht, da die Roboter-Tabellen immer wieder neu
% geladen werden können
if isempty(usr_debug_robname) && isempty(usr_debug_optname) % nur, wenn mehr als ein Roboter und eine Optimierung zusammengefasst werden
  if isempty(StatsTab) % ansonsten werden alle Spalten zu cell-Arrays
    StatsTab = StatsTab_Opt;
  else
    StatsTab = [StatsTab; StatsTab_Opt]; %#ok<AGROW>
  end
  restabfile_i = fullfile(resdir, ['taskred_eval_example_result_', ...
    OptName, '_', datestr(now,'yyyymmdd_HHMM'), '.csv']);
  writetable(StatsTab, restabfile_i, 'Delimiter', ';');
end
