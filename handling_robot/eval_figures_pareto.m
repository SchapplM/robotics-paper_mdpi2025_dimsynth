% Assemble the Pareto fronts for multiple robots. This script is able to
% use results from several independent run of the dimensional synthesis.
% In the synthesis toolbox the Pareto diagram is only created for one run.
%
% Preliminaries:
% * run dimensional synthesis
% 
% This script is based on the same file eval_figures_pareto.m from 
% https://github.com/SchapplM/robotics-paper_ark2022_3T1R/
% (ARK paper "Inverse Kinematics for Task Redundancy of Symmetric 3T1R
% Parallel Manipulators using Tait-Bryan-Angle Kinematic Constraints",
% Schappler 2022)  

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
usr_fig_all = {'default', 'onlypoints', 'relax'};

for i_usr_fig = 1:2
usr_figselection = usr_fig_all{i_usr_fig};
%% Initialisierung
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir.m'));
if isempty(repo_dir)
  error(['You have to create a file HandlingRobot_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = HandlingRobot_dimsynth_data_dir();
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
% outputdir = fullfile(phdthesis_dir, 'manuscript', '05_evaluation', 'figures');
outputdir = fullfile(paper_dir, 'paper', 'Figures');
imescolors = imes_plot_template_colors();
% Für alle Bilder die gleichen Kriterien
pareto_settings = {'actforce', 'installspace'};

SerRobDB = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof', 'N');
ParRobDB = load(fullfile(fileparts(which('parroblib_path_init.m')), ...
  'sym_3T3R', 'sym_3T3R_list_act.mat'));

%% Auswahl der Ergebnisse und Ergebnisse durchgehen
% Optimierungen werden mit config_amun.m aus dem AMUN-Maßsynthese-Repo
% gestartet.
% Ergebnis-Ordner müssen im Datenverzeichnis liegen (entweder Offline-Aus-
% wertung oder neue Maßsynthese).
resdirs = {};
if contains(usr_figselection, 'default')
%   resdirs = {resdirs{:}, 'ARK_3T1R_20230624_phdthesis_v2_all'}; %#ok<CCAT> 
  % Zwischenzeitlich neu hinzugefügte PKM
%   resdirs = {resdirs{:}, 'ARK_3T1R_20230630_phdthesis_v3_P3PRRRR4V1G4P2A1'}; %#ok<CCAT> 
%   resdirs = {resdirs{:}, 'ARK_3T1R_20230701_phdthesis_v1_all'}; %#ok<CCAT> 
  for rr = 1:2
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230728_plfabovejoints_rep%d', rr)}; %#ok<CCAT> 
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230728_plfabovejoints2_rep%d', rr)}; %#ok<CCAT> 
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230729_v1_rep%d', rr)}; %#ok<CCAT> 
  end
  for rr = 1:3
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230729_v2_rep%d', rr)}; %#ok<CCAT> 
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230730_v1_rep%d', rr)}; %#ok<CCAT> 
  end
  for rr = 1:2
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230730_full_rep%d', rr)}; %#ok<CCAT> 
  end
elseif contains(usr_figselection, 'relax')
  for rr = 1:2
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230701_phdthesis_v3_all_relax_rep%d', rr)}; %#ok<CCAT> 
  end
elseif contains(usr_figselection, 'onlypoints')
  for rr = 1:2
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230804_v2_rep%d', rr)}; %#ok<CCAT> 
  end
  for rr = 1:3
    resdirs = {resdirs{:}, sprintf('ARK_3T1R_20230807_v1_rep%d', rr)}; %#ok<CCAT> 
  end
else
  error('Fall nicht definiert');
end
% Suche nach unvollständigen Teilen (falls Merge auf Cluster nicht ging)
for i = 1:length(resdirs)
  tablepath = fullfile(resdirtotal, resdirs{i}, sprintf('%s_results_table.csv', resdirs{i}));
  partslist = dir(fullfile(resdirtotal, [resdirs{i}, '_p*']));
  if exist(tablepath, 'file'), continue; end % Teile sind zusammengefügt und Ordner sind nur Relikte ohne Ergebnis-Dateien
  if ~isempty(partslist)
    fprintf('Auswertung %s unvollständig. Füge %d Teile hinzu.\n', resdirs{i}, length(partslist));
  end
  for j = 1:length(partslist)
    if ~partslist(j).isdir, continue; end
    if partslist(j).name(1) == '.', continue; end
    resdirs = {resdirs{:}, partslist(j).name}; %#ok<CCAT> 
  end
end

clear ResTab_ges
for i = 1:length(resdirs)
  fprintf('Lade Ergebnis_Verzeichnis %s\n', resdirs{i});
  tablepath = fullfile(resdirtotal, resdirs{i}, sprintf('%s_results_table.csv', resdirs{i}));
  % Finde die Zielfunktionen heraus (Auslesen der ersten Einstellungsdatei
  % reicht)
  setfile = dir(fullfile(resdirtotal, resdirs{i}, '*settings.mat'));
  if isempty(setfile)
    warning('Einstellungsdatei und damit Ergebnis %s existiert nicht', resdirs{i});
    continue
  end
  if ~exist(tablepath, 'file')
    warning('Ergebnis-Tabelle und damit Ergebnis %s existiert nicht', resdirs{i});
    continue
  end
  d1 = load(fullfile(resdirtotal, resdirs{i}, setfile(1).name));
  Set_i = cds_settings_update(d1.Set);
  objstr = '';
  for j = 1:length(Set_i.optimization.objective)
    objstr = [objstr, ',', Set_i.optimization.objective{j}]; %#ok<AGROW>
  end
  objstr = objstr(2:end); % Erstes Komma entfernen
  ResTab_i = readtable(tablepath, 'HeaderLines', 1, 'VariableNamingRule', 'preserve');
  if isempty(ResTab_i)
    warning('Ergebnis-Tabelle %s ist leer', resdirs{i});
    continue
  end

    
  % Vergleiche die Einstellungen
  if ~exist('Set_first', 'var')
    Set_first = Set_i;
    fprintf('Geladene Einstellungen: %s\n', Set_first.optimization.optname);
    fprintf('optimization.static_force_only=%d\n', Set_first.optimization.static_force_only);
    fprintf('optimization.nolinkmass=%d\n', Set_first.optimization.nolinkmass);
    fprintf('optimization.noplatformmass=%d\n', Set_first.optimization.noplatformmass);
    fprintf('task.payload.m=%1.2f\n', Set_first.task.payload.m);
    fprintf('task.payload.m=%1.2f\n', Set_first.task.payload.m);
    fprintf('optimization.tilt_base=%d, max_tilt_base=%1.0f°\n', ...
      Set_first.optimization.tilt_base, Set_first.optimization.tilt_base*180/pi*Set_first.optimization.max_tilt_base);
    fprintf('optimization.constraint_obj(6)=%1.1f (Limit Materialspannung)\n', Set_first.optimization.constraint_obj(6));
  elseif ~strcmp(usr_figselection, 'volume') % bei rein geometrischer Betrachtung egal
    % Prüfe, ob die Einstellungen gleich bleiben. Sonst Pareto-Diagramm
    % nicht vergleichbar.
    if ~(Set_first.optimization.static_force_only==Set_i.optimization.static_force_only)
      warning('Einstellung static_force_only passt nicht: %s', Set_i.optimization.optname);
      continue
    end
    if ~(Set_first.optimization.nolinkmass==Set_i.optimization.nolinkmass)
      warning('Einstellung nolinkmass passt nicht: %s', Set_i.optimization.optname);
      continue
    end
    if ~(Set_first.optimization.noplatformmass==Set_i.optimization.noplatformmass)
      warning('Einstellung noplatformmass passt nicht: %s', Set_i.optimization.optname);
      continue
    end
    if ~(Set_first.optimization.tilt_base==Set_i.optimization.tilt_base)
      % Füge diese PKM trotzdem hinzu (bezieht sich nur auf die 3T2R-PKM,
      % die zur Abgrenzung gezeigt werden sollen)
      warning('Einstellung tilt_base passt nicht: %d. Nehme trotzdem', Set_i.optimization.tilt_base);
      % continue
    end
    if ~all(Set_first.optimization.platform_size_limits==Set_i.optimization.platform_size_limits)
      warning('Einstellung platform_size_limits passt nicht: %s ([%s] statt [%s]', ...
        Set_i.optimization.optname, disp_array(1e3*Set_i.optimization.platform_size_limits, '%1.0f'), ...
        disp_array(1e3*Set_first.optimization.platform_size_limits, '%1.0f'));
    end
    % Prüfe ob die Einstellungen so sind wie geplant
    if strcmp(usr_figselection, 'springopt1')
      assert(Set_i.optimization.joint_stiffness_active_revolute==0);
      assert(isnan(Set_i.optimization.joint_stiffness_passive_revolute));
    elseif strcmp(usr_figselection, 'springopt2')
      assert(isnan(Set_i.optimization.joint_stiffness_active_revolute));
      assert(Set_i.optimization.joint_stiffness_passive_revolute==0);
    elseif strcmp(usr_figselection, 'springopt3')
      assert(isnan(Set_i.optimization.joint_stiffness_active_revolute));
      assert(isnan(Set_i.optimization.joint_stiffness_passive_revolute));
    elseif strcmp(usr_figselection, 'springopt4')
      assert(Set_i.optimization.joint_stiffness_active_revolute==0);
      assert(Set_i.optimization.joint_stiffness_passive_revolute==0);
    end
  end

  ResTab_i_headers = readtable(tablepath, 'ReadVariableNames', true);
  ResTab_i.Properties.VariableNames = ResTab_i_headers.Properties.VariableNames;
  ResTab_i = addvars(ResTab_i, repmat(resdirs(i),size(ResTab_i,1),1), 'Before', 1);
  ResTab_i.Properties.VariableNames(1) = {'OptName'};
  ResTab_i = addvars(ResTab_i, repmat({objstr},size(ResTab_i,1),1), 'Before', 8);
  ResTab_i.Properties.VariableNames(8) = {'Zielfunktion'};
  if ~exist('ResTab_ges', 'var')
    ResTab_ges = ResTab_i;
  else
    % Mache die Tabellen kompatibel
    ResTab_i_new = repmat(ResTab_ges(1,:), size(ResTab_i,1), 1);
    for vn = ResTab_ges.Properties.VariableNames
      if any(strcmp(ResTab_i.Properties.VariableNames, vn{1}))
        ResTab_i_new.(vn{1}) = ResTab_i.(vn{1});
      else
        if isa(ResTab_i_new.(vn{1}), 'cell')
          ResTab_i_new.(vn{1}) = cell(size(ResTab_i_new,1),1);
        else
          ResTab_i_new.(vn{1}) = NaN(size(ResTab_i_new,1),1);
        end
      end
    end
    ResTab_ges = [ResTab_ges; ResTab_i_new]; %#ok<AGROW>
  end
end

Robots = unique(ResTab_ges.Name);
mkdirs(fullfile(repo_dir, 'data'));
tablename = sprintf('results_all_reps_pareto_%s', usr_figselection);
writetable(ResTab_ges, fullfile(repo_dir, 'data', [tablename, '.csv']), ...
  'Delimiter', ';');
save(fullfile(repo_dir, 'data', [tablename, '.mat']), 'ResTab_ges');
fprintf('Daten gespeichert: %s\n', tablename);
continue % Plot ist nicht interessant
%% Alle Roboter einzeln durchgehen
markerlist = {'x', 's', 'v', '^', '*', 'o', '+', '<', '>', 'p', 'h', '.'}; % 'h' geht nicht bei matlab2tikz
colorlist =  {'r', 'g', 'b', 'c', 'm', 'k', imescolors.imesorange, ...
  imescolors.imesgruen, imescolors.grau, imescolors.hellrot};

I_robleg = false(length(Robots), 1);
% Betrachte nur Optimierungsläufe mit i.O.-Ergebnis
I_iO = ResTab_ges.Fval_Opt < 1e3;
% Betrachte nur Optimierungsläufe mit passender Zielfunktion
I_objmatch = contains(ResTab_ges.Zielfunktion,pareto_settings{1}) & ...
             contains(ResTab_ges.Zielfunktion,pareto_settings{2});
assert(any(I_objmatch), 'Kein Ergebnis hat die passende Zielfunktion');
fighdl = change_current_figure(10+i_usr_fig);clf; hold all;
axhdl = get(fighdl, 'children');
leghdl = [];
legstr = {}; legstr2 = {};
countrob = 0;
countmarker = 0;
ic = 0; im = 0;
Chain_Name_old = '';
Chains = {};
PlotData = cell2table(cell(0,6), 'VariableNames', {'RobotName', ...
  'Handle', 'ChainName', 'Coupling', 'MarkerNumber', 'ColorNumber'});
for i = 1:length(Robots)
  RobName = Robots{i};
  I_robmatch = strcmp(ResTab_ges.Name, RobName);
  [~, LEG_Names, ~, Coupling_i, ~, ~, ~, PName_Kin] = parroblib_load_robot(RobName, 0);
  Chain_Name = LEG_Names{1};
  II_Robi = find(I_iO&I_objmatch&I_robmatch);
  if isempty(II_Robi)
    continue
  end
  % Alternativ:
  % Generiere die Marker-Symbole bereits vor der Prüfung, ob die richtigen
  % Zielfunktionen gewählt sind. Dadurch wird sichergestellt, dass die
  % Marker in allen Pareto-Diagrammen gleich sind.
  if any(I_iO&I_robmatch)
    countmarker = countmarker + 1; % Hochzählen für die Marker und Farben
    ic = mod((countmarker-1),6)+1; % Index für Farben und Marker generieren
    im = ceil(countmarker/6);
    if im > length(markerlist), im = length(markerlist); end
    marker = [markerlist{im}, colorlist{ic}];
  end
  II_Robi = find(I_iO&I_objmatch&I_robmatch);
  if isempty(II_Robi)
    continue
  end
  fprintf('Rob %d (%s): Lade Daten (%d i.O.-Wiederholungen)\n', i, RobName, length(II_Robi));
  
  numrep_i = 0;
  pt_i = cell2table(cell(0,2), 'VariableNames', {'OptName', 'ParetoIndNr'});
  %% Stelle Pareto-Front aus verschiedenen Durchläufen zusammen
  pf_data = []; % Pareto-Front mit physikalischen Daten. Spalten bezogen auf pareto_settings
  pf_data_fval = []; % Funktionswerte der Pareto-Front (zur einfacheren Zuordnung zu den Optimierungsergebnissen)
  for j = 1:length(II_Robi) % Verschiedene Gut-Durchläufe durchgehen
    OptName = ResTab_ges.OptName{II_Robi(j)};
    LfdNr = ResTab_ges.LfdNr(II_Robi(j));
    resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
    tmp = load(resfile);
    RobotOptRes_j = tmp.RobotOptRes;
    d1 = load(fullfile(resdirtotal, OptName, [OptName, '_settings.mat']));
    Set_i = d1.Set;
    kk1 = find(strcmp(Set_i.optimization.objective, pareto_settings{1}));
    kk2 = find(strcmp(Set_i.optimization.objective, pareto_settings{2}));
    % Wähle nur Durchläufe, bei denen nur die gewählten Kriterien für
    % die Pareto-Front benutzt wurden. Sonst eher Streudiagramm.
    if isempty(kk1) || isempty(kk2) % length(Set_i.optimization.objective)>2 || 
      continue % Entweder zu viele oder die falschen Zielkriterien
    end
    
    pf_data = [pf_data; RobotOptRes_j.physval_pareto(:,[kk1,kk2])]; %#ok<AGROW>
    pf_data_fval = [pf_data_fval; RobotOptRes_j.fval_pareto(:,[kk1,kk2])]; %#ok<AGROW>
    row_i = cell(size(RobotOptRes_j.physval_pareto,1),2);
    row_i(:,1) = repmat({OptName},size(RobotOptRes_j.physval_pareto,1),1);
    for k = 1:size(RobotOptRes_j.physval_pareto,1)
      row_i{k,2} = k;
    end
    pt_i = [pt_i; row_i]; %#ok<AGROW>
    numrep_i = numrep_i + 1;
  end
  assert(~isempty(pf_data_fval), 'Pareto-Front-Daten sind leer');
  [~, Ikk] = sort(pf_data_fval(:,1)); % Sortiere nach erstem Kriterium
  pt_i = pt_i(Ikk,:);
  pf_data = pf_data(Ikk,:);
  pf_data_fval = pf_data_fval(Ikk,:);
  % Erstelle eine einzige Pareto-Front. Die Fronten mehrere Durchläufe sind
  % durch die heuristische Optimierung gegenseitig dominant.
  % Definition Pareto-Front: Siehe CoelloPulLec2004 Gl. (1)-(6)
  Idom_ges = pareto_dominance(pf_data);
  fprintf(['Durch mehrfache Durchführung der Optimierung (oder 3D-Pareto-', ...
    'Optimierung) müssen %d/%d Partikel von der Pareto-Front entfernt ', ...
    'werden.\n'], sum(Idom_ges), length(Idom_ges));
  pf_data = pf_data(~Idom_ges,:);
  pf_data_fval = pf_data_fval(~Idom_ges,:);
  pt_i = pt_i(~Idom_ges,:);
  % Speichere die Ergebnisse der Daten für diesen Roboter
  writetable(pt_i, fullfile(repo_dir, 'data', ...
    sprintf('%s_paretofront.csv', RobName)), 'Delimiter', ';');
  if isempty(pf_data)
    % Durch Filterkriterien wird der Roboter doch wieder aussortiert.
    continue;
  end
  % Ab hier ist ein Roboter erfolgreich
  countrob = countrob + 1; % Für Legende: Nur erfolgreiche PKM zählen
  I_robleg(i) = true;
  hdl = NaN(2,1);
  %% Zeichne die Ergebnisse in das Bild ein
  % Bild mit physikalischen Werten
  Set_dummy = struct('optimization', struct('objective', {pareto_settings}));
  [obj_units, objscale] = cds_objective_plotdetails(Set_dummy);
  if any(strcmp(Set_dummy.optimization.objective, 'colldist'))
    objscale(strcmp(Set_dummy.optimization.objective, 'colldist')) = 1e2;
    obj_units{strcmp(Set_dummy.optimization.objective, 'colldist')} = 'cm';
  end
  % Daten ausdünnen (sonst nicht lesbar). Wähle die Schwellwerte so, dass
  % man die Marker noch halbwegs erkennen kann
  xthresh = NaN; ythresh = NaN;
  if strcmp(pareto_settings{1}, 'footprint')
    xthresh = 0.01; % in m²
  elseif strcmp(pareto_settings{1}, 'actforce')
    xthresh = 1; % in Nm
  end
  if strcmp(pareto_settings{2}, 'colldist')
    ythresh = 3; % in mm
  elseif strcmp(pareto_settings{2}, 'actvelo')
    ythresh = 1; % in deg/s
  end
  if isnan(xthresh) || isnan(ythresh)
    error('Schwellwerte für Punkt-Zusammenfassung nicht gesetzt');
  end
  II = select_plot_indices_downsample_nonuniform(objscale(1)*pf_data(:,1), ...
    objscale(2)*pf_data(:,2), xthresh, ythresh);
  hdl=plot(objscale(1)*pf_data(II,1), objscale(2)*pf_data(II,2), 'x');
  if im <= length(markerlist)
    set(hdl, 'Marker', markerlist{im});
  else
    set(hdl, 'Marker', '.');
  end
  if ic <= length(colorlist)
    set(hdl, 'Color', colorlist{ic});
  else
    set(hdl, 'Color', 'k');
%     warning('Nicht genug Farben definiert');
  end
  xlabel(sprintf('%s in %s', pareto_settings{1}, obj_units{1}));
  ylabel(sprintf('%s in %s', pareto_settings{2}, obj_units{2}));
  grid on;
  leghdl(countrob) = hdl; %#ok<SAGROW>
  legstr{countrob} = sprintf('%d/%d (%s); %d Wdh.', i, length(Robots), Robots{i}, numrep_i); %#ok<SAGROW>
  % Legende für Diss.
  RobName2 = parroblib_format_robot_name(Robots{i}, 1);
  legstr2{countrob} = sprintf('%s %s', RobName2, combine_alignment_names(Coupling_i));
  % Eintragen
  PlotData = [PlotData; {RobName, hdl, Chain_Name, Coupling_i, im, ic}];

end
if isempty(leghdl)
  error('Nichts gezeichnet. Daten unpassend');
end

title(sprintf('Pareto-Front %s vs %s', pareto_settings{1}, pareto_settings{2}));
lh = legend(leghdl(1:countrob), legstr);
set(fighdl, 'name', usr_figselection, 'numbertitle', 'off');

%% Formatieren für Diss-Bild
name = sprintf('amunpkm_pareto_%s_%s_allvar_%s', pareto_settings{1}, pareto_settings{2}, usr_figselection);

export_fig(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)));
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));

end % i_usr_fig
