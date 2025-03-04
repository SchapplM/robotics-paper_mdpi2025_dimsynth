% Werte Pareto-Fronten verschiedener Roboter aus für LuFI-Beispiel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
usr_fig_all = {'motordiagram', 'power_vs_mass', 'power_vs_coll', 'materialstresscolldist'};
% Zusätzliche Auswertungs-Namen für Vergleich GA vs PSO
for k = 1:5
  usr_fig_all = [usr_fig_all, sprintf('motordiagram_ga_%d', k), ... 
                              sprintf('motordiagram_pso_%d', k)]; %#ok<AGROW>
end
for i_usr_fig = 1:length(usr_fig_all)
usr_figselection = usr_fig_all{i_usr_fig};
% Wähle eine Roboterkinematik für ein Detail-Bild dazu aus (leer lassen: Alle Roboter)
% 6-UPS: P6RRPRRR14V6; 6-RUS: P6RRRRRR10V6; 6-PUS: P6PRRRRR6V4
usr_robsel_all = {''};
if strcmp(usr_figselection, 'power_vs_mass')
  usr_robsel_all = {'', 'P6RRPRRR14V6', 'P6RRRRRR10V6', 'P6PRRRRR6V4'};
end
for i_usr_robsel = 1:length(usr_robsel_all)
close all % Damit nicht Legenden o.ä. in den falschen Plot gehen.
usr_robsel = usr_robsel_all{i_usr_robsel};
%% Initialisierung
if isempty(which('lufi_dimsynth_data_dir'))
  error(['You have to create a file lufi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = lufi_dimsynth_data_dir();
paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
assert(~isempty(paper_dir), 'Pfad-Init fehlerhaft');
repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));

outputdir = fullfile(paper_dir, 'paper', 'Figures');
imescolors = imes_plot_template_colors();

if strcmp(usr_figselection, 'power_vs_mass')
  pareto_settings = {'power', 'mass'};
elseif strcmp(usr_figselection, 'power_vs_coll')
  pareto_settings = {'power', 'colldist'};
elseif contains(usr_figselection, 'motordiagram')
  pareto_settings = {'actforce', 'actvelo'};
elseif contains(usr_figselection, 'materialstresscolldist')
  pareto_settings = {'materialstress', 'colldist'};
else
  error('Fall nicht definiert');
end

SerRobDB = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof', 'N');
ParRobDB = load(fullfile(fileparts(which('parroblib_path_init.m')), ...
  'sym_3T3R', 'sym_3T3R_list_act.mat'));

%% Auswahl der Ergebnisse und Ergebnisse durchgehen
% Namensschema: Siehe dimsynth/config_pareto.m
% Ergebnis-Ordner müssen im Datenverzeichnis liegen (entweder Offline-Aus-
% wertung oder neue Maßsynthese).
resdirs = {};

% Anpassung für Bild in MDPI-Paper zur Auswertung GA vs PSO:
% Mache für jede Optimierung einen eigenen Datensatz
expnames_base = {'lufipkm_20250219_optmeth_rep1'};
for k = 1:4, expnames_base = [expnames_base, sprintf('lufipkm_20250220_optmeth_rep%d', k)]; end %#ok<AGROW>

if strcmp(usr_figselection, 'materialstresscolldist')
  % lufipkm_20230219_Diss_processworkspace_allrob
  resdirs = {'lufipkm_20230409_Diss_processworkspace_allrob_matstress'};
  % resdirs = {'lufipkm_20230319_Diss_processworkspace_allrob'};
elseif contains(usr_figselection, 'motordiagram_ga')
  expnum = str2double(usr_figselection(end)); % einstellige Zahlen
  resdirs = {strrep(expnames_base{expnum}, 'optmeth', 'gamultiobj')};
elseif contains(usr_figselection, 'motordiagram_pso')
  expnum = str2double(usr_figselection(end)); 
  resdirs = {strrep(expnames_base{expnum}, 'optmeth', 'mopso')};
else
  resdirs = [resdirs, sprintf('lufipkm_20230325_Diss_processworkspace_allrob')]; 
  resdirs = [resdirs, sprintf('lufipkm_20230330_Diss_processworkspace_allrob')]; 
  resdirs = [resdirs, sprintf('lufipkm_20230413_Diss_processworkspace_allrob_default')];
  resdirs = [resdirs, sprintf('lufipkm_20230415_Diss_processworkspace_allrob_default')];
  resdirs = [resdirs, sprintf('lufipkm_20230417_Diss_processworkspace_robotsfrom0415_default2')];
  resdirs = [resdirs, sprintf('lufipkm_20230418_Diss_processworkspace_robotsfrom0415_default2')];
  resdirs = [resdirs, sprintf('lufipkm_20230418_Diss_processworkspace_motordiagram')];
  % Füge Daten aus beiden PSO/GA-Testläufen hinzu
  resdirs = [resdirs, strrep(expnames_base, 'optmeth', 'mopso')];
  resdirs = [resdirs, strrep(expnames_base, 'optmeth', 'gamultiobj')];
  resdirs = [resdirs, sprintf('lufipkm_20250221_gamultiobj_rep1')];
  resdirs = [resdirs, sprintf('lufipkm_20250221_mopso_rep1')];
  resdirs = [resdirs, sprintf('lufipkm_4to6j20250221_mopso_rep1')];
end
% Suche nach unvollständigen Teilen (falls Merge auf Cluster nicht ging)
for i = 1:length(resdirs)
  partslist = dir(fullfile(resdirtotal, [resdirs{i}, '_p*']));
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
  % Filtere alte Versuche heraus
  if any(strcmp(Set_i.optimization.objective, 'power')) && ...
      Set_i.optimization.obj_power.symmetric_speed_torque_limits == 0
    fprintf('%s hat alte Einstellung für symmetric_speed_torque_limits\n', resdirs{i})
    if ~strcmp(usr_figselection, 'materialstresscolldist')
      fprintf('Überspringe\n');
      continue
    else
      fprintf('Für diese Auswertung egal.\n');
    end
  end

  objstr = '';
  for j = 1:length(Set_i.optimization.objective)
    objstr = [objstr, ',', Set_i.optimization.objective{j}]; %#ok<AGROW>
  end
  % Korrektur der Einstellungen
  objstr = objstr(2:end); % Erstes Komma entfernen
  ResTab_i = readtable(tablepath, 'HeaderLines', 1, 'VariableNamingRule', 'preserve');
  if isempty(ResTab_i)
    warning('Ergebnis-Tabelle %s ist leer', resdirs{i});
    continue
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
% Entferne PKM, die aus der DB gelöscht wurden
[C,IA,IB] = setxor(ResTab_ges.Name, ParRobDB.ActTab.Name);
if any(IA)
  fprintf(['%d Einträge der Gesamt-Ergebnis-Tabelle beziehen sich auf ', ...
    'entfernte PKM:\n %s\n'], length(IA), disp_array(unique(ResTab_ges.Name(IA))', '%s'));
  ResTab_ges.Fval_Opt(IA) = NaN; % Entferne diese Ergebnisse
  ResTab_ges = ResTab_ges(~isnan(ResTab_ges.Fval_Opt),:);
end

mkdirs(fullfile(repo_dir, 'data'));
if isempty(usr_robsel) % Speichere zur Weiterverarbeitung durch eval_figures_pareto_groups.m
  tablename = sprintf('results_all_reps_pareto_%s', usr_figselection);
  writetable(ResTab_ges, fullfile(repo_dir, 'data', [tablename, '.csv']), ...
    'Delimiter', ';');
  save(fullfile(repo_dir, 'data', [tablename, '.mat']), 'ResTab_ges');
  fprintf('Daten gespeichert: %s\n', tablename);
  continue % Bild gar nicht erst zeichnen. Ist zu umfangreich
end
%% Alle Roboter einzeln durchgehen
markerlist = {'x', 's', 'v', '^', '*', 'o', '+', '<', '>', 'p', 'h', '.'}; % 'h' geht nicht bei matlab2tikz
colorlist =  {'r', 'g', 'b', 'c', 'm', 'k', imescolors.imesorange, ...
  imescolors.imesgruen, imescolors.grau, imescolors.hellrot};
% Reihenfolge der Marker: Erst Paarweise, dann einzeln
markercouplingassignment = [5:8, 1:4, 9:10]; % siehe align_base_coupling
Robots = unique(ResTab_ges.Name);
I_robleg = false(length(Robots), 1);
% Betrachte nur Optimierungsläufe mit i.O.-Ergebnis
I_iO = ResTab_ges.Fval_Opt < 1e3;
assert(any(I_iO), 'Kein Ergebnis ist i.O.');
% Betrachte nur Optimierungsläufe mit passender Zielfunktion
I_objmatch = contains(ResTab_ges.Zielfunktion,pareto_settings{1}) & ...
             contains(ResTab_ges.Zielfunktion,pareto_settings{2});
assert(any(I_objmatch), 'Kein Ergebnis hat die passende Zielfunktion');
% Betrachte nur den ausgewählten Roboter
I_robsel = contains(ResTab_ges.Name, usr_robsel);
assert(any(I_robsel), 'Mit usr_robsel gewählter Roboter nicht vorhanden');
if ~isempty(usr_robsel)
  Robots = Robots(contains(Robots, usr_robsel));
  % Sortiere die betrachteten Roboter nach Plattform und Gestell, damit die
  % Reihenfolge der Farben (paarweise, nicht-paarweise) zwischen Bildern
  % gleich bleibt
  Coupl_all = NaN(length(Robots), 2);
%   I_robmatch = contains(ResTab_ges.Name, usr_robsel);
  for ii = 1:length(Robots)
    RobName = Robots{ii};
    I_robmatch = strcmp(ResTab_ges.Name, RobName);
    if ~any(I_iO & I_objmatch & I_robmatch & I_robsel)
      continue
    end
    [~, LEG_Names, ~, Coupling_ii, ~, ~, ~, PName_Kin] = parroblib_load_robot(RobName, 0);
    Coupl_all(ii,:) = Coupling_ii;
  end
  [Coupl_all_sort, IIsort] = sortrows(Coupl_all,[2 1]);
  % Entferne NaN-Elemente
  IIsort = IIsort(~isnan(Coupl_all_sort(:,1)));
  Robots = Robots(IIsort);
end
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
  II_Robi = find(I_iO&I_objmatch&I_robmatch&I_robsel);
  if isempty(II_Robi)
    continue
  end
  if strcmp(usr_figselection, 'motordiagram_revolute')
    if contains(Chain_Name, 'P'), continue; end % Schubgelenk-Ketten ignorieren
  elseif strcmp(usr_figselection, 'motordiagram_prismatic')
    if ~contains(Chain_Name, 'P'), continue; end % Schubgelenk-Ketten ignorieren
  end
  % Alternativ:
  % Generiere die Marker-Symbole bereits vor der Prüfung, ob die richtigen
  % Zielfunktionen gewählt sind. Dadurch wird sichergestellt, dass die
  % Marker in allen Pareto-Diagrammen gleich sind.
  if any(I_iO&I_robmatch)
    countmarker = countmarker + 1; % Hochzählen für die Marker und Farben
    if isempty(usr_robsel) % Standard-Modus: Alle Kinematiken. Benutze Farbe für Kinematik und Marker für Gestell-Variante
      if ~strcmp(Chain_Name_old, Chain_Name)
        im = im + 1;
      end
      % Farb-Code aus vorgefertigter Liste auswählen?
      if isempty(PlotData.Coupling)
        ic = 1;
      else
        I_coupl = all(PlotData.Coupling == repmat(Coupling_i, size(PlotData.Coupling,1), 1), 2);
        if any(I_coupl) % Diese Koppel-Variante gab es schon mal. Farbe ist gesetzt.
          II_coupl = find(I_coupl, 1, 'first');
          ic = PlotData.ColorNumber(II_coupl);
        else
          ic = max(PlotData.ColorNumber) + 1;
        end
      end
    else % Einzel-Kinematik-Modus: Marker und Farben für Koppelgelenk-Varianten (Gestell und Plattform)
      if isempty(PlotData.Coupling) % Erster Eintrag. Setze auf 1
        im = 1;
        ic = 1;
      else
        if any(PlotData.Coupling(:,1)==Coupling_i(1)) % Gestell-Variante gab es schon. Hole Marker
          II_basecoupl = find(PlotData.Coupling(:,1)==Coupling_i(1), 1, 'first');
          im = PlotData.MarkerNumber(II_basecoupl);
        else % Variante neu im Plot. Erhöhe Marker-Nummer
          % Setze auf vordefinierte Marker-Nummer, damit die Marker in
          % allen Plots gleich sind
          im = find(markercouplingassignment==Coupling_i(1));
          % Alte Version: Einfach hochzählen entsprechend der Sortierung
%           im = max(PlotData.MarkerNumber(:)) + 1;
        end
        % Das gleiche für die Marker
        if any(PlotData.Coupling(:,2)==Coupling_i(2)) % Plattform-Variante gab es schon. Hole Farbe
          II_plfcoupl = find(PlotData.Coupling(:,2)==Coupling_i(2), 1, 'first');
          ic = PlotData.ColorNumber(II_plfcoupl);
        else % Variante neu im Plot. Erhöhe Farb-Nummer
          ic = max(PlotData.ColorNumber(:)) + 1;
        end
      end
    end
  end
  Chain_Name_old = Chain_Name;

  fprintf('Rob %d (%s): Lade Daten (%d i.O.-Wiederholungen)\n', i, RobName, length(II_Robi));
  
  countrob = countrob + 1; % Für Legende: Nur erfolgreiche PKM zählen
  I_robleg(i) = true;
  numrep_i = 0;
  pt_i = cell2table(cell(0,2), 'VariableNames', {'OptName', 'ParetoIndNr'});
  %% Stelle Pareto-Front aus verschiedenen Durchläufen zusammen
  pf_data = []; % Pareto-Front mit physikalischen Daten. Spalten bezogen auf pareto_settings
  pf_data_fval = []; % Funktionswerte der Pareto-Front (zur einfacheren Zuordnung zu den Optimierungsergebnissen)
  for j = 1:length(II_Robi) % Verschiedene Gut-Durchläufe durchgehen
    OptName = ResTab_ges.OptName{II_Robi(j)};
    LfdNr = ResTab_ges.LfdNr(II_Robi(j));
    resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
    if ~exist(resfile, 'file')
      warning('Ergebnis-Datei existiert nicht. Daten inkonsistent. %s', resfile);
      continue
    end
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
  fprintf(['Durch mehrfache Durchführung der Optimierung müssen %d/%d Partikel ', ...
    'von der Pareto-Front entfernt werden.\n'], sum(Idom_ges), length(Idom_ges));
  pf_data = pf_data(~Idom_ges,:);
  pf_data_fval = pf_data_fval(~Idom_ges,:);
  pt_i = pt_i(~Idom_ges,:);
  % Speichere die Ergebnisse der Daten für diesen Roboter
  writetable(pt_i, fullfile(repo_dir, 'data', ...
    sprintf('%s_paretofront.csv', RobName)), 'Delimiter', ';');
  %% Zeichne die Ergebnisse in das Bild ein
  % Bild mit physikalischen Werten
  Set_dummy = struct('optimization', struct('objective', {pareto_settings}));
  [obj_units, objscale] = cds_objective_plotdetails(Set_dummy);
  objscale(strcmp(Set_dummy.optimization.objective, 'power')) = 1e-3;
  obj_units{strcmp(Set_dummy.optimization.objective, 'power')} = 'kW';
  % Daten ausdünnen (sonst nicht lesbar). Wähle die Schwellwerte so, dass
  % man die Marker noch halbwegs erkennen kann
  xthresh = NaN; ythresh = NaN;
  if strcmp(pareto_settings{1}, 'power')
    xthresh = 1e-3; % in kW
  end
  if strcmp(pareto_settings{1}, 'actforce')
    xthresh = 1;
  end
  if strcmp(pareto_settings{2}, 'mass')
    ythresh = 1; % in kg
  end
  if strcmp(pareto_settings{2}, 'colldist')
    ythresh = 1e-3; % in m
  end
  if strcmp(pareto_settings{2}, 'actvelo')
    ythresh = 1;
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

title(sprintf('Pareto-Front %s vs %s', pareto_settings{1}, pareto_settings{2}));
lh = legend(leghdl(1:countrob), legstr);
set(fighdl, 'name', usr_figselection, 'numbertitle', 'off');

%% Formatieren für Diss-Bild
name = sprintf('lufipkm_pareto_%s_%s_allvar_%s', pareto_settings{1}, pareto_settings{2}, usr_figselection);
if ~isempty(usr_robsel)
  name = [name, '_', usr_robsel]; %#ok<AGROW> 
end
% Lade Bezeichnungen aus Dissertation, damit die Nummern konsistent sind
tableexportpath = fullfile(paper_dir, ...
  'paper', 'Structural_Synthesis');
PKM_Table_Diss = readtable(fullfile(tableexportpath, ...
    sprintf('%s_PKM_LatexNumbers.csv', '3T3R')), 'Delimiter', ';');
figure_format_publication(); % Schriftart ändern (Achtung: Scheint Reihenfolge zu ändern und macht Legende unsichtbar)
title(''); sgtitle('');
% set(axhdl, 'xlim', [18, 67]);
if strcmp(usr_figselection, 'power_vs_mass')
  pareto_settings = {'power', 'mass'};
  xlabel('Actuators'' rated power in kW');
  ylabel('Moving parts'' mass in kg');
else
  error('Nicht definiert');
end

% Bild erst ohne Legende speichern
delete(lh);
set_size_plot_subplot(fighdl, ...
  16, 6, axhdl,...
  0.06,0.006,0.02,0.12,0.04,0);
export_fig(fighdl, fullfile(outputdir, sprintf('%s_nolegend.pdf', name)));
saveas(fighdl, fullfile(outputdir, sprintf('%s_nolegend.fig', name)));
% Dann mit Legende
if isempty(usr_robsel)
  set_size_plot_subplot(fighdl, ...
    16, 8, axhdl,...
    0.06,0.006,0.27,0.12,0.04,0); % lrud xy
else
  % Legende ist kleiner, falls Gestell Marker ist
  set_size_plot_subplot(fighdl, ...
    16, 7, axhdl,...
    0.06,0.006,0.22,0.12,0.04,0); % lrud xy
end
%% Legende neu erstellen

% lfhdl = legendflex(leghdl, legstr2, 'anchor', {'n','n'}, ...
%   'ref', fighdl, ... % an Figure ausrichten (mitten oben)
%   'buffer', [5 -1], ... % Kein Versatz notwendig, da mittig oben
%   'ncol', 0, 'nrow', 4, ... % eine Zeile für Legende
%   'fontsize', 8, ...
%   'xscale', 0.5, ... % Kleine Symbole
%   'padding', [1,1,0], ... % Leerraum reduzieren
%   'box', 'on', ...
%   'interpreter', 'latex');
%% Legende für Kinematik vs Gestell-Plattform-Kombination
if isempty(usr_robsel)
% Legende erstellen für Kinematik-Typ (Marker-Symbol)
[~,I] = unique(PlotData.ChainName);
leghdl_markers = NaN(length(I), 1);
legstr_markers = cell(length(I), 1);
for i = 1:length(I)
  leghdl_markers(i) = plot(axhdl, NaN,NaN,['k',get(PlotData.Handle(I(i)), 'Marker')]);
  [NLEG, LEG_Names, Actuation, Coupling_i, ~, ~, ~, PName_Kin, PName_Legs] = ...
    parroblib_load_robot(PlotData.RobotName{I(i)}, 2);
  % Bestimme allgemeine Beinkette, falls es Varianten-Kette ist
  ilc = strcmp(SerRobDB.Names, LEG_Names{1});
  ChainNameGen_i = SerRobDB.Names{SerRobDB.AdditionalInfo(ilc, 3)};
  PName_Kin_Gen = sprintf('P%d%s', NLEG, ChainNameGen_i(3:end));
  % Nummer des aktuierten Gelenks
  PName_Kin_ActVar = [PName_Legs, 'GxPx', sprintf('A%d', Actuation{1})];
  % Allgemeinen Beinketten-Namen bestimmen (ohne Variante)
  % Für Einbindung der Latex-Roboternummern mit Tikz:
%   legstr_markers{i} = sprintf('PR\\,\\ref{restabrow:%s}.\\ref{restabrow:%s}', ...
%     PName_Kin_Gen, PName_Kin_ActVar);
  % Latex-Roboternummern direkt einfügen
  I_tab = strcmp(PKM_Table_Diss.RobotName_Diss, PName_Kin_ActVar);
  if ~any(I_tab), error('PKM %s nicht in Diss-Tabelle gefunden', PName_Legs); end
  legstr_markers{i} = sprintf('PR %d.%d', ...
    PKM_Table_Diss.NumberMain(I_tab), PKM_Table_Diss.NumberVar(I_tab));

  if contains(PlotData.RobotName{I(i)}, 'V') % Kurzer Name mit technischen Gelenken
    legstr_markers{i} = [legstr_markers{i}, sprintf(' (%s)', ...
      parroblib_format_robot_name(PlotData.RobotName{I(i)}, 4))];
  else % PKM-Name mit Akzenten
    legstr_markers{i} = [legstr_markers{i}, sprintf(' (%s)', ...
      parroblib_format_robot_name(PlotData.RobotName{I(i)}, 1))];
  end
end
if exist('lf_markers_hdl', 'var'), delete(lf_markers_hdl); end
lf_markers_hdl = legendflex(leghdl_markers, legstr_markers, 'anchor', {'n','n'}, ...
  'ref', fighdl, ... % an Figure ausrichten (mitten oben)
  'buffer', [1 -1], ... % Kein Versatz notwendig, da mittig oben
  'ncol', 4, 'nrow', 0, ... % eine Zeile für Legende
  'fontsize', 10, ...
  'FontName', 'Times New Roman', ...
  'title', 'Marker: leg chain kinematic structure', ...
  'xscale', 0.6, ...
  ... % Für matlab2tikz: [-12,-14,8] (kein zusätzlicher Abstand mehr. Latex macht ??.?? aus den PR-Nummern für die Bestimmung der Größe der Text-Box)
  'padding', [1,1,3], ...
  'box', 'on', ...
  'interpreter', 'latex');
% Legende erstellen für G-/P-Variante (Farbe)
[~,I] = unique(PlotData.ColorNumber);
leghdl_colors = NaN(length(I), 1);
legstr_colors = cell(length(I), 1);
for i = 1:length(I)
  leghdl_colors(i) = plot(axhdl, NaN,NaN,['k','-']);
  set(leghdl_colors(i), 'color', get(PlotData.Handle(I(i)), 'Color'));
  legstr_colors{i} = combine_alignment_names(PlotData.Coupling(I(i),:));
end
% Ändere die Reihenfolge der Farb-Legendeneinträge auf alphabetisch
[~,II] = sort(legstr_colors);
leghdl_colors = leghdl_colors(II);
legstr_colors = legstr_colors(II);
if exist('lf_colors_hdl', 'var'), delete(lf_colors_hdl); end
lf_colors_hdl = legendflex(leghdl_colors, legstr_colors, 'anchor', {'ne','ne'}, ...
  'ref', axhdl, ... % an Figure ausrichten (mitten oben)
  'buffer', [-6 -6], ... % Etwas näher an Ecke ranrücken
  'ncol', 3, 'nrow', 0, ... % eine Zeile für Legende
  'fontsize', 10, ...
  'FontName', 'Times New Roman', ...
  'title', 'Color: joint alignment', ...
  'xscale', 0.6, ... % Kleine Symbole
  'padding', [1,1,2], ... % Abstand nach Text für matlab2tikz: [1 1 10]
  'box', 'on', ...
  'interpreter', 'latex');
% Legende größer machen, damit Text noch passt bei matlab2tikz
% lfpos = get(lf_colors_hdl, 'Position');
% set(lf_colors_hdl, 'Position', [lfpos(1)-50,lfpos(2),lfpos(3)+50,lfpos(4)]);
end
%% Legende für Gestellgelenk und Plattformgelenk (bei einem Roboter)
if ~isempty(usr_robsel)
% Legende erstellen für Gestell-Typ (Marker-Symbol)
for jj = 1:2 % 1=Gestell (Marker), 2=Plattform (Farbe)
[~,I] = unique(PlotData.Coupling(:,jj));
leghdl_jj = NaN(length(I), 1);
legstr_jj = cell(length(I), 1);
for i = 1:length(I)
  % Benennung der GP-Varianten aus der Diss für Legende benutzen.
  tmpstr = combine_alignment_names(PlotData.Coupling(I(i),:));
  [tokens, match] = regexp(tmpstr, '\\mbox{(\w)-(\w)}', 'tokens', 'match');
  if jj == 1
    leghdl_jj(i) = plot(axhdl, NaN, NaN, ['k', get(PlotData.Handle(I(i)), 'Marker')]);
    legstr_jj{i} = tokens{1}{1}; % erster Teil des Strings (vor Bindestrich) für Gestell
  else
    leghdl_jj(i) = plot(axhdl, NaN,NaN,['k','-']);
    set(leghdl_jj(i), 'color', get(PlotData.Handle(I(i)), 'Color'));
    legstr_jj{i} = tokens{1}{2}; % zweiter Teil des Strings (nach Bindestrich) für Plattform
  end
end
legstr_jj_orig = legstr_jj;
% Benenne in vollständige Bezeichnungen um
gpdict = {{'C', 'conical (pairw.)'}, {'R', 'radial (pairw.)'}, ...
          {'T', 'tangential (pairw.)'}, {'V', 'vertical (pairw.)'}, ...
          {'c', 'conical'}, {'p', 'parallel'},{'r', 'radial'}, ...
          {'t', 'tangential'}, {'v', 'vertical'}};
if jj == 1 % Gestell
  % Nehme vollständige Liste von oben
else % Plattform
  % Bei Hexapod gibt es wegen des Kugelgelenks nur v und V.
  if length(intersect(legstr_jj, {'v', 'V'})) == length(legstr_jj)
    gpdict = {{'V', 'pairwise'}, {'v', 'circular'}};
  end
end
for ii = 1:length(legstr_jj)
  for kk = 1:length(gpdict)
    if jj == 1 % Gestell: Abkürzung des Typs noch mit hinschreiben
      legstr_jj{ii} = regexprep(legstr_jj{ii} , ['^',gpdict{kk}{1},'$'], ...
        [gpdict{kk}{1}, ': ', gpdict{kk}{2}]);
    else % Plattform: Nicht machen
      legstr_jj{ii} = regexprep(legstr_jj{ii} , ['^',gpdict{kk}{1},'$'], gpdict{kk}{2});
    end
  end
end
% Ändere die Reihenfolge der Farb-Legendeneinträge auf alphabetisch
% (Großbuchstaben zuerst, also paarweise Anordnungen)
[~,II] = sort(char(legstr_jj_orig));
leghdl_jj = leghdl_jj(II);
legstr_jj = legstr_jj(II);

% Legenden in Plot einfügen
if jj == 1
  leghdl_markers = leghdl_jj;
  legstr_markers = legstr_jj;
  if exist('lf_markers_hdl', 'var'), delete(lf_markers_hdl); end
  lf_markers_hdl = legendflex(leghdl_markers, legstr_markers, 'anchor', {'n','n'}, ...
    'ref', fighdl, ... % an Figure ausrichten (mitten oben)
    'buffer', [1 -1], ... % Kein Versatz notwendig, da mittig oben
    'ncol', 5, 'nrow', 0, ... % eine Zeile für Legende
    'fontsize', 10, ...
    'FontName', 'Times New Roman', ...
    'title', 'Marker: base coupling', ...
    'xscale', 0.6, ...
    'padding', [1,1,3], ...
    'box', 'on', ...
    'interpreter', 'latex');
else
  leghdl_colors = leghdl_jj;
  legstr_colors = legstr_jj;
  if exist('lf_colors_hdl', 'var'), delete(lf_colors_hdl); end
  lf_colors_hdl = legendflex(leghdl_colors, legstr_colors, 'anchor', {'ne','ne'}, ...
    'ref', axhdl, ... % an Figure ausrichten (mitten oben)
    'buffer', [-6 -6], ... % Etwas näher an Ecke ranrücken
    'ncol', 3, 'nrow', 0, ... % drei Zeilen für Legende
    'fontsize', 10, ...
    'FontName', 'Times New Roman', ...
    'title', 'Color: platform coupling', ...
    'xscale', 0.6, ... % Kleine Symbole
    'padding', [1,1,2], ... % Abstand nach Text für matlab2tikz: [1 1 10]
    'box', 'on', ...
    'interpreter', 'latex');
end
end
end
%% Speichern
% saveas(10*ps, fullfile(outputdir, sprintf('%s.fig', name)));
export_fig(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)));
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));

% Funktioniert zwar, aber Dokument ist dann nicht mehr kompilierbar:
% matlab2tikz(fullfile(outputdir, sprintf('%s.tex', name)), ...
%   'parseStrings', false, ...
%   'strictFontSize', false);
end % i_usr_robsel
end % i_usr_fig
