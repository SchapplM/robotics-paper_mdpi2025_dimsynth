% Werte Pareto-Fronten verschiedener Roboter aus unter Gruppierung von PKM
% 
% Vorher ausführen:
% * Maßsynthese
% * Aggregation der Daten mit eval_figures_pareto.m
% 
% Mögliche Bilder:
% materialstresscolldist, materialstresscolldist_34joints, materialstresscolldist_56joints
% (Zuerst ohne Beschränkung der Gelenke, damit Marker bestimmt werden)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2025-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all
usr_fig_all = {'materialstresscolldist', 'materialstresscolldist_34joints', ... % 1-2
  'materialstresscolldist_56joints', ... % 3
  'motordiagram', 'motordiagram_revolute', 'motordiagram_prismatic', ... % 4-6
  'power_vs_mass', 'power_vs_coll', ... % 7-8
  'linkdiam_vs_coll', 'linkdiam_vs_coll_34joints'}; % 9-10
% Zusätzliche Auswertungs-Namen für Vergleich GA vs PSO
for k = 1:5
  usr_fig_all = [usr_fig_all, ...
    sprintf('motordiagram_revolute_ga_%d', k), ... 
    sprintf('motordiagram_prismatic_ga_%d', k), ... 
    sprintf('motordiagram_revolute_pso_%d', k), ... 
    sprintf('motordiagram_prismatic_pso_%d', k)]; %#ok<AGROW>
end

for i_usr_fig = 1:length(usr_fig_all)
usr_figselection = usr_fig_all{i_usr_fig};
% Die Gruppen lieber neu generieren, damit keine veralteten Daten geladen werden
usr_loadgroups = false;
usr_loadmarkers = false;
if any(strcmp(usr_figselection, {'materialstresscolldist_34joints', ...
    'materialstresscolldist_56joints', ...
    'linkdiam_vs_coll', 'linkdiam_vs_coll_34joints', ...
    'power_vs_mass_34joints', 'power_vs_coll_34joints'})) || ...
    any(contains(usr_figselection, {'motordiagram_revolute', ...
      'motordiagram_prismatic'}))
  % Bei der Auswahl aus einer größeren Menge sollten die Gruppen und Marker
  % immer neu geladen werden, damit es keine Überschneidungen gibt.
  usr_loadgroups = true;
  usr_loadmarkers = true;
end

%% Initialisierung

if contains(usr_figselection, 'power_vs_mass')
  pareto_settings = {'power', 'mass'};
elseif contains(usr_figselection, 'power_vs_coll')
  pareto_settings = {'power', 'colldist'};
elseif contains(usr_figselection, 'motordiagram')
  pareto_settings = {'actforce', 'actvelo'};
elseif contains(usr_figselection, 'materialstresscolldist')
  pareto_settings = {'materialstress', 'colldist'};
elseif contains(usr_figselection, 'linkdiam_vs_coll')
  pareto_settings = {'linkdiam', 'colldist'};
else
  error('Fall nicht definiert');
end

resdirtotal = lufi_dimsynth_data_dir();
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));

datadir = fullfile(repo_dir,'data');
outputdir = fullfile(paper_dir, 'paper', 'Figures');

SerRobDB = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof', 'N');
% Lade Bezeichnungen aus Dissertation, damit die Nummern konsistent sind
% (CSV-Tabellen im Paper-Repo speichern). TODO: Könnte vereinheitlicht
% werden, wenn Struktursynthese-Paper publiziert ist.
tableexportpath = fullfile(paper_dir, ...
  'paper', 'Structural_Synthesis');
PKM_Table_Diss = readtable(fullfile(tableexportpath, ...
    sprintf('%s_PKM_LatexNumbers.csv', '3T3R')), 'Delimiter', ';');
%% Daten laden (als mat)
usr_figselection1 = usr_figselection;
if contains(usr_figselection, 'materialstresscolldist')
  usr_figselection1 = 'materialstresscolldist';
elseif contains(usr_figselection, 'motordiagram_revolute_pso') || ...
       contains(usr_figselection, 'motordiagram_revolute_ga')
  usr_figselection1 = strrep(usr_figselection, '_revolute', '');
elseif contains(usr_figselection, 'motordiagram_prismatic_pso') || ...
       contains(usr_figselection, 'motordiagram_prismatic_ga')
  usr_figselection1 = strrep(usr_figselection, '_prismatic', '');
elseif contains(usr_figselection, 'motordiagram') || contains(usr_figselection, 'linkdiam_vs_coll')
  usr_figselection1 = 'motordiagram';
elseif contains(usr_figselection, 'power_vs_mass')
  usr_figselection1 = 'power_vs_mass';
end
fprintf('Starte Pareto-Diagramm %s. Datenquelle. %s\n', usr_figselection, usr_figselection1);
% kommt aus eval_figures_pareto.m
datafile = fullfile(datadir, sprintf('results_all_reps_pareto_%s.mat', usr_figselection1));
assert(exist(datafile, 'file'), 'Daten zu Pareto-Bild existieren nicht');

tmp = load(datafile);
ResTab_ges = tmp.ResTab_ges;

% Betrachte nur Optimierungsläufe mit i.O.-Ergebnis
I_iO = ResTab_ges.Fval_Opt < 1e3;
% ResTab_ges = readtable(fullfile(datadir, 'results_all_reps_pareto.csv'), ...
%   'Delimiter', ';', 'ReadVariableNames', true);
Robots = unique(ResTab_ges.Name(I_iO));

% Namen der Roboter laden (enthält auch Eigenschaften zu Kardan-Gelenken)
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
dimsynth_expeval_robot_names(Robots, namestablepath, 'edit', 'skip', 0);
ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');

%% Roboter gruppieren
% usr_figselection1 = usr_figselection;
if contains(usr_figselection, 'materialstresscolldist')
  % Benutze die Plot-Marker des gemeinsamen Bildes, damit es keine
  % Dopplungen gibt.
  usr_figselection1 = 'materialstresscolldist';
elseif contains(usr_figselection, 'linkdiam_vs_coll')
  usr_figselection1 = 'motordiagram';
elseif contains(usr_figselection, 'motordiagram')
  % Benutze die Plot-Marker des gemeinsamen Bildes, damit es keine
  % Dopplungen gibt.
  usr_figselection1 = 'motordiagram';
else % Benutze dieselben Marker und Gruppen für alle Bilder
  usr_figselection1 = 'motordiagram';
end
groupsfile = fullfile(datadir, sprintf('robot_groups_%s_nodata.mat', usr_figselection1));
if ~usr_loadgroups || ~exist(groupsfile, 'file')
  fprintf('Gruppiere die PKM aus den Ergebnissen\n');
  % Erzeuge die Namen der Gruppen durch weglassen der P- und G-Nummer
  RobotGroups = cell2table(cell(0,5), 'VariableNames', {'GroupName', ...
    'Robots', 'ResultsFound', 'PlotMarker', 'KinematicsNumber'});
  t_lastlog = tic()-1e3;
  for i = 1:length(Robots)
    if toc(t_lastlog) > 10
      fprintf('Verarbeite PKM %d/%d. %1.1fs bis hier.\n', i, length(Robots), toc(t_lastlog));
      t_lastlog = tic();
    end
    row_i = cell(1,5);
    [NLEG, LEG_Names, Actuation, Coupling, ~, symrob, EE_dof0, ...
      PName_Kin, PName_Legs] = parroblib_load_robot(Robots{i}, 2);
    row_i{1} = [PName_Legs, 'G'];
    % Finde alle Roboter, die zur Gruppe passen und schreibe sie in die
    % zweite Spalte
    row_i{2} = Robots(contains(Robots,row_i{1}));
    % Dritte Spalte, Marker, ob i.O.-Ergebnisse in Maßsynthese
    row_i{3} = 0;
    % Vierte Spalte, Plot-Marker (in allen Bildern konsistent)
    row_i{4} = '';
    % Haupt-Kinematiken entsprechend der Diss-Tabelle merken:
    PName_Kin_ActVar = [PName_Legs, 'GxPx', sprintf('A%d', Actuation{1})];
    I_tab = strcmp(PKM_Table_Diss.RobotName_Diss, PName_Kin_ActVar);
    if ~any(I_tab)
      warning('Eintrag %d in PKM_Table_Diss nicht gefunden: %s', i, Robots{i});
    end
    row_i{5} = [PKM_Table_Diss.NumberMain(I_tab), PKM_Table_Diss.NumberVar(I_tab)];
    RobotGroups = [RobotGroups; row_i]; %#ok<AGROW> 
  end
  % Sortiere die Roboter-Gruppen nach Nummer in der Datenbank (damit
  % indirekt nach Beinkettenname und Variante; V10 sollte nicht vor V2 kommen)
  % Nicht machen, da sonst die Marker-Reihenfolge geändert wird gegenüber
  % der Version vor Oktober 2024. Dann müssen die ganzen Kinematik-Bilder
  % neu gemacht werden. Für zukünftige Fallstudien sollte das hier geändert
  % werden.
  % [tmp,I] = sortrows(RobotGroups.KinematicsNumber);
  % RobotGroups = RobotGroups(I,:);
  [~,I] = unique(RobotGroups.GroupName);
  RobotGroups = RobotGroups(I,:);
  save(groupsfile, 'RobotGroups');
else
  % Lade vorrangig die Daten mit bereits generierten Markern
  groupsfile2 = fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection1));
  if exist(groupsfile2, 'file') % Lade die Datei mit den Markern. Nehme dann die gleichen Marker
    tmp = load(groupsfile2);
  else
    tmp = load(groupsfile);
  end
  RobotGroups = tmp.RobotGroups;
  fprintf('Gruppierung der PKM aus Datei geladen\n');
end

%% Alle Roboter einzeln durchgehen
markerlist = {'s', 'x', 'v', '^', '*', '+', '<', '>', 'o', 'd', 'p', 'h', '|'}; % '.' % folgende gehen nicht in matlab2tikz: 'h', '|', '_'
colorlist =  {'r', 'g', 'b', 'c', 'm', 'k'};

I_robleg = false(length(Robots), 1);
% Betrachte nur Optimierungsläufe mit passender Zielfunktion
if any(strcmp(pareto_settings, 'linkdiam')) % ist keine Zielfunktion, wird hier aber später so behandelt
  pareto_settings_search = pareto_settings(~strcmp(pareto_settings, 'linkdiam'));
  I_objmatch = contains(ResTab_ges.Zielfunktion,pareto_settings_search{1});
else
  I_objmatch = contains(ResTab_ges.Zielfunktion,pareto_settings{1}) & ...
               contains(ResTab_ges.Zielfunktion,pareto_settings{2});
end
fighdl = figure(10);clf; hold all;
axhdl = get(fighdl, 'children');
logscales = false(2,1); % Logarithmische Skalierung schon hier festlegen
if strcmp(pareto_settings{1}, 'materialstress')
  logscales(1) = true; % waagerechte Achse log.
end
leghdl = [];
legstr = {};
countrob = 0;
ic = 0;
im = 0;
clear Set_first % Sonst funktioniert die Prüfung der Existenz der Variablen nicht
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  I_groupmatch = contains(ResTab_ges.Name, GroupName);
  II_Robi = find(I_iO&I_objmatch&I_groupmatch);
  if isempty(II_Robi)
    fprintf('Gruppe %d/%d: Keine Ergebnisse\n', i, size(RobotGroups,1));
    continue
  end
  % Filtere anhand der Anzahl der Gelenke in den Beinketten
  ChainShortName = ResTab_NameTrans.Chain_ShortName{find(contains(ResTab_NameTrans.PKM_Name, GroupName), 1, 'first')};
  if contains(usr_figselection, '34joints') && length(ChainShortName) > 4 ||...
     contains(usr_figselection, '56joints') && length(ChainShortName) < 5
    fprintf('Überspringe Gruppe %d (%s) (falsche Gelenkzahl)\n', i, ChainShortName);
    continue
  end
  if contains(usr_figselection, 'revolute') && any(ChainShortName=='P') || ...
     contains(usr_figselection, 'prismatic') && ~any(ChainShortName=='P')
    fprintf('Überspringe Gruppe %d (%s) (falsche Aktuierung)\n', i, ChainShortName);
    continue
  end
  % Generiere die Marker-Symbole bereits vor der Prüfung, ob die richtigen
  % Zielfunktionen gewählt sind. Dadurch wird sichergestellt, dass die
  % Marker in allen Pareto-Diagrammen gleich sind.
  if any(I_iO&I_groupmatch)
    im = im+1; % Index für Farben und Marker generieren
%     if ic == 0 || all( ~all(repmat(RobotGroups.KinematicsNumber(i,:), i-1, 1) ...
%          == RobotGroups.KinematicsNumber(1:i-1,:),2) )
    I_prev_plot = find(RobotGroups.ResultsFound(1:i-1) > 0);
    if all( RobotGroups.KinematicsNumber(i,1) ~= RobotGroups.KinematicsNumber(I_prev_plot,1) ) || ...
        ic == 0 % Ist notwendig, falls erste Kinematiken übersprungen werden
      ic = ic+1; % Farbe aktualisieren, wenn neue Haupt-Nummer genommen wird
      fprintf('PKM-Nummer %d.%d: Neue Farbe Nr. %d\n', RobotGroups.KinematicsNumber(i,1), ...
        RobotGroups.KinematicsNumber(i,2), ic);
    end
    % Prüfe, ob genau diese Kombination aus Farbe und Marker schon belegt ist
    if any(strcmp([markerlist{im}, colorlist{ic}], RobotGroups.PlotMarker(I_prev_plot)))
      fprintf('Marker-Farb-Kombination %s existiert schon. Suche neue.\n', [markerlist{im}, colorlist{ic}])
      success = false;
      for jj = [im:length(markerlist), 1:im-1]
        if ~any(strcmp([markerlist{jj}, colorlist{ic}], RobotGroups.PlotMarker(I_prev_plot)))
          im = jj;
          fprintf('Wähle die Marker %s stattdessen\n', markerlist{im});
          success = true;
          break;
        end
      end
      if ~success
        error('Kein eindeutiger Marker bestimmbar');
      end
    end
    marker = markerlist{im};
    color = colorlist{ic};
    if ~usr_loadmarkers || isempty(RobotGroups.PlotMarker{i}) % Behalte die schon geladenen Marker
      RobotGroups.PlotMarker{i} = [marker,color];
    else
      marker = RobotGroups.PlotMarker{i}(1);
      color = RobotGroups.PlotMarker{i}(2);
      fprintf('Benutze gespeicherten Marker %s%s\n', marker, color);
    end
    % Fange mit der jeweiligen Zählung von vorne an
    if ic == length(colorlist)
      fprintf('Belege die Farben hiernach wieder von vorne\n');
      ic = 1;
    end
    if im == length(markerlist), im = 1; end
  end
  fprintf('Gruppe %d/%d (%s): Lade Daten (%d i.O.-Optimierungs-Läufe)\n', i, ...
    size(RobotGroups,1), GroupName, length(II_Robi));
  % disp(ResTab_ges.OptName(II_Robi));
  numrep_i = 0;
  pt_i = cell2table(cell(0,6), 'VariableNames', {'OptName', 'RobName', 'LfdNr', ...
    'ParetoIndNr', 'Crit1', 'Crit2'});
  %% Stelle Pareto-Front aus verschiedenen Durchläufen zusammen
  pf_data = []; % Pareto-Front mit physikalischen Daten. Spalten bezogen auf pareto_settings
  for j = 1:length(II_Robi) % Verschiedene Gut-Durchläufe durchgehen
    OptName = ResTab_ges.OptName{II_Robi(j)};
    RobName = ResTab_ges.Name{II_Robi(j)};
    LfdNr = ResTab_ges.LfdNr(II_Robi(j));
    % Lade die Ergebnisse
    setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
    d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
    Set_j = cds_settings_update(d1.Set);
    resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
    if ~exist(resfile, 'file')
      warning('Ergebnis-Datei existiert nicht. Daten inkonsistent. %s', resfile);
      continue
    end
    tmp = load(resfile);
    RobotOptRes_j = tmp.RobotOptRes;
    
    % Vergleiche die Einstellungen
    if ~exist('Set_first', 'var')
      Set_first = Set_j;
      fprintf('Geladene Einstellungen: %s\n', Set_first.optimization.optname);
      fprintf('optimization.static_force_only=%d\n', Set_first.optimization.static_force_only);
      fprintf('optimization.nolinkmass=%d\n', Set_first.optimization.nolinkmass);
      fprintf('optimization.noplatformmass=%d\n', Set_first.optimization.noplatformmass);
      fprintf('task.payload.m=%1.2f\n', Set_first.task.payload.m);
      fprintf('optimization.platform_size_limits=[%s]mm\n', disp_array( ...
        Set_first.optimization.platform_size_limits*1e3, '%1.0f'));
      fprintf('optimization.constraint_obj(6)=%1.1f (Limit Materialspannung)\n', Set_first.optimization.constraint_obj(6));
    else
      % Prüfe, ob die Einstellungen gleich bleiben. Sonst Pareto-Diagramm
      % nicht vergleichbar.
      assert(Set_first.optimization.static_force_only==Set_j.optimization.static_force_only, ...
        'Einstellung static_force_only passt nicht');
      assert(Set_first.optimization.nolinkmass==Set_j.optimization.nolinkmass, ...
        'Einstellung nolinkmass passt nicht');
      assert(Set_first.optimization.noplatformmass==Set_j.optimization.noplatformmass, ...
        'Einstellung noplatformmass passt nicht');
    end
    % Stelle alle Daten zur Pareto-Front zusammen. Vermeide NaN-Werte, die
    % auftreten, wenn die Nebenbedingungen verletzt werden und dies trotz-
    % dem Teil der Pareto-Front wird.
    I_notnan = all(~isnan(RobotOptRes_j.physval_pareto),2);
    fval_pareto = RobotOptRes_j.fval_pareto(I_notnan,:);
    p_val_pareto = RobotOptRes_j.p_val_pareto(I_notnan,:);
    physval_pareto = RobotOptRes_j.physval_pareto(I_notnan,:);
    % Erweitere die Daten um die Segmentdurchmesser als Pseudo-Zielfunktion
    I_dop = strcmp(RobotOptRes_j.Structure.desopt_pnames, 'linkdiameter');
    physval_pareto = [physval_pareto, RobotOptRes_j.desopt_pval_pareto(:,I_dop)]; %#ok<AGROW>
    fval_pareto = [fval_pareto, zeros(size(RobotOptRes_j.desopt_pval_pareto,1),1)]; %#ok<AGROW>
    Set_j.optimization.objective = [Set_j.optimization.objective, 'linkdiam'];

    % Lade die Detail-Ergebnisse mit allen Zwischenwerten
    resfile2 = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
    if exist(resfile2, 'file')
      if any(strcmp(Set_j.optimization.objective, 'linkdiam'))
        warning('Noch nicht implementiert!');
      else
        tmp2 = load(resfile2);
        RobotOptRes2_j = cds_convert_pareto_details2front(tmp2.PSO_Detail_Data);
        I_detail_j = all(RobotOptRes2_j.fval_all<1e3,2);
        fval_pareto = [fval_pareto; RobotOptRes2_j.fval_all(I_detail_j,:)]; %#ok<AGROW> 
        p_val_pareto = [p_val_pareto; RobotOptRes2_j.pval_all(I_detail_j,:)]; %#ok<AGROW> 
        physval_pareto = [physval_pareto; RobotOptRes2_j.physval_all(I_detail_j,:)]; %#ok<AGROW> 
      end
    else
      % warning('Vollständige Daten liegen nicht vor. Damit nachträgliche Filterung nicht belastbar')
      % Dummy-Variable anlegen (für Schleife im nächsten Schritt)
      PSO_Detail_Data_j = struct('pval', NaN(3,size(p_val_pareto,2),2), ...
        'physval', NaN(3,size(physval_pareto,2),2), ...
        'fval', NaN(3,size(fval_pareto,2),2));
    end
    
    kk1 = find(strcmp(Set_j.optimization.objective, pareto_settings{1}));
    kk2 = find(strcmp(Set_j.optimization.objective, pareto_settings{2}));
    % Wähle nur Durchläufe, bei denen nur die gewählten Kriterien für
    % die Pareto-Front benutzt wurden. Sonst eher Streudiagramm.
    if isempty(kk1) || isempty(kk2)
      continue % nicht die richtigen Zielkriterien
    end
    
    % Index der Roboter, die Isomorphismen des aktuellen darstellen.
    % Nur sinnvoll, wenn aktueller Roboter ein allgemeiner Typ ist.
    Ir_this = strcmp(ResTab_NameTrans.PKM_Name, RobName); % Index in Namens-Tabelle
    ChainName = ResTab_NameTrans.Chain_Name{Ir_this};
    % Lade Daten der Beinkette
    serroblibpath=fileparts(which('serroblib_path_init.m'));
    NLJ = str2double(ChainName(2));
    mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', ...
      NLJ), sprintf('S%d_list.mat',NLJ));
    l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
    Ir_db = find(strcmp(l.Names_Ndof, ChainName)); % Index dieses Roboters in Datenbank
    Id_db = find(l.AdditionalInfo(:,3)==Ir_db & l.AdditionalInfo(:,2)); % Index daraus abgeleiteter Varianten
    if ~isempty(Id_db) % es gibt aus diesem Roboter abgeleitete Varianten (und es ist nicht dieser Roboter selbst)
      Chain_DepVarMdl = l.Names_Ndof{Id_db}; % Name der Beinkette der Varianten
      % Index der Roboter, die Isomorphismen des aktuellen darstellen
      Ir_jointiso = find(strcmp(ResTab_NameTrans.Chain_Name, Chain_DepVarMdl) & ...
        ResTab_NameTrans.Gnum==ResTab_NameTrans.Gnum(Ir_this) & ...
        ResTab_NameTrans.Pnum==ResTab_NameTrans.Pnum(Ir_this));
      if isempty(Ir_jointiso), Ir_jointiso = []; end % gegen 0x1
    else
      Ir_jointiso = [];
    end
    % Merke Indizes von Gelenken, deren DH-Parameter für diesen allgemeinen
    % Roboter nicht Null sein müssen. Wenn sie Null sind, entspricht dieser
    % Roboter der Variante und beide wären Isomorphismen.
    Ijoints_notnull = false(1,NLJ);
    % Alle Gelenke dieses Roboters (bzw. der Beinkette) durchgehen und
    % prüfen, ob daraus abgeleitete Varianten in der Optimierung
    % existieren)
    for II_jointiso = Ir_jointiso % Es kann mehrere Varianten geben
      % Bestimme Zeichenkette der technischen Gelenke (z.B. "PRUR")
      TJ_this = ResTab_NameTrans.Chain_ShortName{Ir_this};
      TJ_jointiso = ResTab_NameTrans.Chain_ShortName{II_jointiso};
      % Roboter-Klasse definieren und deren Methoden nutzen
      RS_this = serroblib_create_robot_class(ChainName);
      RS_var = serroblib_create_robot_class(Chain_DepVarMdl);
      RS_this.set_techjoints(TJ_this);
      RS_var.set_techjoints(TJ_jointiso);
      % Unterschied der technischen Gelenke
      Ijointdiff = RS_this.DesPar.joint_type ~= RS_var.DesPar.joint_type;
      % Bei einer Kardan-Gruppe wird immer der zweite a/d-Parameter zu Null
      % gesetzt (der erste Parameter führt zum Ort des Gelenks)
      ignorenext = false; % Merker zum Finden des jeweils zweiten Eintrags
      for iii = 1:length(RS_var.DesPar.joint_type)
        if ~Ijointdiff(iii), continue; end
        if RS_var.DesPar.joint_type(iii) == 2 && ~ignorenext
          ignorenext = true;
        elseif RS_var.DesPar.joint_type(iii) == 2 && ignorenext
          Ijoints_notnull(iii) = true;
          ignorenext = false;
        end
      end
    end
    % Markiere jeweils die zweiten DH-Parameter einer U-Gelenk-Gruppe
    Ip_DH_Ujoint = false(length(RobotOptRes_j.Structure.varnames),length(Ijoints_notnull));
    for ii = 1:length(Ijoints_notnull)
      if Ijoints_notnull(ii)
        Ip_DH_Ujoint(contains(RobotOptRes_j.Structure.varnames, sprintf('d%d', ii)),ii) = true;
        Ip_DH_Ujoint(contains(RobotOptRes_j.Structure.varnames, sprintf('a%d', ii)),ii) = true;
      end
    end
    if any(Ip_DH_Ujoint(:))
      fprintf(['Gelenkkette %s (%s): Es existiert die Variante %s (%s). Folgende DH-', ...
        'Parameter müssen ungleich Null sein, damit von Variante verschieden: %s\n'], ...
        ChainName, TJ_this, Chain_DepVarMdl, TJ_jointiso, ...
        disp_array(RobotOptRes_j.Structure.varnames(any(Ip_DH_Ujoint,2)), '[%s]'));
    end
    % Index für Skalierungsparameter
    Ip_scale = strcmp(RobotOptRes_j.Structure.varnames, 'scale');
    % Index für Steigungsparameter (für spätere Auswertung und Auswahl)
    Ip_G4elev = strcmp(RobotOptRes_j.Structure.varnames, 'base_morph_coneelev');
    % Steigungsparameter (für spätere Auswertung und Auswahl)
    if any(Ip_G4elev)
      alpha_pareto_k = p_val_pareto(:,Ip_G4elev);
      % Wähle nur Partikel auf der Pareto-Front aus, bei denen die Steigung
      % einen Wert ungleich 0 hat. Sonst ist der schräge Fall exakt
      % identisch mit dem senkrechten Fall (nicht sinnvoll für Auswertung)
      I_select = abs(alpha_pareto_k) > 5*pi/180 & abs(alpha_pareto_k) < 175*pi/180;
      if any(~I_select)
        fprintf('Schließe %d Lösungen aus, da die Steigung fast senkrecht ist\n', sum(~I_select));
      end
      if Set_j.optimization.min_inclination_conic_base_joint == 5*pi/180 && any(~I_select)
        warning('Eigentlich dürften keine auszuschließenden Lösungen entstanden sein')
      end
    else
      % Steigung ist kein Parameter. Nehme alle.
      I_select = true(size(p_val_pareto,1),1);
    end
    % Bei Strukturen mit Drehgelenken: Prüfe deren Abstand. Wenn es die
    % gleiche Struktur auch mit Kardan-Gelenken gibt, sind die beiden
    % gleichwertig und der Abstand muss bei dieser allgemeinen Struktur
    % mindestens 20mm betragen (willkürlicher Wert)
    if any(Ip_DH_Ujoint(:))
      for ii = 1:length(Ijoints_notnull)
        if ~Ijoints_notnull(ii), continue; end
        DH_ad_pareto_k_rel = p_val_pareto(:, Ip_DH_Ujoint(:,ii));
        DH_ad_pareto_k = DH_ad_pareto_k_rel .* ...
          repmat(p_val_pareto(:,Ip_scale), 1, sum(Ip_DH_Ujoint(:,ii)));
        DH_ad_pareto_k_norm = sqrt(DH_ad_pareto_k(:,1).^2+DH_ad_pareto_k(:,2).^2); % TODO: Hier gab es einen Fehler.
        n_select_prior = sum(I_select);
        I_select_jointdist = any(abs(DH_ad_pareto_k_norm) > 20e-3);
        I_select = I_select & I_select_jointdist;
        fprintf('Schließe %d Lösungen aus, da die Gelenke zu nah beieinander sind\n', ...
          n_select_prior - sum(I_select));
        if Set_j.optimization.min_joint_distance == 20e-3 && any(~I_select_jointdist)
          warning('Eigentlich dürften keine auszuschließenden Lösungen entstanden sein')
        end
      end
    end
    
    % Wende Filter an. Die ersten Generationen werden mehrfach gefiltert.
    % Ist rechentechnisch egal und Code so kompakter.
    p_val_pareto = p_val_pareto(I_select,:);
    physval_pareto = physval_pareto(I_select,:);
    fval_pareto = fval_pareto(I_select,:);
    % Entferne Duplikate. Führt unten zu Logik-Fehlern
    [p_val_pareto, I] = unique(p_val_pareto, 'rows');
    physval_pareto = physval_pareto(I,:);
    fval_pareto = fval_pareto(I,:);
    % Bereits hier wieder Pareto-Dominanz prüfen. Sonst führt das weiter
    % unten zur Überlastung durch die Vielzahl an Partikeln
    if size(physval_pareto,1) > 1
      Idom_ges = pareto_dominance(physval_pareto);
      physval_pareto = physval_pareto(~Idom_ges,:);
      p_val_pareto = p_val_pareto(~Idom_ges,:);
      fval_pareto = fval_pareto(~Idom_ges,:);
    end
    if any(Ip_G4elev)
      alpha_pareto = p_val_pareto(:,Ip_G4elev);
    else
      % Kein Steigungsparameter. Gebe Steigung mit 0 an.
      alpha_pareto = zeros(size(p_val_pareto,1),1);
    end
    pf_data = [pf_data; physval_pareto(:,[kk1,kk2])]; %#ok<AGROW>
    row_i = cell(size(physval_pareto,1),6);
    row_i(:,1:3) = repmat({OptName,RobName,LfdNr},size(row_i,1),1);
    for k = 1:length(alpha_pareto)
      % Nummer des Pareto-Partikels in den ursprünglichen Daten
      k_pareto = find(all(RobotOptRes_j.p_val_pareto == ...
        repmat(p_val_pareto(k,:),size(RobotOptRes_j.p_val_pareto,1),1),2));
      if isempty(k_pareto)
        k_pareto = NaN; % Partikel aus den Zwischenständen, nicht aus finaler Front
      elseif length(k_pareto)>1
        k_pareto = k_pareto(1); % Nehme erstes Vorkommnis. Evtl Duplikat in Pareto-Front
      end
      row_i{k,4} = k_pareto;
      row_i{k,5} = physval_pareto(k,kk1); % strcmp(Set_j.optimization.objective, 'actforce')
      row_i{k,6} = physval_pareto(k,kk2); % strcmp(Set_j.optimization.objective, 'positionerror')
%       row_i{k,7} = alpha_pareto(k);
      assert(all(~isnan(physval_pareto(k,:))), 'physval_pareto ist NaN.');
    end
    pt_i = [pt_i; row_i]; %#ok<AGROW>
    numrep_i = numrep_i + 1;
  end
  if ~isempty(pf_data)
    [~, Ikk] = sort(pf_data(:,1)); % Sortiere nach erstem Kriterium
    pt_i = pt_i(Ikk,:);
    pf_data = pf_data(Ikk,:);
  end
  % Erstelle eine einzige Pareto-Front. Die Fronten mehrere Durchläufe sind
  % durch die heuristische Optimierung gegenseitig dominant.
  % Definition Pareto-Front: Siehe CoelloPulLec2004 Gl. (1)-(6)
  Idom_ges = pareto_dominance(pf_data);
  fprintf(['Durch mehrfache Durchführung der Optimierung müssen %d/%d Partikel ', ...
    'von der Pareto-Front entfernt werden.\n'], sum(Idom_ges), length(Idom_ges));
  pf_data_orig = pf_data; % Daten ohne Entfernen der dominierten
  pf_data = pf_data_orig(~Idom_ges,:);

  % Markiere diese Gruppe als i.O. (da Ergebnisse vorliegen)
  RobotGroups.ResultsFound(i) = size(pf_data,1);

  pt_i = pt_i(~Idom_ges,:);
  % Speichere die Ergebnisse der Daten für diesen Roboter (bzw. die Gruppe)
  writetable(pt_i, fullfile(datadir, ...
    sprintf('group_%s_paretofront_%s.csv', GroupName, usr_figselection)), 'Delimiter', ';');
  save(fullfile(datadir, sprintf('group_%s_paretofront_%s.mat', GroupName, ...
    usr_figselection)), 'pt_i');
  
  if isempty(pf_data)
    % Durch Filterkriterien wird die Gruppe doch wieder aussortiert.
    continue;
  end
  % Ab hier ist ein Roboter erfolgreich
  countrob = countrob + 1; % Für Legende: Nur erfolgreiche PKM zählen
  I_robleg(i) = true;
  %% Zeichne die Ergebnisse in das Bild ein
  % Bild mit physikalischen Werten
  
  % Zeichne eine Linie, falls mehrere Punkte beieinander sind. Ansonsten
  % zeichne einzelne Marker
  % Abstand zwischen zwei Punkten, bei dem die Linie unterbrochen werden
  % soll. Wird per Hand eingestellt. Annahme: Dann kann man nicht davon
  % ausgehen, dass kleine Änderungen der Kinematikparameter möglich sind.
  % (zusammenhängender Lösungsraum)
  % Zahlen beziehen sich auf die Werte vor der Einheitenkorrektur (deg, µm)
  if strcmp(pareto_settings{1}, 'power')
    maxgaplength_x = 2; % bezogen auf x-Achse, in W
  elseif strcmp(pareto_settings{1}, 'actforce')
    maxgaplength_x = 2;
  elseif strcmp(pareto_settings{1}, 'materialstress')
    maxgaplength_x = 5; % Lücke in %
  elseif strcmp(pareto_settings{1}, 'linkdiam')
    maxgaplength_x = 1; % Lücke in mm
  else
    error('Value for maxgaplength_x not set yet');
  end
  xminmax = minmax2(pf_data(:,1)');
  if strcmp(pareto_settings{1}, 'power')
    dx = 0.1; % Prüfe in 0.1W-Schritten
  elseif strcmp(pareto_settings{1}, 'actforce')
    dx = 0.1;
  elseif strcmp(pareto_settings{1}, 'materialstress')
    dx = 5; % in Prozent
  elseif strcmp(pareto_settings{1}, 'linkdiam')
    dx = 1; % in mm
  else
    error('Value for dx not set yet');
  end
  gapdata_x = (xminmax(1):dx:xminmax(2))';
  gapdata_y = NaN(length(gapdata_x),1);
  for jj = 1:length(gapdata_x)
    mindist_jj_x = min(abs(gapdata_x(jj) - pf_data(:,1)));
    if mindist_jj_x > maxgaplength_x/2 % der Abstand gilt immer zu beiden Seiten
      % Setze NaN, damit hier eine Lücke gemacht wird
      gapdata_y(jj) = NaN;
    else
      gapdata_y(jj) = 0; % Setze Null, damit Wert am Ende entfernt wird
    end
  end
  pf_plotdata = [pf_data; [gapdata_x(gapdata_y~=0), gapdata_y(gapdata_y~=0)]];
  % Daten neu stetig sortieren
  [~,II] = sort(pf_plotdata(:,1));
  pf_plotdata = pf_plotdata(II,:);
  % Dünne die Daten aus, damit die Marker sich nicht komplett gegenseitig
  % überdecken (nur auf der Linie)
  % Daten ausdünnen (sonst nicht lesbar). Wähle die Schwellwerte so, dass
  % man die Marker noch halbwegs erkennen kann
  xthresh = NaN; ythresh = NaN;
  if strcmp(pareto_settings{1}, 'power')
    xthresh = 2; % in W
  end
  if strcmp(pareto_settings{1}, 'actforce')
    if contains(usr_figselection, 'prismatic')
      xthresh = 0.4; % kN
    else
      xthresh = 0.15; % kNm
    end
  end
  if strcmp(pareto_settings{1}, 'materialstress')
    xthresh = 1.10; % (Log-Skalierung. Nächster Punkt muss 10% weiter sein als vorheriger (also ein Kästchen bei log.-Skalierung)
  end
  if strcmp(pareto_settings{1}, 'linkdiam')
    xthresh = 1; % in mm
  end
  if strcmp(pareto_settings{2}, 'mass')
    ythresh = 1; % in kg
  end
  if strcmp(pareto_settings{2}, 'colldist')
    ythresh = 1.5; % in cm
  end
  if strcmp(pareto_settings{2}, 'actvelo')
    if contains(usr_figselection, 'prismatic')
      ythresh = 0.5; % m/s
    else
      ythresh = 3; % deg/s
    end
  end
  assert(~any(isnan([xthresh;ythresh])), 'xthresh oder ythresh nicht gesetzt');
  [obj_units, objscale] = cds_objective_plotdetails(Set_j);
  % Ändere Skalierung für die erreichten Werte
  objscale(strcmp(Set_j.optimization.objective, 'power')) = 1e-3; % kW
  objscale(strcmp(Set_j.optimization.objective, 'actforce')) = 1e-3; % kN
  if strcmp(pareto_settings{2}, 'actvelo') && contains(usr_figselection, 'revolute')% Benutze deg/s
     objscale(strcmp(Set_j.optimization.objective, 'actvelo')) = 180/pi;
  end
  objscale(strcmp(Set_j.optimization.objective, 'colldist')) = -1e2; % Benutze positive Werte (und cm)
  I_pm = select_plot_indices_downsample_nonuniform(objscale(kk1)*pf_data(:,1), ...
    objscale(kk2)*pf_data(:,2), xthresh, ythresh, logscales);
  fprintf('Es werden %d/%d Marker zum Plotten gewählt nach dem Ausdünnen.\n', ...
    sum(I_pm), length(I_pm));
  % Linienstil bestimmen und zeichnen
%   if all(isnan(gapdata_y)) % Alle Werte sind einzeln, keine Linie zu zeichnen
    linestyle = '';
%   else % TODO: Hier kann noch eine andere Strichelung genommen werden
%     linestyle = '-';
%   end

  % Zeichne durchgezogene Linie (nur bei Materialspannungs-Paretobild)
  if contains(usr_figselection, 'materialstresscolldist')
    plot(objscale(kk1)*pf_plotdata(:,1), objscale(kk2)*pf_plotdata(:,2), [color,linestyle]);
  end
  % Zeichne Marker
  plot(objscale(kk1)*pf_data(I_pm,1), objscale(kk2)*pf_data(I_pm,2), [color,marker]);
  % Dummy-Handle für Legende
  hdl = plot(0, NaN, [color, linestyle,marker]);
  xlabel(sprintf('%s in %s', pareto_settings{1}, obj_units{kk1}));
  ylabel(sprintf('%s in %s', pareto_settings{2}, obj_units{kk2}));
  grid on;
  leghdl(countrob) = hdl; %#ok<SAGROW>
  legstr{countrob} = sprintf('%s; %d Wdh.', GroupName, numrep_i); %#ok<SAGROW>
end
set(fighdl, 'numbertitle', 'off', 'name', sprintf('%s_groups', usr_figselection));
title(sprintf('Pareto-Front %s vs %s (Fitness-Werte physikalisch)', pareto_settings{1}, pareto_settings{2}));
lh = legend(leghdl, legstr);
% set(fighdl, 'name', sprintf('pareto_groups_%s_%s', pareto_settings{1}(1:4), pareto_settings{2}(1:4)));
% saveas(fighdl, fullfile(outputdir, sprintf('figure_pareto_groups_%s_%s.fig', pareto_settings{1}, pareto_settings{2})));
% export_fig(fighdl, fullfile(outputdir, sprintf('figure_pareto_groups_%s_%s.pdf', pareto_settings{1}, pareto_settings{2})));

%% Gruppen abspeichern (mit Ergebnis-Anzahl)
% GroupTab = cell2table(cell(0,3), 'VariableNames', {'GroupName', 'RobotNames', 'StructName'});
save(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)), 'RobotGroups');
writetable(RobotGroups, fullfile(datadir, sprintf('robot_groups_%s.csv', ...
  usr_figselection)), 'Delimiter', ';');

%% Ausgewählte Lösung in Pareto-Diagramm eintragen
% Aus eval_existing_design.m
if ~contains(usr_figselection, 'materialstresscolldist') && ...
    ~contains(usr_figselection, 'revolute') && ...
    ~contains(usr_figselection, 'linkdiam_vs_coll') && ...
    ~any(contains(usr_figselection,{'_pso', '_ga'})) % für PSO/GA-Auswertung egal
erg_eng = load(fullfile(datadir, 'detail_result_engineering_solution.mat'));
if any(erg_eng.fval > 1e3)
  error('Gewählte eigene Lösung ist ungültig')
end
assert(Set_first.optimization.static_force_only==erg_eng.Set.optimization.static_force_only, ...
  'Einstellung static_force_only passt nicht für Konstruktionslösung. Nicht vergleichbar.');
assert(Set_first.optimization.nolinkmass==erg_eng.Set.optimization.nolinkmass, ...
  'Einstellung nolinkmass passt nicht für Konstruktionslösung. Nicht vergleichbar.');
assert(Set_first.optimization.noplatformmass==erg_eng.Set.optimization.noplatformmass, ...
  'Einstellung noplatformmass passt nicht für Konstruktionslösung. Nicht vergleichbar.');
Set_eng = erg_eng.Set;
kk1_eng = find(strcmp(Set_eng.optimization.objective, pareto_settings{1}));
kk2_eng = find(strcmp(Set_eng.optimization.objective, pareto_settings{2}));
if isempty(kk1_eng) || isempty(kk2_eng)
  error(['Die gesuchten Kriterien fehlen in den Daten zur eigenen Lösung. ', ...
    'eval_existing_design entweder neu ausführen oder überarbeiten!']);
end
row_eng = RobotGroups(strcmp(RobotGroups.GroupName, 'P6RRPRRR14V6G'),:);
row_eng.GroupName = 'Eng. Sol.';
legstr{countrob+1} = 'engineering solution';
[obj_units_eng, objscale_eng] = cds_objective_plotdetails(Set_eng);
% Skalierung manuell ändern (konsistent zu oben)
objscale_eng(strcmp(Set_eng.optimization.objective, 'actforce')) = 1e-3;
objscale_eng(strcmp(Set_eng.optimization.objective, 'power')) = 1e-3;
objscale_eng(strcmp(Set_eng.optimization.objective, 'colldist')) = -1e2; % cm

% Wähle den gleichen Markertyp wie PUU-v, aber größer und schwarz
leghdl(countrob+1) = plot(objscale_eng(kk1_eng)*erg_eng.physval(kk1_eng), ...
  objscale_eng(kk2_eng)*erg_eng.physval(kk2_eng), 'kh', 'MarkerSize', 10);
else
erg_eng = []; % Anzeige, dass kein Ergebnis geladen wurde
end

%% Leistungs-Kennlinie einzeichnen
if contains(usr_figselection, 'motordiagram_revolute')
  ydata = 1:1:150; % deg/s
  for pwr = [1 2 3 5] % in kW
    xdata = pwr ./ (ydata*pi/180); % in kNm bei vorgegebener Bemessungsleistung
    plot(axhdl(1), xdata, ydata, 'k--');
  end
  % Textfelder werden an PDF-Formatierung vom Ende des Skriptes angepasst
  texthdl_pwr = NaN(4,1);
  texthdl_pwr(1) = text(2.3522, 22.8113, 0,'1kW');
  texthdl_pwr(2) = text(4.2794, 26.0040,0,'2kW');
  texthdl_pwr(3) = text(5.3784, 33.7951,0,'3kW');
  texthdl_pwr(4) = text(6.6252, 38.0520,0,'5kW');
  for ii = 1:length(texthdl_pwr)
    set(texthdl_pwr(ii), 'BackgroundColor', 'w');
  end
elseif contains(usr_figselection, 'motordiagram_prismatic')
  ydata = 0.3:0.02:1.3; % m/s
  for pwr = [1 2 3] % in kW
    xdata = pwr ./ (ydata); % in kN bei vorgegebener Bemessungsleistung
    plot(axhdl(1), xdata, ydata, 'k--');
  end
  % Textfelder werden an PDF-Formatierung vom Ende des Skriptes angepasst
  % Setze zwei der Textfelder (1kW und 2kW) direkt in das Zoom-Fenster
  texthdl_pwr1 = text(4.0005, 0.8361 ,0,'3kW');
  % Kein weißer Hintergrund. Bild ist zu voll.
end
%% Formatieren für Diss-Bild
% uiopen(fullfile(outputdir, sprintf('figure_pareto_groups_%s_%s.fig', ...
%   pareto_settings{1}, pareto_settings{2})), 1);

figure_format_publication(axhdl);
if strcmp(pareto_settings{1}, 'actforce') && contains(usr_figselection, 'motordiagram_revolute')
  xlabel('Actuator torque in kNm');
  xlim([0.5,7]);
elseif strcmp(pareto_settings{1}, 'actforce') && contains(usr_figselection, 'motordiagram_prismatic')
  xlabel('Actuator force in kN');
  xlim([0.8,24]);
elseif strcmp(pareto_settings{1}, 'actforce') && contains(usr_figselection, 'motordiagram')
  xlabel('Actuator force/torque (mixed units)');
elseif strcmp(pareto_settings{1}, 'power')
  xlabel('Actuators'' rated power in kW');
  xlim([0.7, 8]); % Jenseits von 3kW sind nicht viele Punkte, aber das wird durch das Zoom-Fenster überschrieben
elseif strcmp(pareto_settings{1}, 'linkdiam')
  xlabel('Link diameter in mm');
  xlim([10, 50.5]);
elseif strcmp(pareto_settings{1}, 'materialstress')
  xlabel('Material stress in % of yield stress limit');
  if strcmp(usr_figselection, 'materialstresscolldist_34joints')
    xlim([4, 3e3]);
    set(gca, 'xtick', [5, 10, 50, 100, 500, 1000]); % sonst nur glatte Zehner-Potenz
  else
    xlim([40, 1e4]);
    set(gca, 'xtick', [50, 100, 200, 500, 1000, 2000, 5000, 9e3]);
  end
else
  error('Value for xlabel not set yet');
end
if logscales(1)
  set(axhdl, 'XScale', 'Log');
end
if strcmp(pareto_settings{2}, 'mass')
  ylabel('Moving parts'' mass in kg');
  ylim([50, 109]);
elseif strcmp(pareto_settings{2}, 'colldist')
  ylabel('Collision distance in cm');
  ylimorig = get(axhdl, 'ylim');
  if strcmp(usr_figselection, 'materialstresscolldist_56joints')
    ylim([-0.5 14.5]); % es gibt keine mit größerem Kollisionsabstand
  else
    ylim([-0.5 16.5]); % passend zu Skalierung oben
  end
  set(axhdl, 'Ydir', 'reverse'); % da oben die Werte negiert wurden. 0 soll oben sein
elseif strcmp(pareto_settings{2}, 'actvelo') && contains(usr_figselection, 'motordiagram_revolute')
  ylabel('Actuated-joint velocity in deg/s');
elseif strcmp(pareto_settings{2}, 'actvelo') && contains(usr_figselection, 'motordiagram_prismatic')
  ylabel('Actuated-joint velocity in m/s');
  ylim([0.35, 1.25]);
elseif strcmp(pareto_settings{2}, 'actvelo') && contains(usr_figselection, 'motordiagram')
  ylabel('Actuated-joint velocity (mixed units)');
else
  error('Value for ylabel not set yet');
end
title('');

if contains(usr_figselection, 'materialstresscolldist')
  % Markierung bei 100%
  plot([100;100], [-5;20], 'k-', 'LineWidth', 1);
end

% Plot-Grenzen manuell setzen (für Bild ohne Legende)
% if strcmp(usr_figselection, 'power_vs_mass')
%   set(axhdl, 'xlim', [16 80]);
% end

% Bild erst ohne Legende speichern
delete(lh);

set_size_plot_subplot(fighdl, ...
  18.3,5,axhdl,...
  0.045,0.005,0.01,0.17,0,0);


% Erzeuge Zoom-Bild
if strcmp(usr_figselection, 'power_vs_coll')
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [950e-3, 2050e-3; -0.3, 6.5]);
  set(zoomhdl1, 'position', [0.5041 0.2222 0.4678 0.7196]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.4893 0.3228 -0.2215 0.4339]);
end


name = sprintf('lufipkm_pareto_%s_%s_groups_%s_nolegend', ...
  pareto_settings{1}, pareto_settings{2}, usr_figselection);
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));
% export_fig(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)));
exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)),'ContentType','vector');

% Plot-Größe anhand der Legende auswählen
if strcmp(usr_figselection, 'motordiagram_prismatic')
  % Diagramm ist nicht so informationsdicht, nicht so hoch, dafür Zoom
  set_size_plot_subplot(fighdl, ...
    18.3,9,axhdl,...
    0.045,0.01,0.34,0.09,0,0);
elseif length(legstr) > 40
  set_size_plot_subplot(fighdl, ...
    18.3,12,axhdl,...
    0.045,0.01,0.32,0.07,0,0);
elseif length(legstr) > 36 % getestet für linkdiam_vs_coll, 15.01.2025
  set_size_plot_subplot(fighdl, ...
    18.3,10,axhdl,...
    0.045,0.005,0.32,0.08,0,0);
elseif length(legstr) > 32 % getestet für power_vs_mass, 14.04.2023
  set_size_plot_subplot(fighdl, ...
    18.3,10,axhdl,...
    0.045,0.005,0.39,0.08,0,0);
elseif length(legstr) > 28 % getestet für motordiagram_prismatic am 16.4.23
  set_size_plot_subplot(fighdl, ...
    18.3,10,axhdl,...
    0.045,0.005,0.35,0.09,0,0);
elseif length(legstr) > 24 % getestet für materialstresscolldist_34joints
  set_size_plot_subplot(fighdl, ...
    18.3,8,axhdl,...
    0.04,0.005,0.28,0.10,0,0);
elseif length(legstr) > 20
  set_size_plot_subplot(fighdl, ...
    18.3,8,axhdl,...
    0.04,0.005,0.24,0.10,0,0);
elseif length(legstr) > 16 % getestet für materialstresscolldist_56joints
  set_size_plot_subplot(fighdl, ...
    18.3,10,axhdl,...
    0.04,0.005,0.19,0.08,0,0);
elseif length(legstr) <= 8 % getestet für motordiagram_revolute am 16.4.23
  set_size_plot_subplot(fighdl, ...
    18.3,6,axhdl,...
    0.045,0.005,0.19,0.15,0,0);
else
  set_size_plot_subplot(fighdl, ...
    18.3,9,axhdl,...
    0.045,0.005,0.32,0.12,0,0);
end

% Plot-Grenzen manuell setzen, so dass noch rechts Platz für die große
% Legende ist. Hier also nur noch xlim einstellen.
% if strcmp(usr_figselection, 'power_vs_mass') 
% %   set(axhdl, 'xlim', [16 97]);
% else
%   error('Value for ps not set yet');
% end

% Bezug zwischen Plot-Legende und gespeicherten Gruppen abspeichern
InfoTab = cell2table(repmat(legstr(:), 1,6));
InfoTab.Properties.VariableNames = {'TextFix', 'TextLink', 'GroupName', ...
  'GroupNum', 'NumberMain', 'NumberVar'};

% Legende und Roboternamen aktualisieren
legstr2 = cell(length(legstr),1);
legstr2_PR_link = legstr2;
if ~isempty(erg_eng)
  legstr2{end} = legstr{end}; % Engineering Solution
end
% Ersetze den ursprünglichen Legendennamen durch die Bezeichnung aus der Diss
for i = 1:length(legstr)
  for j = 1:size(RobotGroups,1) % muss nicht identisch mit Legende sein
    if contains(legstr{i}, RobotGroups.GroupName{j})
      % Suche Latex-Namen
      Robots_j = RobotGroups.Robots{j};
      if ~isa(Robots_j, 'cell'), Robots_j = {Robots_j}; end
      for k = 1:length(Robots_j)
        I_k = strcmp(ResTab_NameTrans.PKM_Name, Robots_j{k});
        % Bestimme allgemeine Beinkette, falls es Varianten-Kette ist
        ilc = strcmp(SerRobDB.Names, ResTab_NameTrans.Chain_Name{I_k});
        ChainNameGen_i = SerRobDB.Names{SerRobDB.AdditionalInfo(ilc, 3)};
        % Allgemeinen Namen für Latex-Verweis generieren
        [NLEG, LEG_Names, Actuation, Coupling_i, ~, ~, ~, PName_Kin, PName_Legs] = ...
          parroblib_load_robot(ResTab_NameTrans.PKM_Name{I_k}, 2);
        PName_Kin_Gen = sprintf('P%d%s', NLEG, ChainNameGen_i(3:end));
        PName_Kin_ActVar = [PName_Legs, 'GxPx', sprintf('A%d', Actuation{1})];
        % Für Einbindung der Latex-Roboternummern mit Tikz:
        legstr2_PR_link{i} = sprintf('PR\\,\\ref{restabrow:%s}.\\ref{restabrow:%s}', ...
          PName_Kin_Gen, PName_Kin_ActVar);
        % Latex-Roboternummern direkt einfügen
        I_tab = strcmp(PKM_Table_Diss.RobotName_Diss, sprintf('%sGxPxA%d', PName_Legs, Actuation{1}));
        assert(any(I_tab), 'Nicht in Datenbank gefunden');
        % Bei MDPI-Paper
        legstr2{i} = ''; %sprintf('PR %d.%d', ...
          % PKM_Table_Diss.NumberMain(I_tab), PKM_Table_Diss.NumberVar(I_tab));
        
        if length(ResTab_NameTrans.Chain_ShortName{I_k}) < str2double(LEG_Names{1}(2)) % Name zeichnet sich nicht durch Substitution von Gelenken aus
            % alte Abfrage: contains(ResTab_NameTrans.PKM_Name{I_k}, 'V') % Kurzer Name mit technischen Gelenken
          legstr2{i} = [legstr2{i}, sprintf('%s', ...
            parroblib_format_robot_name(ResTab_NameTrans.PKM_Name{I_k}, 4))];
        else % PKM-Name mit Akzenten
          legstr2{i} = [legstr2{i}, sprintf('%s', ...
            parroblib_format_robot_name(ResTab_NameTrans.PKM_Name{I_k}, 1))];
        end
        InfoTab.TextFix{i} = legstr2{i};
        InfoTab.TextLink{i} = legstr2_PR_link{i};
        InfoTab.GroupName{i} = RobotGroups.GroupName{j};
        InfoTab.GroupNum{i} = j;
        InfoTab.NumberMain{i} = PKM_Table_Diss.NumberMain(I_tab);
        InfoTab.NumberVar{i} = PKM_Table_Diss.NumberVar(I_tab);
        % Unterstreichung mit Akzent funktioniert nicht mit Latex-Interpreter.
%         legstr2{i} = strrep(legstr2{i}, 'P', '\underline{P}');
        break;
      end
      break;
    end
    if ~isempty(legstr2{i})
      break;
    end
  end
end
% Ändere die Reihenfolge der Legende so, dass die PR-Nummern aufsteigend sind
[~,I] = sort(cell2mat(InfoTab.NumberMain(1:countrob)) + ...
             cell2mat(InfoTab.NumberVar( 1:countrob))/100);
legstr3 = [legstr2(I); legstr2(countrob+1:end)];
leghdl3 = leghdl([I(:)', countrob+1:end]);
InfoTab = InfoTab(I,:);
if strcmp(usr_figselection, 'materialstresscolldist')
  % Füge bei identischen Einträgen ein "variant x hinzu"
  for i = 1:length(legstr3)
    I = strcmp(legstr3{i},legstr3);
    if sum(I) > 1
      for j = find(I)'
        legstr3{j} = [legstr3{j}, sprintf(' var. %d', find(j==find(I),1,'first'))];
        InfoTab.TextFix{j} = legstr3{j};
      end
    end
  end
else
  % Lade das "variant x" von materialstresscolldist, da dort alle Roboter
  % stehen. Dadurch bleibt die Benennung gleich.
  InfoTabRef = readtable(fullfile(outputdir, ...
    sprintf('lufipkm_groups_legend_%s.csv', 'materialstresscolldist')));
  for i = 1:size(InfoTab,1)
    j = find(strcmp(InfoTabRef.GroupName, InfoTab.GroupName{i}));
    if isempty(j)
      warning('Gruppe %s existiert nicht', InfoTab.GroupName{i});
      continue
    end
    if length(j)>1, error('Logik-Fehler. Existiert mehrfach'); end
    InfoTab.TextFix{i} = InfoTabRef.TextFix{j};
    legstr3{i} = InfoTab.TextFix{i};
  end
end
% Speichere die Legende als Datei ab für späteren Bezug im Dokument
% (Rauskopieren der Daten für Roboter-Bild)
writetable(InfoTab, fullfile(outputdir, ...
  sprintf('lufipkm_groups_legend_%s.csv', usr_figselection)), 'Delimiter', ';');

xscale_add = 0; % Zur Verkleinerung der Legende bei langen Namen
if strcmp(usr_figselection, 'materialstresscolldist_56joints') || ...
    strcmp(usr_figselection, 'materialstresscolldist')
  xscale_add = -0.1;
end
if strcmp(usr_figselection, 'motordiagram_prismatic')
  xscale_add = -0.1;
end
% Die Legendeneinträge habe unterschiedliche Länge (wegen var....)
% Ausprobieren, wie viele in eine Zeile passen.
num_legend_entries_per_row = 5;
if strcmp(usr_figselection, 'materialstresscolldist_34joints') || ...
   strcmp(usr_figselection, 'linkdiam_vs_coll_34joints') || ...
   strcmp(usr_figselection, 'linkdiam_vs_coll')
  num_legend_entries_per_row = 6;
end
if exist('lf_hdl', 'var'), delete(lf_hdl); end
lf_hdl = legendflex(leghdl3, legstr3, 'anchor', {'n','n'}, ...
  'ref', fighdl, ...
  'buffer', [0 -2], ... % Kein Versatz notwendig, da mittig oben
  'ncol', 0, 'nrow', ceil(length(legstr3)/num_legend_entries_per_row), ... % pro Zeile passen 5 Namen auf die Legende in Seitenbreite
  'fontsize', 9, ...
  'FontName', 'Times New Roman', ...
  'xscale', 0.7+xscale_add, ... % Kleine Symbole
  'padding', [0,1,2], ... % Leerraum reduzieren
  'box', 'on', ...
  'interpreter', 'latex');

% Erzeuge Zoom-Bild (mit unterschiedlichen Einstellungen als bei Bild ohne
% Legende, wegen der anderen Skalierung)
if strcmp(usr_figselection, 'power_vs_mass')
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [950e-3, 2050e-3; 55, 75]);
  set(zoomhdl1, 'position', [0.4694 0.1746 0.4992 0.4021]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.4612 0.2910 -0.2264 -0.1032]);
end
if strcmp(usr_figselection, 'power_vs_coll')
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [950e-3, 2050e-3; -0.3, 6.5]);
  set(zoomhdl1, 'position', [0.5289 0.1005 0.4479 0.4683]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.5143 0.2602 -0.2697 0.2662]);
end
if strcmp(usr_figselection, 'motordiagram_revolute')
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [0.6, 1.2; 60, 120]);
  set(zoomhdl1, 'position', [0.2975 0.5507 0.6859 0.2731]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.2810 0.6564 -0.1008 -0.0573]);
end
if strcmp(usr_figselection, 'motordiagram_prismatic')
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [1.7, 4; 0.39, 0.61]);
  set(zoomhdl1, 'position', [0.3041 0.2324 0.6612 0.3647]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.2893 0.3735 -0.0909 -0.1235]);
  % Beschriftung der Antriebsleistung nur in Zoom-Fenster
  axes(zoomhdl1);
  texthdl_pwr1 = text(2.0460, 0.4359, 0,'1kW','FontName','Times','FontSize',8);
  texthdl_pwr2 = text(3.7652, 0.5526, 0,'2kW','FontName','Times','FontSize',8);
end

% Speichern und Abschluss
name = sprintf('lufipkm_pareto_%s_%s_groups_%s', pareto_settings{1}, pareto_settings{2}, usr_figselection);
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));
% export_fig(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)));
exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)),'ContentType','vector');
% export_fig(fighdl, fullfile(paperfigdir, 'pareto_all.pdf'));
fprintf('Auswertung %s gespeichert\n', name);

% Funktioniert nicht so gut
% matlab2tikz(fullfile(outputdir, sprintf('%s.tex', name)), ...
%   'parseStrings', false, ...
%   'strictFontSize', false);
end
