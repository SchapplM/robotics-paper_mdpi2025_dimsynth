% Evaluate Pareto fronts for groups of robots. The groups are selected by
% similar kinematic properties to reduce the amount of solutions.
%
% Preliminaries:
% * run dimensional synthesis with dimsynth_3T1R_example.m
% * Generate robot names with robot_names.m
% * Aggregate results with  eval_figures_pareto.m
%
% modified for PhD thesis
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
close all

usr_fig_all = {'default', 'default_revolute', 'default_prismatic', ...
  'onlypoints', 'onlypoints_revolute', 'onlypoints_prismatic', ...
  'relax', 'relax_revolute', 'relax_prismatic'};
for i_usr_fig = 1:6
usr_figselection = usr_fig_all{i_usr_fig};
% Die Gruppen lieber neu generieren, damit keine veralteten Daten geladen werden
usr_loadgroups = false;
usr_loadmarkers = false;
if any(contains(usr_figselection, {'revolute', 'prismatic'}))
  % Marker sollen von anderer Auswertung geladen werden
  usr_loadgroups = true;
  usr_loadmarkers = true;
end
%% Initialisierung

if true % contains(usr_figselection, 'default')
  pareto_settings = {'actforce', 'installspace'};
else
  error('Fall nicht definiert');
end

resdirtotal = HandlingRobot_dimsynth_data_dir();
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir.m'));

datadir = fullfile(repo_dir,'data');
outputdir = fullfile(paper_dir, 'paper', 'Figures');

SerRobDB = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof', 'N');
% Lade Bezeichnungen aus Dissertation, damit die Nummern konsistent sind
tableexportpath = fullfile(paper_dir, ...
  'paper', 'Structural_Synthesis');
PKM_Table_Diss_3T0R = readtable(fullfile(tableexportpath, ...
    sprintf('%s_PKM_LatexNumbers.csv', '3T0R')), 'Delimiter', ';');
PKM_Table_Diss_3T1R = readtable(fullfile(tableexportpath, ...
    sprintf('%s_PKM_LatexNumbers.csv', '3T1R')), 'Delimiter', ';');
% Die 3T1R-PKM sollen zuerst kommen damit die Marker einheitlich sind mit
% Bildern, in denen nur 3T1R-PKM stehen (vgl. mit gemischten Bildern mit 3T0R)
% Dafür noch weitere Sortierungen unten notwendig.
PKM_Table_Diss = [PKM_Table_Diss_3T1R; PKM_Table_Diss_3T0R];
%% Daten laden (als mat)
usr_figselection1 = usr_figselection;
if contains(usr_figselection, 'default')
  % Lade die PKM-Liste der gesamten (Schub- und Drehantrieb). Dadurch
  % doppeln sich keine Marker.
  usr_figselection1 = 'default';
elseif contains(usr_figselection, 'onlypoints')
  usr_figselection1 = 'onlypoints';
elseif contains(usr_figselection, 'relax')
  usr_figselection1 = 'relax';
end
datafile = fullfile(datadir, sprintf('results_all_reps_pareto_%s.mat', usr_figselection1));
assert(exist(datafile, 'file'), 'Daten zu Pareto-Bild existieren nicht');

tmp = load(datafile);
ResTab_ges = tmp.ResTab_ges;

% Betrachte nur Optimierungsläufe mit i.O.-Ergebnis
I_iO = ResTab_ges.Fval_Opt < 1e3;
% ResTab_ges = readtable(fullfile(datadir, 'results_all_reps_pareto.csv'), ...
%   'Delimiter', ';', 'ReadVariableNames', true);
Robots = unique(ResTab_ges.Name(I_iO), 'stable');
% Sortiere so, dass erst die 6FG und dann die 5FG kommen
I_4FG = ~cellfun(@isempty,regexp(Robots, '^P6', 'match'));
Robots = [Robots(I_4FG); Robots(~I_4FG)];

% Namen der Roboter laden (enthält auch Eigenschaften zu Kardan-Gelenken)
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
dimsynth_expeval_robot_names(Robots, namestablepath, 'edit', 'skip', 0);
ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');

%% Roboter gruppieren
if contains(usr_figselection, 'default') || contains(usr_figselection, ...
    'onlypoints')  % Nehme die gleichen Marker der Auswertung mit Trajektorie.
  % Benutze die Plot-Marker des gemeinsamen Bildes, damit es keine
  % Dopplungen gibt.
  usr_figselection1 = 'default';
else % Benutze dieselben Marker und Gruppen für alle Bilder
  % Nehme neue Marker, weil theoretisch neue PKM gegenüber der default-
  % Einstellung mit enthalten sein können.
  usr_figselection1 = usr_figselection;
end

groupsfile = fullfile(datadir, sprintf('robot_groups_%s_nodata.mat', usr_figselection1));
if ~usr_loadgroups || ~exist(groupsfile, 'file')
  fprintf('Gruppiere die PKM aus den Ergebnissen\n');
  % Erzeuge die Namen der Gruppen durch weglassen der P- und G-Nummer
  RobotGroups = cell2table(cell(0,8), 'VariableNames', {'GroupName', ...
    'Robots', 'ResultsFound', 'PlotMarker', 'PlotColor', 'KinematicsNumber', ...
    'NameActVar' 'NameManual'});
  t_lastlog = tic()-1e3;
  for i = 1:length(Robots)
    if toc(t_lastlog) > 10
      fprintf('Verarbeite PKM %d/%d. %1.1fs bis hier.\n', i, length(Robots), toc(t_lastlog));
      t_lastlog = tic();
    end
    row_i = cell(1,8);
    [NLEG, LEG_Names, Actuation, Coupling, ~, symrob, EE_dof0, ...
      PName_Kin, PName_Legs] = parroblib_load_robot(Robots{i}, 2);
    row_i{1} = [PName_Legs, 'G'];
    % Finde alle Roboter, die zur Gruppe passen und schreibe sie in die
    % zweite Spalte
    row_i{2} = Robots(contains(Robots,row_i{1}));
    % Dritte Spalte, Marker, ob i.O.-Ergebnisse in Maßsynthese
    row_i{3} = 0;
    % Vierte+Fünfte Spalte, Plot-Marker (in allen Bildern konsistent)
    row_i{4} = ''; row_i{5} = '';
    % Haupt-Kinematiken entsprechend der Diss-Tabelle merken:
    PName_Kin_ActVar = [PName_Legs, 'GxPx', sprintf('A%d', Actuation{1})];
    I_tab = strcmp(PKM_Table_Diss.RobotName_Diss, PName_Kin_ActVar);
    if ~any(I_tab)
      warning('Eintrag %d in PKM_Table_Diss nicht gefunden: %s', i, Robots{i});
    end
    row_i{6} = [PKM_Table_Diss.NumberMain(I_tab), PKM_Table_Diss.NumberVar(I_tab)];
    row_i{7} = PName_Kin_ActVar;
    RobotGroups = [RobotGroups; row_i]; %#ok<AGROW> 
  end
  % Sortiere zuerst nach dem Namen bzgl. der Diss-PKM-Datenbank. Damit
  % bleiben unterschiedliche Aktuierungen erhalten
  [~,I] = unique(RobotGroups.NameActVar, 'stable');
  RobotGroups = RobotGroups(I,:);
  % Entferne Gruppen, wenn sie eine distalere Aktuierung haben
  [~,I] = sort(RobotGroups.NameActVar); % proximale Aktuierung zuerst innerhalb der Gruppen
  RobotGroups = RobotGroups(I,:);
  [~,I] = unique(RobotGroups.GroupName, 'stable'); % damit wird die niedrigste Aktuierung behalten (steht als erstes)
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
ctmp = imes_plot_template_colors();
colorlist =  {'r', 'g', 'b', 'c', 'm', 'k', ctmp.imesorange, ctmp.imesgruen, ctmp.imesblau};
colorlist_str = colorlist;
colorlist_str{end-2} = 'imesorange';
colorlist_str{end-1} = 'imesgruen';
colorlist_str{end} = 'imesblau';
I_robleg = false(length(Robots), 1);
% Betrachte nur Optimierungsläufe mit passender Zielfunktion
I_objmatch = contains(ResTab_ges.Zielfunktion,pareto_settings{1}) & ...
             contains(ResTab_ges.Zielfunktion,pareto_settings{2});
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
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  I_groupmatch = contains(ResTab_ges.Name, GroupName);
  II_Robi = find(I_iO&I_objmatch&I_groupmatch);
  if isempty(II_Robi)
    fprintf('Gruppe %d/%d (%s): Keine Ergebnisse\n', i, size(RobotGroups,1), ...
      GroupName);
    continue
  end
  ChainShortName = ResTab_NameTrans.Chain_ShortName{find(contains(ResTab_NameTrans.PKM_Name, GroupName), 1, 'first')};
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
    if any( strcmp(markerlist{im}, RobotGroups.PlotMarker(I_prev_plot)) & ...
       cellfun(@(x)isequal(x,colorlist{ic}),RobotGroups.PlotColor(I_prev_plot)) )
      fprintf('Marker-Farb-Kombination %s/%s existiert schon. Suche neue.\n', ...
        markerlist{im}, colorlist_str{ic})
      success = false;
      % Probiere alle möglichen Marker und dann alle Farben durch
      for kk = [ic:length(colorlist), 1:ic-1]
        for jj = [im:length(markerlist), 1:im-1]
          if ~any(strcmp(markerlist{jj}, RobotGroups.PlotMarker(I_prev_plot)) & ...
             cellfun(@(x)isequal(x,colorlist{kk}),RobotGroups.PlotColor(I_prev_plot)))
            im = jj;
            ic = kk;
            fprintf('Wähle die Marker %s (Farbe: %s) stattdessen\n', ...
              markerlist{im}, colorlist{ic});
            success = true;
            break;
          end
        end
        if success, break; end
      end
      if ~success
        error('Kein eindeutiger Marker bestimmbar');
      end
    end
    marker = markerlist{im};
    color = colorlist{ic};
    if ~usr_loadmarkers || isempty(RobotGroups.PlotMarker{i}) % Behalte die schon geladenen Marker
      RobotGroups.PlotMarker{i} = marker;
      RobotGroups.PlotColor{i} = color;
    else
      marker = RobotGroups.PlotMarker{i};
      color = RobotGroups.PlotColor{i};
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
    if isempty(setfile)
      warning('Einstellungs-Datei existiert nicht. Daten inkonsistent: %s', OptName);
      continue
    end
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
    elseif ~strcmp(usr_figselection, 'volume') % bei rein geometrischer Betrachtung egal
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
    % Lade die Detail-Ergebnisse mit allen Zwischenwerten
    resfile2 = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
    detail_data_valid = false;
    if exist(resfile2, 'file') && false % Detail-Daten sind so teilweise viele, dass Matlab abbricht wegen Speicher (bei pareto_dominance)
      tmp2 = load(resfile2);
      RobotOptRes2_j = cds_convert_pareto_details2front(tmp2.PSO_Detail_Data);
      I_detail_j = all(RobotOptRes2_j.fval_all<1e3,2);
      if all(all(~isnan(RobotOptRes2_j.physval_all(I_detail_j,:))))
        detail_data_valid = true;
        % Erst hier eintragen. Durch unbekannte Ursachen kann NaN drin sein
        fval_pareto = [fval_pareto; RobotOptRes2_j.fval_all(I_detail_j,:)]; %#ok<AGROW> 
        p_val_pareto = [p_val_pareto; RobotOptRes2_j.pval_all(I_detail_j,:)]; %#ok<AGROW> 
        physval_pareto = [physval_pareto; RobotOptRes2_j.physval_all(I_detail_j,:)]; %#ok<AGROW> 
      else
        warning('In Detail-Daten NaN in physval_all: %s', resfile2);
      end
    end
    if ~detail_data_valid
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
    sprintf('group_%s_paretofront.csv', GroupName)), 'Delimiter', ';');
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
  if strcmp(pareto_settings{1}, 'footprint')
    maxgaplength_x = 0.01; % bezogen auf x-Achse, in m²
  elseif strcmp(pareto_settings{1}, 'actforce')
    maxgaplength_x = 2;
  else
    error('Value for maxgaplength_x not set yet');
  end
  xminmax = minmax2(pf_data(:,1)');
  if strcmp(pareto_settings{1}, 'footprint')
    dx = 0.1; % Prüfe in 0.1m²-Schritten
  elseif strcmp(pareto_settings{1}, 'actforce')
    dx = 0.01;
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
  if strcmp(pareto_settings{1}, 'actforce')
    xthresh = 1; % Nm
  end
  if strcmp(pareto_settings{2}, 'positionerror')
    ythresh = 10; % in µm
  elseif strcmp(pareto_settings{2}, 'installspace')
    ythresh = 0.01; % in m³
  elseif strcmp(pareto_settings{2}, 'actvelo')
    ythresh = 3; % deg/s
  end
  assert(~any(isnan([xthresh;ythresh])), 'xthresh oder ythresh nicht gesetzt');
  [obj_units, objscale] = cds_objective_plotdetails(Set_j);
  % Ändere Skalierung für die erreichten Werte
  objscale(strcmp(Set_j.optimization.objective, 'actvelo')) = 180/pi;
  objscale(strcmp(Set_j.optimization.objective, 'colldist')) = -1e2; % Benutze positive Werte (und cm)
  I_pm = select_plot_indices_downsample_nonuniform(objscale(kk1)*pf_data(:,1), ...
    objscale(kk2)*pf_data(:,2), xthresh, ythresh, logscales);
  fprintf('Es werden %d/%d Marker zum Plotten gewählt nach dem Ausdünnen.\n', ...
    sum(I_pm), length(I_pm));
  % Linienstil bestimmen und zeichnen
%   if all(isnan(gapdata_y)) % Alle Werte sind einzeln, keine Linie zu zeichnen
    linestyle = 'none';
%   else % TODO: Hier kann noch eine andere Strichelung genommen werden
%     linestyle = '-';
%   end
  % Zeichne keine durchgezogene Linie
  if false
    plot(objscale(kk1)*pf_plotdata(:,1), objscale(kk2)*pf_plotdata(:,2), [color,linestyle]);
  end
  % Zeichne Marker
  plot(objscale(kk1)*pf_data(I_pm,1), objscale(kk2)*pf_data(I_pm,2), ...
    'Marker', marker, 'Color', color, 'LineStyle', linestyle);
  % Dummy-Handle für Legende
  hdl = plot(0, NaN, 'Marker', marker, 'Color', color, 'LineStyle', linestyle);
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
% Aus eval_existing_design.m: Einstellung ist egal
% usr_figselection_eng = usr_figselection;
countadd = 0;
if strcmp(usr_figselection, 'default_prismatic') || ... % bei relax_prismatic sind die Bedingungen für die Reproduktion nicht vergleichbar
    strcmp(usr_figselection, 'onlypoints_prismatic')
  for k = 1:5
    countadd = countadd + 1;
    switch k
      case 1
        engshortname = 'CRR';
        engtexname = '3-\underline{C}RR with parameters from Prause et al. 2015, Table 5. Likewise:';
        enggroupname = 'P3PRRR1G';
      case 2
        engshortname = 'PUU';
        engtexname = '3-\underline{P}UU';
        enggroupname = 'P3PRRRR8V1G';
      case 3
        engshortname = 'CRU';
        engtexname = '3-\underline{C}RU';
        enggroupname = 'P3PRRRR8V2G';
      case 4
        engshortname = 'CUR';
        engtexname = '3-\underline{C}UR';
        enggroupname = 'P3PRRRR4V1G';
      case 5
        engshortname = 'UPU';
        engtexname = '3-U\underline{P}U';
        enggroupname = 'P3RRPRR12V3G';
    end
    if contains(usr_figselection, 'default')
      usr_figselection_eng = 'default'; % dort sind alle Kriterien berechnet
    else
      usr_figselection_eng = 'onlypoints';
    end
    erg_eng = load(fullfile(datadir, sprintf(['detail_result_' ...
      'PrauseChaCor2015_%s_%s_collcheck1_config1.mat'], engshortname, usr_figselection_eng)));
    if any(erg_eng.fval > 1e3)
      error('Gewählte Vergleichslösung ist ungültig')
    end
    Set_eng = erg_eng.Set;
    kk1_eng = find(strcmp(Set_eng.optimization.objective, pareto_settings{1}));
    kk2_eng = find(strcmp(Set_eng.optimization.objective, pareto_settings{2}));
    if isempty(kk1_eng) || isempty(kk2_eng)
      error(['Die gesuchten Kriterien fehlen in den Daten zur eigenen Lösung. ', ...
        'eval_existing_design entweder neu ausführen oder überarbeiten!']);
    end
    row_eng = RobotGroups(strcmp(RobotGroups.GroupName, enggroupname),:);
    row_eng.Robots = {erg_eng.R.mdlname};
    row_eng.GroupName = {engshortname};
    legstr{countrob+countadd} = engshortname; %#ok<SAGROW> % wird später durch NameManual ersetzt
    [obj_units_eng, objscale_eng] = cds_objective_plotdetails(Set_eng);
  %   % Skalierung manuell ändern (konsistent zu oben)
  %   objscale_eng(strcmp(Set_eng.optimization.objective, 'colldist')) = -1e2; % cm
  %   objscale_eng(strcmp(Set_eng.optimization.objective, 'actvelo')) = 180/pi; % deg/s
    row_eng.NameManual = {sprintf('%s', engtexname)}; %  (PR %d.%d), row_eng.KinematicsNumber(1), row_eng.KinematicsNumber(2))};{engtexname};
    % Wähle den Markertyp genauso wie in den bestehenden Lösungen
    % Nehme keinen größeren Marker (blöd für Legende), sondern eine andere,
    % noch unbenutzte Farbe
    row_eng.PlotColor{1} = ctmp.hellrot;
    for size_offset = [-2:0.1:0, 2:-0.1:0] % Mache den Marker sehr fett, indem er mit verschiedenen Größen übereinander geplottet wird.
      leghdl(countrob+countadd) = plot(axhdl, objscale_eng(kk1_eng)*erg_eng.physval(kk1_eng), ...
        objscale_eng(kk2_eng)*erg_eng.physval(kk2_eng), ...
        'LineStyle', 'none', 'Marker', row_eng.PlotMarker{1}, 'Color', ...
        row_eng.PlotColor{1}, 'MarkerSize', 6+size_offset); % Alt: 'Color', row_eng.PlotColor{1}, 'MarkerSize', 10);
    end
    if k == 1
      TabEng = row_eng;
    else
      TabEng = [TabEng; row_eng]; %#ok<AGROW> 
    end
  end
  % Speichere Tabelle ab (für andere Skripte zur Vorbereitung)
  save(fullfile(datadir, sprintf('engsol_groups_%s.mat', usr_figselection)), 'TabEng');
else
  erg_eng = [];
end

%% Formatieren für Diss-Bild
% uiopen(fullfile(outputdir, sprintf('figure_pareto_groups_%s_%s.fig', ...
%   pareto_settings{1}, pareto_settings{2})), 1);

figure_format_publication(axhdl);
if strcmp(pareto_settings{1}, 'footprint')
  xlh = xlabel('Footprint area in m²');
elseif strcmp(pareto_settings{1}, 'actforce')
  if contains(usr_figselection, 'revolute')
    xlh = xlabel('Actuator torque in Nm');
    set(axhdl, 'xlim', [0 60]);
  elseif contains(usr_figselection, 'prismatic')
    xlh = xlabel('Actuator force in N');
    set(axhdl, 'xlim', [3.5 44]);
  else
    xlh = xlabel('Actuator force/torque (mixed units N/Nm)');
  end
else
  error('Value for xlabel not set yet');
end
if logscales(1)
  set(axhdl, 'XScale', 'Log');
end

if strcmp(pareto_settings{2}, 'positionerror')
  ylabel('Position error in µm');
elseif strcmp(pareto_settings{2}, 'installspace')
  ylabel('Installation space in m³');
  % es soll keine Zahl oben an der Achse stehen
  if contains(usr_figselection, 'revolute')
    set(axhdl, 'ylim', [0, 0.34]);
  elseif strcmp(usr_figselection, 'default_prismatic')
    set(axhdl, 'ylim', [0, 0.59]);
  end
else
  error('Value for ylabel not set yet');
end
title('');

% x-Beschriftung hochziehen, damit Bild kleiner werden kann
xlhp = get(xlh, 'position');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl, 'y');
xtl = get(axhdl, 'xticklabel');
if contains(usr_figselection, 'revolute')
  set(xlh, 'position', [xlhp(1), Y_off+Y_slope*(-1.05), 0]);
  xtl{ceil((length(xtl))/2)} = '';
  set(axhdl, 'xticklabel', xtl);
elseif strcmp(usr_figselection, 'default_prismatic')
  set(xlh, 'position', [xlhp(1), Y_off+Y_slope*(-1.04), 0]);
  xtl{ceil((length(xtl))/2)} = '';
  xtl{ceil((length(xtl))/2)+1} = '';
  set(axhdl, 'xticklabel', xtl);
end

% Bild erst ohne Legende speichern
delete(lh);

set_size_plot_subplot(fighdl, ...
  18.3,5,axhdl,...
  0.045,0.01,0.01,0.17,0,0);

name = sprintf('handlingpkm_pareto_%s_%s_groups_%s_nolegend', ...
  pareto_settings{1}, pareto_settings{2}, usr_figselection);
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));
% export_fig(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)));
exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)),'ContentType','vector');

%% Vorbereiten für Legende
% Plot-Größe anhand der Legende auswählen
if contains(usr_figselection, 'revolute')
  set_size_plot_subplot(fighdl, ...
    18.3,7,axhdl,...
    0.05,0.01,0.27,0.08,0,0);
elseif any(strcmp(usr_figselection, {'default_prismatic'}))
  set_size_plot_subplot(fighdl, ...
    18.3,8.5,axhdl,...
    0.05,0.01,0.39,0.06,0,0);
elseif any(strcmp(usr_figselection, {'onlypoints_prismatic'}))
  % Das Bild ist sowieso auf einer ganzen Seite im Anhang. Größer machen.
  set_size_plot_subplot(fighdl, ...
    18.3,15,axhdl,...
    0.05,0.01,0.225,0.06,0,0);
elseif strcmp(usr_figselection, 'relax_prismatic')
  % Andere Maße, da Prause-PKM-Legende fehlt
  set_size_plot_subplot(fighdl, ...
    18.3,11,axhdl,...
    0.05,0.01,0.45,0.08,0,0);
else
  warning('Nicht implementiert')
  set_size_plot_subplot(fighdl, ...
    18.3,8,axhdl,...
    0.05,0.01,0.24,0.12,0,0);
end

% Bezug zwischen Plot-Legende und gespeicherten Gruppen abspeichern
InfoTab = cell2table(repmat(legstr(:), 1,6));
InfoTab.Properties.VariableNames = {'TextFix', 'TextLink', 'GroupName', ...
  'GroupNum', 'NumberMain', 'NumberVar'};

% Legende und Roboternamen aktualisieren
legstr2 = cell(length(legstr),1);
legstr2_PR_link = legstr2;
% if ~isempty(erg_eng)
%   legstr2(end-countadd+1:end) = legstr(end-countadd+1:end); % Engineering Solution
% end
% Ersetze den ursprünglichen Legendennamen durch die Bezeichnung aus der Diss
% Erstelle auch vollständige Tabellenzeilen für die Referenzlösungen
if ~isempty(erg_eng)
  RobotGroups2 = [RobotGroups; TabEng];
else
  RobotGroups2 = RobotGroups;
end
for i = 1:length(legstr)
  for j = 1:size(RobotGroups2,1) % muss nicht identisch mit Legende sein
    if contains(legstr{i}, RobotGroups2.GroupName{j})
      % Suche Latex-Namen
      Robots_j = RobotGroups2.Robots{j};
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
        legstr2{i} = sprintf(''); %'PR %d.%d', ...
          % PKM_Table_Diss.NumberMain(I_tab), PKM_Table_Diss.NumberVar(I_tab));
        
        if length(ResTab_NameTrans.Chain_ShortName{I_k}) < str2double(LEG_Names{1}(2)) % Name zeichnet sich nicht durch Substitution von Gelenken aus
            % alte Abfrage: contains(ResTab_NameTrans.PKM_Name{I_k}, 'V') % Kurzer Name mit technischen Gelenken
          legstr2{i} = [legstr2{i}, sprintf('%s', ...
            parroblib_format_robot_name(ResTab_NameTrans.PKM_Name{I_k}, 4))];
        else % PKM-Name mit Akzenten
          legstr2{i} = [legstr2{i}, sprintf('%s', ...
            parroblib_format_robot_name(ResTab_NameTrans.PKM_Name{I_k}, 1))];
        end
        if ~isempty(RobotGroups2.NameManual{j})
          legstr2{i} = RobotGroups2.NameManual{j};
        end
        InfoTab.TextFix{i} = legstr2{i};
        InfoTab.TextLink{i} = legstr2_PR_link{i};
        InfoTab.GroupName{i} = RobotGroups2.GroupName{j};
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
for i = 1:size(InfoTab,1)
  assert(strcmp(InfoTab.TextFix{i}, legstr2{i}), 'Fehler beim Aufbau von InfoTab');
end
% Ändere die Reihenfolge der Legende so, dass die PR-Nummern aufsteigend sind
% Erst FG, dann PR-Nummern
LegNum = cellfun (@(x) x(1),InfoTab.TextFix);
InfoTab_orig = InfoTab;
for ln = 3:4
  % Suche alle PKM mit diesen Beinketten und sortiere nach PR-Nummer
  I_ln = contains(InfoTab.TextFix, sprintf('%d-', ln)) & ...
         contains(InfoTab.GroupName, sprintf('P%d', ln)); % Dadurch Ausschluss der Zeilen für Ref-Einträge
  [~, I] = sortrows([InfoTab.NumberMain(I_ln), InfoTab.NumberVar(I_ln)]);
  InfoTab_ln = InfoTab(I_ln,:); % Auswahl
  InfoTab_ln = InfoTab_ln(I,:); % Teil Sortieren
  InfoTab(I_ln,:) = InfoTab_ln; % neu zuweisen
end
% Bestimme nochmal die Permutationsreihenfolge, die oben mit dem
% abschnittsweise sortrows gefunden wurde
I = 1:size(InfoTab,1);
for i = 1:size(InfoTab,1)
  I(i) = find(strcmp(InfoTab.TextFix{i}, InfoTab_orig.TextFix) & ...
              strcmp(InfoTab.GroupName{i}, InfoTab_orig.GroupName));
end
% [~,I] = sort(cell2mat(InfoTab.NumberMain(1:countrob)) + ...
%              cell2mat(InfoTab.NumberVar( 1:countrob))/100);
legstr3 = legstr2(I); % Einträge aus Paper (countrob+1:end) nicht in gleiche Legende
leghdl3 = leghdl(I(:)');
% Probe: Wurden die Legendeneinträge dabei nicht durcheinander gebracht?
for i = 1:size(InfoTab,1)
  assert(strcmp(InfoTab.TextFix{i}, legstr3{i}), 'Fehler beim Permutieren');
end

if strcmp(usr_figselection, 'default')
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
    sprintf('handlingpkm_groups_legend_%s.csv', 'default')));
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
  sprintf('handlingpkm_groups_legend_%s.csv', usr_figselection)), 'Delimiter', ';');

xscale_add = 0; % Zur Verkleinerung der Legende bei langen Namen
% if strcmp(usr_figselection, 'volume')
%   xscale_add = -0.1;
% else
%   xscale_add = -0.2;
% end
% Die Legendeneinträge habe unterschiedliche Länge (wegen var....)
% Ausprobieren, wie viele in eine Zeile passen.
num_legend_entries_per_row = 5;
if false % contains(usr_figselection, 'default') || contains(usr_figselection, 'relax')
  % Erzeuge zwei Legenden, eine für 3FG-PKM, eine für 4FG-PKM
  if exist('lf_hdl1', 'var'), delete(lf_hdl1); end
  I1 = contains(legstr3, '3-');
  lf_hdl1 = legendflex(leghdl3(I1), legstr3(I1), 'anchor', {'n','n'}, ...
    'ref', fighdl, ...
    'buffer', [0 -2], ... % Kein Versatz notwendig, da mittig oben
    'ncol', 0, 'nrow', ceil(length(legstr3(I1))/num_legend_entries_per_row), ...
    'fontsize', 9, ...
    'FontName', 'Times New Roman', ...
    'title', '3T0R parallel robots:', ...
    'xscale', 0.7+xscale_add, ... % Kleine Symbole
    'padding', [2,1,2], ... % Leerraum reduzieren
    'box', 'on', ...
    'interpreter', 'latex');
  if exist('lf_hdl2', 'var'), delete(lf_hdl2); end
  I2 = ~I1;
  lf_hdl2 = legendflex(leghdl3(I2), legstr3(I2), 'anchor', {'s','n'}, ...
    'ref', lf_hdl1, ...
    'buffer', [0 -2], ... % Kein Versatz notwendig, da mittig oben
    'ncol', 0, 'nrow', ceil(length(legstr3(I2))/num_legend_entries_per_row), ...
    'fontsize', 9, ...
    'FontName', 'Times New Roman', ...
    'title', '3T1R parallel robots:', ...
    'xscale', 0.7+xscale_add, ... % Kleine Symbole
    'padding', [2,1,2], ... % Leerraum reduzieren
    'box', 'on', ...
    'interpreter', 'latex');
  % Zusätzliche Legende für PKM aus Prause-Paper
  if ~isempty(erg_eng)
    if exist('lf_hdl3', 'var'), delete(lf_hdl3); end
    lf_hdl3 = legendflex(leghdl(countrob+1:end), legstr2(countrob+1:end), ...
      'anchor', {'s','n'}, ...
      'ref', lf_hdl2, ...
      'buffer', [0 -2], ... % Kein Versatz notwendig, da mittig oben
      'ncol', 0, 'nrow', 1, ... % Es müssten 5 (kurze) Namen passen
      'fontsize', 9, ...
      'FontName', 'Times New Roman', ...
      'title', '3T0R robots with parameters from reference [PCC15], Table 5', ...
      'xscale', 0.7+xscale_add, ... % Kleine Symbole
      'padding', [-1,0,1], ... % Leerraum reduzieren
      'box', 'on', ...
      'interpreter', 'latex');
    p3 = get(lf_hdl3, 'Position');
  else
    p3 = NaN(1,4);
  end
  % Breite der Legenden angleichen
  p1 = get(lf_hdl1, 'Position'); p2 = get(lf_hdl2, 'Position');
  lf_xpos = min([p1(1), p2(1), p3(1)]);
  lf_width = max([p1(3), p2(3), p3(3)]);
  set(lf_hdl1, 'Position', [lf_xpos, p1(2), lf_width, p1(4)]);
  set(lf_hdl2, 'Position', [lf_xpos, p2(2), lf_width, p2(4)]);
  if all(~isnan(p3))
    set(lf_hdl3, 'Position', [lf_xpos, p3(2), lf_width, p3(4)]);
  end
else
  % Nur eine Legende, die hauptsächlich aus 4FG-Robotern besteht.
  % Kennzeichne 3FG-Roboter gesondert. 4FG-Roboter sind dann klar.
  % --> für MDPI-Paper ohne PR-Nummern nicht notwendig
  legstr4 = legstr3;
  % Nehme nicht die Einträge der Paper-Referenz
  I_3FG = find(contains(legstr3(1:countrob), '3-'));
  % for kk = find(I_3FG(:)')
  %   legstr4{kk} = strrep(legstr4{kk}, 'PR ', '3T0R-PR ');
  % end
  I_4FG = find(contains(legstr4, '4-'));
  % for kk = find(I_4FG(:)') % nur für PKM der ersten Legenden-Spalte eintragen
  %   if ~any(contains(legstr4{kk}, {'42.1', '42.5'})), continue; end
  %   legstr4{kk} = strrep(legstr4{kk}, 'PR ', '3T1R-PR ');
  % end
  Ileg = [I_3FG; I_4FG];
  if exist('lf_hdl', 'var'), delete(lf_hdl); end
  lf_hdl = legendflex(leghdl3(Ileg), legstr3(Ileg), 'anchor', {'n','n'}, ...
    'ref', fighdl, ...
    'buffer', [0 -2], ... % Kein Versatz notwendig, da mittig oben
    'ncol', 5, 'nrow', 0, ... % pro Zeile passen 5 Namen auf die Legende in Seitenbreite
    'fontsize', 9, ...
    'FontName', 'Times New Roman', ...
    ...'xscale', 0.7+xscale_add, ... % Kleine Symbole
    'padding', [0,1,15], ... % Mehr Leerraum nach Schrift
    'box', 'on', ...
    'interpreter', 'latex');
  % Zusätzliche Legende für PKM aus Prause-Paper
  if ~isempty(erg_eng)
    if exist('lf_hdl3', 'var'), delete(lf_hdl3); end
    lf_hdl3 = legendflex(leghdl(countrob+1:end), legstr2(countrob+1:end), ...
      'anchor', {'s','n'}, ...
      'ref', lf_hdl, ...
      'buffer', [0 -2], ... % Kein Versatz notwendig, da mittig oben
      'ncol', 0, 'nrow', 1, ... % Es müssten 5 (kurze) Namen passen
      'fontsize', 9, ...
      'FontName', 'Times New Roman', ...
      ... 'title', '3T0R robots with parameters from reference [PCC15], Table 5', ...
      'xscale', 0.8+xscale_add, ... % Kleine Symbole
      'padding', [-5,-3,6], ... % Leerraum reduzieren
      'box', 'on', ...
      'interpreter', 'latex');
    p3 = get(lf_hdl3, 'Position');
  else
    p3 = NaN(1,4);
  end
end
% Erzeuge Zoom-Bild (mit unterschiedlichen Einstellungen als bei Bild ohne
% Legende, wegen der anderen Skalierung)
if any(strcmp(usr_figselection, {'default_revolute', 'onlypoints_revolute'}))
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [0.8, 5; 0.02, 0.20]);
  set(zoomhdl1, 'position', [0.3215 0.3129 0.6480 0.3559]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.3091 0.3862 -0.1735 -0.0581]);
elseif any(strcmp(usr_figselection, {'default_prismatic'}))
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [3.9, 10; 0.02, 0.20]);
  set(zoomhdl1, 'position', [0.2860 0.3278 0.6942 0.2587]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.3512 0.3115 -0.1347 -0.1225]);
elseif any(strcmp(usr_figselection, {'onlypoints_prismatic'}))
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [3.9, 10; 0.02, 0.20]);
  set(zoomhdl1, 'position', [0.2813 0.4384 0.6946 0.3165]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.3526 0.4303 -0.1312 -0.19102]);
elseif strcmp(usr_figselection, 'relax_prismatic')
  if exist('zoomhdl1', 'var'), delete(zoomhdl1); end
  if exist('boxhdl', 'var'), delete(boxhdl); end
  [zoomhdl1, boxhdl] = add_zoom_axis(axhdl, [4.1, 10; 0.02, 0.30]);
  set(zoomhdl1, 'position', [0.4496 0.1971 0.5323 0.3312]);
  if exist('arrhdl1', 'var'), delete(arrhdl1); end
  arrhdl1 = annotation(fighdl,'arrow');
  set(arrhdl1, 'position', [0.4364 0.2837 -0.2314 -0.0625]);
end

% Speichern und Abschluss
name = sprintf('handlingpkm_pareto_%s_%s_groups_%s', pareto_settings{1}, pareto_settings{2}, usr_figselection);
saveas(fighdl, fullfile(outputdir, sprintf('%s.fig', name)));
exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', name)),'ContentType','vector');
fprintf('Auswertung %s gespeichert\n', name);

end
