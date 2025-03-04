% Passe die Einstellungen der Maßsynthese an (Strukturauswahl)
%
% Jannik Fettin, jannik.fettin@stud.uni-hannover.de, 2022-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Set = load_whitelist(Set, configset)
loadwhitelist = configset.loadwhitelist;
if ~loadwhitelist
  return
end
%% Lade mögliche Verzeichnisse mit Ergebnissen
dimsynthpath = fileparts(which('structgeomsynth_path_init.m'));
datadir_local = fullfile(dimsynthpath, 'results');
datadirs = {datadir_local};
if ~isempty(which('lufi_dimsynth_data_dir.m'))
  datadir2 = lufi_dimsynth_data_dir();
  datadirs{2} = datadir2;
else
  warning(['Datei lufi_dimsynth_data_dir.m existiert nicht. Diese Datei ', ...
    'sollte eigentlich auf das Seafile-Laufwerk zeigen']);
end
%% Definiere Namen der bisherigen Optimierungen
% Es sollten Versuchsläufe sein, in denen alle in Frage kommenden PKM
% einmal ausprobiert wurden
% Hier Anpassen für das Laden der "funktionierenden" Ergebnisse
optlist = configset.whitelist;
%% Gehe Ergebnisordner durch und lade die Liste gültier PKM
if ~isempty(optlist)
whitelist = {};
for i = 1:length(datadirs)
  for j = 1:length(optlist)
    dir_ij = fullfile(datadirs{i}, optlist{j});
    if ~exist(dir_ij, 'file')
      continue
    end
    ResTab = readtable(fullfile(dir_ij, [optlist{j}, '_results_table.csv']));
    % Alternative 1: Nur i.O.-Ergebnisse
    fval_thresh = 1e3;
    % Alternative 2: Nur Ergebnisse, bei denen die Trajektorie berechnet
    % werden kann (und die dann nicht singulär sind). Werte aus 
    % cds_constraints_traj mit Faktor 1e4 multipliziert.
    % fval_thresh = 9e3*1e4;
    % Tabelle nur mit ausgewählten Zeilen
    ResTab_filt = ResTab(ResTab.Fval_Opt<fval_thresh,:);
    % Alternative 3: Wähle bestimmten Wertebereich aus
    % Nur Ergebnisse, die in Entwurfsoptimierung fehlgeschlagen sind
    % ResTab_filt = ResTab(ResTab.Fval_Opt>1e4 & ResTab.Fval_Opt<1e5,:);
    whitelist = [whitelist; ResTab_filt.Name]; %#ok<AGROW>
  end
end
Set.structures.whitelist = unique(whitelist');
fprintf('Positiv-Liste aus vorhandenen Versuchen geladen: %d Einträge\n', length(Set.structures.whitelist));
end
%% Lade nur die PKM, die in Pareto-Diagrammen optimal sind
if configset.whitelist_from_paperdata
serroblibpath=fileparts(which('serroblib_path_init.m'));
SerRob_DB_all = load(fullfile(serroblibpath, 'serrob_list.mat'));
whitelist = {};
% Annahme: Bisherige Versuche sind ausreichend gut konvergiert
repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');
usr_figselection = 'motordiagram';
usr_figselection1 = usr_figselection;
% usr_figselection1 = 'power_vs_mass_34joints';
groupsfile = fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection1));
tmp = load(groupsfile);
for i = 1:size(tmp.RobotGroups,1)
  GroupName = tmp.RobotGroups.GroupName{i};
  gfi = fullfile(datadir, sprintf('group_%s_paretofront_%s.mat', GroupName, ...
    usr_figselection));
  tmp_i = load(gfi);
  roblist_i = unique(tmp_i.pt_i.RobName);
  % Filtere Roboter
  for j = 1:length(roblist_i)
    [~, LEG_Names] = parroblib_load_robot(roblist_i{j}, 0);
    ilc = find(strcmp(SerRob_DB_all.Names, LEG_Names{1}));
    SName_TechJoint = fliplr(regexprep(num2str(SerRob_DB_all.AdditionalInfo(ilc,7)), ...
      {'1','2','3','4','5'}, {'R','P','C','U','S'}));
    if strcmp(SName_TechJoint, 'SPU')
      % Entferne von Nach-Untersuchung. Ist zu unintuitiv für Paper
      roblist_i{j} = '';
    end
    if ~any(length(SName_TechJoint) == Set.structures.num_tech_joints)
      % Roboter hat nicht die geforderte Anzahl Beinketten. Ignoriere.
      roblist_i{j} = '';
    end
  end
  whitelist = [whitelist; roblist_i(~strcmp(roblist_i,''))]; %#ok<AGROW>
end
Set.structures.whitelist = unique(whitelist');
fprintf('Insgesamt %d Roboter aus Pareto-optimalen Ergebnissen bestimmt.\n', length(Set.structures.whitelist));
end