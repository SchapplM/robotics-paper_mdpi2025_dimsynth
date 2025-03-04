% Starte das Skript "cds_taskred_eval_example" auf dem Cluster

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover
clc
clear

%% Liste aller Roboter erstellen, um diese dann parallel zu starten
usr_debug_optname = ''; % Muss konsistent in beiden Skripten sein
% Aus dieser Optimierung werden die verfügbaren Roboternamen geholt
% (nur relevant, falls alle Roboter nochmal neu gerechnet werden)
usr_debug_optname_robnames = 'ARK_3T1R_20230730_full_rep1';
% usr_debug_optname_findstr = {'ARK_3T1R_20230730_full_rep1', 'ARK_3T1R_20230730_full_rep2', ...
%   'ARK_3T1R_20230729_v2_rep1', 'ARK_3T1R_20230729_v2_rep2', 'ARK_3T1R_20230729_v2_rep3', ...
%   'ARK_3T1R_20230730_full_rep1', 'ARK_3T1R_20230730_full_rep2'};  % 'amunpkm_20230605','amunpkm_20230606'
usr_debug_optname_findstr = 'ARK_3T1R_20230730_full_rep1';
usr_all_robots_one_job = false;
usr_all_optim_one_job = true;
[resdir, datadir] = cds_taskred_eval_path_config(false);
% Sammle Namen der Roboter
ResTab = readtable(fullfile(datadir, usr_debug_optname_robnames, ...
  [usr_debug_optname_robnames, '_results_table.csv']));
% RobNames = unique(ResTab.Name(ResTab.Fval_Opt<1e3))';
% RobNames = RobNames(strcmp(RobNames, 'P4RRRRR10V1G1P1A1')); % P4RPRRR8V2G9P1A1 P4RRRRR8V2G1P1A1
% RobNames = {'P4RPRRR8V2G3P1A1', 'P4RRRRR10V1G1P1A1', 'P4RRRRR5V1G2P1A1'};
% RobNames = {'P4RPRRR8V2G3P1A1'};
RobNames = {'P4RRRRR10V1G1P1A1', 'P4RRRRR5V1G2P1A1', 'P4RRRRR5G2P1A1', 'P4RRRRR5G3P1A1', 'P4RRRRR5V1G9P1A1', 'P4PRRRR4V1G1P8A1'};
% Sammle Namen der Optimierungen
dirres = dir(fullfile(datadir, 'ARK_3T1R_*'));
OptNames = {};
for i = 1:length(dirres)
  [tokens, match] = regexp(dirres(i).name, '_p(\d+)$', 'tokens', 'match');
  if ~isempty(tokens)
    continue % Teil-Ergebnisse nicht mit reinnehmen
  end
  if ~any(contains(dirres(i).name, usr_debug_optname_findstr))
    continue % Gesuchte Optimierungen nicht gefunden.
  end
  OptNames = [OptNames, dirres(i).name]; %#ok<AGROW> 
end
if usr_all_optim_one_job
  J = 0;
else
  J = 1:length(OptNames);
  fprintf('Starte %d Jobs: für jede Optimierung einen: %s\n', ...
    length(OptNames), disp_array(OptNames, '%s'));
end
for j = J

if usr_all_robots_one_job
  I = 0;
else
  I = 1:length(RobNames);
  fprintf('Starte %d Jobs: für jeden Roboter einen\n',length(RobNames));
end
for i = I
%% Skript erzeugen
% Pfade und Dateien initialisieren
this_dir = fileparts(which('cds_taskred_eval_example_cluster_start.m'));
script_file = fullfile(this_dir, 'cds_taskred_eval_example.m');
sgpath = fileparts(which('structgeomsynth_path_init.m'));
clusterheaderfile=fullfile(sgpath, 'dimsynth','dimsynth_cluster_header.m');
jobdir = fullfile(this_dir, 'job', sprintf('%d_%d', j, i));
mkdirs(jobdir);

targetfile = fullfile(jobdir, sprintf('start_script_%d_%d.m', j, i));
% Hierdurch Initialisierung der Pfade
copyfile(clusterheaderfile, targetfile);
% Eigentliches Skript anfügen
f = fileread(script_file);
f = strrep(f, 'usr_cluster = false;', ...
  'usr_cluster = true;');
% Roboter-Namen einsetzen (falls Job nur einen Roboter analysieren soll)
if i > 0
  f = strrep(f, '%#CLUSTER usr_debug_robname = '''';', ...
    sprintf('usr_debug_robname = ''%s'';', RobNames{i}));
end
% Optimierungs-Namen einsetzen
if j > 0
  f = strrep(f, '%#CLUSTER usr_debug_optname = '''';', ...
    sprintf('usr_debug_optname = ''%s'';', OptNames{j}));
end

fid  = fopen(targetfile,'a');
fprintf(fid,'%s',f);
fclose(fid);
fprintf('Skript %s erzeugt\n', targetfile);

%% Weitere Dateien dort hineinkopieren
copyfile(fullfile(this_dir, 'cds_taskred_eval_path_config.m'), jobdir);
copyfile(fullfile(this_dir, 'cds_taskred_eval_definitions.m'), jobdir);
%% Skript auf Cluster laden und starten
computation_name = sprintf('taskred_eval_p%d_%d_%s', j, i, datestr(now,'yyyymmdd_HHMMSS'));
if j > 0
  computation_name = [computation_name, '_', OptNames{j}]; %#ok<AGROW>
end
if i > 0
  computation_name = [computation_name, '_', RobNames{i}]; %#ok<AGROW>
end
% continue
[~, targetfilename] = fileparts(targetfile);
dependstruct = struct('afterany', []);
jobStart(struct( ...
  'name', computation_name, ...
  'ppn', 1, ...
  'matFileName', [targetfilename, '.m'], ...
  'locUploadFolder', jobdir, ...
  'mem', 10, ... % GB
  'time',12), ... % So lange (in h) sollte es aber nicht dauern
  dependstruct);
pause(5);
end % i (Roboter)
end % j (Optimierungen)
