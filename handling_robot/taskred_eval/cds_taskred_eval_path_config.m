% Pfad-Initialisierung für Auswertung der Aufgabenredundanz in der Maßsynthese

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [resdir, datadir] = cds_taskred_eval_path_config(usr_cluster)

padir = fileparts(which('robsynth_projektablage_path.m'));

if usr_cluster
  resdir= '/bigwork/nhkbscha/REPO/structgeomsynth_dbg/taskred_eval/results';
  mkdirs(resdir);
else
  % run('robsynth_projektablage_path.m');
  
  % if isempty(padir)
  %   warning('Pfad zur Projektablage nicht gefunden');
  %   padir = 'P:\Für mich freigegeben\imes-projekt-dfg_robotersynthese';
  % end
  % für Auswertung basierend auf AMUN-Synthese
  % resdir = fullfile(padir, ...
  %     '03_Entwicklung', 'Auswertung_Maßsynthese_IK_Zielkriterium', 'Diss');
  % Prause-Beispiel für MDPI-Paper
  resdir = fullfile(padir, ...
      '06_Publikationen', '2025_MDPI_Maßsynthese', 'Ergebnisse_Auswertung_Aufgabenredundanz');
  % resdir = PrauseDemo_dimsynth_data_dir();
  if isempty(padir)
    error('Seafile Robotersynthese nicht verfügbar')
  end
  % Debug: Ohne Projektablage
  % resdir = fullfile(fileparts(which('struktsynth_bsp_path_init.m')), ...
  %   'taskred_eval', 'results');
  % mkdirs(resdir);
  % resdir = fullfile(padir, '03_Entwicklung', ...
  %   'Auswertung_Maßsynthese_IK_Zielkriterium', 'Statistiken');
  % 
  % resdir = 'C:\Users\Schappler_Moritz\Documents\Repo\structgeomsynth_dbg\taskred_eval\results';
  % resdir = 'D:\Ubuntu_Transfer\CLUSTER\REPO\structgeomsynth_dbg\taskred_eval\results';

end

if usr_cluster
  datadir = '/bigwork/nhkbscha/REPO/structgeomsynth/results/';
else
%   % Nutze lokal die Projektablage
%   datadir = i4sdg2023_dimsynth_data_dir();%

  % für Auswertung basierend auf AMUN-Synthese
  % datadir = amun_dimsynth_data_dir();
  % Prause-Beispiel für MDPI-Paper
  datadir = HandlingRobot_dimsynth_data_dir();
%   datadir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
%   datadir = fullfile(padir, '03_Entwicklung', ...
%     'Auswertung_Maßsynthese_IK_Zielkriterium', 'Ergebnisse');
end
