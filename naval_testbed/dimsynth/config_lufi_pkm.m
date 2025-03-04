% Kombinierte Struktur- und Maßsynthese für LuFI-PKM
% 
% See also:
%                   cds_settings_defaults.m
%                   cds_gen_traj.m
%
% Author: Jannik Fettin
% Institute of Mechatronic Systems (imes), Leibniz Universität Hannover
% email address: jannik.fettin@stud.uni-hannover.de
% Website: http://imes.uni-hannover.de
% 2022-08; Last revision: 25.08.2022

clc; clear; close all;

usr_compare_pso_vs_ga = false;
nreps = 1;
%% Name der Optimierung
for i_optmeth = 2
if i_optmeth == 1, optmeth = 'gamultiobj';
else,              optmeth = 'mopso';
end
for i_rep = 1:nreps
% Definition: configset = struct('optname', ['lufipkm_' datestr(now,'yyyymmdd') '_custom_name_resDir']);
configset = struct('optname', ['lufipkm_4to6j', datestr(now,'yyyymmdd'),...
  sprintf('_%s_rep%d', optmeth, i_rep)]);
% Debug:
% configset.optname = 'lufipkm_20250215_Diss_processworkspace_optmeth_rep1_gamultiobj';
% configset = struct('optname', '20230407_Diss_processworkspace_allrob_matstress_open');
% configset = struct('optname', ['lufipkm_20220811_[Evaluation_V6]_fullworkspace']);

%% Berechnung auf Cluster durchführen
configset.cluster = true;
% Abhängigkeiten von vorherigen Jobs
% configset.jobok = 1367106;

%% Einstellungen für Programmfluss (lokal)
% Whitelist-Einstellungen: Lade i.O.-Ergebnisse dieser Optimierung:
configset.whitelist = {};%'lufipkm_20230415_Diss_processworkspace_allrob_default', ...
  % 'lufipkm_20230417_Diss_processworkspace_robotsfrom0415_default2'};
% Zusammenfassen einer aufgeteilten Berechnung (Nachverarbeiten der vom Cluster heruntergeladenen Ergebnisse)
configset.merge_results_only = false;
% Parellele Berechnung nutzen
configset.parallel = true; % nicht lokal
% Für abgebrochene Berechnungen oder nachträgliche Auswertung:
configset.repair = false;
% Nach abgebrochenem Durchlauf auf Cluster: Unfertige Optimierungen mit vorläufigem Stand abschließen
configset.finish_aborted = false;
% Benutze nur PKM, die vorher schon funktioniert haben
configset.loadwhitelist = true;
configset.whitelist_from_paperdata = true; % Benutze die Pareto-optimalen Roboter aus den Ergebnissen in der Diss
% Anzahl der Wiederholungen der Optimierung
configset.num_repetitions = 1;

%% Allgemeine Einstellugen (general)
% Freiheitsgrad des Roboters
DoF = [1 1 1 1 1 1];    % 3T3R-PKM
% Einstellungs-Struktur initialisieren
Set = cds_settings_defaults(struct('DoF', DoF));
% Abhänigkeiten von vorherigen Jobs
% Set.general.cluster_dependjobs = struct('afterok', configset.jobok);
% Ausgabe der Log-Ergebnisse
Set.general.verbosity = 4;
% Set.general.matfile_verbosity = 3;
% Am Ende von jedem Roboter ein Video erstellen und das Pareto-Bild
Set.general.animation_styles = {'3D'};
% Set.general.save_animation_file_extensions = {'mp4','gif'};
Set.general.save_animation_file_extensions = {}; % keine Animation
Set.general.eval_figures = {'pareto_all_phys'}; % , 'pareto'
% Set.general.eval_figures = {'pareto_all_phys', 'pareto', 'robvisuanim'}; % Hohe Speicherbedarf
Set.general.maxduration_animation = 10;     % Länge der Animation begrenzen

%% Debug-Optionen
% Bilder nicht nur generieren, sondern auch speichern
% Set.general.save_robot_details_plot_fitness_file_extensions = {''};
Set.general.save_robot_details_plot_fitness_file_extensions = {'png', 'fig'};

% Vorlagen neu erzeugen, falls veraltete Dateien vorliegen
% Set.general.create_template_functions = true;

% Debug: Wähle Ausmaß der Debug-Plots während der Maßsynthese
% Set.general.plot_details_in_fitness = 4e9;        % Selbstkollision
% Set.general.plot_details_in_fitness = 1e4;        % Probleme in Traj.
Set.general.plot_robot_in_fitness = 0;

%% Einstellungen zur Auswahl der verwendeten Strukturen (structures)

% whitelist={'P6RRPRRR14V4G5P4A1'};
% whitelist = {'P6RRPRRR10V3G3P2A1'};
whitelist = {};

Set.structures.whitelist= unique(whitelist');

% Strukturauswahl (automatisch)
% Für den Fall, dass keine Positiv-Liste genommen wird:
% Set.structures.joint_filter = {'RPRRRR', 'RRPRRR', 'RRRPRR'};
Set.structures.max_index_active = 3;            % angetriebene Gelenke nicht an EE-Plattform
Set.structures.use_serial = false;              % keine seriellen Roboter
% Anzahl der Strukturen gegenüber der vollständigen Untersuchung reduzieren
if usr_compare_pso_vs_ga
  Set.structures.num_tech_joints = 3;%:4;
else
  Set.structures.num_tech_joints = 4:6;
end
Set.structures.max_index_active_revolute=1;     % Setzt den maximalen Index aktuierter Gelenke fest, nur bezogen auf Drehgelenke

% Laden der bisher erfolgreichen Versuche, dadurch weniger Rechenaufwand.
% Vorherige Optimierungen zur Erkundung möglicher PKM sollten möglichst vollständig gewesen sein.
Set = load_whitelist(Set, configset);
% wenn keine Strukturen in der Whitelist, dann flag negativ setzen. Nur
% dann funktionieren eingestellte joint-filter
if isempty(Set.structures.whitelist)
  configset.loadwhitelist = false;
end

% Benutze den Index aller Ergebnisse zum schnelleren Start (auf dem Cluster)
if configset.cluster
  Set.optimization.InitPopFromGlobalIndex = true;
end
% Alte Ergebnisse laden
if ~configset.cluster % auf Cluster ungültige Pfade
  Set.optimization.result_dirs_for_init_pop = {fullfile(fileparts( which(...
    'robsynth_projektablage_path.m')), '03_Entwicklung', 'LuFI_PKM'), ...
    'C:\Users\Schappler_Moritz\Ubuntu_Transfer\CLUSTER\REPO\structgeomsynth\results'};
end
if usr_compare_pso_vs_ga
  % Keine alten Ergebnisse laden (zur Überprüfung der Optimierungsverfahren)  
  Set.optimization.InitPopRatioOldResults = 0;
end
% Spezifische Umgebungseinstellungen laden
Set = lufi_synth_env(Set);

%% Einstellungen für Aufgabe (Trajektorie, Bauraum, Hindernisse)
Set.task.Ts = 10e-3;    % sample time (muss eher kleiner als 5e-2 gewählt werden)
Set.task.Tv = 1e-1;     % Verschliffzeit Beschleunigung (max Ruck)
% Die maximale Beschleunigung für den konkreten Anwendungsfall mit
% T_min=0.8s und y_max=35mm beträgt ca. 0.3 m/s (a_max=(2pi/T)^2*y_max)
Set.task.amax = 3;      % m/s^2

Set.task.Ts = 50e-3;    % sample time (muss eher kleiner als 5e-2 gewählt werden)
Set.task.Tv = 1e-1;     % Verschliffzeit Beschleunigung (max Ruck)
Set_task = Set.task;
% Trajektorienauswahl (Definition in lufi_synth_traj(Set.task))
% Besteht aus surge-roll, sway-pitch und heave-yaw
% Set.task.refTraj = 'fullprocess';               % Samples: 2832, Eckpunkte 42
% Set.task.refTraj = 'workspacePoints';         % Samples: 1114, Eckpunkte 17
% Set.task.refTraj = 'workspaceOrientation';    % Samples: 5964, Eckpunkte 85
Set_task.refTraj = 'process+workspace';       % Samples: 12990, Eckpunkte 183

% Vollständige Trajektorien (mit zeitlichem Verlauf) nur für prozessnahe
% Bewegungen, da deren Dynamik wichtig ist.
% Set.task.profile = 0;     % Nur Eckpunkte

% Trajektoriendaten laden

Traj = lufi_synth_traj(Set_task);
Set.task.DoF = logical(DoF);    % Zurücksetzen auf Vorgabe für Strukturauswahl
if Set.task.profile==0
    warning('Es werden nur die Eckpunkte, nicht die Trajektorie betrachtet!');
end

% Debug: Visualisierung der Aufgabe
% cds_show_task(Traj, Set);

%% Optimierungs-Einstellungen
Set.optimization.NumIndividuals = 200;      % Kleinere Werte erlauben auch sehr kurze Test-Durchführung.
Set.optimization.MaxIter = 100;             % Anzahl Generationen (wird voraussichtlich sowieso vor Ende abgebrochen)
if ~configset.cluster
  Set.optimization.max_time = 5*60; % 5min lokal
end
% Damit nach jedem Eckpunkt die Kollision bereits geprüft wird, für
% schnelleren Abbruch und damit kürzere Rechenzeit
Set.optimization.single_point_constraint_check = true;
% Abbruchkriterien so definieren, dass nur eine einzige gültige 
% Lösung direkt genommen wird, nur zum aussieben von vielen Strukturen benutzen!
% Set.optimization.obj_limit = [1e3;1e3;1e3]; % Unter 1e3 ist gültig.

% Set.optimization.abort_pareto_front_size = 5; % dadurch früherer Abbruch

% Konditionszahl darf nicht total schlecht sein. Dann werden
% Antriebskräfte und innere Kräfte zu hoch (Erfahrungswerte)
Set.optimization.constraint_obj(4) = 500;   % max. Wert für Konditionszahl
% Die Antriebskraft sollte nicht zu groß werden.
%Set.optimization.constraint_obj(3) = 100;  % max. Wert in Nm / N
% Kollisionsprüfung ist notwendig.
Set.optimization.constraint_collisions = true;
Set.optimization.algorithm = optmeth;

% Debug: Nur statische Kräfte und keine Masse der Beinketten annehmen.
% Auch keine Betrachtung der Plattform-Masse jenseits der definierten
% Traglast. Dadurch keine Betrachtung von Beschleunigungskräften und rein
% statische Auslegung.
% Set.optimization.static_force_only = true;
Set.optimization.nolinkmass = false;
Set.optimization.noplatformmass = false;  

% Segmentstärke optimieren
Set.optimization.desopt_vars = {'linkstrength'};
Set.optimization.constraint_obj(6) = 1; % Materialspannung als Nebenbedingung;
% Set.optimization.safety_link_yieldstrength = 1.5; % 50% Aufschlag
% Bei Optimierung der Segmentdurchmesser die Kollisionen beachten
Set.optimization.constraint_collisions_desopt = true;

% Zielkriterien (2 oder 3 für Pareto-Optimierung)
% Zielfunktion:
% mass, energy, condition, valid_kin, valid_act, actforce, materialstress, stiffness, jointrange, jointlimit
% manipulability, minjacsingval, positionerror, actvelo, chainlength, installspace, footprint, colldist.
Set.optimization.objective = {'actforce','actvelo'};
% Set.optimization.objective = {'power','mass','colldist'};
% Set.optimization.objective = {'materialstress', 'power', 'colldist'};
% Set.optimization.objective = {'actforce','actvelo','stiffness'};
% Set.optimization.obj_limit = 1e3 * ones(length( Set.optimization.objective ), 1);   % Abbruch sobald Lösung gefunden worden ist 


if Set.task.profile == 0 % nur Eckpunkte. Keine Geschwindigkeit bestimmbar
  Set.optimization.objective = {'actforce', 'condition'};
end
  
%% Starten der Maßsynthese
% Einstellung für Jobs auf dem Cluster
Set = adapt_settings(Set, configset);
start_dimsynth(Set, Traj, configset);
end % i_rep
end % optmeth
