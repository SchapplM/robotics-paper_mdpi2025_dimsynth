% Perform the combined structural and dimensional synthesis for 3T0*R-PKM
% Use example from PrauseChaCor2015

% References: Prause, I.; Charaf Eddine, S.; Corves, B. Comparison of
% parallel kinematic machines with three translational degrees of freedom 
% and linear actuation. Chin. J. Mech. Eng. 2015, 28, 841–850. https://doi.org/10.3901/CJME.2015.0128.052.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
% User Settings for this file
usr_cluster = true; % if a computing cluster is configured.
usr_relax_requirements = false;
usr_notrajectory = true;
nreps = 3;
%% Optimierung aller Roboter für alle Aufgabentypen starten
for i_rep = 1:nreps
% Einstellungen der Optimierung
Set = cds_settings_defaults(struct('DoF', logical([1 1 1 0 0 0])));
Set.general.verbosity = 4;
Set.task.pointing_task = true;
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
% Traj = cds_gen_traj(Set.task.DoF, 1, Set.task);

% Keine Zusatzmasse
Set.task.payload.m = 0;
Set.task.payload.Ic(:) = 0;
Set.task.payload.rS(:) = 0;

%% Trajektorie definieren
% PrauseChaCor2015], Fig. 3, Gl. 10
h = 0.100; b = 0.200;
x_MS = [0;0;h/2];
XE = NaN(3^3, 6);
delta = 0.5*[b, b, h];
signmat = allcomb([-1,0,+1], [-1,0,+1], [-1,0,+1]);
for i = 1:size(XE,1)
  XE(i,1:3) = x_MS' + signmat(i,:) .* delta;
  XE(i,4:6) = 0;
end

% Sortiere die Eckpunkte so, dass der Weg dazwischen minimal wird
% (Ist besser für Konvergenz der Positions-IK)
% TODO

% Bestimme Trajektorie im Arbeitsraum
if ~usr_notrajectory
  Set.task.profile = 2; % 0= nur Eckpunkte, 2=4851 Punkte im Arbeitsraum (Diff.Quot.)
  % Erzeuge Zick-Zack-Bahnen von unten nach oben
  dz = 0.01;
  dx = 0.01;
  dy = 0.01;
  vmax = 1; % PrauseChaCor2015, Gl. 8
  dt = 0.01 / vmax;
  % dt = 0.2; % 10mm in 200ms -> v=0.05m/s (sonst werden Gelenkgrenzen verletzt)
  corners = minmax2(XE(:,1:3)');
  % Zig-Zag 3D
  z = corners(3,1);
  last_x = corners(1,1); last_y = corners(2,1);
  xyz_traj = [];
  IE = 1;
  while z < corners(3,2)
    if last_y == corners(1,1), next_y = corners(2,2);
    else,                      next_y = corners(2,1);
    end
    if last_x == corners(1,1), next_x = corners(1,2);
    else,                      next_x = corners(1,1);
    end
    % Zick-Zack zur nächsten Ecke
    xval = last_x:dx*sign(next_x-last_x):next_x;
    yval = last_y:dy*sign(next_y-last_y):next_y;
    [xmesh,ymesh] = meshgrid(xval,yval);
    x_zigzag = zigzag(xmesh);
    y_zigzag = zigzag(ymesh);
    z_zigzag = ones(length(x_zigzag),1)*z;
    xyz_traj = [xyz_traj; [x_zigzag(:), y_zigzag(:), z_zigzag(:)]]; %#ok<AGROW> 
    % Als Eckpunkt der Trajektorie eintragen (für DP um red. Koord. zu ändern)
    IE = [IE; size(xyz_traj,1)]; %#ok<AGROW> 
    z = z + dz;
    last_x = next_x;
    last_y = next_y;
  end
  % Debug: Trajektorie
  %   figure(1);clf;
  %   title(sprintf('Zick-Zack-Trajektorie für z=%1.3f', z));
  %   subplot(2,1,1); hold on; grid on; title('xyz');
  %   stairs(xyz_traj(:,1), 'rx-');
  %   stairs(xyz_traj(:,2), 'go--');
  %   stairs(xyz_traj(:,3), 'b+:');
  %   subplot(2,1,2); hold on; grid on; title('diff xyz');
  %   stairs(diff(xyz_traj(:,1)), 'rx-');
  %   stairs(diff(xyz_traj(:,2)), 'go--');
  %   stairs(diff(xyz_traj(:,3)), 'b+:');
  % Bestimme die Indizes der Eckpunkte in der neuen Trajektorie
  % (Erfordert neue Sortierung der Eckpunkte, damit sie stetig sind)
  % j_next = 1;
  % IE = NaN(size(XE,1),1); % Indizes der Eckpunkte in der Trajektorie
  % for i = 1:size(xyz_traj)
  %   for j = 1:size(XE,1)
  %     if all(abs(xyz_traj(i,:) - XE(j, 1:3)) < 1e-8)
  %       IE(j) = i;
  %     end
  %   end
  % end
  % Sortiere die Eckpunkte neu, damit sie fortlaufend mit Traj. passen
  % [~, Isort] = sort(IE);
else
  Set.task.profile = 0;
end
Traj.XE = XE; % nicht sortieren, da die Indizes bei profile=2 nicht konsistent sein müssen.
% Belegung der Trajektorien-Struktur
if Set.task.profile == 0
  Traj.X = Traj.XE;
  Traj.XD = zeros(size(XE));
  Traj.XDD = Traj.XD;
  Traj.t = (1:size(XE,1))';
  Traj.IE = Traj.t;
else
  Traj.X = [xyz_traj, zeros(size(xyz_traj,1),3)];
  Traj.XD = [zeros(1,6); [diff(xyz_traj)/dt, zeros(size(xyz_traj,1)-1,3)]];
  Traj.XDD = [zeros(1,6); diff(Traj.XD)/dt];
  Traj.t = dt*(0:size(Traj.X,1)-1)';
  Traj.IE = IE;
  % Prüfe vorgegebene Maximalgeschwindigkeit
  assert(all(all( abs(Traj.XD(:,1:3)) < vmax*1.01 )), sprintf(['XD ist zu ' ...
    'groß: max [%s]m/s. Erlaubt %1.1f'], disp_array(max(abs(Traj.XD(:,1:3))), '%1.1f'), vmax) );
end

% Externe Kraft
% PrauseChaCor2015], Gl. 8
Traj.Fext = repmat([2, 2, -10, 0, 0, 0], size(Traj.X,1), 1);

% Kollisionskörper über der Trajektorie, damit es nicht unplausibel von
% oben kommt (einheitliche Lösungen von einer Seite der Aufgabe)
% Durchmesser so wie Breite der Aufgabe
r = mean(max(Traj.XE(:,1:2)) - min(Traj.XE(:,1:2)))/2;
% Höhe des Mittelpunkts der Kugel so, dass Kugel die Aufgabe von oben
% berührt
h = max(Traj.XE(:,3)) + r + 2*r; % mit +r ist die Kugel genau anschließend an Punkte. Dann noch Kollision mit Plattform-Körper. Daher nochmal +2r
Set.task.obstacles = struct( ...
  'type', uint8(4), ... % Kugel
  'params', [[0,0,h], r]); % Mittelpunkt, Radius
% Das erfordert, dass es keine Kugel in der Plattform-Mitte gibt. Sonst
% würde eine große Plattform immer auch automatisch mit der Begrenzung
% kollidieren
Set.optimization.collshape_platform = {'star', 'ring'};

% Bauraumbegrenzung: Kein Teil des Roboters über dem Arbeitsraum
if ~usr_relax_requirements
  n_instspcbodies = 2;
  Set.task.installspace = struct( ...
    'type', uint8(zeros(n_instspcbodies,1)), ... % Nummern der Geometrie-Typen, siehe Implementierung.
    'params', NaN(n_instspcbodies,10), ...
    'links', {cell(n_instspcbodies,1)});
  r_Zylinder = 2;
  p_Zylinder = [[0,0,-2], ... % Erster Punkt (unten)
    [0,0,max(Traj.XE(:,3))+10e-3], ... % Zweiter Punkt (oben)
    r_Zylinder, NaN(1,3)];
  Set.task.installspace.type(1) =  uint8(2); % Zylinder
  Set.task.installspace.params(1,:) = p_Zylinder;
  Set.task.installspace.links(1) = {1:6};  % Alle bewegten Teile des Roboters müssen im Bauraum sein
  % Die ersten Gelenk des Roboters sollen auch nicht über den unteren Rand
  % des Arbeitsraums gehen
  p_Zylinder2 = [[0,0,-2], ... % Erster Punkt (unten)
    [0,0,min(Traj.XE(:,3))], ... % Zweiter Punkt (oben)
    r_Zylinder, NaN(1,3)];
  Set.task.installspace.type(2) =  uint8(2); % Zylinder
  Set.task.installspace.params(2,:) = p_Zylinder2;
  Set.task.installspace.links(2) = {1:3};  % Alle bewegten Teile des Roboters müssen im Bauraum sein
end
%% Optimierungseinstellungen
% moderate Anzahl an Wiederholungen (reicht für gute Konvergenz)
Set.optimization.NumIndividuals = 400;
Set.optimization.MaxIter = 500;
Set.structures.min_task_redundancy = 0; % 0=erlaube auch 3T0R; 1=nur 3T1R
Set.structures.max_task_redundancy = 1; % 1=Erlaube Aufgabenredundanz, 0=nur 3T0R
% Beschränkung auf parallel
Set.structures.use_serial = false;
Set.structures.use_parallel = true;
Set.structures.max_index_active_revolute = 1; % Nur aktives Gestellgelenk bei Drehantrieb
Set.structures.max_index_active_prismatic = 3; % RP, RRP oder UP als Beginn der Beinkette erlauben
% Parallel und auf Cluster
Set.general.computing_cluster = usr_cluster; % Auf Cluster rechnen
Set.general.computing_cluster_cores = 12; % Weniger Kerne: Mehr Jobs, dafür schnellerer Start
Set.general.cluster_maxrobotspernode = Set.general.computing_cluster_cores;
Set.general.computing_cluster_max_time = 8*3600;
% Debug: Auf bestimmten Cluster-Job warten (für bessere Startwerte)
Set.general.cluster_dependjobs.afterany = 1852526;

% Debug-Bilder (werden auf Cluster nicht erzeugt)
% Set.general.plot_robot_in_fitness = 4.5e9;
% Set.general.plot_details_in_fitness = 3e9; % Bauraumverletzung zeigen
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig', 'png'};

if usr_cluster
  Set.general.plot_robot_in_fitness = 0;
  Set.general.plot_details_in_fitness = 0;
end
% Erlaubt in Desktop-Modus den Abschluss abgebrochener Optimierungen
if ~usr_cluster
  Set.general.matfile_verbosity = 3;
end
% Bilder nicht speichern
Set.general.save_robot_details_plot_fitness_file_extensions = {};
% Für jeden Roboter eine Animation erstellen
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.eval_figures = {'pareto_all_phys'};
Set.general.noprogressfigure = true;

% Dynamische Programmierung benutzen. Ähnliche Einstellungen wie AMUN
% Set.optimization.objective_ik = 'poserr_ee';
Set.optimization.objective_ik = 'maxactforce';
Set.general.taskred_dynprog = true;
Set.general.taskred_dynprog_and_gradproj = false;
Set.general.taskred_dynprog_only = true;
Set.optimization.traj_ik_abort_on_success = true;
Set.optimization.ee_rotation_limit_rel_traj = [-60, 60]*pi/180;
Set.general.taskred_dynprog_numstates = [5 0];
Set.general.debug_taskred_perfmap = 0; % Anzeigen und Speichern der Redundanzkarte

% Keine singulären PKM nehmen
Set.optimization.condition_limit_sing = 1e4;
Set.optimization.condition_limit_sing_act = 1e4;
% Debug: auch singuläre PKM zulassen
% Set.optimization.condition_limit_sing = inf;
% Set.optimization.condition_limit_sing_act = inf;
% Bewusst PKM untersuchen, die einen Rangverlust haben
% Set.structures.use_parallel_rankdef = 6;

% Weitere Rahmenbedingungen der Maßsynthese
% Konditionszahl darf nicht total schlecht sein. Dann werden
% Antriebskräfte und innere Kräfte zu hoch (Erfahrungswerte)
% Set.optimization.constraint_obj(4) = 5000*1e3; % max. Wert für Konditionszahl (vermeide quasi-singuläre)
% Die Antriebskraft sollte nicht zu groß werden.
Set.optimization.constraint_obj(3) = 100*1e3; % max. Wert in N oder Nm (damit Pareto-Diagramm übersichtlicher ist)
% Der Positionsfehler darf nicht zu groß sein
Set.optimization.constraint_obj(7) = 0.5e-3; % in m, am Endeffektor

% Zielkriterium Antriebskraft in PrauseChaCor2015
Set.optimization.objective = {'actforce', 'installspace', 'positionerror'};
% Set.optimization.objective = {'actforce', 'workspace', 'positionerror'};
% Set.optimization.objective = {'positionerror', 'power', 'installspace'};
if Set.structures.use_parallel_rankdef > 0
  % Bei Rangverlust ist die Untersuchung der Konditionszahl sinnlos.
  Set.optimization.objective = {'chainlength', 'installspace'};
  % Die Aktuierung wird auch ignoriert, es geht nur um die Mechanismen
  Set.structures.max_index_active_revolute = 6;
end
% Debug: Bei erstem funktionierendem Ergebnis aufhören
% Set.optimization.obj_limit = 1e3*ones(length(Set.optimization.objective),1);
% Debug: Bei mehreren dominierenden Ergebnissen aufhören
% Set.optimization.abort_pareto_front_size = 5;

Set.optimization.constraint_collisions = true;
% In z-Richtung frei lassen (mit plausiblen Grenzen). Sonst mittig.
% PKM ist unterhalb der Aufgabe
Set.optimization.basepos_limits = [[0,0];[0,0];[-1.0,-0.1]];
% Debug: Benutze auch außermittige Basis, da 3T1R-PKM sonst evtl. immer
% singulär
% Set.optimization.basepos_limits(1:2,:) = repmat([-0.1, 0.1],2,1);
% Dimension von Plattform und Gestell: PrauseChaCor2015, Text unter Table 4
Set.optimization.base_size_limits = [0.100, 0.500];
Set.optimization.platform_size_limits = [0.100, 0.500];
if usr_relax_requirements
  Set.optimization.base_size_limits = [0.100, 0.750];
end
% Grenzen der Schubgelenke aus PrauseChaCor2015, Abschnitt 4.3 Ende
Set.optimization.max_range_prismatic = 0.400;
if usr_relax_requirements
  Set.optimization.max_range_prismatic = 0.600;
end
% PrauseChaCor2015, Gl. 5
Set.optimization.max_velocity_active_prismatic = 3.5;
% Länge der Beinkette begrenzen, PrauseChaCor2015, Gl. 7
Set.optimization.max_chain_length = 1;
if usr_relax_requirements
  Set.optimization.max_chain_length = 1.5;
end
% Verhältnis Plattform zu Gestell begrenzen, PrauseChaCor2015, Gl. 6; [Prause2016], Gl. 8.11
Set.optimization.max_platform_base_ratio = 1;
% Positionsfehler so wie im Paper berechnen: PrauseChaCor2015, nach Gl. 19
% (Keine Vorgabe für Drehgelenke)
Set.optimization.obj_positionerror.prismatic = 0.052e-3;
% Gestellgelenk mit max. 30° Neigung, PrauseChaCor2015, S.846 oben rechts
% Set.structures.parrob_basejointfilter = 4; % Nur mit Gestellneigung, Debug
Set.optimization.max_inclination_conic_base_joint = 30*pi/180;
% Debug: Schubzylinder dürfen über Gelenk hinausgehen
if usr_relax_requirements
  Set.optimization.prismatic_cylinder_allow_overlength = true;
end
% Debug: Gelenkabstand mind. so dass es konstruierbar ist?
% Set.optimization.min_joint_distance

% Das Eigengewicht der Struktur wird nicht betrachtet, PrauseChaCor2015, Gl. 9
Set.optimization.noplatformmass = true;
Set.optimization.nolinkmass = true;
Set.optimization.static_force_only = true; % hat keine Auswirkungen, da keine Massen vorhanden.

% Betrachte dann auch keine Beschleunigungs-Grenzen, da die Trajektorie
% sowieso kein konsistentes Bahnprofil erzeugt (wegen Rechteckregel)
Set.optimization.max_acceleration_prismatic = inf; % ca. 5g Beschleunigung ist technisch kaum zu realisieren
Set.optimization.max_acceleration_revolute = inf; % Maximale Geschw. (20) wäre in 0.2s erreicht. Sehr hoher Wert für frühe Erkennung schlechter Konditionierung


Set.general.debug_calc = false;
Set.optimization.base_morphology = true;
Set.optimization.platform_morphology = true;
Set.optimization.rotate_base = true; % Müsste eigentlich egal sein bei Aufgabenredundanz
Set.optimization.ee_rotation = false; % Kein Einfluss bei Aufgabenredundanz

% Debug: Visualisierung der Aufgabe
% cds_show_task(Traj, Set)
% return

% Nicht-symmetrische Länge der Schubzylinder ermöglicht mehr Lösungen, aber
% dafür unplausibler. daher symmetrisch wählen.
Set.optimization.joint_limits_symmetric_prismatic = true;

% Benutze den Index aller Ergebnisse zum schnelleren Start
Set.optimization.InitPopFromGlobalIndex = true;

% Setze Ordner mit bisherigen Ergebnissen für Startwerte der Optimierung.
% Dauert teilweise etwas länger beim Laden, da auf Projektablage
% zugegriffen wird.
if ~usr_cluster
  Set.optimization.result_dirs_for_init_pop = {HandlingRobot_dimsynth_data_dir()}; %, ...
%     'C:\Users\Schappler_Moritz\Ubuntu_Transfer\CLUSTER\REPO\structgeomsynth\results'};
end

% Debug: Nur einen Roboter erzeugen
% Set.structures.whitelist = {'P3PRRR1G4P7A1'}; % 3PRRR
% Set.structures.whitelist = {'P3PRRRR8V1G1P2A1'}; % 3PUU
% Set.structures.whitelist = {'P4RRRRR5V1G1P2A1'}; % 4RRUR
% Set.structures.whitelist = {'P4RRRRR10V1G1P1A1'}; % 4RUU
% Set.structures.whitelist = {'P3PRRRR7G4P9A1'}; % konische Basis
% Set.structures.whitelist = {'P3PRRRR4V1G4P2A1'}; % bisher fehlende PKM, aus Paper
% Set.structures.whitelist = {'P3RRRRR6G2P9A1'}; % Beispiel für Gelenke oberhalb der Plattform


% Nehme Roboter aus vorherigen Auswertungen mit bestimmtem Status:
% resdir_wl = ['C:\Users\Schappler_Moritz\Ubuntu_Transfer\' ...
%   'CLUSTER\REPO\structgeomsynth\results'];
resdir_wl = HandlingRobot_dimsynth_data_dir();
optname_wl = {'ARK_3T1R_20230624_phdthesis_v1_perfmap',...
  'ARK_3T1R_20230701_phdthesis_v2_all_rep1', 'ARK_3T1R_20230701_phdthesis_v2_all_rep2', ...
  'ARK_3T1R_20230701_phdthesis_v3_all_relax_rep1', 'ARK_3T1R_20230701_phdthesis_v3_all_relax_rep2'};
for ii = 1:length(optname_wl)
ResTab_ii = readtable(fullfile(resdir_wl, optname_wl{ii}, [optname_wl{ii}, '_results_table.csv']));
if ii == 1, ResTab = ResTab_ii; else, ResTab = [ResTab; ResTab_ii]; end %#ok<AGROW> 
end
% Alle die erfolgreich waren
% Set.structures.whitelist = ResTab.Name(ResTab.Fval_Opt<1e3)';
% Alle die bis zur Trajektorie gekommen sind:
% Set.structures.whitelist = ResTab.Name(ResTab.Fval_Opt<1e9)';
% Alle mit Ausschlussgrund wegen Beinkettenlänge
% Set.structures.whitelist = ResTab.Name(ResTab.Fval_Opt>=4.9e9 & ResTab.Fval_Opt<=5e9)';
% Set.structures.whitelist = Set.structures.whitelist(1);
% Alle die nicht bis zur Trajektorie gekommen sind:
% Set.structures.whitelist = ResTab.Name(ResTab.Fval_Opt>1e9)';
% Ergebnisse bei denen die Schubzylinderlänge problematisch war:
% Set.structures.whitelist = ResTab.Name(ResTab.Fval_Opt>=4.25e9 & ResTab.Fval_Opt<=4.5e9)';
% Set.structures.whitelist = unique(Set.structures.whitelist);
% Debug: Struktur einschränken (nur Schubgelenke)
% Set.structures.joint_filter = {'P****', '*P****', '**P***'};

% Set.structures.num_tech_joints = 4; % PRUR
% Set.structures.parrob_basejointfilter = 10;

% Sieht auf Bildern übersichtlicher aus im Gegensatz zu Deckenmontage
Set.structures.mounting_parallel = 'floor';
if Set.general.computing_cluster && length(Set.structures.whitelist) == 1
  warning('Es soll nur ein Roboter auf dem Cluster berechnet werden. Sicher?');
  pause(5); % Bedenkzeit
end
Set.optimization.optname = sprintf('ARK_3T1R_20230807_v1');
if usr_relax_requirements
  Set.optimization.optname  = [Set.optimization.optname, '_relax']; %#ok<UNRCH> 
end
if nreps > 1
  Set.optimization.optname = [Set.optimization.optname, sprintf('_rep%d', i_rep)];
end

% Debug: Abgebrochenen Durchlauf abschließen
Set.general.only_finish_aborted = false;
% Debug: Falls Berechnung abgebrochen ist und Videos nachträglich erstellt
% werden sollen
Set.general.regenerate_summary_only = false; 
cds_start(Set,Traj);
if nreps > 1 && i_rep < nreps
  pause(4);
end
end

return
% Debug: Vergleiche Einstellungen
resdir = 'C:\Users\Schappler_Moritz\Ubuntu_Transfer\CLUSTER\REPO\structgeomsynth\results';
optname1 = 'ARK_3T1R_20230701_phdthesis_v1_all';
optname2 = 'ARK_3T1R_20230701_phdthesis_v2_all_rep1';
tmp1 = load(fullfile(resdir, optname1, [optname1, '_settings.mat']));
Set1 = cds_settings_update(tmp1.Set);
tmp2 = load(fullfile(resdir, optname2, [optname2, '_settings.mat']));
Set2 = cds_settings_update(tmp2.Set);
compare_structs(Set1, Set2)
