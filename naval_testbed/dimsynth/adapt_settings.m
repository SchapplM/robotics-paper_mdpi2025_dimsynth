% Passe die Einstellungen der Maßsynthese benutzerspezifisch an

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Set = adapt_settings(Set, configset)

cluster = configset.cluster;
parallel = configset.parallel;
repair = configset.repair;
finish_aborted = configset.finish_aborted;

if cluster || parallel
  Set.general.parcomp_struct = true; % Maßsynthese parallel durchführen
  Set.general.parcomp_plot = true; % Bilder parallel erzeugen
end
if cluster
  Set.general.computing_cluster = true; % Auf Cluster rechnen
  % Wenn weniger Kerne gefordert sind, wird der Job schneller gestartet.
  % Sollte zu den verfügbaren Knoten passen.
  Set.general.computing_cluster_cores = 4;
  % Nur 1 Roboter pro Kern und maximal parallel, nicht sequentiell
  Set.general.cluster_maxrobotspernode = Set.general.computing_cluster_cores;
  % Debug: Rechenzeit auf dem Cluster begrenzen (für schnellere Bearbeitung)
  Set.general.computing_cluster_max_time = 7*3600; % in Sekunden
  % Debug: Erst folgende Jobs auf Cluster fertig werden lassen:
  % (z.B. wenn in Job die Mex-Dateien aktualisiert werden)
  % Set.general.cluster_dependjobs = struct('afterok', [], 'afternotok', [], 'afterany', []);
  % Debug: Wenn beim Hochladen ein Fehler passiert ist, überspringe die
  % schon gestarteten Teile
%   Set.general.continue_with_part = 155;
else
  Set.optimization.max_time = 10*60; % lokal sehr schnell fertig werden
end
if repair
  Set.general.regenerate_summary_only = true;
  Set.general.nosummary = false; % Tabelle und Bilder
end
if finish_aborted
  Set.general.only_finish_aborted = true;
end
