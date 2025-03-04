% Geometrie der LuFi-Aufgabe definieren: Bauraum, Kollisionskörper
% 
% Eingabe:
%       Set         Einstellungen für Trajektorie
%                   (Siehe cds_settings_defaults.m)
%
% Ausgabe:
%       Set         wie Eingang, mit mehr Informationen (durchgeschleift)
%                   (Zeit, Position, Geschw.)
%
% See also:
%                   cds_settings_defaults.m
%
% Author: Jannik Fettin
% Institute of Mechatronic Systems (imes), Leibniz Universität Hannover
% email address: jannik.fettin@stud.uni-hannover.de
% Website: http://imes.uni-hannover.de
% 2022-08; Last revision: 24.08.2022

function Set = lufi_synth_env(Set)
%% Bauraum
% Der Roboter soll innerhalb eines Quaders arbeiten
% Annahme: Nicht größer als der Strömungskanal (Breite 2m), Sicherheitsabstand zum Rand jeweils 20 cm
b_Quader = 1.6;     % Breite des Quaders
h_Quader = 1.5;     % Höhe des Quaders
p_Quader = [[-b_Quader/2, -b_Quader/2, 1], ...    % Aufpunkt (unten links);
  [b_Quader,0,0], [0,b_Quader,0], ...             % Zwei Kantenvektoren
  h_Quader];                                      % Länge des letzten Kantenvektors
Set.task.installspace = struct( ...
  'type', uint8(1), ...     % Nummern der Geometrie-Typen, siehe Implementierung (SerRob, check_collisionset_simplegeom)
  'params', p_Quader, ...   % Quader
  'links', {{0:6}});        % Alle Gelenke müssen in dem Quader sein

%% Gestell
% Das Gestell sollte nicht größer als die Bauraumgrenze (Quader) sein.
% In Anforderungen auf Durchmesser des Behälters festgelegt. Setze die
% maximale Größe etwas größer (hier: Radien (xy-Ebene) [min, max]).
Set.optimization.base_size_limits = [b_Quader/4, b_Quader/2-0.05];
% Die Basis kann in der Höhe verschoben werden 
Set.optimization.basepos_limits = zeros(3,2);
Set.optimization.basepos_limits(3,:) = [1.1, 2.5];
% Optimiere auch Paarabstand von Gelenken
Set.optimization.base_morphology = true;
% Auch Verschiebung der Basis in der Ebene zulassen. Für die Aufgabe ist es
% nicht wichtig, ob das Objekt genau mittig unter dem Roboter gehalten
% wird. Es gibt bei einigen PKM Singularitäten in der Mitte des Roboters.
% Diese können damit vermieden werden.
% Set.optimization.basepos_limits(1:2,:) = repmat([-0.300, 0.300], 2, 1);
% Roboter soll hängend von oben nach unten arbeiten.
Set.structures.mounting_parallel = 'ceiling';
% Form der Roboterbasis: Keine Kollisionskörper vorgeben (weniger
% Ausschluss durch Selbstkollision, konstruktion schwieriger).
% Set.optimization.collshape_base = {'joint'};
% Einzelnen Glieder sollen nicht durch die Basisebene fahren
Set.optimization.collshape_base = {'star', 'ring'};

%% Endeffektor-Plattform
% Die Plattform sollte nicht größer als das Gestell sein, eher viel kleiner
Set.optimization.platform_size_limits = [0.1, b_Quader/4];
% Annahme: Symmetrische Maschine ohne Schrägstellung des Endeffektors ist
% günstiger. Sonst ist der Arbeitsraum nicht symmetrisch. Optimiere eine
% Drehung um die z-Achse des EE aber nicht die Kippwinkel um die x-/y-Achse
% Damit kann eine günstige Drehung der PKM gegen die Aufgabe gefunden
% werden. Das geht vielleicht noch über die Drehung der Basis hinaus.
Set.optimization.ee_rotation = true;
Set.optimization.ee_rotation_fixed = [0,0,NaN]; % damit phi_x=phi_y=0 für EE-Trafo.
% Annahme, die Referenztrajektorie bezieht sich auf dei Bewegung des
% Schiffes am Endeffektor (Annahme: 20 cm unter dem Endeffektor Dicke des Kraftsensors: 33mm + Abstand Schwerpunkt vom Schiffsmodell: 76 mm),
% Orientierung bleibt gleich. Der Wert ist bezogen auf das Plattform-KS, nicht das Endeffektor-KS. Das Plf-KS zeigt aus dem Roboter raus
Set.optimization.ee_translation = false;
Set.optimization.ee_translation_fixed=[0,0,0.2];
% Optimiere auch das Aussehen der Plattform (Abstand von paarweise
% angeordneten Koppelgelenken)
Set.optimization.platform_morphology = true;
% Plattform-Drehung nur sehr langsam zulassen. Ansonsten entstehen recht
% starke Oszillationen, da die Grenzen für Kollision und Singularität nahe
% beieinander liegen
Set.optimization.max_velocity_ee_rotation = 45*pi/180;
% Beschleunigung der redundanten Koordinate eher groß wählen. Abbau der max. Geschwindigkeit ist damit sehr schnell möglich
Set.optimization.max_acceleration_ee_rotation = 2*pi/0.200;

%% Gelenke
% Technisch unrealistisch, aber für Ergebnis-Veranschaulichung besser.
Set.optimization.max_range_passive_universal = 180*pi/180;
Set.optimization.max_range_passive_spherical = 180*pi/180;  % Vorher 120
% Begrenzung des Verfahrweges von Schubgelenken. Mit NaN deaktiviert (wird dann aus Größe des Roboters plausibel abgeleitet).
Set.optimization.max_range_prismatic=1; % Normaler Hub von Linearmotoren in m

%% Zusätzliche Masse am Endeffektor
% Repräsentiert das Werkzeug/Schiff. CAD-Gewicht: 54,2 kg
Set.task.payload = struct('m', 54.2, 'rS', zeros(3,1), 'Ic', zeros(6,1));
% Trägheitsmomente: ( Gramm *  Quadratmillimeter )
% Massenträgheitsmomente im Schwerpunkt [kg mm^2]
% 	Ixx = 429592.48		Ixy = 0.129		Ixz = -162622.918
% 	Iyx = 0.129		Iyy = 8260442.053	Iyz = 0.085
% 	Izx = -162622.918	Izy = 0.085		Izz = 8335501.64
% Massenträgheitsmoment xx, yy, zz
Set.task.payload.Ic(1) = 429592.48*10^-6;
Set.task.payload.Ic(2) = 8260442.053*10^-6;
Set.task.payload.Ic(3) = 8335501.64*10^-6;
% Annahme: Erst xy, xz, yz
% Nicht symmetrische EE-Last noch nicht definiert
Set.task.payload.Ic(4) = 0.129*10^-6;
Set.task.payload.Ic(5) = -162622.918*10^-6;
Set.task.payload.Ic(6) = 0.085*10^-6;