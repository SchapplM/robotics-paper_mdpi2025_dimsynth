% Definition der Beispiel-Trajektorie für die Maßsynthese der LuFI-PKM
% 
% Eingabe:
%       trajset     Einstellungen für Trajektorie
%                   (Siehe cds_settings_defaults.m)
%
% Ausgabe:
%       Traj        Struktur mit Trajektorie des Endeffektors
%                   (Zeit, Position, Geschw.)
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

function Traj = lufi_synth_traj(trajset)
%% Initialisierung
Fx=0; Fy=0; Fz=0; Mx=0; My=0; Mz=0; Fext=[];
% Kräfte und Momente der Trajektorie (siehe Kraftmodell)
FzHeave=403; % [N]
MxRoll=562; % [Nm]
MyPitch=63; % [Nm]
F_max=[33.5588  451.0889  659.0945   67.4671  608.3941  162.5326];
% Eckpunkte für Beispiel-Trajektorie bestimmen (minimalmaße Arbeitsraum)
b_ws = 0.100;               % minimale Breite workspace (+- Bereich um Null, Anforderungen)
h_ws = 0.200;               % minimale Höhe workspace (Anforderungen)
phi_ws=15*pi/180;           % minimale Orientierung aller Achsen (Anforderungen)
% Punkte im Welt-KS.
X0 = [ [0;0;1.1]; [0;0;0]*pi/180 ];  % Ruhestellung außerhalb des Wassers
X1 = X0' + [0, 0, -h_ws/2,0,0,0];    % Arbeitshöhe auf Höhe des Wassers
if strcmp(trajset.refTraj,'process+fullworkspace')
    xp=pi/4;
    yp=pi/4;
    xSchritt=0.5;
    ySchritt=0.5;
    zSchritt=0.5;
else
    xp=0;
    yp=0;
    xSchritt=1;
    ySchritt=1;
    zSchritt=1;
end
%% Surgebewegung und Roll (x-Achse)
k=1;   SR(k,:) = X1;
for alpha=0:pi/4:xp
    % Über Wasser, Arbeitshöhe, unter Wasser
    for z=1:-zSchritt:-1
        % Surgebewegung
        Pos=rotx(alpha)*[b_ws;0;z*h_ws/2];
        k=k+1; SR(k,:) = X1 + [ Pos' r2eulxyz(rotx(0))'];
        Pos=rotx(alpha)*[-b_ws;0;z*h_ws/2];
        k=k+1; SR(k,:) = X1 + [ Pos', r2eulxyz(rotx(0))'];
        % Rollbewegung
        Pos=rotx(alpha)*[0;0;z*h_ws/2];
        k=k+1; SR(k,:) = X1 + [ Pos', r2eulxyz(rotx(phi_ws))'];
        k=k+1; SR(k,:) = X1 + [ Pos', r2eulxyz(rotx(-phi_ws))'];
    end
end
k=k+1; SR(k,:) = X1;
%% Sway und Pitch (y-Achse)
k=1;   SP(k,:) = X0;
for beta=0:pi/4:yp
    for z=1:-zSchritt:-1
        % Swaybewegung
        Pos=roty(beta)*[0;b_ws;z*h_ws/2];
        k=k+1; SP(k,:) = X1 + [ Pos' r2eulxyz(roty(0))'];
        Pos=roty(beta)*[0;-b_ws;z*h_ws/2];
        k=k+1; SP(k,:) = X1 + [ Pos', r2eulxyz(roty(phi_ws))'];
        % Pitchbewegung
        Pos=roty(beta)*[0;0;z*h_ws/2];
        k=k+1; SP(k,:) = X1 + [ Pos', r2eulxyz(roty(0))'];
        k=k+1; SP(k,:) = X1 + [ Pos', r2eulxyz(roty(phi_ws))'];
    end
end
k=k+1; SP(k,:) = X1;
%% Heave und Yaw
k=1;   HY(k,:) = X1;
for x=0:xSchritt:0
    for y=0:ySchritt:0
       for z=1:-1:-1
           % Heavebewegung
            k=k+1; HY(k,:) = X1 + [ x*b_ws,y*b_ws,z*h_ws/2, r2eulxyz(rotz(0))'];
            % Yawbewegung
            k=k+1; HY(k,:) = X1 + [ x*b_ws,y*b_ws,z*h_ws/2, r2eulxyz(rotz(phi_ws))'];
            k=k+1; HY(k,:) = X1 + [ x*b_ws,y*b_ws,z*h_ws/2, r2eulxyz(rotz(-phi_ws))'];
            k=k+1; HY(k,:) = X1 + [ x*b_ws,y*b_ws,z*h_ws/2, r2eulxyz(rotz(0))'];
       end
    end   
end 
k=k+1; HY(k,:) = X1;
%% Quader in zwei Quadranten des Arbeitsraumes
% Vereinfachte Darstellung in zwei Quadranten
k=1; AR =X0';
% Beginn Würfel mit zusätzlicher 3D-EE-Drehung obere Ebene
k=k+1; AR(k,:) = AR(k-1,:) + [b_ws,0,0, 0,0,0];
k=k+1; AR(k,:) = AR(k-1,:) + [0,b_ws,0  0,0, phi_ws];
k=k+1; AR(k,:) = AR(k-1,:) + [-b_ws,0,0, 0,0,-phi_ws];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0,-h_ws, phi_ws,0, 0];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0, h_ws, -phi_ws,0,0];
k=k+1; AR(k,:) = AR(k-1,:) + [0,-b_ws,0,  0,phi_ws,0];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0,-h_ws, 0,-phi_ws,0];
% untere Ebene: Fahre anders herum, damit Drehgelenke nicht mehrfache
% Umdrehungen machen müssen
k=k+1; AR(k,:) = AR(k-1,:) + [ 0,b_ws,0, phi_ws,-phi_ws,0];
k=k+1; AR(k,:) = AR(k-1,:) + [b_ws,0,0  -phi_ws,phi_ws,0];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0, h_ws, -phi_ws,phi_ws,phi_ws];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0,-h_ws, -phi_ws,phi_ws,0];
k=k+1; AR(k,:) = AR(k-1,:) + [0,-b_ws,0, phi_ws,-phi_ws,phi_ws];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0, h_ws, phi_ws,-phi_ws,phi_ws];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0,-h_ws, phi_ws,-phi_ws,phi_ws];
k=k+1; AR(k,:) = AR(k-1,:) + [-b_ws,0,0,  -phi_ws,0,-phi_ws];
k=k+1; AR(k,:) = AR(k-1,:) + [0,0, h_ws, -phi_ws,phi_ws,phi_ws];
%% Eckpunkte die die Orientierung am Arbeitsraumrand abdecken
k=0;
for i=1:-1:-1
    jPoints=[1,1,-1,-1]; lPoints=[1,-1,-1,1];
    for m=1:length(jPoints)
        j=jPoints(m); l=lPoints(m);
        % Einfacher Winkel
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,0,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,0,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,phi_ws,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,-phi_ws,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,0,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,0,-phi_ws];
        % Zweifacher Winkel
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,phi_ws,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,phi_ws,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,-phi_ws,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,-phi_ws,0];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,0,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,0,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,0,-phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,0,-phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,phi_ws,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,phi_ws,-phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,-phi_ws,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, 0,-phi_ws,-phi_ws];
        % Dreichfacher Winkel
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,phi_ws,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,phi_ws,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,-phi_ws,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,phi_ws,-phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,-phi_ws,phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,phi_ws,-phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, phi_ws,-phi_ws,-phi_ws];
        k=k+1; IL(k,:) = X1 + [ j*b_ws,l*b_ws,i*0.1, -phi_ws,-phi_ws,-phi_ws];
    end
    k=k+1; IL(k,:) = X1 + [ jPoints(1)*b_ws,lPoints(m)*b_ws,i*0.1, phi_ws,phi_ws,phi_ws];
end
%% Fallunterscheidung
% In der Trajektorie wird jeweils nur die maximale Kraft berücksichtigt!
% Fext=[Fx,Fy,Fz,Mx,My,Mz];
if strcmp(trajset.refTraj,'fullprocess')
    warning('Es werden die maximal wirkenden Kräfte und Momente verwendet!');
    Mx=MxRoll;
    My=MyPitch;
    Fz=FzHeave;
    XE = [SR;SP;HY];
elseif strcmp(trajset.refTraj,'workspaceOrientation')
    XE = IL;
    Fext=zeros(length(AR),6);
    warning('Externe Kraft wird hier vernachlässigt!');
elseif strcmp(trajset.refTraj,'workspacePoints')
    XE = AR;
    Fext=zeros(length(AR),6);
    warning('Externe Kraft wird hier vernachlässigt!');
elseif strcmp(trajset.refTraj,'process+workspace')
    Fx=F_max(1); Fy=F_max(2); Fz=F_max(3);
    Mx=F_max(4); My=F_max(5); Mz=F_max(6);
    warning('Es werden alle maximal wirkenden Kräfte und Momente verwendet!');
    XE=[SR;SP;HY;IL]; 
else 
    error("trajset.refTraj not defined!");
end
%% Trajektorie generieren
if trajset.profile == 1
    [X_ges,XD_ges,XDD_ges,T_ges,IE] = traj_trapez2_multipoint(XE, ...
        trajset.vmax, trajset.vmax/trajset.amax, trajset.Tv, trajset.Ts, 0);
    if strcmp(trajset.refTraj,'fullprocess') || strcmp(trajset.refTraj,'process+workspace')
        Fext=nan(length(T_ges),6);
        for i=1:1:length(T_ges)
            Fext(i,:)=[Fx, Fy, Fz, Mx, My, Mz];
        end
    end 
elseif trajset.profile == 0 % Nur Eckpunkte
  X_ges = XE; XD_ges = XE*0; XDD_ges = XE*0;
  T_ges = (1:size(XE,1))'; IE = (1:size(XE,1))';
else
  error('Profil nicht definiert');
end

%% Ausgabe
% Doppelte Eckpunkte entfernen zur schnelleren Prüfung. Sortiere die Punkte
% entsprechend der ursprünglichen Reihenfolge (sonst Fehler / Unplausibel)
% (ist für Redundanzkarte nachteilig, die aber hier irrelevant ist)
[XE_unique, I_unique] = unique(XE, 'rows', 'stable');
Traj = struct('X', X_ges, 'XD', XD_ges, 'XDD', XDD_ges, 't', T_ges, ...
  'Fext', Fext, 'XE', XE_unique, 'IE', IE(I_unique));