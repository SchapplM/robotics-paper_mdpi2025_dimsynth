% Auswertung des aktuellen Entwurfs zum Vergleich mit der Maßsynthese
% (Handling-PKM, Beispiel aus PrauseChaCor2015)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
rng(0);

usr_fig_all = {'default', 'onlypoints'};
for i_usr_fig = 1:2
usr_figselection = usr_fig_all{i_usr_fig};
% Auswahl des Roboters: Siehe PrauseChaCor2015, Tab. 4
usr_robot_selection_all = {'CRR', 'UPU', 'PUU', 'CRU', 'CUR'};
for i_usr_rob = 1:length(usr_robot_selection_all)
for i_collcheck = 0:1
for i_configcrit = 1:2
usr_robot_selection = usr_robot_selection_all{i_usr_rob};
fprintf('Berechne Fitness-Funktion für %s (i_collcheck=%d, i_configcrit=%d)\n', ...
  usr_robot_selection, i_collcheck, i_configcrit)
%% Lade Roboterdefinition (aus anderem Versuch, zur Vereinfachung)
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir'));
if isempty(repo_dir)
  error(['You have to create a file HandlingRobot_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = HandlingRobot_dimsynth_data_dir();
% Maßsynthese-Durchlauf auswählen, dessen Einstellungen benutzt werden,
% um die ausgewählte Parametrierung auszuwerten.
% Sollte übereinstimmen mit Datengrundlage für Pareto-Bilder.
if strcmp(usr_figselection, 'default')
  OptName = 'ARK_3T1R_20230730_full_rep1';
elseif strcmp(usr_figselection, 'onlypoints')
  OptName = 'ARK_3T1R_20230804_v2_rep2'; % keine Trajektorie
end
% Es wird der passende Robotertyp ausgewählt (vgl. PrauseChaCor2015, Tab 3)
if strcmp(usr_robot_selection, 'CRR')
  RobName = 'P3PRRR1G4P7A1';
elseif strcmp(usr_robot_selection, 'PUU')
  % Sternförmige konische Basis in Paper. Gab nur eine funktionierende
  % Konfiguration der Plattform bei mir
  RobName = 'P3PRRRR8V1G4P2A1';
elseif strcmp(usr_robot_selection, 'CRU')
  RobName = 'P3PRRRR8V2G4P2A1';
elseif strcmp(usr_robot_selection, 'CUR')
  RobName = 'P3PRRRR4V1G4P2A1';
elseif strcmp(usr_robot_selection, 'UPU')
  % Dreieckige Basis in Paper. Bei Kardangelenk hier mit allen
  % Gestellgelenk-Ausrichtungen möglich (G1, G2, G3)
  RobName = 'P3RRPRR12V3G2P2A1';
else
  error('Fall nicht berücksichtigt')
end
% Daten laden
setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
ws_mean = mean(minmax2(d1.Traj.XE(:,1:3)')'); %#ok<UDIM> 
fprintf('Mitte des Arbeitsraums: [%s]mm\n', disp_array(1e3*ws_mean, '%1.1f'));
Set_i = cds_settings_update(d1.Set);
OptName2 = 'ARK_3T1R_20230730_full_rep1'; % evtl. anders, falls PKM nicht in erster Auswertung drin ist (ist bei onlypoints der Fall)
resfiles = dir(fullfile(resdirtotal, OptName2, sprintf('Rob*_%s_Endergebnis.mat', RobName)));
resfile = fullfile(resfiles(1).folder, resfiles(1).name);
tmp = load(resfile);
RobotOptRes_i = tmp.RobotOptRes;
Structure = tmp.RobotOptRes.Structure;
parroblib_update_template_functions({RobName});
% parroblib_addtopath({RobName});
% Set_i.optimization.max_range_passive_revolute
% Set_i.optimization.max_range_passive_universal = inf;
% Optionen anpassen (vor der Initialisierung)
if strcmp(usr_robot_selection, 'CRU')
%   Set_i.optimization.collision_bodies_size = 20e-3; % verkleinern
%   Set_i.optimization.collision_bodies_safety_distance = 0; % verkleinern
%   Set_i.task.obstacles = struct('type', [], 'params', []);
elseif strcmp(usr_robot_selection, 'CUR')
%   Set_i.optimization.constraint_collisions = false;
  Set_i.optimization.max_range_prismatic = 4; % Effektiv deaktivieren. TODO: noch unklar, warum
end
% Bauraumgrenzen deaktivieren (sind in Prause-Paper nicht enthalten)
Set_i.task.installspace = struct('type', [], 'links', [], 'params', []);
Set_i.optimization.base_tolerance_prismatic_guidance = 5; % effektiv deaktivieren
Set_i.optimization.collshape_base = {}; % Damit die Kollisionsprüfungen damit richtig initialisiert werden
Set_i.optimization.collshape_platform = {'ring'}; % Bei vorherigen Einstellungen noch teilweise riesige Kugel in Plattform
[R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, Structure, true);

%% Ersetze die Parameter
vn = Structure.varnames;
pval = NaN(length(vn),1);
pval(strcmp(vn, 'scale')) = 1;
% Siehe PrauseChaCor2015, Tab. 4.
% In Paper: Angabe von z_ms bezeichnet die Höhe der Arbeitsraummitte über
% der PKM-Basis.
pval(strcmp(vn, 'baserotation z')) = 0; % dort nicht angegeben. Also Null. Müsste nicht wichtig sein
if strcmp(usr_robot_selection, 'CRR')
  pval(strcmp(vn, 'base z')) = ws_mean(3) - 0.12;
  pval(strcmp(vn, 'base radius')) = 0.3; % Sternförmig. Hat keinen Einfluss
  pval(strcmp(vn, 'platform radius')) = 0.11;
  pval(strcmp(vn, 'pkin 2: a3')) = 0.29; % l1
  pval(strcmp(vn, 'pkin 5: d3')) = 0;
  pval(strcmp(vn, 'pkin 3: a4')) = 0.25; % l2
  pval(strcmp(vn, 'pkin 6: d4')) = 0;
  pval(strcmp(vn, 'pkin 7: theta1')) = 0;
  pval(strcmp(vn, 'base_morph_coneelev')) = (90+30)*pi/180 - 1e-12;
elseif strcmp(usr_robot_selection, 'PUU')
  pval(strcmp(vn, 'base z')) = ws_mean(3) - 0.32;
  pval(strcmp(vn, 'base radius')) = 0.25; % Hat keine Wirkung bzw. nicht im Paper angegeben
  pval(strcmp(vn, 'platform radius')) = 0.23;
  pval(strcmp(vn, 'pkin 2: a4')) = 0.45; % nur eine Länge im Paper gegeben
  pval(strcmp(vn, 'pkin 5: d4')) = 0;
%   pval(strcmp(vn, 'base_morph_coneelev')) = (90-5)*pi/180 - 1e-12; % Numerische Genauigkeit beachten
  pval(strcmp(vn, 'base_morph_coneelev')) = (-85)*pi/180 + 1e-12; % Numerische Genauigkeit beachten
elseif strcmp(usr_robot_selection, 'CRU')
  pval(strcmp(vn, 'base z')) = ws_mean(3) - 0.42;
  pval(strcmp(vn, 'base radius')) = 300e-3; % Hat keine kinematische Wirkung bzw. nicht im Paper angegeben. Muss groß gewählt werden, damit die Führungsschienen nicht kollidieren
  pval(strcmp(vn, 'platform radius')) = 0.42;
  pval(strcmp(vn, 'pkin 2: a3')) = 0.1; % l1
  pval(strcmp(vn, 'pkin 6: d3')) = 0;
  pval(strcmp(vn, 'pkin 3: a4')) = 0.42; % l2
  pval(strcmp(vn, 'pkin 7: d4')) = 0;
  pval(strcmp(vn, 'base_morph_coneelev')) = (90+25)*pi/180;
elseif strcmp(usr_robot_selection, 'CUR')
  pval(strcmp(vn, 'base z')) = ws_mean(3) - 0.92;
  pval(strcmp(vn, 'base radius')) = 200e-3; % s.o. bei CRU
  pval(strcmp(vn, 'platform radius')) = 0.15;
  pval(strcmp(vn, 'pkin 2: a3')) = 0.49; % l1
  pval(strcmp(vn, 'pkin 5: d3')) = 0;
  pval(strcmp(vn, 'pkin 3: a5')) = 0.49; % l2
  pval(strcmp(vn, 'pkin 6: d5')) = 0;
  pval(strcmp(vn, 'pkin 7: theta1')) = 0; % darf eigentlich nicht vorkommen
  pval(strcmp(vn, 'base_morph_coneelev')) = (90+30)*pi/180 - 1e-12;
elseif strcmp(usr_robot_selection, 'UPU')
  pval(strcmp(vn, 'base z')) = ws_mean(3) - 0.45;
  pval(strcmp(vn, 'base radius')) = 0.5;
  pval(strcmp(vn, 'platform radius')) = 0.12;
else
  error('Fall nicht berücksichtigt')
end
assert(all(~isnan(pval)), 'pval enthält NaN');
pval_phys_test = cds_update_robot_parameters(R, Set_i, Structure, pval);
fprintf('Daten des Roboters:\n');
fprintf('Gestell-Radius: %1.3fmm\n', 1e3*R.DesPar.base_par(1));
fprintf('Plattform-Radius: %1.3fmm\n', 1e3*R.DesPar.platform_par(1));

% fprintf('Oberarm-Länge: a2=%1.3fmm, d2=%1.3fmm\n', 1e3*R.Leg(1).MDH.a(2), 1e3*R.Leg(1).MDH.d(2));
% fprintf('Unterarm-Länge: a4=%1.3fmm, d4=%1.3fmm\n', 1e3*R.Leg(1).MDH.a(4), 1e3*R.Leg(1).MDH.d(4));

%% Berechne die Zielfunktion. Dadurch Detail-Kennzahlen zur Kinematik
Set = Set_i;
% Speicherort für Bilder (z.B. Redundanzkarte aus Dynamischer Programmierung)
% Set.optimization.resdir = fullfile(repo_dir, 'data', 'existing_design');
% Abweichungen von den gespeicherten Einstellungen vornehmen.
% Achtung: Das gefährdet die Vergleichbarkeit der Konstruktionslösung mit
% der aktuellen Maßsynthese
Set.general.plot_details_in_fitness = 0;
Set.general.plot_robot_in_fitness = 0;
% Debug-Bilder, falls keine Lösung gefunden wird.
% Set.general.plot_details_in_fitness = -1e3; % Nur bei Fehler Bilder erstellen
Set.general.plot_robot_in_fitness = 1e12;
Set.optimization.pos_ik_abort_on_success = false; % Manchmal ist die zweite IK-Lösung erst sinnvoll
Set.optimization.traj_ik_abort_on_success = false;

existingdesign_dataname = sprintf('existing_design_%s', usr_figselection);
Set.optimization.resdir = fullfile(repo_dir, 'data', existingdesign_dataname); % 'existing_design_collopt'
Set.optimization.optname = existingdesign_dataname; % Damit es immer im gleichen Ordner landet
defstruct = cds_definitions();

Set.optimization.objective = defstruct.obj_names_all; % Berechne alle Kriterien (dann Ergebnis für jede Auswertung nutzbar)
% Wähle die Konfiguration, die die kleinste Fläche benötigt. Entspricht am
% ehesten der plausiblen Lösung (gut für Bild) und entspricht dem
% Optimierungsziel. Zusätzlich Antriebskraft, da es im Pareto-Bild ist.
% TODO: Kommentar noch veraltet.
if i_configcrit == 1 % Durch die automatische Auswahl des Optimierungsziels wird dann doch die kleinste Konditionszahl benutzt
  Set.optimization.criteria_config_selection = {'actforce'};
elseif i_configcrit == 2
  Set.optimization.criteria_config_selection = {'installspace'}; % Nehme nur die kleinste PKM
else
  error('Fall nicht definiert');
end

% Arbeitsraumberechnung dauert zu lange und wird an anderer Stelle gemacht
Set.optimization.objective = Set.optimization.objective(~strcmp(Set.optimization.objective, 'workspace'));
Set.optimization.obj_limit = zeros(length(Set.optimization.objective), 1);
Set.optimization.obj_limit_physval = Set.optimization.obj_limit;

Set.optimization.pos_ik_tryhard_num = 200;
% Es muss auch die Schnittkraft berechnet werden
Structure.calc_cut = true;
% Ordner für Speicherung von tmp-Bildern erzeugen.
[~,~,~,resdir] = cds_get_new_figure_filenumber(Set, Structure, '');
mkdirs(resdir);
% Ohne die Vorgabe von q0_traj funktionierte es mal nicht. Jetzt doch wieder. Unklar, warum.)
% Structure.q0_traj = cat(1,R_ref.Leg(:).qref);
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig', 'png'}; % Speicherung der Redundanzkarte als Bild
cds_log(0, '', 'init', Set);
cds_save_particle_details(); cds_fitness();
[fval, physval, Q, QD, QDD, TAU, ~, Jinv_ges] = cds_fitness(R, Set,d1.Traj, Structure, pval);
if any(fval > 1e3)
  error('Berechnung nicht erfolgreich');
end

PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
Traj_0 = cds_transform_traj(R, d1.Traj);
X = Traj_0.X;
objective_names = Set_i.optimization.objective;
existingdesign_resname = sprintf('detail_result_PrauseChaCor2015_%s_%s_collcheck%d_config%d', ...
  usr_robot_selection, usr_figselection, i_collcheck, i_configcrit);
save(fullfile(repo_dir, 'data', sprintf('%s.mat', existingdesign_resname)), ... % '.mat'
  'R', 'pval', 'fval', 'physval', 'Structure', 'Q', 'QD', 'TAU', 'Jinv_ges', 'X', ... % Daten zum Ergebnis
  'objective_names', 'Set', 'Traj_0');

fprintf('Bereich der Antriebskräfte: %1.1f...%1.1f\n', min(max(abs(TAU),[],2)),max(abs(TAU(:))));

%% Bild erstellen (Code aus robot_images.m)
fighdl = change_current_figure(1); clf; hold on;
% Dicke der Segmente reduzieren
for kk = 1:R.NLEG
  R.Leg(kk).DesPar.seg_par(:,1) = 1e-3;
  R.Leg(kk).DesPar.seg_par(:,2) = 1e-2;
end
s_plot = struct('ks_legs', [], 'ks_platform', [], 'mode', 4, ...
  'straight', 0, ... % Damit Quader in der Mitte der Verbindung ist
  'nojoints', 2); % Damit Quader für Gelenk gezeichnet werden
if any(contains(Structure.varnames, ': a'))
  s_plot.straight = 1;
end
s_plot.jointsize = [0.06, 0.03];
% Nehme den untersten Punkt der Trajektorie
[xwmin,I_plot] = min(d1.Traj.X(:,3)); % Welt-KS
R.plot(Q(I_plot,:)', X(I_plot,:)', s_plot);
view(3);
title('');xlabel('');ylabel('');zlabel('');
LegColors = [ [0 1 0]; [0 0 0]; [1 0 1]; [0 0 1]; [1 0 0]; [0 1 1] ]; % Grün, Schwarz, Violett, blau, rot, cyan
% Automatisch generierten Plot nachverarbeiten
ch = get(gca, 'Children');
for jj = 1:length(ch)
  % KS entfernen
  if strcmp(ch(jj).Type, 'hgtransform')
    delete(ch(jj)); continue
  end
  % Weise den Beinketten neue Farben zu
  for kk = 1:R.NLEG
    if contains(get(ch(jj), 'DisplayName'), sprintf('Leg_%d_Link', kk))
      set(ch(jj), 'EdgeColor', LegColors(kk,:));
    end
  end
end
xw_mean = mean(minmax2(d1.Traj.X(:,1:3)'),2);
ws_off_z = 0.1; % Arbeitsraum etwas weiter oben zeichnen, damit EE mit 100mm Abstand angenommen wird (Bild übersichtlicher)
if exist('wshdl', 'var'), delete(wshdl); end
wshdl = drawCuboid([xw_mean(1), xw_mean(2), ws_off_z+xw_mean(3), diff(minmax2(d1.Traj.X(:,1)')), ...
  diff(minmax2(d1.Traj.X(:,2)')), diff(minmax2(d1.Traj.X(:,3)'))], ...
  'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', 0.3);
view([76,7.5]);
if strcmp(usr_robot_selection, 'CRR')
  view([76,7.5]);
elseif strcmp(usr_robot_selection, 'CRU')
  view([8,10]);
elseif strcmp(usr_robot_selection, 'CUR')
  view([74,5.5]);
end
set(gca,'XTICKLABEL',{});set(gca,'YTICKLABEL', {});set(gca,'ZTICKLABEL',{});
set(gca,'xtick',[],'ytick',[],'ztick',[]);
set(get(gca, 'XAxis'), 'visible', 'off');
set(get(gca, 'YAxis'), 'visible', 'off');
set(get(gca, 'ZAxis'), 'visible', 'off');
figure_format_publication(gca);
set(gca, 'Box', 'off');
set(1, 'windowstyle', 'normal');
set_size_plot_subplot(1, ...
  8,8,gca,...
  0,0,0,0,0,0)
drawnow();
name = sprintf('RobotFig_existingdesign_%s_collcheck%d_config%d', ...
  usr_robot_selection, i_collcheck, i_configcrit);
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
% outputdir = fullfile(phdthesis_dir, 'manuscript', '05_evaluation', 'figures', ...
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures', ...
  sprintf('robots_handlingpkm_existingdesign'));
mkdirs(outputdir);
prevdir = pwd();
cd(outputdir);
export_fig(fighdl, [name, '_r864.png'], '-r864');
savefig(fighdl, [name, '.fig']);
cd(prevdir);
end % i_configcrit
end % i_collcheck
end % i_usr_rob
end % i_usr_fig
return
%% Testen der Konfigurationen

for i_usr_rob = 1:length(usr_robot_selection_all)
  usr_robot_selection = usr_robot_selection_all{i_usr_rob};  
  usr_figselection = 'default';
  resname1 = sprintf('detail_result_PrauseChaCor2015_%s_%s_collcheck%d_config%d', ...
    usr_robot_selection, usr_figselection, 0, 1);
  data1 = load(fullfile(repo_dir, 'data', sprintf('%s.mat', resname1)));

  
  fprintf('Vergleiche Ergebnisse für %s\n', usr_robot_selection)
  for i_collcheck = 0:1
  for i_configcrit = 1:2
  resname2 = sprintf('detail_result_PrauseChaCor2015_%s_%s_collcheck%d_config%d', ...
    usr_robot_selection, usr_figselection, i_collcheck, i_configcrit);
  data2 = load(fullfile(repo_dir, 'data', sprintf('%s.mat', resname2)));
  
  test_physival = data1.physval - data2.physval;
  if all(test_physival == 0)
    fprintf('collcheck=%d, config=%d: identisch zu Fall 0/1\n', i_collcheck, i_configcrit);
  else
    fprintf('collcheck=%d, config=%d: unterschiedlich zu Fall 0/1\n', i_collcheck, i_configcrit);
    if i_collcheck == 0 && i_configcrit == 1
      error('Logik-Fehler. Ergebnisse anders obwohl es der gleiche Fall ist.');
    end
  end

end
end
end
