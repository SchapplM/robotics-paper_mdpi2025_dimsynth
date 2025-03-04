% Erzeuge hochaufgelöste Bilder der einzelnen Roboter aus den
% Optimierungsergebnissen.
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% * Zusammenstellen der Details mit select_eval_robot_examples.m
% 
% This script is based on the same file eval_figures_pareto.m from 
% https://github.com/SchapplM/robsynth-paper_mhi2021
% (MHI paper "Combined Structural and Dimensional Synthesis of a Parallel
% Robot for Cryogenic Handling Tasks", Schappler et al. 2022)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

usr_fig_all = {'default_revolute', 'default_prismatic'};
for i_usr_fig = 1:2
usr_figselection = usr_fig_all{i_usr_fig}; % siehe andere Skripte. Möglichkeiten: volume, motordiagram

warning('off', 'Coder:MATLAB:rankDeficientMatrix'); % für invkin2
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
% outputdir = fullfile(phdthesis_dir, 'manuscript', '05_evaluation', 'figures', ...
paperrepo_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir = fullfile(paperrepo_dir, 'paper', 'Figures', ...
  sprintf('robots_handlingpkm_%s', usr_figselection));
mkdirs(outputdir);
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');
importdir = HandlingRobot_dimsynth_data_dir();
tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)));
RobotGroups = tmp.RobotGroups;
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');

InfoTab = readtable(fullfile(outputdir, '..', ...
  sprintf('handlingpkm_groups_legend_%s_add.csv', usr_figselection)), 'Delimiter', ';');

%% Alle Gruppen durchgehen
countrob = 0; % Nummern aus Legende
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  if RobotGroups.ResultsFound(i) == 0, continue; end % keine Ergebnisse vorliegend
  if contains(GroupName(2:end), 'P') && contains(usr_figselection, 'revolute') || ...
    ~contains(GroupName(2:end), 'P') && contains(usr_figselection, 'prismatic')
    continue % Gruppe wird hier nicht betrachtet.
  end
%   if ~strcmp(GroupName, 'P4RPRRR8V2G'), continue; end % Debug
  fprintf('Zeichne Bild für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  resfile = fullfile(datadir, sprintf('detail_result_group_%s_%s.mat', GroupName, usr_figselection));
  if ~exist(resfile, 'file')
    warning('Ergebnis für Gruppe existiert nicht. select_eval_robot_examples ausführen!');
    continue
  end
  erg = load(resfile);
  % countrob = countrob + 1; % nur gültige PKM-Gruppen zählen
  countrob = i; % Geht nur, wenn alle Gruppen gültig sind.
  % Daten der Ergebnisse laden
  fprintf('Anzahl Variablen: %d\n', length(erg.Structure.vartypes));
  disp(strjoin(erg.Structure.varnames));
  fprintf('Parametertypen: [%s]\n', disp_array(erg.Structure.vartypes', '%1.0f'));
  close all
%   if strcmp(usr_figselection, 'volume') && GroupName(2)=='6'
%     continue % nur 5FG-PKM für kompakten Bauraum zeichnen
%   end
%   if strcmp(usr_figselection, 'motordiagram') && GroupName(2)=='5'
%     continue % nur 6FG-PKM für Motordiagramm zeichnen
%   end
%   if ~strcmp(GroupName, 'P3PRRRR4V1G1'), continue; end
  setfile = dir(fullfile(importdir, erg.OptName, '*settings.mat'));
  assert(~isempty(setfile), sprintf('Für Optimierung %s fehlt die Einstellungsdatei', erg.OptName));
  d1 = load(fullfile(importdir, erg.OptName, setfile(1).name));
  Set_i = cds_settings_update(d1.Set);
  %% Roboter initialisieren
  R = erg.R;
  % Dicke der Segmente reduzieren
  for kk = 1:R.NLEG
    R.Leg(kk).DesPar.seg_par(:,1) = 1e-3;
    R.Leg(kk).DesPar.seg_par(:,2) = 1e-2;
  end
  parroblib_addtopath({R.mdlname});
  %% Anpassung der Visualisierung
  Q = erg.Q;
  X = erg.X;
  Q_plot = Q;
  X_plot = X;
  % Trajektorie in Welt-KS (für Plot) umrechnen
  Traj_W = R.transform_traj(struct('X', X), false);
  XW_plot = Traj_W.X;
  xw_mean = mean(minmax2(XW_plot(:,1:3)'),2);
  %% Bild zeichnen und Formatieren
  for imgset = 2 % (Vorerst ohne Bauraum-Quader zeichnen)
  change_current_figure(1); clf; hold on;
  s_plot = struct('ks_legs', [], 'ks_platform', [], 'mode', 4, ...
    'straight', 0, ... % Damit Quader in der Mitte der Verbindung ist
    'nojoints', 2); % Damit Quader für Gelenk gezeichnet werden
  % Bei Beinketten mit a-Parametern muss straight=1 gewählt werden. Sonst
  % werden die Ersatz-Gelenke in die Luft gezeichnet.
  if any(contains(erg.Structure.varnames, ': a'))
    s_plot.straight = 1;
  end
  % Bei einigen PKM sind die Gelenke so dicht beeinander, dass sie kleiner
  % gezeichnet werden müssen
%   if GroupName(2) == '6'% any(strcmp(GroupName, {'P6RRRRRR10G', 'P6RRRRRR5G', 'P6RRRRRR5V2G'}))
    s_plot.jointsize = [0.06, 0.03];
%   end
  % Debug: Plotte in mittlerer Stellung des Roboters
  x_mean = mean(minmax2(X_plot(:,1:3)'),2);
  % Finde den Punkt der Trajektorie, der der Mitte am nächsten ist.
  if size(Q,1) == 1
    warning('Keine Trajektorie gegeben')
    I_plot = 1; % Nehme den ersten Punkt
  else
    % Nehme den untersten Punkt der Trajektorie
    [xwmin,I_plot] = min(XW_plot(:,3));
    % Alternativ: Nehme einen Punkt aus der Mitte
%     [~,I_plot] = min(sum((repmat(x_mean',size(X_plot,1),1)-X_plot(:,1:3)).^2,2));
  end
  R.plot(Q_plot(I_plot,:)', X_plot(I_plot,:)', s_plot);
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

  % Optional: Zusätzlich Bauraum als Rahmenlinien einzeichnen.
  % (macht Bild aber unübersichtlich)
  % Umrechnen in Format der plot-Funktion
  if imgset == 1
    q_W = Set_i.task.installspace.params(1:3)';
    u1_W = Set_i.task.installspace.params(4:6)';
    u2_W = Set_i.task.installspace.params(7:9)';
    % letzte Kante per Definition senkrecht auf anderen beiden.
    u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*Set_i.task.installspace.params(10);
    u3_W(3) = 1.2; % Reduziere Ausdehnung nach oben
    cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
    cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
    cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
    drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
      'FaceColor', 'c', 'FaceAlpha', 0.01);
  end
  % Arbeitsraum einzeichnen (vorher in Welt-KS umrechnen)
%   pts = allcomb(minmax2(X_plot(:,1)'), minmax2(X_plot(:,2)'), minmax2(X_plot(:,3)'));
%   plot3(pts(:,1), pts(:,2), pts(:,3), 'co');
  ws_off_z = 0.1; % Arbeitsraum etwas weiter oben zeichnen, damit EE mit 100mm Abstand angenommen wird (Bild übersichtlicher)
  if exist('wshdl', 'var'), delete(wshdl); end
  wshdl = drawCuboid([xw_mean(1), xw_mean(2), ws_off_z+xw_mean(3), diff(minmax2(XW_plot(:,1)')), ...
    diff(minmax2(XW_plot(:,2)')), diff(minmax2(XW_plot(:,3)'))], ...
    'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', 0.3);

  % Dummy-Plot für Legende mit Roboter-Marker
  hdl=plot3(0,0,NaN, 'Marker', RobotGroups.PlotMarker{i}, 'Color', RobotGroups.PlotColor{i}, 'LineStyle', 'None');
%   I_latexname = strcmp(ResTab_NameTrans.PKM_Name, RobotGroups.Robots{i});
%   legstr_i = [sprintf('%d: ', countrob), ResTab_NameTrans.ChainStructure{I_latexname}];
%   legstr_i = strrep(legstr_i, 'P', '\underline{P}');
  % abgespeicherten Marker-Typ einzeichnen (zur Wiedererkennung beim
  % Zusammenstellen der Bilder)
  % lh = legend(hdl, legstr_i, 'interpreter', 'latex'); 
  % set(lh, 'location', 'northeastoutside');
  
  % Nehme überall die gleiche Perspektive für Vergleichbarkeit der Bilder?
  % Nein. Dann sind die Verdeckungen zu hoch.
  view(43,16); % Standard-Perspektive
  % Manuelle Anpassungen für einzelne Bilder möglichst wenig Verdeckungen)
  % Debug: [a,b] = view()
  if strcmp(GroupName, 'P3PRRR1G')
    view(87,18);
  elseif strcmp(GroupName, 'P3PRRRR3V1G')
    view(83,8);
  elseif strcmp(GroupName, 'P3PRRRR4G')
    view(102,17);
  elseif strcmp(GroupName, 'P3PRRRR4V1G')
    view(103,15);
  elseif strcmp(GroupName, 'P3PRRRR6G')
    view(62,13);
  elseif strcmp(GroupName, 'P3PRRRR6V1G')
    view(38,14);
  elseif strcmp(GroupName, 'P3PRRRR7G')
    view(87,10);
  elseif strcmp(GroupName, 'P3PRRRR7V1G')
    view(99,6.5);
  elseif strcmp(GroupName, 'P3PRRRR8G')
    view(93,13);
  elseif strcmp(GroupName, 'P3PRRRR8V1G')
    view(73,13);
  elseif strcmp(GroupName, 'P3PRRRR8V2G')
    view(80,9);
  elseif strcmp(GroupName, 'P3RPRRR12V2G')
    view(69,3);
  elseif strcmp(GroupName, 'P3RPRRR12V3G')
    view(79,10);
  elseif strcmp(GroupName, 'P3RPRRR2V1G')
    view(114,12);
  elseif strcmp(GroupName, 'P3RPRRR6V2G')
    view(82,10);
  elseif strcmp(GroupName, 'P3RPRRR6V3G')
    view(30,10);
  elseif strcmp(GroupName, 'P3RPRRR8V2G')
    view(46,15);
  elseif strcmp(GroupName, 'P3RPRRR8V3G')
    view(105,21);
  elseif strcmp(GroupName, 'P3RPRRR9V2G')
    view(22,13);
  elseif strcmp(GroupName, 'P3RPRRR9V3G')
    view(100,7);
  elseif strcmp(GroupName, 'P3RRPRR12V3G')
    view(54,21);
  elseif strcmp(GroupName, 'P3RRPRR12V4G')
    view(52,22);
  elseif strcmp(GroupName, 'P3RRPRR12V5G')
    view(98,20);
  elseif strcmp(GroupName, 'P3RRPRR4V1G')
    view(8,12);
  elseif strcmp(GroupName, 'P3RRPRR5V1G')
    view(11,13);
  elseif strcmp(GroupName, 'P4RPRRR8V2G')
    view(51,17);
  elseif strcmp(GroupName, 'P4RPRRR8V3G')
    view(36,14);
  elseif strcmp(GroupName, 'P4RRPRR12V4G')
    view(73,19);
  elseif strcmp(GroupName, 'P4RRPRR12V5G')
    view(58.5,9);
  elseif strcmp(GroupName, 'P3RRRRR10G')
    view(57,12);
  elseif strcmp(GroupName, 'P3RRRRR10V1G')
    view(91,15);
  elseif strcmp(GroupName, 'P3RRRRR10V2G')
    view(41,7);
  elseif strcmp(GroupName, 'P3RRRRR5G')
    view(64,24);
  elseif strcmp(GroupName, 'P3RRRRR5V1G')
    view(91,15);
  elseif strcmp(GroupName, 'P3RRRRR6G')
    view(24,18);
  elseif strcmp(GroupName, 'P3RRRRR6V1G')
    view(16,24);
  elseif strcmp(GroupName, 'P3RRRRR7G')
    view(88,6);
  elseif strcmp(GroupName, 'P3RRRRR7V2G')
    view(89,22);
  elseif strcmp(GroupName, 'P3RRRRR8G')
    view(195,18);
  elseif strcmp(GroupName, 'P3RRRRR8V2G')
    view(82,19.6);
  elseif strcmp(GroupName, 'P4RRRRR10G')
    view(53,13);
  elseif strcmp(GroupName, 'P4RRRRR10V1G')
    view(114,10);
  elseif strcmp(GroupName, 'P4RRRRR10V2G')
    view(27,12);
  elseif strcmp(GroupName, 'P4RRRRR5G')
    view(91,16);
  elseif strcmp(GroupName, 'P4RRRRR5V1G')
    view(73,12);
  elseif strcmp(GroupName, 'P4RRRRR8G')
    view(111,8);
  elseif strcmp(GroupName, 'P4RRRRR8V2G')
    view(101,11);
  else
    fprintf('Keine manuell gewählte Perspektive. Nehme Standard.\n')
  end
  % Debug: [a,b] = view()

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
  % Bild speichern
  i_IT = strcmp(InfoTab.GroupName, GroupName);
  if ~any(i_IT)
    error('Keine Daten in Tabelle. select_eval_robot_examples.m neu ausführen.');
  end
  [tokens, match] = regexp(InfoTab.TextFix{i_IT}, ...
    'PR (\d+)\.(\d+) .*', 'tokens', 'match');
  % TODO: 3T1R in Dateinamen
  name = sprintf('RobotFig_%dT%dR_PR_%s_%s_%s_Group_%s', sum(R.I_EE(1:3)), ...
    sum(R.I_EE(4:6)), tokens{1}{1}, tokens{1}{2}, ...
    strrep(InfoTab.CouplingOnly{i_IT}, '-', '_'), GroupName);
  if imgset == 1
    name = [name, '_box']; %#ok<AGROW> 
  end
  cd(outputdir);
  export_fig([name, '_r864.png'], '-r864');
  savefig([name, '.fig'])
  end
end

end
