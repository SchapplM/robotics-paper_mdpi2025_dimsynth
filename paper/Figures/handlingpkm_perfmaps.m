% Erzeuge ein Bild mit vielen Redundanzkarten
% Nehme zwei verschiedene Einstellungen (Kriterien)
% 
% Before execution, select_eval_robot_examples.m has to be run with activated performance maps

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
cdsdef = cds_definitions();
% Gebe das Kriterium für die Redundanzkarte vor
usr_fig_all = {'default_revolute', 'default_prismatic', 'detail_P4RPRRR8V2G'};
usr_save_fig = true;
for i_usr_fig = 1:3
usr_figselection = usr_fig_all{i_usr_fig};
usr_xaxis = 'samples'; % Optionen: time, normalized, samples
colorlimit_all = [];
for i_cases = [1 2 4]
if i_cases == 1
  usr_pm_criterion = 'positionerror';
elseif i_cases ==2
  usr_pm_criterion = 'cond_ikjac';
elseif i_cases == 3
  usr_pm_criterion = 'cond_jac';
elseif i_cases == 4
  usr_pm_criterion = 'actforce';
else
  error('i_cases nicht definiert')
end
if strcmp(usr_figselection, 'detail_P4RPRRR8V2G') && i_cases ~= 4
  continue % Nur Antriebskraftbild für einzelne PKM
end
usr_objective_ik = 'maxactforce'; % konsistent mit select_eval_robot_examples.m
%% Initialization
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir'));
if isempty(repo_dir)
  error(['You have to create a file PrauseDemo_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
% Datenquelle wählen
if contains(usr_figselection, 'prismatic')
  usr_figselection1 = 'default_prismatic';
elseif contains(usr_figselection, 'revolute')
  usr_figselection1 = 'default_revolute';
elseif contains(usr_figselection, 'detail')
  usr_figselection1 = 'default_prismatic'; % nehme die passende Datenquelle
else
  error('Fall nicht definiert');
end
% Muss konsistent mit eval_existing_design.m sein.
datadir = fullfile(repo_dir,'data');
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');
InfoTab = readtable(fullfile(outputdir, sprintf('handlingpkm_groups_legend_%s.csv', ...
  usr_figselection1)), 'Delimiter', ';');
tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection1)));
RobotGroups = tmp.RobotGroups;
%% Create Performance Maps
i_ax = 0;
fighdl = change_current_figure(1); clf;
 % TODO: Layout ändern? Insgesamt sind es 12 PKM. Bei 5x3 gibt es Platz für zwei Farbskalen
if contains(usr_figselection, 'prismatic')
  axhdl = gobjects(2,3); % für 5 PKM
elseif contains(usr_figselection, 'revolute')
  axhdl = gobjects(3,3); % für 7 PKM
elseif contains(usr_figselection, 'detail')
  axhdl = gobjects(1,1); % für eine PKM
else
  error('Fall nicht definiert');
end
cbhdl = NaN;
legdata = cell(10,5);
idx_legdata = 1; % Nächster freier Index
fprintf('Zeichne Redundanzkarten\n');
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups.GroupName{i};
  if RobotGroups.ResultsFound(i) == 0, continue; end % keine Ergebnisse vorliegend
  if GroupName(2)=='3'
    continue % nur 3T1R-PKM zeichnen
  end
  if contains(usr_figselection, 'detail') && ~contains(usr_figselection,GroupName)
    continue % Wähle nur eine einzige PKM aus
  end
  fprintf('Zeichne Redundanzkarte für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  resfile = fullfile(datadir, sprintf('detail_result_group_%s_%s.mat', ...
    GroupName, usr_figselection1)); % es gibt nur eine IK-Zielfunktion. Nicht mehr im Dateinamen vermerkt
  if ~exist(resfile, 'file')
    warning('Ergebnis für Gruppe existiert nicht. select_eval_robot_examples ausführen!');
    continue
  end
  erg = load(resfile);
  
  % Pfad zum temporären Optimierungsordner für die Reproduktion aus Skript
  % eval_existing_design.m
  tmpdir_i = fullfile(datadir, sprintf('select_eval_reproduce_opt_%s', usr_objective_ik), ...
    erg.OptName, 'tmp', sprintf('%d_%s', 1, erg.RobName));
  if ~exist(tmpdir_i, 'file')
    warning('Tmp-Verzeichnis existiert nicht. select_eval_robot_examples?');
    continue % Plot leer lassen
  end
  fprintf('Ergebnis-Ordner: %s\n', tmpdir_i);
  % Hole Roboterklasse aus geladenen Daten. Kein Neugenerieren notwendig.
  R = copy(erg.R); % Sonst wird beim Debuggen die Original-Struktur überschrieben
  parroblib_addtopath({R.mdlname});
  trajdatafiles = dir(fullfile(tmpdir_i, '*_Traj*.mat'));
  perfmapfiles = dir(fullfile(tmpdir_i, '*Konfig*TaskRedPerfMap_Data.mat'));
  trajstats = array2table(NaN(length(trajdatafiles),4), 'VariableNames', ...
    {'ConfigNum', 'perfmapfileidx', 'Fval', 'TrajNum'});
  % Trajektorie laden (Eingabe-Trajektorie und EE-Traj. aus Fitness-Eval.)
  importdir = HandlingRobot_dimsynth_data_dir();
  setfile = dir(fullfile(importdir, erg.OptName, '*settings.mat'));
  d1 = load(fullfile(importdir, erg.OptName, setfile(1).name));
  % Zeige Einstellungen zur Beschreibung im Text
  fprintf('IK-Einstellungen: taskred_dynprog_numstates = [%s]\n', ...
    disp_array(d1.Set.general.taskred_dynprog_numstates, '%d'));  
  PHIz_traj = NaN(length(d1.Traj.t), length(trajdatafiles));
  TrajLegendText = cell(1,2);
  for k = 1:length(trajdatafiles)
    tmp = load(fullfile(tmpdir_i, trajdatafiles(k).name));
    [tokens,~] = regexp(trajdatafiles(k).name, 'Konfig(\d)+_', 'tokens', 'match');
    confignum = str2double(tokens{1}{1});
    perfmapfileidx = find(contains({perfmapfiles.name}, sprintf('Konfig%d_', confignum)));
    assert(isscalar(perfmapfileidx), 'Logik-Fehler');
    row_k = array2table([confignum,  perfmapfileidx(end), tmp.fval, tmp.i_ar-1]);
    row_k.Properties.VariableNames = trajstats.Properties.VariableNames;
    trajstats(k,:) = row_k;
    PHIz_traj(:,k) = tmp.PM_phiz_plot(:,end);
    TrajLegendText{k} = sprintf('Traj. %d', k);
  end
  k_iO = trajstats.Fval <= 1e3;
  if ~any(k_iO)
    warning('Keine gültige Trajektorie. select_eval_robot_examples?');
    continue % Plot leer lassen
  end
  k_plot = find(k_iO, 1, 'first');
  % Wähle weitere i.O.-Trajektorien dieser Konfiguration
  k_iOc = k_iO & (trajstats.ConfigNum == trajstats.ConfigNum(k_plot));
  fprintf('%d/%d Trajektorien führen zu erfolgreichem Ergebnis. %d für diese Konfiguration.\n', ...
    sum(k_iO), length(trajdatafiles), sum(k_iOc));
  % Vorbereiten zum Plotten
  i_ax = i_ax + 1;
  % Fülle die Plots zeilenweise auf, nicht spaltenweise
  i_axrow = floor((i_ax-1)/3)+1;
  i_axcol = mod(i_ax-1,3)+1;
  % fprintf('%d: %d, %d\n', i_ax, i_axrow, i_axcol); % Debug.
  axhdl(i_axrow,i_axcol) = subplot(size(axhdl,1), size(axhdl,2), ...
    sprc2no(5,3,i_axrow,i_axcol)); hold on;
  % Redundanzkarte laden (passend zur Nummer der Konfiguration)
  if isempty(perfmapfiles), error('Datei nicht gefunden'); end
  perfmapfile_k = perfmapfiles(trajstats.perfmapfileidx(k_plot)).name;
  dpm = load(fullfile(tmpdir_i, perfmapfile_k));
  
  wn_ik = zeros(R.idx_ik_length.wnpos,1);
  wn_phys = zeros(4,1);
  critnames_withphys = [fields(R.idx_ikpos_wn)', ... % siehe cds_constraints_traj und cds_debug_taskred_perfmap
    {'coll_phys', 'instspc_phys', 'cond_ik_phys', 'cond_phys', ...
    'maxactforce'}];  % zusätzliches Feld
  wn_perfmap = [wn_ik; wn_phys; 0]; % Eintrag für Antriebskraft
  % Klassen-Variablen erweitern, damit physikalische Kriterien auch geplottet
  % werden
  for k = 1:length(critnames_withphys)
    R.idx_ikpos_wn.(critnames_withphys{k}) = k;
  end
  R.idx_ik_length.hnpos = length(wn_perfmap);
  % Zusätzliches Feld auch in Daten der Redundanzkarte einfügen (ans Ende)
  H_all = NaN(size(dpm.H_all,1), size(dpm.H_all,2), length(critnames_withphys));
  for jj = 1:size(dpm.H_all,3)
    H_all(:,:,jj) = dpm.H_all(:,:,jj);
  end
  assert(size(H_all,3)==length(critnames_withphys), 'Dimensionen inkonsistent')

  if strcmp(usr_pm_criterion, 'positionerror')
    wn_perfmap(strcmp(fields(R.idx_ikpos_wn), 'poserr_ee')) = 1;
  elseif strcmp(usr_pm_criterion, 'actforce')
    wn_perfmap(strcmp(critnames_withphys, 'maxactforce')) = 1;
  elseif strcmp(usr_pm_criterion, 'cond_ikjac')
    wn_perfmap(strcmp(critnames_withphys, 'cond_ik_phys')) = 1;
  elseif strcmp(usr_pm_criterion, 'cond_jac')
    wn_perfmap(strcmp(critnames_withphys, 'cond_phys')) = 1;
  else
    error('Fall nicht definiert');
  end
  
  % Marker für Singularität früher einzeichnen (unendlich wird nicht erreicht)
  abort_thresh_h = inf(R.idx_ik_length.hnpos, 1);
  abort_thresh_h(R.idx_ikpos_hn.qlim_hyp) = NaN; % Keine Gelenkgrenzen beachten. Noch nicht plausibel, evtl. Konfigurationssprünge
  % Schwellwerte für Singularitäten so wählen wie in Optimierung
  abort_thresh_h(R.idx_ikpos_hn.jac_cond) = d1.Set.optimization.condition_limit_sing_act;
  abort_thresh_h(R.idx_ikpos_hn.ikjac_cond) = d1.Set.optimization.condition_limit_sing;
  % Marker für Positionsfehler-Grenzüberschreitung auch eintragen.
  % Je nach Heatmap-Farbe ist die Skalierung anders
  if strcmp(usr_pm_criterion, 'positionerror')
    colorscale_poserr = 1e6 / 100; % Skalierung in 100 µm
  else
    colorscale_poserr = 1; % Wert in m aus gespeicherter Redundanzkarte
  end
  abort_thresh_h(R.idx_ikpos_hn.poserr_ee) = d1.Set.optimization.constraint_obj( ...
    strcmp(cdsdef.objconstr_names_all, 'positionerror')) * colorscale_poserr;
  
  if strcmp(usr_pm_criterion, 'positionerror')
    H_poserr = dpm.H_all(:,:,R.idx_ikpos_hn.poserr_ee);
    fprintf('Maximaler Positionsfehler in Redundanzkarte: %1.1fmm\n', 1e3*max(H_poserr(:)));
    colorscale = 1e6 / 100; % Skalierung in 100 µm
    logscale = true;
    % Begrenze den Positionsfehler auf ausprobierten Wert, damit die Farbskala einheitlich ist
    if contains(usr_figselection1, 'revolute')
      colorlimit = [0.25, 2]; % in 100µm
    elseif contains(usr_figselection1, 'prismatic') % Schubantriebe haben ganz anderen Wertebereich. Kann unterschiedlich gewählt werden
      colorlimit = [1, 6]; % in 100µm (bei 500µm ist sowieso Schluss
    end
    I_cutul = H_poserr > colorlimit(2)/colorscale;
    if any(I_cutul(:))
      fprintf('%d Werte oben abgeschnitten\n', sum(I_cutul(:)));
      H_poserr(I_cutul) = colorlimit(2)/colorscale;
    end
    I_cutll = H_poserr < colorlimit(1)/colorscale;
    if any(I_cutll(:))
      fprintf('%d Werte unten abgeschnitten\n', sum(I_cutll(:)));
      H_poserr(I_cutll) = colorlimit(1)/colorscale;
    end
    if all(H_poserr(:)==H_poserr(1))
      warning('Nach Abschneiden des Farbbereichs sind alle Werte gleich');
    end
    H_all(:,:,R.idx_ikpos_hn.poserr_ee) = colorscale*H_poserr;
  elseif strcmp(usr_pm_criterion, 'actforce')
    % Berechne die maximale Antriebskraft neu aus Jacobi-Matrix und
    % externer Kraft (keine Dynamik-Anteile in Prause-Demo)
    % Da die Berechnung sehr lange dauert, wird zusätzlich das Ergebnis
    % gespeichert und bei Neudurchführung wieder geladen
    perfmapfile_k_actforce = strrep(perfmapfile_k, '.mat', '_actforce.mat');
    if exist(fullfile(tmpdir_i, perfmapfile_k_actforce), 'file')
      fileinfo = dir(fullfile(tmpdir_i, perfmapfile_k_actforce));
      if fileinfo.datenum < datenum(2023,8,10,21,50,0)
        fprintf('Datei ist veraltet. Bestimme neu.\n');
        tmp = [];
      else
        tmp = load(fullfile(tmpdir_i, perfmapfile_k_actforce));
        H_maxforce = tmp.H_maxforce;
      end
      % Prüfe, ob kompatibel
      if isempty(tmp)
        H_maxforce = [];
      elseif all(tmp.q0 == dpm.q)
        fprintf('Kräfte für Redundanzkarte erfolgreich aus %s geladen\n', ...
          perfmapfile_k_actforce);
      else
        fprintf('Anfangswerte aus %s nicht kompatibel. Berechne Kräfte neu.\n', ...
          perfmapfile_k_actforce);
        H_maxforce = [];
      end
    else
      H_maxforce = [];
      fprintf('Keine gespeicherten Ergebnisse für Antriebskraft gefunden. Berechne neu.\n');
    end
    if isempty(H_maxforce)
      % Neu berechnen
      H_maxforce = NaN(size(H_all,1), size(H_all,2));
      Traj_0 = cds_transform_traj(R, d1.Traj);
      t0 = tic(); t_lastinfo = t0;
      for ii1 = 1:size(H_all,1) % waagerechte Dimension der Karte
        [~,I_traj] = min(abs(dpm.s_ref(ii1)-dpm.s_tref));
        for ii2 = 1:size(H_all,2) % senkrechte Dimension
          xE_ii = [Traj_0.X(I_traj,1:5)';dpm.phiz_range(ii2)];
          if ~isfield(dpm, 'Q_all')
            [q_ii, Phi_ii] = R.invkin_ser(xE_ii, dpm.q);
          else
            q_ii = dpm.Q_all(ii2,:,ii1)'; % wird in neuer Version ab 8.8.23 gespeichert
            if any(isnan(q_ii)), continue; end % Punkt nicht erreichbar
            % Prüfe ob der Punkt passt
            Phi_ii = R.constr1(q_ii, xE_ii);
            if any(abs(Phi_ii)>1e-6)
              error('Fehler in Zuordnung von q_ii zu xE_ii');
            end
          end
          Ja_ii = R.jacobi_qa_x(q_ii, xE_ii);
          cond_ii = cond(Ja_ii);
          test_cond_abs = abs(cond_ii - H_all(ii1,ii2,strcmp(critnames_withphys, 'cond_phys'))); % siehe perfmap_taskred_ik
          test_cond_rel = test_cond_abs / H_all(ii1,ii2,strcmp(critnames_withphys, 'cond_phys'));
          if test_cond_abs > 1e-3 && test_cond_rel > 1e-2 && cond_ii < 1e6 % bei fast-singulären Posen kann die Abweichung größer sein
            warning(['Konditionszahl der Jacobi-Matrix (%1.1f) entspricht ' ...
              'nicht der gespeicherten (%1.1f). Delta: %1.1e bzw. %1.1fs%%' ...
              ' Index1: %d/%d, Index2: %d/%d'], cond_ii, H_all(ii1,ii2,strcmp(critnames_withphys, 'cond_phys')), ...
              test_cond_abs, test_cond_rel*100, ...
              ii1, size(H_all,1), ii2, size(H_all,2));
          end
          % Berechnung der Antriebsmomente aus externer Kraft, siehe cds_obj_dependencies
          if all(R.I_EE==ones(1,6)), error('TODO: Für 3T3R muss noch die Euler-Umrechnung passieren'); end
          tau_a_max_ii = max(abs(-Ja_ii' \ Traj_0.Fext(I_traj, R.I_EE)'));
          H_maxforce(ii1,ii2) = tau_a_max_ii;
          if toc(t_lastinfo) > 10
            fprintf('Kräfte berechnet: %d/%d. Bereits %1.1fs Rechenzeit\n', ...
              ii1, size(H_all,1), toc(t0));
            t_lastinfo = tic();
          end
        end % for ii2
        % Debug: Vergleiche Trajektorie mit der Redundanzkarte
        q_ii_traj = erg.Q(I_traj,:)';
        x_ii_traj = R.fkineEE_traj(q_ii_traj')'; % TODO: Müsste eigentlich in erg.X stehen. Dort war es aber Stand 10.08.23 noch nicht das korrigierte X
        [~,Phi_ii] = R.constr1(q_ii_traj, x_ii_traj);
        if any(abs(Phi_ii)>1e-6)
          error('ii1=%d. Fehler in Zuordnung von q_ii_traj zu x_ii_traj', ii1);
        end
        test_x = x_ii_traj(1:5) - Traj_0.X(I_traj,1:5)';
        if any(abs(test_x)>1e-6)
          error('ii1=%d. Fehler in Zuordnung von x_ii_traj zu Traj. aus Redundanzkarte', ii1);
        end
        [delta_phi_pm,ii2] = min(abs(angleDiff(x_ii_traj(6), dpm.phiz_range)));
        if abs(delta_phi_pm)>10*pi/180
          error('ii1=%d. Zuordnungsfehler der redundanten Koordinate ist zu groß', ii1);
        end
        xE_ii = [Traj_0.X(I_traj,1:5)';dpm.phiz_range(ii2)];
        % Abgleich der Daten der Redundanzkarte mit der Trajektorie
        % TODO: Besser mit linearer Interpolation, wenn zwischen zwei Punkten
        delta_tau = H_maxforce(ii1, ii2) - max(abs(erg.TAU(I_traj,:)));
        if abs(delta_tau) > 5
          warning('ii1=%d. Unterschied der Antriebskraft zu groß', ii1);
        end
        q_ii = dpm.Q_all(ii2,:,ii1)';
        test_q = q_ii_traj - q_ii;
        if any(abs(test_q(R.MDH.sigma==0)) > 5*pi/180) || any(abs(test_q(R.MDH.sigma==1)) > 5*1e-3)
          warning(['ii1=%d. Gelenkkonfiguration in Traj. unterscheidet ' ...
            'sich stark von Karte: [%s]'], ii1, disp_array(test_q', '%1.3f'));
        end
%         [Jinv_ii, Jinvges_ii] = R.jacobi_qa_x(q_ii, xE_ii);
%         Jinvges_ii_traj = reshape(erg.Jinv_ges(I_traj,:), R.NJ, sum(R.I_EE));
%         Jinv_ii_traj = Jinvges_ii_traj(R.I_qa, :);
%         test_Jinv = Jinv_ii_traj - Jinv_ii;
%         Jinv_ii_traj'\Traj_0.Fext(I_traj, R.I_EE)'
      end % for ii1
      % Ergebnis speichern
      fprintf('Antriebskräfte berechnet. Dauer: %1.1fs\n', toc(t0));
      q0 = dpm.q;
      save(fullfile(tmpdir_i, perfmapfile_k_actforce), 'H_maxforce', 'q0');
    end
    fprintf('Minimale Antriebskraft in Redundanzkarte: %1.1f %s\n', ...
      min(H_maxforce(:)), R.Leg(1).tauunit_sci{R.Leg(1).MDH.mu==2});
    fprintf('Maximale Antriebskraft in Redundanzkarte: %1.1f %s\n', ...
      max(H_maxforce(:)), R.Leg(1).tauunit_sci{R.Leg(1).MDH.mu==2});
    % Plausibilitätsprüfung
    tau_a_traj_minmax = [min(max(abs(erg.TAU),[],2)), max(max(abs(erg.TAU),[],2))];
    fprintf('Max. Antriebskraft in Ergebnis-Trajektorie: min %1.1f, max %1.1f %s\n', ...
      tau_a_traj_minmax(1), tau_a_traj_minmax(2), R.Leg(1).tauunit_sci{R.Leg(1).MDH.mu==2});
    if tau_a_traj_minmax(2) < min(H_maxforce(:))
      error('Größte Antriebskraft der Trajektorie ist kleiner als kleinste der Redundanzkarte. Sehr unplausibel.');
    end
    colorscale = 1; % Skalierung in N/Nm
    logscale = false; % Werte werden nicht übermäßig groß
    % Grenzen müssen hart kodiert werden, damit sie für alle Plots gleich
    % sind
    if contains(usr_figselection1, 'prismatic')
      colorlimit = [5, 20]; % in N
    else % revolute
      colorlimit = [1, 15]; % in Nm
%       colorlimit = [min(H_maxforce(:)), max(H_maxforce(:))];
    end
    I_cutul = H_maxforce > colorlimit(2)/colorscale;
    if any(I_cutul(:))
      fprintf('%d Werte oben abgeschnitten\n', sum(I_cutul(:)));
      H_maxforce(I_cutul) = colorlimit(2)/colorscale;
    end
    I_cutll = H_maxforce < colorlimit(1)/colorscale;
    if any(I_cutll(:))
      fprintf('%d Werte unten abgeschnitten\n', sum(I_cutll(:)));
      H_maxforce(I_cutll) = colorlimit(1)/colorscale;
    end
    H_all(:,:,strcmp(critnames_withphys, 'maxactforce')) = H_maxforce;
  elseif strcmp(usr_pm_criterion, 'cond_jac')
    logscale = true;
    H_cond = dpm.H_all(:,:,strcmp(critnames_withphys, 'cond_phys'));
    fprintf('Wertebereich für Kondition Jacobi: %1.1e...%1.1e\n', ...
      min(H_cond(:)), max(H_cond(:)) );
    colorscale = 1;
    colorlimit = [1, 1e4];
%     colorlimit = [min(H_cond(:)), max(H_cond(:))];
%     warning('todo')
    I_cutul = H_cond > colorlimit(2)/colorscale;
    if any(I_cutul(:))
      fprintf('%d Werte oben abgeschnitten\n', sum(I_cutul(:)));
      H_cond(I_cutul) = colorlimit(2)/colorscale;
    end
    I_cutll = H_cond < colorlimit(1)/colorscale;
    if any(I_cutll(:))
      fprintf('%d Werte unten abgeschnitten\n', sum(I_cutll(:)));
      H_cond(I_cutll) = colorlimit(1)/colorscale;
    end
    H_all(:,:,strcmp(critnames_withphys, 'cond_phys')) = H_cond;
  elseif strcmp(usr_pm_criterion, 'cond_ikjac')
    logscale = true;
    H_ikcond = dpm.H_all(:,:,strcmp(critnames_withphys, 'cond_ik_phys'));
    fprintf('Wertebereich für Kondition IK-Jacobi: %1.1e...%1.1e\n', ...
      min(H_ikcond(:)), max(H_ikcond(:)) );
    colorscale = 1;
    if contains(usr_figselection1, 'revolute')
      colorlimit = [100, 1e4];
    else % wesentlich bessere Konditionszahlen
      colorlimit = [10, 1000];
    end
    I_cutul = H_ikcond > colorlimit(2)/colorscale;
    if any(I_cutul(:))
      fprintf('%d Werte oben abgeschnitten\n', sum(I_cutul(:)));
      H_ikcond(I_cutul) = colorlimit(2)/colorscale;
    end
    I_cutll = H_ikcond < colorlimit(1)/colorscale;
    if any(I_cutll(:))
      fprintf('%d Werte unten abgeschnitten\n', sum(I_cutll(:)));
      H_ikcond(I_cutll) = colorlimit(1)/colorscale;
    end
    H_all(:,:,strcmp(critnames_withphys, 'cond_ik_phys')) = H_ikcond;
%     colorlimit = [min(H_ikcond(:)), max(H_ikcond(:))];
%     warning('todo')
  else
    error('Fall für usr_pm_criterion nicht definiert');
  end
  
  % Ändere von normalisierter Skalierung zu Zeit-Skalierung
  if strcmp(usr_xaxis, 'time')
    [s_tref_unique, I_unique] = unique(dpm.s_tref,'legacy');
    t_s_ref = interp1(s_tref_unique, d1.Traj.t(I_unique), dpm.s_ref, 'nearest');
    s_ref = t_s_ref;
    s_tref = d1.Traj.t;
    pm_refmode = 'time';
  elseif strcmp(usr_xaxis, 'samples')
    [s_tref_unique, I_unique] = unique(dpm.s_tref,'legacy');
    t_s_ref = interp1(s_tref_unique, d1.Traj.t(I_unique), dpm.s_ref, 'nearest');
    % Rechne die Trajektorienzeit in Samples um
    t_s_ref = t_s_ref * length(d1.Traj.t) / d1.Traj.t(end);
    s_ref = t_s_ref;
    s_tref = d1.Traj.t * length(d1.Traj.t) / d1.Traj.t(end);
    pm_refmode = 'samples';
  else
    s_ref = dpm.s_ref;
    s_tref = dpm.s_tref;
    pm_refmode = 'normalized';
  end
  
  % Bestimme Grenzen des Datenbereichs (anhand der Trajektorie)
  ylim1 = 180/pi*minmax2(PHIz_traj(:,k_plot)')+[-45, +45];
  ylim1(ylim1>180) = 185; % Bei großer Verfahrbewegung wird der Ausschnitt zu groß
  ylim1(ylim1<-180) = -185;

  % Abstand der Marker anhand des Plot-Bereichs einstellen
  markermindist = [(max(s_ref)-min(s_ref))/20, diff(ylim1)/15];

  % Redundanzkarte plotten und nachverarbeiten
  [Hdl_all, settings_perfmap_plot, PlotData] = R.perfmap_plot(H_all, dpm.phiz_range, ...
    s_ref, struct( ...
    'log', logscale, ...
    'PM_limit', false, ...
    'abort_thresh_h', abort_thresh_h, ...
    'markermindist', markermindist, ...
    'reference', pm_refmode, 'wn', wn_perfmap));
  colorlimit_all = [colorlimit_all; colorlimit];
  if all(~isnan(colorlimit))
    if Hdl_all.cb.Limits(1) < colorlimit(1) || Hdl_all.cb.Limits(2) > colorlimit(2)
      error(['Plot (%1.2e...%1.2e) übersteigt die manuell gewählte ' ...
        'Farbskalierung (%1.2e...%1.2e)'], Hdl_all.cb.Limits(1), ...
        Hdl_all.cb.Limits(2), colorlimit(1), colorlimit(2));
    end
    clim(colorlimit); % Farb-Skalierung manuell wählen
    if i_ax ~= 1 % nur eine (gemeinsame) Colorbar übrig behalten
      delete(Hdl_all.cb);
    else
      cbhdl = Hdl_all.cb;
    end
  end
  for kk = 1:length(Hdl_all.VM)
    if ~isnan(Hdl_all.VM(kk))
      set(Hdl_all.VM(kk), 'MarkerSize', 5)
    end
  end
  
  % Zeichne Trajektorien ein (die für diese Konfiguration gelten)
  for k = find(k_iOc)'
    trajhdl = plot(s_tref, 180/pi*PHIz_traj(:,k), 'k-', 'LineWidth', 2);
    set(trajhdl, 'DisplayName', 'Traj');
    break % nur die erste zeichnen
  end
  if ~any(k_iOc)
    error('Keine gültige Trajektorie. Darf nicht sein.');
  end
  % Zusätzlich die Grenzen einzeichnen
  for dplim = d1.Set.optimization.ee_rotation_limit_rel_traj
    limhdl = plot(s_tref([1; end]), 180/pi*PHIz_traj(1,find(k_iOc,1,'first')) + ...
      repmat(dplim,2,1)*180/pi, 'k--', 'LineWidth', 0.5);
    set(limhdl, 'DisplayName', 'DP Limit');
  end
  % Plot-Grenzen jetzt erst setzen (werden oben nochmal überschrieben)
  ylim(ylim1); % bereits oben bestimmt wegen Markern
  xlim([0-0.02*max(s_tref), 1.02*max(s_tref)]);

  % Titel setzen (Robotername)
  i_IT = strcmp(InfoTab.GroupName, GroupName);
  if ~any(i_IT)
    error('Keine Daten in Tabelle. select_eval_robot_examples.m neu ausführen.');
  end
  if ~contains(usr_figselection, 'detail') % nur in Plot mit mehreren Subplots relevant
    abcstr = ['(\textbf{', char(96+i_ax), '})\quad '];
    title([abcstr, InfoTab.TextFix{i_IT}], 'interpreter', 'latex');
  end
  % Achsbeschriftung
  if strcmp(usr_xaxis, 'time')
    xlabel('Trajectory time $t$ in s', 'interpreter', 'latex');
  elseif strcmp(usr_xaxis, 'samples')
    xlabel('Trajectory sample', 'interpreter', 'latex');
  else
    xlabel('Normalized trajectory progress $s$', 'interpreter', 'latex');
  end
  if ~contains(usr_figselection, 'detail')
    ylh = ylabel('Red. coord. $\varphi_z$ in deg', 'interpreter', 'latex');
  else % Mehr Platz in Detail-Bild. Ausschreiben.
    ylh = ylabel('Redundant coordinate $\varphi_z$ in deg', 'interpreter', 'latex');
  end
  % MDPI verlangt, dass die Minus-Zeichen En-Dash sind.
  set(axhdl(i_axrow,i_axcol), 'TickLabelInterpreter', 'latex');
  ytickformat('$%g$')
end % for i

testcl = repmat(colorlimit_all(1,:),size(colorlimit_all,1),1)-colorlimit_all;
if any(abs(testcl(:)) ~= 0)
  warning(['Die Grenzen für die Farbskala sind unterschiedlich. Eine ' ...
    'einzige Farb-Legende nicht sinnvoll']);
end
%% Formatiere Gesamtbild
figure_format_publication(fighdl);
remove_inner_labels(axhdl,1);
remove_inner_labels(axhdl,4);
% Beide Bilder sollen gemeinsam auf eine DIN A4 Seite passen
% (Gemeinsame Höhe ca. 21cm)
if contains(usr_figselection, 'prismatic') % 2x3, siehe oben
  set_size_plot_subplot(fighdl, ...
    18.3,9,axhdl,...
    0.06,0.01,0.055,0.09,... %l r u d
    0.05,0.07) % x y
elseif contains(usr_figselection, 'revolute')  % 3x3, siehe oben
  set_size_plot_subplot(fighdl, ...
    18.3,12,axhdl,...
    0.06,0.01,0.04,0.07,... %l r u d
    0.05,0.07) % x y
elseif contains(usr_figselection, 'detail')  % 1x1, einzelnes Bild im Hauptteil
  set_size_plot_subplot(fighdl, ... % einheitlich mit trajectory.m (unten/oben)
    9,6,axhdl,...
    0.12,0.18,0.12,0.13,... %l r u d
    0,0) % x y
else
  error('Fall nicht definiert');
end
% Achsbeschriftungen bündig links
for i = 1:size(axhdl,1)
  if isa(axhdl(i,1), 'matlab.graphics.GraphicsPlaceholder'), continue; end
  ylh = get(axhdl(i,1), 'Ylabel');
  ylhp = get(ylh, 'position');
  [X_off, X_slope] = get_relative_position_in_axes(axhdl(i,1), 'x');
  set(ylh, 'position', [X_off+X_slope*(-1.25), ylhp(2), 0]);
end
if contains(usr_figselection, 'detail')
  % Kompakter: ylabel nach rechts ziehen
  set(ylh, 'Position', [X_off+X_slope*(-1.17), ylhp(2), 0]);
end

% Colorbar zurechtrücken (waagerecht machen auf Platz von fehlendem Subplot)
if ~contains(usr_figselection, 'detail')
  set(cbhdl, 'location', 'north'); % damit wieder senkrecht
end
if ~contains(usr_figselection, 'detail')
  cbyh = ylabel(cbhdl,'TODO', 'Rotation', 0);
else
  cbyh = ylabel(cbhdl,'TODO', 'Rotation', 90); % senkrecht daneben
end
if contains(usr_figselection, 'prismatic')
  line_separator = newline(); % Farblegende ist unten rechts kompakter -> Zeilenumbruch
elseif contains(usr_figselection, 'revolute')
  line_separator = ' ';  % Legende hat doppelte Breite (unten Mitte bis unten rechts)
elseif contains(usr_figselection, 'detail')
  line_separator = newline();
else
  error('Fall nicht definiert');
end
if strcmp(usr_pm_criterion, 'positionerror')
  set(cbhdl, 'Ticks', [10, 20, 50, 100, 200:100:700]/100);
  set(cbyh, 'String', sprintf(['Precision (position error) in 100 µm', line_separator, ...
    '(values beyond the scale are cut off)']));
elseif strcmp(usr_pm_criterion, 'colldist')
  set(cbyh, 'String', sprintf(['Collision distance in mm', ...
    line_separator, '(positive value represents collision)']));
elseif strcmp(usr_pm_criterion, 'actforce')
  if contains(usr_figselection1, 'prismatic')
    set(cbyh, 'String', ['Actuator force in N' ...
      line_separator, '(values beyond the scale are cut off)']);
  else
    set(cbyh, 'String', ['Actuator torque in Nm' ...
      line_separator, '(values beyond the scale are cut off)']);
  end
  if contains(usr_figselection, 'detail')
    set(cbyh, 'String', 'Max. actuator force in N');
  end
  if strcmp(usr_figselection1, 'default_prismatic')
    set(cbhdl, 'Ticks', 5:5:20); % siehe oben bei colorlimit
  else
%     set(cbhdl, 'Ticks', 10:10:50);
    warning('colorticks nicht definiert');
  end
elseif strcmp(usr_pm_criterion, 'cond_jac')
  set(cbhdl, 'Ticks', [1e1, 1e2, 1e3, 1e4, 1e5], 'TickLabels', ...
    {'10', '10^{2}', '10^{3}', '10^{4}', '10^{5}'})
  set(cbyh, 'String', ['Jacobian condition number' ...
    line_separator, '(values beyond the scale are cut off)']);
%   set(cbhdl, 'Limits', [10, 1e4]);
elseif strcmp(usr_pm_criterion, 'cond_ikjac')
  set(cbhdl, 'Ticks', [1e1, 1e2, 1e3, 1e4, 1e5], 'TickLabels', ...
    {'10', '10^{2}', '10^{3}', '10^{4}', '10^{5}'})
  set(cbyh, 'String', ['IK Jacobian condition number' ...
    line_separator, '(values beyond the scale are cut off)']);
%   set(cbhdl, 'Limits', colorlimit);
else
  error('Fall nicht definiert');
end

if contains(usr_figselection, 'prismatic')
%   ahabovepos = get(axhdl(1,3), 'Position');
%   set(cbhdl, 'Position', [ahabovepos(1) 0.0374 ahabovepos(3) 0.0286]);
  set(cbhdl, 'Position', [0.7067 0.0874 0.2734 0.0685]);
elseif contains(usr_figselection, 'revolute') % Beide Bilder haben eine andere Dimension
%   ahabovepos = get(axhdl(1,3), 'Position');
%   set(cbhdl, 'Position', [ahabovepos(1) 0.0374 ahabovepos(3) 0.0286]);
  set(cbhdl, 'Position', [0.5067 0.0286 0.2784 0.0579]);
elseif contains(usr_figselection, 'detail')
  set(cbhdl, 'Position', [0.8448 0.1233 0.0492 0.7255]);
  set(cbyh, 'Position', [2 12 0]);
else
  warning('cbhdl position nicht definiert');
end
% set(cbyh, 'Position', [1.5048 2.8782 0]);

% Legende generieren
legdummyhdl = [];
leglbl = {};
for k = 1:size(settings_perfmap_plot.violation_markers,2)
  if strcmp(settings_perfmap_plot.violation_markers{1,k}, 'qlim_hyp')
    continue % Gelenkgrenzen nicht einzeichnen. Sind noch fraglich
  end
  if contains(usr_figselection, 'detail')
    if strcmp(settings_perfmap_plot.violation_markers{1,k}, 'invalid')
      continue % Gelenkgrenzen nicht einzeichnen. Sind noch fraglich
    end
    if isnan(Hdl_all.VM(k)), continue; end % Legende kompakt halten.
  end
  legdummyhdl = [legdummyhdl; plot(NaN, NaN, settings_perfmap_plot.violation_markers{2,k})];
  leglbl = [leglbl; settings_perfmap_plot.violation_markers{1,k}];
end
leglbl(strcmp(leglbl,'ikjac_cond')) = {'singularity type I'};
if ~contains(usr_figselection, 'detail')
  leglbl(strcmp(leglbl,'jac_cond')) = {'singularity type II'};
elseif strcmp(usr_figselection, 'detail_P4RPRRR8V2G')
  leglbl(strcmp(leglbl,'jac_cond')) = {'singularity'}; % etwas kürzer
else
  leglbl(strcmp(leglbl,'jac_cond')) = {'sing.'}; % kompakter
end
leglbl(strcmp(leglbl,'qlim_hyp')) = {'joint limits'};
if ~contains(usr_figselection, 'detail') || strcmp(usr_figselection, 'detail_P4RPRRR8V2G')
  leglbl(strcmp(leglbl,'coll_hyp')) = {'collision'};
else
  leglbl(strcmp(leglbl,'coll_hyp')) = {'coll.'}; % kompakter
end

leglbl(strcmp(leglbl,'instspc_hyp')) = {'installation space'};
leglbl(strcmp(leglbl,'poserr_ee')) = {'low precision'};
leglbl(strcmp(leglbl,'invalid')) = {'out of range'};

% Trajektorie und Grenzen einzeichnen
legdummyhdl = [legdummyhdl; plot(NaN, NaN, 'k-')];
leglbl = [leglbl; {'trajectory'}];
if ~contains(usr_figselection, 'detail')
  % Nicht in 1x1-Detailbild die Grenzen einzeichnen. Treten im betrachteten
  % Fall nicht auf.
  legdummyhdl = [legdummyhdl; plot(NaN, NaN, 'k--')];
  leglbl = [leglbl; {'DP limits'}];
end
if contains(usr_figselection, 'prismatic')
  legref = axhdl(1,3);
elseif contains(usr_figselection, 'revolute')
  legref = axhdl(2,2); % Bild ist 3x3. Legende ist in Element 3,2
elseif contains(usr_figselection, 'detail')
  % Bezug der Legende auf Figure
else
  error('Fall nicht definiert');
end
if contains(usr_figselection, 'revolute')
  if exist('lfhdl', 'var'), delete(lfhdl); end
  lfhdl = legendflex(legdummyhdl, leglbl, 'anchor', {'se','n'}, ...
    'ref', legref, ... % an anderem Subplot ausrichten
    'buffer', [0 -35], ...
    'ncol', 2, 'nrow', 0, ... % eine Zeile für Legende
    'fontsize', 9, ...
    'xscale', 1, ... % Kleine Symbole
    'padding', [1,2,3], ... % Leerraum reduzieren
    'box', 'on');
elseif contains(usr_figselection, 'prismatic')
  % Bei prismatic-Bild ist weniger Platz. Da beide Bilder auf der gleichen
  % Seite sind, kann die Marker-Legende hier weggelassen werden.
elseif contains(usr_figselection, 'detail')
  if exist('lfhdl', 'var'), delete(lfhdl); end
  lfhdl = legendflex(legdummyhdl, leglbl, 'anchor', {'n','n'}, ...
    'ref', fighdl, ... % an Figure ausrichten (mitten oben)
    'buffer', [0 -1], ... % Kein Versatz notwendig, da mittig oben
    'ncol', 0, 'nrow', 1, ... % eine Zeile für Legende
    'fontsize', 9, ...
    'xscale', 0.7, ... % Kleine Symbole
    'padding', [-3,0,2], ... % Leerraum reduzieren
    'box', 'on');
else
  error('Fall nicht definiert');
end
% Schriftart der Legende aktualisieren
figure_format_publication(fighdl);
drawnow();
t1 = tic();
if usr_save_fig % Debug: Dauert zu lange
  if length(axhdl(:))>1
    multistring = '_multi';
  else
    multistring = '';
  end
  name = ['handlingpkm_perfmaps_', usr_figselection, multistring, '_' ...
    usr_pm_criterion, '_', usr_objective_ik];
  exportgraphics(fighdl, fullfile(outputdir, [name, '.pdf']), 'ContentType','vector');
  saveas(fighdl, fullfile(outputdir, [name, '.fig']));
  fprintf('Exported performance map as vector graphics: %s. Duration: %1.1fs\n', name, toc(t1));
end
end % i_cases
end % i_usr_fig
