% Vergleiche das IK-Ergebnis für verschiedene IK-Zielkriterien
% Zeichne dazu die Redundanzkarte mit verschiedenen Trajektorien
% 
% Siehe auch: cds_taskred_eval_example.m (muss vorher ausgeführt werden)
% Erzeugt Fig. 5.20a in Diss.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
usr_cluster = false; % Zum Umschalten der Pfade auf dem Cluster
% Auswahl des zu analysierenden 
% In der Diss-Version bis zum 1.11.2024 stand hier
% amunpkm_20230605_3T3R_obj3_colldist_ikobj2_rep1.
% Das war nicht konsistent mit cds_taskred_eval_example_diss_figure.m.
% Dort (rechter Teil von Fig. 5.20) wird ...colldist_ikobj2_rep2 benutzt.
OptName = 'ARK_3T1R_20230730_full_rep1';
usr_LfdNr = 230;
usr_xaxis = 'time';
usr_pm_criterion = 'positionerror';
usr_select_pnr = 1; % Pareto-Nummer auswählen
usr_debug_plot = false;
% Dateiendung für den Fall, dass Dynamische Programmierung für IK benutzt
% wird. Die Ergebnisse sind dann nicht direkt vergleichbar
dpsuffix = '_dp';

taskredeval_dir = fileparts(which('cds_taskred_eval_check_case.m'));
% outputdir = fullfile(taskredeval_dir, 'results');
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');


%% Pfade initialisieren
[resdir, datadir] = cds_taskred_eval_path_config(usr_cluster);

%% Prüfe, welche Roboter verfügbar sind
restabfiles_rob = dir(fullfile(resdir, OptName, 'taskred_eval_example_result_rob*.csv'));
if isempty(restabfiles_rob)
  warning('Keine passenden Dateien gefunden');
end
for ii_robfile = 1%:length(restabfiles_rob)

[tokens, match] = regexp(restabfiles_rob(ii_robfile).name, 'rob(\d+)', 'tokens', 'match');
LfdNr = str2double(tokens{1}{1});
if ~isempty(usr_LfdNr) && LfdNr ~= usr_LfdNr
   continue;
end
fprintf('Untersuche Roboter %d (Datei %d/%d)\n', LfdNr, ii_robfile, length(restabfiles_rob));

% Tabelle laden
restabfile_rob = fullfile(resdir, OptName, sprintf(['taskred_eval_example_' ...
  'result_rob%d%s.csv'], LfdNr, '_dp'));
StatsTab_Rob = readtable(restabfile_rob);

% Definiere die Namen der IK-Optimierungskriterien
% Muss konsistent mit anderem Skript sein.
defstruct = cds_definitions();
objective_ik_cases = defstruct.objective_ik_names_all;
defstruct2 = cds_taskred_eval_definitions();
objective_ik_cases_leg = defstruct2.objective_ik_cases_leg;
n_meth = length(objective_ik_cases);
%% Ergebnisse analysieren
ident_matrix = false(n_meth, n_meth);
if ~isempty(usr_select_pnr)
  II_pareto = usr_select_pnr;
else
  II_pareto = 1:size(StatsTab_Rob, 1);
end
fprintf('Gehe über %d/%d Pareto-Partikel\n', length(II_pareto), size(StatsTab_Rob, 1));
for ii_pareto = II_pareto(:)'
PNr = ii_pareto;
Q_all = cell(n_meth,1);
for i_set = 1:n_meth
  resfile_iset = fullfile(resdir, OptName, sprintf(['taskred_eval_example_' ...
    'result_rob%d_par%d_set%d%s.mat'],LfdNr, ii_pareto, i_set, dpsuffix));
  resfile_iset2 = strrep(resfile_iset, '.mat', '_short.mat');
  if ~exist(resfile_iset, 'file')
    continue;
  end
  tmp = load(resfile_iset);
  Q_all{i_set} = tmp.Q;
end
for i = 1:n_meth
  if isempty(Q_all{i}), continue; end % Daten liegen nicht vor.
  for j = 1:n_meth
    if isempty(Q_all{j}), continue; end % Daten liegen nicht vor.
    if i == j, continue; end
    test_Q = Q_all{i} - Q_all{j};
    if any(test_Q(:))
      % fprintf('Partikel %d: Fall %d und %d sind gleich\n', PNr, i, j);
    else
      ident_matrix(i,j) = false;
    end
  end
end
end
%% Zeichne Redundanzkarte mit verschiedenen Trajektorien
% Für jedes Partikel eine Karte. Die Karten sehen unterschiedlich aus,
% da die Kinematik-Parameter anders sind.
for ii_pareto = II_pareto
PNr = ii_pareto;
RobName = StatsTab_Rob.RobName{1};
% Initialisiere Roboter und Einstellungen der Opt.
settingsfile = fullfile(datadir, OptName, [OptName,'_settings.mat']);
d1 = load(settingsfile);
Set = cds_settings_update(d1.Set, 1);
Set.general.save_robot_details_plot_fitness_file_extensions = {};
[R, Structure] = cds_dimsynth_robot(Set, d1.Traj, d1.Structures{LfdNr}, true);
critnames = fields(R.idx_ikpos_wn)';

% Lade Redundanzkarte für alle Fälle und prüfe, ob sie gleich sind. Wenn
% nicht, liegt eine andere IK-Konfiguration vor
dpm = struct();
PM_available_list = false(n_meth,1);
for i_set = 0:n_meth
  pmfile = fullfile(resdir, OptName, sprintf('%s_rob%d_par%d_set%d%s', ...
    OptName, LfdNr, PNr, i_set, dpsuffix), ...
    'tmp', sprintf('%d_%s', LfdNr, RobName), 'Gen-1_Ind-1_Konfig1_TaskRedPerfMap_Data.mat');
  if ~exist(pmfile, 'file')
    fprintf('Datei existiert nicht: %s\n', pmfile);
    continue; % es könnten weitere Dateien noch existieren
  end
  dpm = load(pmfile);
  if i_set == 0
    dpm2 = dpm;
  else
    % Vergleiche mit vorher gespeichertem
    H_diff = dpm2.H_all - dpm.H_all;
    fn = fieldnames(R.idx_ikpos_hn)';
    for kk = 1:length(fn)
      test_kk = H_diff(:,:,kk);
      if any(abs(test_kk(:)) > 1e-6)
        fprintf(['Redundanzkarte unterschiedlich zwischen Einstellung ', ...
          '%d und %d für Kriterium %d/%d "%s". Max. Abw.: %1.1e\n'], ...
          i, i_set, kk, length(fn), fn{kk}, max(abs(test_kk(:))));
        if i_set == 2 && strcmp(fn{kk}, 'ikjac_cond') || ...
           i_set == 3 && strcmp(fn{kk}, 'jac_cond')
           fprintf('Unterschied ist erwartbar, da Schwellwert für Jacobi-Aktivierung geändert\n');
           continue
        end
        % Debug; Zeichne beide Redudanzkarten
        continue
        for jj = 1:2
          if jj == 1, d_jj = dpm2; else, d_jj = dpm; end
          % Fragliches Kriterium aktivieren zum Zeichnen
          wn = zeros(R.idx_ik_length.wnpos,1);
          wn(R.idx_ikpos_wn.(fn{kk})) = 1;
          cds_debug_taskred_perfmap(Set, d_jj.Structure, d_jj.H_all, d_jj.s_ref, d_jj.s_tref(1:d_jj.nt_red), ...
            d_jj.phiz_range, NaN(d_jj.nt_red,0), NaN(d_jj.nt_red,0), struct('wn', wn, ...
            'i_ar', jj, 'name_prefix_ardbg', '', 'fval', 0, ...
            'TrajLegendText', {{}},  'ignore_h0', false, ...
            'deactivate_time_figure', true, ... % Bild nicht wirklich brauchbar
            'critnames', {critnames}, 'constrvioltext', sprintf('Alternative %d', jj)));
        end
      else
        fprintf(['Redundanzkarte identisch zwischen Einstellung ', ...
          '%d und %d für Kriterium %d/%d "%s".\n'], ...
          i, i_set, kk, length(fn), fn{kk});
      end
    end
    fprintf('Konsistenz der Redundanzkarte für Methode %d gegen Methode 1 geprüft\n', i_set);
    if i_set>0, PM_available_list(i_set) = true; end
  end
end
if isempty(dpm) || isempty(fields(dpm))
  fprintf('Unvollständige Daten. Überspringe Partikel\n');
  continue
end
% Weitere Daten vorbereiten
wn = zeros(R.idx_ik_length.wnpos,1);
dpm.H_all(:,:,R.idx_ikpos_hn.qlim_hyp) = NaN; % Gelenkwinkel-Grenzen in Plot ignorieren
% Lade Liste deaktivierter Trajektorien aus Datei zu anderem Bild
% Wird erzeugt von cds_taskred_eval_example_diss_figure.m
% tmp = load(fullfile(outputdir, sprintf('amunpkm_pareto_compare_ik_obj_from%d_p%d_info.mat', 2, usr_select_pnr)));

% Auswahl entsprechend der Plot-Nummer (füge die Linien nach und nach hinzu)
I_sel = false(length(objective_ik_cases),1);
% Nehme nicht alle Linien, die möglich sind. Das überfrachtet den Vortrag
I_sel(strcmp(objective_ik_cases, 'constant')) = true;
I_sel(strcmp(objective_ik_cases, 'poserr_ee')) = true;
I_sel(strcmp(objective_ik_cases, 'maxactvelo')) = true;
I_sel(strcmp(objective_ik_cases, 'maxactforce')) = true;
I_sel(strcmp(objective_ik_cases, 'default')) = true;


% Trajektorien vorbereiten zum Einzeichnen in Bild
TrajLegendText = {};
PM_phiz_plot = [];
Line_valid = true(n_meth+1, 1);
for ii = find(I_sel)'
  % Plotte nicht alle Linien. Soll konsistent mit Diss-Bild für
  % Pareto-Diagramm hierzu sein.
  if ii <= n_meth
    i_set = ii;
  else
    i_set = 0; % Reproduktion des Ergebnisses aus der Optimierung
  end
  resfile_iset = fullfile(resdir, OptName, sprintf(['taskred_eval_example_' ...
    'result_rob%d_par%d_set%d%s.mat'],LfdNr, ii_pareto, i_set, dpsuffix));
  resfile_iset2 = strrep(resfile_iset, '.mat', '_short.mat');
  resfile_iset_info = dir(resfile_iset);
  resfile_iset2_info = dir(resfile_iset2);
  if ~isempty(resfile_iset_info) && ~isempty(resfile_iset2_info) && ...
      resfile_iset2_info.datenum > resfile_iset_info.datenum + 1
    warning('Datei %s (%s) ist wesentlich älter als %s (%s). Ignoriere sie.', ...
      resfile_iset_info.name, resfile_iset_info.date, ...
      resfile_iset2_info.name, resfile_iset2_info.date);
    resfile_iset = '';
  end
  if ~exist(resfile_iset, 'file') && exist(resfile_iset2, 'file')
    tmp2 = load(resfile_iset2);
    Phiz_Traj = tmp2.Phiz_traj;
  elseif exist(resfile_iset, 'file')
    tmp2 = load(resfile_iset);
    Phiz_Traj = tmp2.X6Traj(:,1);
  else
    error('Datei %s existiert nicht', resfile_iset)
  end
  
  if strcmp(objective_ik_cases{i_set}, 'constant')
    if any(abs(Phiz_Traj(1,1)-Phiz_Traj(:))>1e-6)
      error('Geladene Trajektorie für Fall "constant" ist nicht konstant');
    end
  end
  PM_phiz_plot = [PM_phiz_plot, Phiz_Traj]; %#ok<AGROW>
  if i_set == 0
    legtext_i = sprintf('Original trajectory');
  else
    legtext_i = sprintf('%s', objective_ik_cases_leg{i_set});
  end
  if any(tmp2.fval_i > 1e3)
    legtext_i = [legtext_i, ' (invalid)']; %#ok<AGROW>
    fprintf('Linie %s ist ungültig (kein Fehler)\n', legtext_i);
    Line_valid(i_set) = false;
  end
  TrajLegendText = [TrajLegendText, legtext_i]; %#ok<AGROW>
  if ~PM_available_list(ii)
    warning(['Linie %d (%s) eingezeichnet, obwohl keine Redundanzkarte vorlag. ' ...
      'Dadurch nicht 100%% sicher, ob die Konfiguration gleich war.'], ...
      i_set, objective_ik_cases{i_set});
  end
end
% Bild zeichnen (Benutze Redundanzkarten-Plotfunktion)
fighdl = change_current_figure(2); clf; hold on;
axhdl = subplot(1,1,1);
% Ändere von normalisierter Skalierung zu Zeit-Skalierung
if strcmp(usr_xaxis, 'time')
  [s_tref_unique, I_unique] = unique(dpm.s_tref,'legacy');
  t_s_ref = interp1(s_tref_unique, d1.Traj.t(I_unique), dpm.s_ref, 'nearest');
  s_ref = t_s_ref;
  s_tref = d1.Traj.t;
  pm_refmode = 'time';
else
  s_ref = dpm.s_ref;
  s_tref = dpm.s_tref;
  pm_refmode = 'normalized';
end

wn_ik = zeros(R.idx_ik_length.wnpos,1);
wn_phys = zeros(4,1);
critnames_withphys = [fields(R.idx_ikpos_wn)', ... % siehe cds_constraints_traj und cds_debug_taskred_perfmap
  {'coll_phys', 'instspc_phys', 'cond_ik_phys', 'cond_phys'}];
wn_perfmap = [wn_ik; wn_phys];
% Klassen-Variablen erweitern, damit physikalische Kriterien auch geplottet
% werden
for k = 1:length(critnames_withphys)
  R.idx_ikpos_wn.(critnames_withphys{k}) = k;
end
R.idx_ik_length.hnpos = length(wn_perfmap);
if strcmp(usr_pm_criterion, 'positionerror')
  wn_perfmap(strcmp(fields(R.idx_ikpos_wn), 'poserr_ee')) = 1;
else
  warning('Fall nicht definiert');
end
% Gleiche Einstellungen wie für andere Redundanzkarten in Diss
% Marker für Singularität früher einzeichnen (unendlich wird nicht erreicht)
abort_thresh_h = inf(R.idx_ik_length.hnpos, 1);
% abort_thresh_h(R.idx_ikpos_hn.qlim_hyp) = NaN; % Keine Gelenkgrenzen beachten? Doch.
abort_thresh_h(R.idx_ikpos_hn.jac_cond) = 5e3;
abort_thresh_h(R.idx_ikpos_hn.ikjac_cond) = 5e3;

if strcmp(usr_pm_criterion, 'positionerror')
  H_poserr = dpm.H_all(:,:,R.idx_ikpos_hn.poserr_ee);
  fprintf('Maximaler Positionsfehler in Redundanzkarte: %1.1fmm\n', 1e3*max(H_poserr(:)));
  colorscale_poserr = 1e6 / 100; % Skalierung in 100 µm
  logscale = true;
  % Begrenze den Positionsfehler damit die Farbskala einheitlich ist
  % konsistent mit manuscript/05_evaluation/figures/handlingpkm_perfmaps.m
  colorlimit = [1, 6]; % in 100µm (bei 500µm ist sowieso Schluss)
  I_cutul = H_poserr > colorlimit(2)/colorscale_poserr;
  if any(I_cutul(:))
    fprintf('%d Werte (von %d) oben abgeschnitten\n', sum(I_cutul(:)), numel(H_poserr));
    H_poserr(I_cutul) = colorlimit(2)/colorscale_poserr;
  end
  I_cutll = H_poserr < colorlimit(1)/colorscale_poserr;
  if any(I_cutll(:))
    fprintf('%d Werte (von %d) unten abgeschnitten\n', sum(I_cutll(:)), numel(H_poserr));
    H_poserr(I_cutll) = colorlimit(1)/colorscale_poserr;
  end
  dpm.H_all(:,:,R.idx_ikpos_hn.poserr_ee) = colorscale_poserr*H_poserr;
else
  colorscale_poserr = 1; % Wert in m aus gespeicherter Redundanzkarte
end
% Anzeige von Markern für Positionsverletzung
abort_thresh_h(R.idx_ikpos_hn.poserr_ee) = d1.Set.optimization.constraint_obj( ...
  strcmp(defstruct.objconstr_names_all, 'positionerror')) * colorscale_poserr;

% ylim1 = 180/pi*minmax2(PM_phiz_plot(:)')+[-40, +80]; % über oberen Bereich wird die Legende gelegt
ylim1 = [17, 93];
markermindist = [(max(s_ref)-min(s_ref))/30, diff(ylim1)/20];
[Hdl_all, settings_perfmap_plot, PlotData] = R.perfmap_plot(dpm.H_all, dpm.phiz_range, ...
  s_ref, struct( ...
  'log', logscale, ...
  'abort_thresh_h', abort_thresh_h, ...
  'markermindist', markermindist, ...
  'reference', pm_refmode, 'wn', wn_perfmap));

defstruct2 = cds_taskred_eval_definitions();
format = defstruct2.format;
% Trajektorien einzeichnen und danach formatieren
linhdl = NaN(size(PM_phiz_plot,2),1);
for k = 1:size(PM_phiz_plot,2)
  linhdl(k) = plot(s_tref, 180/pi*PM_phiz_plot(:,k));
end
leglinhdl = line_format_publication(linhdl, format(I_sel,:), TrajLegendText);

drawnow()

ylim(ylim1);

if strcmp(usr_xaxis, 'time')
  xlabel('Trajectory time $t$ in s', 'interpreter', 'latex');
else
  xlabel('Normalized trajectory progress $s$', 'interpreter', 'latex');
end
ylh = ylabel('Redundant coordinate $\varphi_z$ in deg', 'interpreter', 'latex');
cbhdl = Hdl_all.cb;
cbyh = ylabel(cbhdl,'TODO', 'Rotation', 90);
if strcmp(usr_pm_criterion, 'positionerror')
  set(cbhdl, 'Limits', [1,6]);
  set(cbhdl, 'Ticks', [50 100, 200:100:700]/100);
  set(cbyh, 'String', 'Precision (position error) in 100 µm');
elseif strcmp(usr_pm_criterion, 'colldist')
  set(cbyh, 'String', 'Collision distance in mm');
elseif strcmp(usr_pm_criterion, 'cond_jac')
  set(cbhdl, 'Ticks', [1e1, 1e2, 1e3, 1e4, 1e5], 'TickLabels', ...
    {'10', '10^{2}', '10^{3}', '10^{4}', '10^{5}'})
  set(cbyh, 'String', 'Jacobian condition number');
  set(cbhdl, 'Limits', [100, 1e5]);
else
  error('Fall nicht definiert');
end

figure_format_publication(fighdl);

% Legende eintragen
legdummyhdl = Hdl_all.VM;
leglbl = settings_perfmap_plot.violation_markers(1,:);

leglbl(strcmp(leglbl,'ikjac_cond')) = {'sing. type I'};
leglbl(strcmp(leglbl,'jac_cond')) = {'singularity'};
leglbl(strcmp(leglbl,'qlim_hyp')) = {'joint limits'};
leglbl(strcmp(leglbl,'coll_hyp')) = {'collision'};
leglbl(strcmp(leglbl,'instspc_hyp')) = {'installation space'};
leglbl(strcmp(leglbl,'poserr_ee')) = {'low precision'};
leglbl(strcmp(leglbl,'invalid')) = {'out of range'};
Ileg = ~isnan(legdummyhdl);
Ileg = Ileg & ~ strcmp(leglbl','out of range'); % diesen nicht zeichnen. Im sichtbaren Bereich nicht drin.
if exist('lfhdl', 'var'), delete(lfhdl); end
lfhdl = legendflex(legdummyhdl(Ileg), leglbl(Ileg), 'anchor', {'n','n'}, ...
  'ref', fighdl, ...
  'buffer', [1 -2], ... % Kein Versatz notwendig, da mittig oben
  'ncol', 0, 'nrow', 1, ... % eine Zeile für Legende
  'fontsize', 9, ...% Sollte konsistent mit figure_format_publication sein (eins größer geht aber auch)
  'FontName', 'Times New Roman', ...
  'xscale', 0.8, ... % Kleine Symbole
  'padding', [0,0,3], ... % Leerraum reduzieren
  'box', 'on');

% Legende für Trajektorien. Nur zum Debuggen. In 1x2-Bild ist die Legende
% im anderen Bild schon enthalten.
if usr_debug_plot
  if exist('lfhdl2', 'var'), delete(lfhdl2); end
  lfhdl2 = legendflex(leglinhdl, TrajLegendText, 'anchor', {'w','w'}, ...
    'ref', fighdl, ...
    ...'buffer', [1 -2], ... % Kein Versatz notwendig, da mittig oben
    'ncol', 1, 'nrow', 0, ... % eine Zeile für Legende
    'fontsize', 9, ...% Sollte konsistent mit figure_format_publication sein (eins größer geht aber auch)
    'FontName', 'Times New Roman', ...
    'xscale', 0.8, ... % Kleine Symbole
    'padding', [0,0,2], ... % Leerraum reduzieren
    'box', 'on');
end

% Speichere das Bild im Diss-Format (halbe Seitenbreite)
set_size_plot_subplot(fighdl, ... % einheitlich mit trajectory.m (unten/oben)
  10.5,6,axhdl,...
  0.07,0.13,0.03,0.13,... %l r u d (u/d konsistent mit cds_taskred_eval_example_diss_figure.m)
  0,0) % x y

% Angleichen an axhdl Position. Etwas kompakter und gleich hoch.
% Muss nach letzter Aktualisierung der Plot-Größe gemacht werden
axpos = get(axhdl, 'Position');
set(cbhdl, 'Position', [0.88 axpos(2) 0.04 axpos(4)]);
set(cbyh, 'Position', [2.0 2.5 0]);

ylhp = get(ylh, 'Position');
[~, ~, A_pos] = get_relative_position_in_axes(axhdl, 'Y', -0.1);
set(ylh, 'Position', [ylhp(1), A_pos, 0]);

ZOrderSet(lfhdl, 1); % Legende in Vordergrund bringen
% Dateiname: "from2" sollte konsistent zu Optimierung oben sein.
figname = sprintf('handlingpkm_perfmap_compare_ik_obj_%s_p%d', StatsTab_Rob.RobName{1}, PNr);
saveas(fighdl,     fullfile(outputdir, [figname,'.fig']));
exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', figname)),'ContentType','vector');
fprintf('Bild gespeichert: %s. Pfad: %s\n', figname, outputdir);
end % ii_pareto
end % ii_robfile
