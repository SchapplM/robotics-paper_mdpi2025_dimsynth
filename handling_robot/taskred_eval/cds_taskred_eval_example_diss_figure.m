% Erzeuge ein Bild für die Dissertation mit der Variation der Pareto-Front
% Erzeugt Fig. 5.20b in Diss.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
usr_cluster = false; % Zum Umschalten der Pfade auf dem Cluster
% Zur Erstellung der Bilder im Paper muss usr_plotnum mit 1 und 2 ausgeführt werden
usr_plotnum = 2;
OptName = 'ARK_3T1R_20230730_full_rep1'; % ARK_3T1R_20230730_full_rep2';
LfdNr = 230; %230;% 231;
RobName = 'P4RPRRR8V2G3P1A1'; % 'P4RPRRR8V2G3P1A1';

[resdir, datadir] = cds_taskred_eval_path_config(usr_cluster);
usr_plot_debug = false;
% Trage Nummer des verbundenen Subplots ein, in dem die Redundanzkarte
% gezeigt wird
usr_select_pnr = 1; % konsistent mit anderem Skript (eval_check_case)

defstruct = cds_definitions();
objective_ik_cases = defstruct.objective_ik_names_all;
defstruct2 = cds_taskred_eval_definitions();

imescolors = imes_plot_template_colors();

taskredeval_dir = fileparts(which('cds_taskred_eval_check_case.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');

%% Ergebnisse laden
dpsuffix = '_dp';
restabfile_rob = fullfile(resdir, OptName, ...
  sprintf('taskred_eval_example_result_rob%d%s.csv', LfdNr, dpsuffix));
if ~exist(restabfile_rob, 'file')
  error('Datei %s existiert nicht', restabfile_rob);
end
StatsTab_Rob = readtable(restabfile_rob);

%% Pareto-Diagramm für einzelnen Roboter
% Daten laden
settingsfile = fullfile(datadir, OptName, [OptName,'_settings.mat']);
if ~exist(settingsfile, 'file'), errpr('Einstellungsdatei fehlt'); end
d1 = load(settingsfile);
Set = cds_settings_update(d1.Set, 1);
resfile = fullfile(datadir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
if ~exist(resfile, 'file')
  error('Ergebnis-Datei existiert nicht: %s', resfile);
end
tmp = load(resfile);
RobotOptRes_j = tmp.RobotOptRes;

% Zeichne Pareto-Diagramm für diesen Roboter mit den Ergebnissen der
% verschiedenen Durchläufe.
StatsTab_RobPlot = StatsTab_Rob;
% Prüfe Integrität der Tabelle
assert(length(unique(StatsTab_RobPlot.ParetoNr)) == length(StatsTab_RobPlot.ParetoNr), 'Tabelle ist nicht eindeutig');
[~,I_sort] = sort(StatsTab_RobPlot.ParetoNr);
if ~all(StatsTab_RobPlot.ParetoNr(I_sort) == StatsTab_RobPlot.ParetoNr)
  warning('Tabelle ist nicht fortlaufend');
  StatsTab_RobPlot = StatsTab_RobPlot(I_sort,:);
end

% Zusammenhang zwischen Zielfunktion der Optimierung und der dieser
% Auswertung
if usr_plotnum == 1
  obj_cases_plot = {'actforce', 'positionerror'};
elseif usr_plotnum == 2
  obj_cases_plot = {'actforce', 'installspace'}; % Auch Bild generieren um zu zeigen, dass die Auswirkung auch gering sein kann.
end

kk1 = find(strcmp(obj_cases_plot{1}, Set.optimization.objective));
kk2 = find(strcmp(obj_cases_plot{2}, Set.optimization.objective));
% Bild vorbereiten
fighdl = change_current_figure(1); clf; hold on;
axhdl = subplot(1,1,1);
set(fighdl, 'name', sprintf('%s_Rob%d_eval', OptName, LfdNr), 'NumberTitle', 'off');
Set_plot = Set;
Set_plot.optimization.objective = obj_cases_plot;
[obj_units, objscale] = cds_objective_plotdetails(Set_plot, {RobotOptRes_j.Structure});
% obj_units{strcmp(Set_plot.optimization.objective ,'actvelo')} = 'deg/s';
% objscale(strcmp(Set_plot.optimization.objective ,'actvelo')) = 180/pi;
if any(strcmp(Set_plot.optimization.objective ,'positionerror')) % in 100µm anzeigen
  objscale(strcmp(Set_plot.optimization.objective ,'positionerror')) = 1/(100*1e-6);
  obj_units{strcmp(Set_plot.optimization.objective ,'positionerror')} = '100µm';
end
xlh = xlabel(sprintf('%s in %s', obj_cases_plot{1}, obj_units{1}));
ylabel(sprintf('%s in %s', obj_cases_plot{2}, obj_units{2}));
I_ikorig = find(strcmp(objective_ik_cases, Set.optimization.objective_ik));

n_iO = NaN(1, length(objective_ik_cases));
% Daten zusammentragen
I = StatsTab_RobPlot.ParetoNr(:)';
pareto_data = NaN(length(I), length(obj_cases_plot), length(objective_ik_cases)+1);
for i_set = 1:length(objective_ik_cases)+1
  if i_set <= length(objective_ik_cases)
    % Nehme die Daten aus der Neuberechnung
    for ii = 1:length(obj_cases_plot)
      Col_ii = StatsTab_RobPlot.(sprintf('physval_%s_ik_%s', obj_cases_plot{ii}, ...
          objective_ik_cases{i_set}));
      pareto_data(:,ii, i_set) = Col_ii(:);
    end
  else
    % Nehme die Daten der ursprünglichen Optimierung
    if isempty(kk1) || isempty(kk2)
      plot(NaN,NaN, ...
        defstruct2.format{i_set,2}, 'Color', defstruct2.format{i_set,1});
      continue;
    end
    pareto_data(:,:, i_set) = RobotOptRes_j.physval_pareto(I, [kk1,kk2]);
  end
end

linhdl = NaN(size(pareto_data,3),1); % Für Legende
% Erzeuge Pareto-Front mit dominierenden Partikeln
% für Ergebnisse der ursprünglichen Optimierung.
if ~isempty(kk1) && ~isempty(kk2)
  Idom = ~pareto_dominance(pareto_data(:,:, end));
  linhdl(end) = plot(objscale(1)*pareto_data(Idom,1,end), objscale(2)*pareto_data(Idom,2,end)); % , [styles{end},lines{end}]);
  set(linhdl(end), 'color', imescolors.orange);
  set(linhdl(end), 'LineWidth', 2);
  % Auch normale Partikel plotten (nur die fehlenden, nicht dominierenden)
  if usr_plot_debug
    plot(objscale(1)*pareto_data(~Idom,1,end), objscale(2)*pareto_data(~Idom,2,end), 'd'); % styles{end});
  end
end
% Wähle die zu zeichnenden Fälle aus (konsistent mit ...check_case.m)
I_ikselect = NaN(1, length(objective_ik_cases));
I_ikselect(strcmp(objective_ik_cases, 'constant')) = 1;
I_ikselect(strcmp(objective_ik_cases, 'poserr_ee')) = 1;
I_ikselect(strcmp(objective_ik_cases, 'maxactvelo')) = 1;
I_ikselect(strcmp(objective_ik_cases, 'maxactforce')) = 1;
I_ikselect(strcmp(objective_ik_cases, 'default')) = 1;
I_ikselect = find(~isnan(I_ikselect));


Idomorig = Idom; % Dominierende Partikel (2D) der ursprünglichen Opt.
% Daten plotten (neu berechnete Partikel)
pareto_data_dom_all = NaN(0,2);
for i_set = I_ikselect
  pareto_data_ii = pareto_data(:,:,i_set);
%   pareto_data(pareto_data>1e3) = NaN; % NB verletzt
  n_iO(i_set) = sum(~isnan(pareto_data_ii(:,1)));
  % Pareto-Fronten einzeichnen (dominierend)
  Idom = ~pareto_dominance(pareto_data_ii);
  pareto_data_iidom = pareto_data_ii(Idom,:);
  [~,Isort] = sort(pareto_data_iidom(:,1));
  Isort = Isort(~isnan(pareto_data_iidom(Isort,1)));
  pareto_data_iidom = pareto_data_iidom(Isort,:);
  % Platzhalter, falls keine Daten vorhanden
  if isempty(pareto_data_iidom), pareto_data_iidom = NaN(1,2); end
  linhdl(i_set) = plot(objscale(1)*pareto_data_iidom(:,1), objscale(2)*pareto_data_iidom(:,2), ...
    defstruct2.format{i_set,2}, 'LineStyle', defstruct2.format{i_set,3}, ...
    'Color', defstruct2.format{i_set, 1}, ...
    'LineWidth', defstruct2.format{i_set, 5}); % [styles{i_set},lines{i_set}]);
  % Alle berechneten Punkte einzeichnen (auch nicht-dominierend);
  if usr_plot_debug
    plot(objscale(1)*pareto_data_ii(~Idom,1), objscale(2)*pareto_data_ii(~Idom,2), ...
      'LineStyle', defstruct2.format{i_set,3}, 'Color', defstruct2.format{i_set,1});
  end
  % Zeichne eine Verbindung von dem zugehörigen Punkt der ursprünglichen
  % Pareto-Front (mit Wert "default" für die IK) zu dem jeweiligen
  % aktuellen Punkt
  if usr_plot_debug
    pareto_data_ref = pareto_data(:,:,end); % Alte Ergebnisse
    if i_set > 1
      for j = find(~isnan(pareto_data_ii(:,1)))'
        plot(objscale(1)*[pareto_data_ref(j,1);pareto_data_ii(j,1)], objscale(2)*[pareto_data_ref(j,2);pareto_data_ii(j,2)], 'k--');
      end
    end
  end
  % Speichere die dominierenden Partikel ab
  pareto_data_dom_all = [pareto_data_dom_all; pareto_data_iidom]; %#ok<AGROW>
end

% Legende mit legendflex.
objective_ik_cases_leg = objective_ik_cases;
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'default', 'heuristic');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'ikjac_cond', 'IK Jac. cond.');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'jac_cond', 'Jac. cond.');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'coll_par', 'collision');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'instspc_par', 'installation space');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'poserr_ee', 'precision');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'none', 'none');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'constant', 'constant');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'maxactvelo', 'act. velo.');
objective_ik_cases_leg = strrep(objective_ik_cases_leg, 'maxactforce', 'act. force');

legnames = cell(1, length(objective_ik_cases)+1);
for i = 1:length(objective_ik_cases)
%   legnames{i} = sprintf('ik obj.: %s (%d i.O.)', objective_ik_cases{i}, n_iO(i)); % Text "default" geht nicht
  legnames{i} = objective_ik_cases_leg{i};
end
legnames{end} = sprintf('Original Pareto front');
grid on;
% title(sprintf('Pareto front based on IK objective %s', objective_ik_cases_leg{I_ikorig}));
% Bild formatieren

if strcmp(obj_cases_plot{1}, 'actforce')
  xlh = xlabel('Actuator joint force in N');
  xlim([7, 24]);
end
if strcmp(obj_cases_plot{2}, 'installspace')
  ylabel('Installation space in m³');
  ylim([0.221, 0.379]);
elseif strcmp(obj_cases_plot{2}, 'positionerror')
  if usr_plotnum == 1
    ylabel(''); % Weglassen, da Farb-Legende direkt links daneben ist
    ylim([2.7, 4.3]);
  else
    ylabel('position error in 100 µm');
  end
else
  % Lasse Standard ylabel
end
% ylim([110, 210]); % Manuell anpassen, damit weniger weißer Leerraum
sgtitle(''); title('');
if usr_plotnum == 1 % keine ylabel wegen daneben liegender Farb-Legende
  set_size_plot_subplot(fighdl, ...
    4, 6, axhdl,... 
    0.13,0.009,  0.03,0.13,  0,0); % lrud xy (u/d konsistent mit cds_taskred_eval_check_case.m)
elseif usr_plotnum == 2
  set_size_plot_subplot(fighdl, ...
    3.7, 6, axhdl,... 
    0.25,0.013,  0.03,0.13,  0,0);
end
% Ursprüngliche Trajektorie nicht zeichnen. Wirft in Diss Fragen zu
% Reproduzierbarkeit auf.
delete(linhdl(end));
linhdl(end) = NaN;

% Textfeld auf das Detail setzen
for k = 1:length(usr_select_pnr)
  pnr = usr_select_pnr(k);
  % Es kann sein, dass die Daten nicht für alle Partikel-Nummern neu
  % generiert wurden. In dem Fall stimmt der Index nicht mit `pnr` überein.
  pnr_in_pd = find(StatsTab_RobPlot.ParetoNr==pnr);
  if isempty(pnr_in_pd)
    warning('Der groß zu zeichnende Marker für Trajektorien aus Redundanzkarte ist nicht in Tabelle enthalten!');
    continue
  end
  % Zeichne den Marker für die ausgewählten Parameter etwas größer
  for ii = 1:length(legnames)
    if isnan(linhdl(ii))
      continue
    end
    hdltmp = plot(objscale(1)*pareto_data(pnr_in_pd,1,ii), ...
                  objscale(2)*pareto_data(pnr_in_pd,2,ii), 'kx');
    set(hdltmp, 'Marker', get(linhdl(ii), 'Marker'));
    set(hdltmp, 'Color', get(linhdl(ii), 'Color'));
    set(hdltmp, 'MarkerSize', 12);
  end
  % Alternative Darstellung zum Bezug auf Redundanzkarte: (ungenauer)
%   % Suche den Schwerpunkt der Pareto-Punkte
%   pareto_data_k = squeeze(pareto_data(pnr,:,:))';
%   thdl = text(mean(objscale(1)*pareto_data_k(:,1)), mean(objscale(2)*pareto_data_k(:,2)), ...
%     sprintf('(%s)', char(96+k))); % Fange bei Buchstaben a an (Bild b ist das Pareto-Diagramm und wird übersprungen)
%   set(thdl, 'BackgroundColor', 'w');
end

figure_format_publication(fighdl); % vor Legendflex machen (wegen Schriftgröße)

% Legende erst nach Größensetzung schreiben
if usr_plotnum == 1
  if exist('lfhdl', 'var'), delete(lfhdl); end
  lfhdl = legendflex(linhdl(~isnan(linhdl)), legnames(~isnan(linhdl)), ...
    'anchor', {'ne','ne'}, ...
    'ref', axhdl, ...
    'buffer', [-4 -4], ...
    'ncol', 1, 'nrow', 0, ... % eine Spalte für Legende
    'title', 'IK objective:', ...
    'fontsize', 9, ... % Sollte konsistent mit figure_format_publication sein
    'FontName', 'Times New Roman', ...
    'xscale', 0.7, ...
    'padding', [1,1,3], ...
    'box', 'on');
  ZOrderSet(lfhdl, 1); % Legende in Vordergrund bringen
end

if usr_plotnum == 2
  % X-Achse mehr nach links verschieben, weil das Bild so klein ist
  [~, ~, X_pos] = get_relative_position_in_axes(axhdl, 'x', -0.2);
  xlhp = get(xlh, 'Position');
  set(xlh, 'Position', [X_pos, xlhp(2) 1]);
end

if length(usr_select_pnr) > 1
  pnr_str = '';
else
  pnr_str = sprintf('_p%d', usr_select_pnr);
end
figname = sprintf('handlingpkm_pareto_%s_%s_compare_%s%s', ...
  obj_cases_plot{1}, obj_cases_plot{2}, RobName, pnr_str);
saveas(fighdl,     fullfile(outputdir, [figname,'.fig']));
exportgraphics(fighdl, fullfile(outputdir, sprintf('%s.pdf', figname)),'ContentType','vector');
save(fullfile(outputdir, [figname,'_info.mat']), 'linhdl');
fprintf('Bild %s gespeichert nach %s\n', figname, outputdir);
