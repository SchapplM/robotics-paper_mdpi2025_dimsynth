% Erstelle ein Bild mit Rechenzeit vs Zielfunktionswert
% Dieses Skript benötigt die vollständigen Daten aus der Simulation
% (diese wurden nicht hochgeladen, da es mehrere Gigabyte sind)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2025-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
close all

usr_classes_type = 'detailed';  % Alternative: 'decades' (10er-Potenzen)

if isempty(which('lufi_dimsynth_data_dir'))
  error(['You have to create a file lufi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');

% Ort wo die Ergebnisse mit Detail.mat liegen
resdirtotal = 'D:\Ubuntu_Transfer\CLUSTER\REPO\structgeomsynth\results';

paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir = fullfile(paper_dir, 'paper', 'Figures');

% Lade Tabelle der Wertebereiche für die Constraints
constrfvalfile = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
  'dimsynth', 'misc', 'constraints_fval_limits.csv');
if ~exist(constrfvalfile, 'file')
  error('Datei constraints_fval_limits.csv nicht im Pfad')
end
ConstrLimTab = readtable(constrfvalfile, 'Delimiter', ';');
%% Zusammenfassungen der bisherige Versuche laden
% Hole die Namen der Optimierungen aus den Ergebnissen des Skripts
% eval_figures_pareto.m

usr_figselection1 = 'motordiagram';
datafile = fullfile(datadir, sprintf('results_all_reps_pareto_%s.mat', usr_figselection1));
assert(exist(datafile, 'file'), 'Daten zu Pareto-Bild existieren nicht');
tmp = load(datafile);
ResTab_unfilt = tmp.ResTab_ges;
StartTimes = datetime(ResTab_unfilt.Startzeit,'InputFormat','yyyy-MM-dd HH:mm:ss');
I = StartTimes > datetime('2025-02-18','InputFormat','yyyy-MM-dd');
ResTab_ges = ResTab_unfilt(I,:);

%% Detail-Ergebnisse nachladen
TimeEvalData = [];
I_act = false(size(ResTab_ges,1),1);
for i = 1:size(ResTab_ges,1)
  OptName = ResTab_ges.OptName{i};
  LfdNr = ResTab_ges.LfdNr(i);
  RobName = ResTab_ges.Name{i};
  dbgstr = sprintf('%s/Rob%d_%s', OptName, LfdNr, RobName);
  if exist('Robots_TL_List', 'var') && ~strcmp(Robots_TL_List{1}{1}, RobName)
    continue
  end
  resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
  if ~exist(resfile, 'file')
    warning('Datei fehlt: %s', resfile);
    continue;
  end
  I_act(i) = true;
  tmp = load(resfile);
  fval_mean = squeeze(mean(tmp.PSO_Detail_Data.fval,2))';
  if isfield(tmp, 'PSO_Detail_Data')
    fval_mean2 = tmp.PSO_Detail_Data.fval_mean;
    test_mean = fval_mean - fval_mean2;
    assert(all(~any(abs(test_mean)>1e-9)), 'Fehler bei Berechnung von fval_mean');
  end
  comptime_data = tmp.PSO_Detail_Data.comptime;
  fval_mean_stack = fval_mean(:);
  comptime_data_stack = comptime_data(:);
  % Suche nach Ausreißern (hier sind sie noch zuzuordnen)
  I_test1 = comptime_data_stack > 3600;
  if any(I_test1)
    I_gen = find(any(comptime_data' > 3600),1,'first');
    I_ind = find(comptime_data(I_gen,:)>3600,1,'first');
    warning(['%s: Partikel %d/%d hat ungewöhnlich lange gerechnet: %1.1fs. ' ...
      'Warum?'], dbgstr, I_gen, I_ind, comptime_data(I_gen, I_ind));
  end

  I_nonnan = ~isnan(fval_mean_stack);
  TimeEvalData = [TimeEvalData; [fval_mean_stack(I_nonnan), comptime_data_stack(I_nonnan)]]; %#ok<AGROW>
end
% Daten in Klassen aufteilen
if strcmp(usr_classes_type, 'decades')
  I_kl = false(size(TimeEvalData,1), 15); % Binär-Inidizes für Klassen
  for i = 1:size(I_kl,2)
    I_kl(TimeEvalData(:,1) > 10^(i-1) & TimeEvalData(:,1) <= 10^(i),i) = true;
  end
else
  I_kl = false(size(TimeEvalData,1), size(ConstrLimTab,1));
  for i = 1:size(ConstrLimTab,1)
    if i > 1 && ConstrLimTab.fval_low(i) < ConstrLimTab.fval_high(i-1)
      warning('Klassen sind nicht aufsteigend');
    end
    I_is_in_class = any(I_kl,2);
    if strcmp(ConstrLimTab.relation_high(i), '<=')
      I_kl(:,i) = (TimeEvalData(:,1) >  ConstrLimTab.fval_low(i) & ...
                 TimeEvalData(:,1) <= ConstrLimTab.fval_high(i));
    elseif strcmp(ConstrLimTab.relation_high(i), '<')
      I_kl(:,i) = (TimeEvalData(:,1) >  ConstrLimTab.fval_low(i) & ...
                 TimeEvalData(:,1) <  ConstrLimTab.fval_high(i));
    elseif strcmp(ConstrLimTab.relation_high(i), '==')
      I_kl(:,i) = (TimeEvalData(:,1) == ConstrLimTab.fval_high(i));
    else
      error('Fall nicht definiert');
    end
  end
end
% Teste, ob die Klassenzuteilung funktioniert hat.
I_noclass = ~any(I_kl,2);
I_firstnoclass = find(I_noclass, 1, 'first');
if any(I_firstnoclass) % Zum Debuggen:
  TimeEvalData(I_firstnoclass,1) >= ConstrLimTab.fval_low & ...
  TimeEvalData(I_firstnoclass,1) <= ConstrLimTab.fval_high
end
TimeEvalDataClasses_sum=sum(I_kl,2);
I_notassigned = TimeEvalDataClasses_sum == 0;
if any(I_notassigned)
  warning('%d Datenpunkte nicht zugeordnet', sum(I_notassigned))
end
I_multiassigned = TimeEvalDataClasses_sum > 1;
if any(I_multiassigned)
  warning('%d Datenpunkte mehrfach zugeordnet', sum(I_multiassigned));
  I_multiassigned1 = find(I_multiassigned, 1, 'first');
  TimeEvalData(I_multiassigned1,:)
  I_kl_ma1 = I_kl(I_multiassigned1,:);
end
assert(size(TimeEvalData,1) == sum(I_kl(:)), ...
  'Nicht alle Einträge wurden einer Klasse zugeordnet');
% Klassen neu stapeln (Eingabeformat für boxplot)
BPData = NaN(size(TimeEvalData,1),1);
GVar = NaN(size(TimeEvalData,1),1);
k = 0;
for i = 1:size(I_kl,2)
  l = sum(I_kl(:,i));
  if l == 0 
    if strcmp(usr_classes_type, 'decades')
      % Leere Klassen sollen als Platzhalter da sein
      BPData(k+1) = NaN;
      GVar(k+1) = i;
      l = 1;
    else % leere Klassen werden übersprungen. Wird sonst zu eng im Bild
      continue
    end
  else
    BPData(k+1:k+l) = TimeEvalData(I_kl(:,i),2);
    GVar(k+1:k+l) = i;
  end
  k = k + l;
end

fprintf('%d Optimierungen, %d verschiedene Roboter\n', ...
  length(unique(ResTab_ges.OptName(I_act))), ...
  length(unique(ResTab_ges.Name(I_act))));
fprintf('Anzahl der Partikel in den Klassen: [%s]\n', disp_array(sum(I_kl), '%d'));

% Bild zeichnen
I_class_select = find(any(I_kl));
fighdl = figure(1);clf;hold on;
grid on;
bphdl = boxplot(BPData, GVar);
% Die Ausreißer sollen ausgedünnt werden, damit das Tikz-Bild gezeichnet
% werden kann
n_removed_outlier = 0;
n_total_outlier = 0;
for k = 1:size(bphdl,2)
  hdl_outlier_k = get(bphdl(7,k));
  xy_k = [hdl_outlier_k.XData', hdl_outlier_k.YData'];
  II = select_plot_indices_downsample_nonuniform(hdl_outlier_k.XData(:), ...
    hdl_outlier_k.YData(:), 0.5, 1.3, [false,true]);
  n_removed_outlier = n_removed_outlier + sum(~II);
  n_total_outlier = n_total_outlier + length(II);
  set(bphdl(7,k), 'XData', xy_k(II,1)', 'YData', xy_k(II,2)');
end
fprintf('Insgesamt %d/%d Outlier entfernt bzw. %d übrig gelassen\n', ...
  n_removed_outlier, n_total_outlier, n_total_outlier-n_removed_outlier);

if strcmp(usr_classes_type, 'decades')
  xlim([2.5, 12.5]);
  xlabel('Obere Grenze von $\mathrm{log}(f)$', 'interpreter', 'latex');
else
  % Beschriftung der Achsen mit den Bezeichnungen aus der Tabelle.
  xtltxt = cell(1,length(I_class_select));
  for kk = 1:length(I_class_select)
    % xtltxt{kk} = sprintf('%d', kk);
    if ~strcmp(ConstrLimTab.latex_label{I_class_select(kk)}, '')
      xtltxt{kk} = sprintf('constr. \\ref*{%s}', ConstrLimTab.latex_label{I_class_select(kk)});
    else
      xtltxt{kk} = ConstrLimTab.text{I_class_select(kk)};
    end
    % Nachbearbeiten für Paper
    if strcmp(xtltxt{kk}, 'i.O.')
      xtltxt{kk} = 'success';
    elseif strcmp(xtltxt{kk}, 'Beinkettenlänge Eckpkt.')
      % Schubgelenke führen dazu, dass die Beinkette zu lang ist
      xtltxt{kk} = 'chainlength';
    elseif strcmp(xtltxt{kk}, 'constr. \ref*{itm:constr_param_inclination}')
      % Effektiv sind es mehrere Nebenbedingungen
      xtltxt{kk} = 'constr. \ref*{itm:constr_param_inclination}--\ref*{itm:constr_param_chainlength}';
    end
  end
  set(gca, 'xticklabel', xtltxt);
end
set(gca, 'XTickLabelRotation', 90); % matlab2tikz geht nicht gut mit 45
set(gca, 'YScale', 'log');

set(gca, 'YTick', [0.01, 0.1, 1, 10, 100, 1000]);
set(gca, 'YTickLabel', {'0.01', '0.1', '1', '10', '100', '1000'});
ylim_old = get(gca, 'YLIM');
set(gca, 'YLIM', [5e-3, ylim_old(2)]); % untere Grenze etwas erhöhen

ylabel('Time per fitness evaluation in s');

figure_format_publication(gca);
set(get(gca, 'Title'), 'FontWeight', 'normal');
set(get(gca, 'Title'), 'Units', 'normalized');
set(get(gca, 'Title'), 'Position', [0.35, 1.06,0]);

set_size_plot_subplot(fighdl, 18, 8, gca,...
  0.07,0.02,0.02,0.18,0,0); % bl,br,hu,hd,bdx,bdy
exportgraphics(fighdl, fullfile(outputdir, 'calctime_vs_fval.png'), 'Resolution', 300);
matlab2tikz(fullfile(outputdir, 'calctime_vs_fval.tex'), ...
  'parseStrings', false, ...
  'strictFontSize', false);

fprintf('Auswertung zu Rechenzeit beendet. %d verschiedene Roboter-Optimierungen. %d Partikel.\n', ...
  length(unique(ResTab_ges.Name)), sum(I_kl(:)));
