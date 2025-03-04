% Erzeuge Latex-Tabellen mit den Ergebnissen
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% * Zusammenstellen der Details mit select_eval_robot_examples.m
% * Ingenieurslösung aus eval_existing_design.m
% 
% Erzeugt tab_kinpar.tex. Der Inhalt wird dann in die Diss kopiert.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
usr_fig_all = {'default_revolute', 'default_prismatic'};
for i_usr_fig = 1:2
usr_figselection = usr_fig_all{i_usr_fig}; % siehe andere Skripte.

SerRobDB = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof', 'N');
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_handling_robot.m'));
outputdir = fullfile(paper_dir, 'paper', 'Tables');
outputdir1 = fullfile(paper_dir, 'paper', 'Figures');
mkdirs(outputdir);
repo_dir = fileparts(which('HandlingRobot_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');

tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)));
RobotGroups = tmp.RobotGroups;
% Sortiere die Gruppen aufsteigend mit den PR-Nummern (wird in
% eval_figures_pareto_groups.m noch nicht gemacht).
[~,I] = sortrows(RobotGroups.KinematicsNumber);
RobotGroups = RobotGroups(I,:);

existingdesignsuffix_all = {'CRR', 'PUU', 'CRU', 'CUR', 'UPU'}; % Reihenfolge wie in Paper, konsistent zu Legende in Pareto-Bild

for i_mode = 1:3 % 3T0R, 3T1R, Referenzlösungen
if i_mode < 3
  % Wird gefüllt mit Detail-Daten zu den ausgewählten Robotern (enthält
  % Koppelgelenk-Namen)
  InfoTab = readtable(fullfile(outputdir1, ...
    sprintf('handlingpkm_groups_legend_%s_add.csv', usr_figselection)), 'Delimiter', ';');
else % Ohne Zusatz "add" laden. Dann direkt aus eval_figures_pareto_groups generiert.
  InfoTab = readtable(fullfile(outputdir1, ...
    sprintf('handlingpkm_groups_legend_%s.csv', usr_figselection)), 'Delimiter', ';');
end
% Aus pareto_groups erzeugte Legende
InfoTab2 = readtable(fullfile(outputdir1, ...
  sprintf('handlingpkm_groups_legend_%s.csv', usr_figselection)), 'Delimiter', ';');
for i_dh = 0:1 % 0=PKM-Parameter, 1=DH-Parameter der Beinkette

if i_dh == 0
  suffix_dh = '';
else
  suffix_dh = '_dh';
end
%% Alle Gruppen durchgehen
countrob = 0; % Nummern aus Legende
Name_LegChain_Gen_vorher = '';
if i_mode == 1
  suffix_mode = '_3T0R';
  I = find(contains(RobotGroups.GroupName, 'P3'));
elseif i_mode == 2
  suffix_mode = '_3T1R';
  I = find(contains(RobotGroups.GroupName, 'P4'));
elseif i_mode == 3
  suffix_mode = '_Ref';
  if strcmp(usr_figselection, 'default_prismatic')
    I = size(RobotGroups,1)+(1:5); % fünf Extra-Zeilen für existierende Modelle aus PrauseChaCor2015
  else
    I = [];
  end
end
% Filtere die auszuwählenden Roboter nach Robotertyp (bereits hier, damit
% klar ist, welches die letzte Zeile der Tabelle ist)
for j = 1:length(I)
  i = I(j);
  if i <= size(RobotGroups,1)
    GroupName = RobotGroups.GroupName{i};
    if RobotGroups.ResultsFound(i) == 0
      I(j) = 0; % keine Ergebnisse vorliegend. Unklar, PKM wurde vermutlich nachträglich beim Pareto-Diagramm wegen zusätzlicher Kriterien ausasortiert.
    end
    if contains(GroupName(2:end), 'P') && contains(usr_figselection, 'revolute') || ...
      ~contains(GroupName(2:end), 'P') && contains(usr_figselection, 'prismatic')
      I(j) = 0; % Gruppe wird hier nicht betrachtet.
    end
  end
end
I = I(I~=0);
% Schreibe Latex-Daten in Textdatei
results_latex_file = fullfile(outputdir, sprintf('handlingpkm_tab_details_%s%s%s.tex', ...
  strrep(usr_figselection, 'default_', ''), suffix_mode, suffix_dh));
fid = fopen(results_latex_file, 'w');
for i = I(:)'
  if i <= size(RobotGroups,1)
    GroupName = RobotGroups.GroupName{i};
    fprintf('Schreibe Tabellenzeile für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
    resfile = fullfile(datadir, sprintf('detail_result_group_%s_%s.mat', GroupName, usr_figselection));
    if ~exist(resfile, 'file')
      warning('Ergebnis %s existiert nicht. Überspringe Zeile.', resfile);
      continue
    end
    erg = load(resfile);
  else % Siehe eval_existing_design.m
    existingdesign_resname = sprintf('detail_result_PrauseChaCor2015_%s_%s', ...
      existingdesignsuffix_all{i-size(RobotGroups,1)}, 'default');
    erg = load(fullfile(datadir, sprintf('%s.mat', existingdesign_resname)));
    erg.RobName = erg.R.mdlname;
    GroupName = erg.RobName;
  end
  countrob = countrob + 1;
  
  % Daten der Ergebnisse laden
  fprintf('Anzahl Variablen: %d\n', length(erg.Structure.vartypes));
  fprintf('Variablen: %s\n', disp_array(erg.Structure.varnames, '%s'));
  fprintf('Parametertypen: [%s]\n', disp_array(erg.Structure.vartypes', '%1.0f'));
  
  % Kinematik-Parameter zusammenstellen
  R = erg.R;
  d_mdh_all =  [R.Leg(1).MDH.d; NaN]; % NaN für 3PRRR
  a_mdh_all =  [R.Leg(1).MDH.a; NaN];
  alpha_mdh_all =  [R.Leg(1).MDH.alpha];
  theta_mdh_all =  [R.Leg(1).MDH.theta];
  d_mdh_all_fix = false(size(d_mdh_all));
  a_mdh_all_fix  = false(size(d_mdh_all));
  alpha_mdh_all_fix  = false(size(d_mdh_all));
  theta_mdh_all_fix  = false(size(d_mdh_all));
  r_base = R.DesPar.base_par(1);
  if any(strcmp(erg.Structure.varnames, 'base_morph_pairdist'))
    d_basepair = R.DesPar.base_par(2);
  else
    d_basepair = NaN;
  end
  r_plf = R.DesPar.platform_par(1);
  if any(strcmp(erg.Structure.varnames, 'platform_morph_pairdist'))
    d_plfpair = R.DesPar.platform_par(2);
  else
    d_plfpair = NaN;
  end
  % qoff = R.Leg(1).DesPar.joint_offset(1);
  Ip_G4elev = strcmp(erg.Structure.varnames, 'base_morph_coneelev') | ...
              strcmp(erg.Structure.varnames, 'base_morph_pyrelev');
  if ~any(Ip_G4elev)
    phi_base = 0;
  else
    phi_base = erg.pval(Ip_G4elev);
  end
  Ip_P9elev = strcmp(erg.Structure.varnames, 'platform_morph_axtiltangle');
  if erg.Structure.Coupling(2) == 7 % Ist auch schief, aber Parameter nicht direkt ablesbar
    R_P_B1 = eulxyz2r(erg.R.phi_P_B_all(:,1));
    phi_platform = acos([0;0;1]' * R_P_B1(:,3));
  elseif ~any(Ip_P9elev)
    phi_platform = 0;
  else
    phi_platform = erg.pval(Ip_P9elev);
  end
  % Trage Parameter, die nicht optimiert werden und Null sind, als NaN ein
  for kk = 1:6
    if length(d_mdh_all) < kk, continue; end
    if ~any(contains(erg.Structure.varnames, sprintf(': d%d', kk))) && d_mdh_all(kk)==0
      d_mdh_all_fix(kk) = true;
    end
    if ~any(contains(erg.Structure.varnames, sprintf(': a%d', kk))) && a_mdh_all(kk)==0
      a_mdh_all_fix(kk) = true;
    end
    if ~any(contains(erg.Structure.varnames, sprintf(': alpha%d', kk)))
      alpha_mdh_all_fix(kk) = true;
    end
    if ~any(contains(erg.Structure.varnames, sprintf(': theta%d', kk)))
      if GroupName(2+kk) == 'R'
        theta_mdh_all(kk) = NaN; % ist die Koordinate (aber passiv, daher Striche)
      else
        theta_mdh_all_fix(kk) = true;
      end
    end
  end
  
  % Erzeuge den Vektor der in der Tabelle einzutragenden Parameter. Nehme
  % den Betrag, da das negative Vorzeichen bei den DH-Parametern ohne die
  % genauen Werte für alpha und theta nicht aussagekräftig ist und eher
  % verwirrt.
  kinparvec = [1e3*r_base, 180/pi*phi_base, 1e3*r_plf, 180/pi*phi_platform];%, ...
  kinparvec_dh = [];
  kinparvec_dh_fix = [];
  for kk = 1:5
%     if length(d_mdh_all) < kk, continue; end % für 3PRRR
    if any(kk == [2 3 4]) % alpha
      kinparvec_dh = [kinparvec_dh; abs(180/pi*alpha_mdh_all(kk))]; %#ok<AGROW> 
      kinparvec_dh_fix = [kinparvec_dh_fix, alpha_mdh_all_fix(kk)]; %#ok<AGROW>
    % elseif ~isnan(alpha_mdh_all(kk))
    %   error('alpha %d ist belegt, aber nicht in Tabelle', kk);
    end
    if kk > 1 % a
      kinparvec_dh = [kinparvec_dh; abs(1e3*a_mdh_all(kk))]; %#ok<AGROW> 
      kinparvec_dh_fix = [kinparvec_dh_fix, a_mdh_all_fix(kk)]; %#ok<AGROW>
    % elseif ~isnan(a_mdh_all(kk))
    %   error('a %d ist belegt, aber nicht in Tabelle', kk);
    end
%     if false % theta; nur bei PKM mit Schubgelenk relevant
%       kinparvec_dh = [kinparvec_dh; abs(180/pi*theta_mdh_all(kk))]; %#ok<AGROW> 
%     end 
    % Parameter d
    if kk > 1 % d
      kinparvec_dh = [kinparvec_dh; abs(1e3*d_mdh_all(kk))]; %#ok<AGROW> 
      kinparvec_dh_fix = [kinparvec_dh_fix, d_mdh_all_fix(kk)]; %#ok<AGROW>
    % elseif ~isnan(d_mdh_all(kk))
    %   error('d %d ist belegt, aber nicht in Tabelle', kk);
    end
  end
  if i <= size(RobotGroups,1)
    objectives_i = erg.objectives_test;
  else
    objectives_i = erg.Set.optimization.objective;
  end
  physval_i = erg.physval;
  m_sum = physval_i(strcmp(objectives_i, 'mass'));
  
  if i <= size(RobotGroups,1)
    % Laufende Nummer (aus Legende) eintragen (war in Paper so)
    latex_row = ''; % sprintf('%d', countrob);
    % Symbol des Markers eintragen (zum schnelleren Blick in Legende; war im Paper)
%     latex_row = [latex_row, sprintf([' & \\vcenteredinclude', ...
%       '{group%d_marker.pdf}'], i)]; %#ok<AGROW>
%     latex_row = [latex_row, sprintf(' & %d', i)]; %#ok<AGROW>
    % Latex-Verweis auf PKM-Datenbank erzeugen
%     Robots_i = RobotGroups.Robots{i};
%     if isa(Robots_i, 'char'), Robots_i = {Robots_i}; end
%     I_k = strcmp(ResTab_NameTrans.PKM_Name, Robots_i{1});
%     ilc = strcmp(SerRobDB.Names, ResTab_NameTrans.Chain_Name{I_k});
%     ChainNameGen_i = SerRobDB.Names{SerRobDB.AdditionalInfo(ilc, 3)};
    % Allgemeinen Namen für Latex-Verweis (bereits vorher generiert)
    i_IT = strcmp(InfoTab.GroupName, GroupName);
    if ~any(i_IT), error('Fehler bei Zuordnung von Legende zu Gruppenname'); end
    couplstr = InfoTab.CouplingOnly{i_IT};
    i_IT2 = strcmp(InfoTab2.GroupName, GroupName);
    [tokens, match] = regexp(InfoTab2.TextLink{i_IT2}, '\.\\ref\{(.*)\}', 'tokens', 'match');
    latexlabel = tokens{1}{1};
    RobName_noPR = InfoTab2.TextFix{i_IT2};
    latex_row = [latex_row, '\hyperrefl{', latexlabel, '}{', RobName_noPR, '},\,', couplstr]; %#ok<AGROW> 
    % if i_dh == 0 % Kurzer Name in erster Tabelle ("PR 35.5, r-r")
    %   latex_row = [latex_row, ' ', InfoTab.TextLink{i_IT}, ', ', ...
    %     InfoTab.CouplingOnly{i_IT}]; %#ok<AGROW> 
    % else
    %   % längere Beschreibung in zweiter Tabelle ("PR 20.4 C-V, 6-RUS")
    %   latex_row = [latex_row, ' ', InfoTab.TextLink{i_IT}, ', ', ... % couplstr, ...
    %     '', InfoTab.TextCoupling{i_IT}, '']; %#ok<AGROW> 
    % end
  elseif i > size(RobotGroups,1)
    i_IT = size(InfoTab,1)-5 + i-size(RobotGroups,1);
    % [tokens, match] = regexp(InfoTab.TextFix{i_IT}, '(.*)[$ ]+[.*]?', 'tokens', 'match');
    shortname_eng = InfoTab.TextFix{i_IT}(1:17); % 3-CRR (mit Latex-Unterstreichung) (glücklicherweise alle Strings gleich lang
    % if i_dh == 0 % Kurzer Name: 3-CRR (mit Unterstreichung)
      latex_row = shortname_eng;
    % else % etwas längerer Name: 3-CRR (3T0R PR 1.1)
    %   latex_row = [shortname_eng, ' (3T0R ', InfoTab.TextLink{i_IT}, ')'];
    % end
    fprintf('Benutze Legendeneintrag %s für %s\n', InfoTab.TextFix{i_IT}, ...
      existingdesignsuffix_all{i - size(RobotGroups,1)});
  end
  % Namen des Roboters erzeugen (nicht mehr in Tabelle einfügen, da bereits
  % in Legende des Bildes).
  % jj_rn = strcmp(RobNamesTab.PKM_Name, erg.RobName);
  % latex_row = [latex_row, ' & ', sprintf('%s', RobNamesTab.ChainStructure{jj_rn})]; %#ok<AGROW>
  % if length(RobNamesTab.Chain_ShortName{jj_rn}) < 5
  %   latex_row = [latex_row, sprintf(' (%s)', RobNamesTab.Chain_ShortName{jj_rn})]; %#ok<AGROW>
  % end
  %% Tabelle für Leistungsmerkmale und PKM-Parameter
  if i_dh == 0
    % Leistungsmerkmale eintragen: TODO: Bauraum und Fußabdruck
    % Nummer der Zielfunktion bestimmen. 
    if all(~isnan(physval_i))
      latex_row = [latex_row, sprintf(' & %1.0f', ...
        -physval_i(strcmp(objectives_i, 'colldist'))*1e3)]; %#ok<AGROW>
      latex_row = [latex_row, sprintf(' & %1.2f', ...
        physval_i(strcmp(objectives_i, 'installspace')))]; %#ok<AGROW>
      latex_row = [latex_row, sprintf(' & %1.2f', ...
        physval_i(strcmp(objectives_i, 'footprint')))]; %#ok<AGROW>
  
      latex_row = [latex_row, sprintf(' & %1.1f', ...
        physval_i(strcmp(objectives_i, 'power')))]; %#ok<AGROW>
      % Antriebskraft eintragen
      latex_row = [latex_row, sprintf(' & %1.1f', 1*...
        physval_i(strcmp(objectives_i, 'actforce')))]; %#ok<AGROW>
      latex_row = [latex_row, sprintf(' & %1.0f', 180/pi* ...
        physval_i(strcmp(objectives_i, 'actvelo')))]; %#ok<AGROW>
      % Konditionszahl eintragen
      latex_row = [latex_row, sprintf(' & %1.1f', ...
        physval_i(strcmp(objectives_i, 'condition')))]; %#ok<AGROW>
      else % Keine Ergebnisse vorhanden. Einträge leerlassen
      for kk = 1:7, latex_row = [latex_row, ' & \multicolumn{1}{c}{---}']; end %#ok<AGROW> 
    end
    % Masse eintragen (nicht mehr machen. Platz ist knapp.
  %   latex_row = [latex_row, sprintf(' & %1.1f', m_sum)]; %#ok<AGROW>
    % Anzahl der Optimierungsparameter
    latex_row = [latex_row, sprintf(' & %d',  length(erg.Structure.vartypes))]; %#ok<AGROW>
    
    % Kinematikparameter eintragen (s.o., nur PKM-Parameter)
    for i_kinpar = 1:length(kinparvec)
      if isnan(kinparvec(i_kinpar)) % Trage einen Strich ein (zentriert)
        latex_row = [latex_row, ' & \multicolumn{1}{c}{---}']; %#ok<AGROW>
      else % Trage Parameter als Zahlenwert ein
        latex_row = [latex_row, sprintf(' & %1.0f', kinparvec(i_kinpar))]; %#ok<AGROW>
      end
    end
  end

  %% Tabelle für DH-Parameter
  if i_dh == 1
    % Kinematikparameter eintragen (DH-Parameter)
    for i_kinpar = 1:length(kinparvec_dh)
      if isnan(kinparvec_dh(i_kinpar)) % Trage einen Strich ein
        latex_row = [latex_row, sprintf(' & \\multicolumn{1}{c}{---}')]; %#ok<AGROW>
      elseif isinf(kinparvec_dh(i_kinpar)) % Markiere aktives Gelenk
        latex_row = [latex_row, sprintf(' & \\multicolumn{1}{c}{act.}')]; %#ok<AGROW>
      else % Trage Parameter als Zahlenwert ein
        if kinparvec_dh_fix(i_kinpar) % ist fixiert. Nehme andere Farbe
          latex_row = [latex_row, sprintf(' & \\textcolor{lightgray}{%1.0f}', kinparvec_dh(i_kinpar))]; %#ok<AGROW>
        else
          latex_row = [latex_row, sprintf(' & %1.0f', kinparvec_dh(i_kinpar))]; %#ok<AGROW>
        end
      end
    end
  end
  %% Tabelle Abschließen
  % Strich ziehen, wenn die grundlegende Kinematik gewechselt hat.
  % Kriterium dafür die die Beinketten-Kinematik
  [~,LEG_Names]=parroblib_load_robot(erg.RobName);
  LegName = LEG_Names{1};
  serroblibpath=fileparts(which('serroblib_path_init.m'));
  NLJ = str2double(LegName(2));
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', ...
    NLJ), sprintf('S%d_list.mat',NLJ));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
  Ir_db = find(strcmp(l.Names_Ndof, LegName));
  I_genmdl = l.AdditionalInfo(Ir_db,3);
  Name_LegChain_Gen = l.Names_Ndof{I_genmdl};
  if countrob > 1 && ... % Erste Zeile darf nicht die hline sein (sonst Latex-Fehler)
      (~strcmp(Name_LegChain_Gen, Name_LegChain_Gen_vorher) || i>size(RobotGroups,1))
    fprintf(fid, '\\midrule\n');
  end
  Name_LegChain_Gen_vorher = Name_LegChain_Gen;

  % Durch Rundung entsteht "-0". Nicht sinnvoll.
  latex_row = strrep(latex_row, ' -0 ', ' 0 ');

  % Nachverarbeitung: Minus-Zeichen im Mathe-Modus. TODO: Besser RegExp.
  latex_row = strrep(latex_row, ' -', ' $-$');
  latex_row = strrep(latex_row, ' $-$-', ' --');

  % Eigentliche Datenzeile schreiben
  fprintf(fid, '%s', latex_row);
  if i ~= I(end) % Bei letzter Zeile kein Zeilenumbruch mit \\ am Ende der Tabelle
    fprintf(fid, ' \\\\ ');
  end
  % Zeilenabschluss mit Kommentar
  if i <= size(RobotGroups,1)
    fprintf(fid, '%% group %d; %s/Rob%d_%s (Pareto-Index %d)\n', ...
      i, erg.OptName, erg.LfdNr, erg.RobName, erg.Ipar);
  else 
    fprintf(fid, '%% Reference Solution; see eval_existing_design.m\n');
  end

end
% fprintf(fid, '\\hline\n');
fclose(fid);
fprintf('Tabelle nach %s geschrieben\n', results_latex_file);
end % i_dh
end % Roboter-Auswahl i_mode
end % Dreh-/Schubgelenk
