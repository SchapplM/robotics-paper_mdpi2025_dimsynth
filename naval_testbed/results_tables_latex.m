% Erzeuge Latex-Tabellen mit den Ergebnissen
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% * Zusammenstellen der Details mit select_eval_robot_examples.m
% * Ingenieurslösung aus eval_existing_design.m
% 
% Erzeugt tab_kinpar.tex. Der Inhalt wird dann in die Diss kopiert.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_figselection = 'power_vs_coll'; % siehe andere Skripte

SerRobDB = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof', 'N');
%phdthesis_dir = fileparts(which('phd_thesis_schappler_path_init.m'));
paper_dir = fileparts(which('run_evaluation_naval_testbed.m'));
outputdir = fullfile(paper_dir, 'paper', 'Tables');
outputdir1 = fullfile(paper_dir, 'paper', 'Figures');
mkdirs(outputdir);
repo_dir = fileparts(which('lufi_dimsynth_data_dir.m'));
datadir = fullfile(repo_dir,'data');
% Namen der Roboter laden (enthält auch Eigenschaften zu Kardan-Gelenken)
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');

tmp = load(fullfile(datadir, sprintf('robot_groups_%s.mat', usr_figselection)));
RobotGroups = tmp.RobotGroups;
% Sortiere die Gruppen aufsteigend mit den PR-Nummern (wird in
% eval_figures_pareto_groups.m noch nicht gemacht).
[~,I] = sortrows(RobotGroups.KinematicsNumber);
RobotGroups = RobotGroups(I,:);
% Wird gefüllt mit Detail-Daten zu den ausgewählten Robotern (enthält
% Koppelgelenk-Namen)
InfoTab = readtable(fullfile(outputdir1, ...
  sprintf('lufipkm_groups_legend_%s_add.csv', usr_figselection)), 'Delimiter', ';');
% Aus pareto_groups erzeugte Legende
InfoTab2 = readtable(fullfile(outputdir1, ...
  sprintf('lufipkm_groups_legend_%s.csv', usr_figselection)), 'Delimiter', ';');
for i_dh = 0:1 % 0=PKM-Parameter, 1=DH-Parameter der Beinkette
for i_gf = 1:2 % 1=Drehantriebe, 2=Schubantriebe

I_Pjoint = false(size(RobotGroups,1),1);
for i = 1:length(I_Pjoint)
  if isa(RobotGroups.Robots{i}, 'cell'), robot_i = RobotGroups.Robots{i}{1};
  else, robot_i = RobotGroups.Robots{i}; end
  [~, LEG_Names, Actuation, ~, ~, ~, ~, ~] = parroblib_load_robot(robot_i, 2);
  if contains(LEG_Names, 'P'), I_Pjoint(i) = true; end
end
if i_gf == 1
  suffix_gf = 'revolute';
  I_gf = find(~I_Pjoint);
else
  suffix_gf = 'prismatic';
  I_gf = [find(I_Pjoint); size(RobotGroups,1)+1]; % Ingenieurslösung hier eintragen
end
if i_dh == 0
  suffix_dh = '';
else
  suffix_dh = '_dh';
end
%% Alle Gruppen durchgehen
countrob = 0; % Nummern aus Legende
% Schreibe Latex-Daten in Textdatei
results_latex_file = fullfile(outputdir, sprintf('lufipkm_tab_details_%s%s.tex', ...
  suffix_gf, suffix_dh));
fid = fopen(results_latex_file, 'w');
Name_LegChain_Gen_vorher = '';
for i = I_gf(:)'
  if i <= size(RobotGroups,1)
    GroupName = RobotGroups.GroupName{i};
    if RobotGroups.ResultsFound(i) == 0, continue; end % keine Ergebnisse vorliegend
    fprintf('Schreibe Tabellenzeile für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
    erg = load(fullfile(datadir, sprintf('detail_result_group_%s_%s.mat', GroupName, usr_figselection)));
  else
    erg = load(fullfile(datadir, 'detail_result_engineering_solution.mat'));
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
  d_mdh_all =  [R.Leg(1).MDH.d]; % NaN für 3PRRR
  a_mdh_all =  [R.Leg(1).MDH.a];
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
  % Trage Parameter, die nicht optimiert werden und Null sind, als NaN ein
  for kk = 1:6
    if length(d_mdh_all) < kk, continue; end % für 3PRRR
    if ~any(contains(erg.Structure.varnames, sprintf(': d%d', kk))) && d_mdh_all(kk)==0
      d_mdh_all_fix(kk) = true;
    end
    % Bei Schubantrieb ist ein d-Parameter die Koordinate
    if i_gf==2 && GroupName(2+kk) == 'P'
      d_mdh_all(kk) = inf; % als Markierung
    end
    if ~any(contains(erg.Structure.varnames, sprintf(': a%d', kk))) && a_mdh_all(kk)==0
      a_mdh_all_fix(kk) = true;
    end
    if ~any(contains(erg.Structure.varnames, sprintf(': alpha%d', kk)))
      % Setze auf konstanten Wert aus Datenbank
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
  kinparvec = [1e3*r_base, 1e3*d_basepair, 180/pi*phi_base, ...
               1e3*r_plf, 1e3*d_plfpair];%, ...
  kinparvec_dh = [];
  kinparvec_dh_fix = [];
  for kk = 1:6
    if any(kk == [2 3 4]) % alpha
      kinparvec_dh = [kinparvec_dh; abs(180/pi*alpha_mdh_all(kk))]; %#ok<AGROW>
      kinparvec_dh_fix = [kinparvec_dh_fix, alpha_mdh_all_fix(kk)]; %#ok<AGROW>
    end
    if kk > 1 % a
      kinparvec_dh = [kinparvec_dh; abs(1e3*a_mdh_all(kk))]; %#ok<AGROW> 
      kinparvec_dh_fix = [kinparvec_dh_fix, a_mdh_all_fix(kk)]; %#ok<AGROW>
    end
    if any(kk == [2 3 4]) && i_gf == 2 % theta; nur bei PKM mit Schubgelenk relevant
      kinparvec_dh = [kinparvec_dh; abs(180/pi*theta_mdh_all(kk))]; %#ok<AGROW> 
      kinparvec_dh_fix = [kinparvec_dh_fix, theta_mdh_all_fix(kk)]; %#ok<AGROW>
    end 
    % Parameter d
    kinparvec_dh = [kinparvec_dh; abs(1e3*d_mdh_all(kk))]; %#ok<AGROW>
    kinparvec_dh_fix = [kinparvec_dh_fix, d_mdh_all_fix(kk)]; %#ok<AGROW>
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
    % if i_dh == 0 % längere Beschreibung ("PR 20.4 C-V") für erste PKM-Tabelle
    latex_row = [latex_row, '\hyperrefl{', latexlabel, '}{', RobName_noPR, '},\,', couplstr]; %#ok<AGROW> 
    % else % kürzere Beschreibung: "20.4" für DH-Tabelle
    %   latex_row = [latex_row, ' ', strrep(InfoTab.TextLink{i_IT}, 'PR\,', '')]; %#ok<AGROW> 
    % end
  else % Doppelspalte für "Eng." (im Paper, nicht mehr in Diss; da in Diss anderes Format)
    if i_dh == 0
      latex_row = 'Eng. Sol.';
    else
      latex_row = 'Eng.'; % \multicolumn{2}{|c|}{Eng.}';
    end
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
    % Leistungsmerkmale eintragen
    % Nummer der Zielfunktion bestimmen. 
    latex_row = [latex_row, sprintf(' & %1.0f', ...
      -physval_i(strcmp(objectives_i, 'colldist'))*1e3)]; %#ok<AGROW>
    latex_row = [latex_row, sprintf(' & %1.2f', ...
      physval_i(strcmp(objectives_i, 'power'))/1e3)]; %#ok<AGROW>
    % Antriebskraft eintragen
    if i_gf == 1 % Drehmoment ist <1000
      latex_row = [latex_row, sprintf(' & %1.0f', 1*...
        physval_i(strcmp(objectives_i, 'actforce')))]; %#ok<AGROW>
    else % Kraft ist > 1000 (dann in kN)
      latex_row = [latex_row, sprintf(' & %1.2f', 1e-3*...
        physval_i(strcmp(objectives_i, 'actforce')))]; %#ok<AGROW>
    end
    if i_gf == 1 % Drehgeschw. in deg/s
    latex_row = [latex_row, sprintf(' & %1.0f', 180/pi* ...
      physval_i(strcmp(objectives_i, 'actvelo')))]; %#ok<AGROW>
    else % Geschw. ist ca. 1m/s (dann in kN)
    latex_row = [latex_row, sprintf(' & %1.2f', 1* ...
      physval_i(strcmp(objectives_i, 'actvelo')))]; %#ok<AGROW>
    end
  
    % Konditionszahl eintragen
    latex_row = [latex_row, sprintf(' & %1.0f', ...
      physval_i(strcmp(objectives_i, 'condition')))]; %#ok<AGROW>
  
    % latex_row = [latex_row, sprintf(' & %1.1f', erg.jointrange_with_springrest*180/pi')]; %#ok<AGROW>
    % Masse eintragen (nicht mehr machen. Platz ist knapp.
  %   latex_row = [latex_row, sprintf(' & %1.1f', m_sum)]; %#ok<AGROW>
    % Anzahl der Optimierungsparameter
    latex_row = [latex_row, sprintf(' & %d',  length(erg.Structure.vartypes))]; %#ok<AGROW>
    
    % Kinematikparameter eintragen (s.o., nur PKM-Parameter)
    for i_kinpar = 1:length(kinparvec)
      if isnan(kinparvec(i_kinpar)) % Trage einen Strich ein
        latex_row = [latex_row, sprintf(' & \\multicolumn{1}{c}{---}')]; %#ok<AGROW>
      else % Trage Parameter als Zahlenwert ein
        latex_row = [latex_row, sprintf(' & %1.0f', kinparvec(i_kinpar))]; %#ok<AGROW>
      end
    end
  
    % Entwurfsparameter eintragen (siehe cds_dimsynth_design.m bzw. SerRob.m)
  %   pval_ls = erg.pval_desopt(erg.Structure.desopt_ptypes == 2);
    % Wandstärke (1) und Durchmesser (2)
    % Berechne die Masse pro Länge? Lieber Material-Querschnitt 
    % (siehe cds_dimsynth_design.m / Funktion data_hollow_cylinder)
    if isfield(erg, 'pval_desopt')
      % In Roboter-Klasse sind die Werte nur passend, wenn die Fitness-Funktion
      % neu ausgewertet wird: 
      e_i = erg.pval_desopt(strcmp(erg.Structure.desopt_pnames, 'linkmaterialstrength'));
      R_i = erg.pval_desopt(strcmp(erg.Structure.desopt_pnames, 'linkdiameter'))/2;
    else
      % Für "existing design" direkt die Roboterklasse nehmen
      e_i = erg.R.Leg(1).DesPar.seg_par(1,1);
      R_i = erg.R.Leg(1).DesPar.seg_par(1,2)/2;
    end
    A_mat = pi*(R_i^2) - (pi*((R_i-e_i)^2)); % doch nicht zeigen, Erklärung schwieriger.
    latex_row = [latex_row, sprintf(' & %1.0f', e_i*1e3)]; %#ok<AGROW>
    latex_row = [latex_row, sprintf(' & %1.0f', 2*R_i*1e3)]; %#ok<AGROW>
  end

  %% Tabelle für DH-Parameter
  if i_dh == 1
    % Kinematikparameter eintragen (DH-Parameter)
    for i_kinpar = 1:length(kinparvec_dh)
      if isnan(kinparvec_dh(i_kinpar)) % Trage zentriert einen Strich ein (in Diss, wenn Parameter vorgegeben wird)
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
  if i > I_gf(1) && ... % Erste Zeile darf nicht die hline sein (sonst Latex-Fehler)
      ~strcmp(Name_LegChain_Gen, Name_LegChain_Gen_vorher) || i>size(RobotGroups,1)
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
  if i ~= I_gf(end) % Bei letzter Zeile kein Zeilenumbruch mit \\ am Ende der Tabelle
    fprintf(fid, ' \\\\ ');
  end
  % Zeilenabschluss mit Kommentar
  if i <= size(RobotGroups,1)
    fprintf(fid, '%% group %d; %s/Rob%d_%s (Pareto-Index %d)\n', ...
      i, erg.OptName, erg.LfdNr, erg.RobName, erg.Ipar);
  else 
    fprintf(fid, '%% Engineering Solution; see eval_existing_design.m\n');
  end

end
% fprintf(fid, '\\hline\n');
fclose(fid);
fprintf('Tabelle nach %s geschrieben\n', results_latex_file);
end % i_gf
end % i_dh
