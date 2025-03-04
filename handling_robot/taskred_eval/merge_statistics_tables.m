% Kombiniere zwei Ergebnis-Tabellen aus cds_taskred_eval_example.m.
% Notwendig, wenn für eine IK-Methode die Daten neu generiert werden.
% Dann sind NaN-Spalten in der Tabelle.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2025-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

tabfile1='D:\Seafile\imes-projekt-dfg_robotersynthese\06_Publikationen\2025_MDPI_Maßsynthese\Simulationsergebnisse\ARK_3T1R_20230730_full_rep1\taskred_eval_example_result_rob231_dp.csv';
% tabfile2='C:\Users\Schappler_Moritz\Documents\Repo\structgeomsynth_dbg\taskred_eval\results\amunpkm_20230605_3T3R_obj3_colldist_ikobj2_rep2\taskred_eval_example_result_rob1_dp.csv';
tabfile2='D:\Seafile\imes-projekt-dfg_robotersynthese\06_Publikationen\2025_MDPI_Maßsynthese\Simulationsergebnisse\ARK_3T1R_20230730_full_rep1\taskred_eval_example_result_rob231_dp_repro.csv';

TabGes = readtable(tabfile1);
TabImport = readtable(tabfile2);

for i = 1:size(TabImport,1)
  I_ges = strcmp(TabImport.OptName{i}, TabGes.OptName) & ...
          strcmp(TabImport.RobName{i}, TabGes.RobName) & ...
          TabImport.RobNr(i) == TabGes.RobNr & ...
          TabImport.ParetoNr(i) == TabGes.ParetoNr;
  assert(sum(I_ges)==1, 'Index-Fehler');
  % for j = 5:size(TabImport,2)
  vn = TabImport.Properties.VariableNames;
  for j = 5:length(vn)
    ColImport = TabImport.(vn{j});
    if ~isnan(ColImport(i))
      fprintf('Daten gefunden: Pareto %d, %s\n', TabImport.ParetoNr(i), vn{j});
      TabGes(I_ges,j) = TabImport(i,j);
    end
  end
end

writetable(TabGes, tabfile1, 'Delimiter',';');