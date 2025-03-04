% Teste den Status von P3PRRRR4V1G4P2A1 in der Datenbank

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
addpath(fullfile(fileparts(which('structgeomsynth_path_init.m')),'struktsynth_par'));

settings = struct( ...
  'EE_FG', logical([1 1 1 0 0 0]), ... % 3T0R
  'whitelist_SerialKin', {{'S5PRRRR4', 'S5PRRRR4V1'}}, ...
  'base_couplings', 4, ... % siehe ParRob/align_base_coupling
  ...'plf_couplings', 2, ... % siehe ParRob/align_platform_coupling
  'dryrun', false, ...
  'parcomp_structsynth', false, ...
  'parcomp_mexcompile', false, ...
  'max_actuation_idx', 1); 
  
parroblib_add_robots_symact(settings);