# Dimensional Synthesis of Parallel Robots Using Bilevel Optimization for Design Optimization and Resolution of Functional Redundancy

Author: Moritz Schappler

This code is supplemental material to the following [publication](https://doi.org/10.3390/robotics14030029) to MDPI Robotics:

```bib
@Article{Schappler2025,
  author       = {Schappler, Moritz},
  title        = {Dimensional Synthesis of Parallel Robots Using Bilevel Optimization for Design Optimization and Resolution of Functional Redundancy},
  number       = {3},
  volume       = {14},
  date         = {2025},
  doi          = {10.3390/robotics14030029},
  groups       = {Maßsynthese},
  journaltitle = {Robotics},
  publisher    = {Multidisciplinary Digital Publishing Institute},
}
```

## Contents and Useage

This repository contains Matlab scripts and Latex code to reproduce results and figures of the paper. Before being able to run the Matlab scripts, the steps from the [prerequesites section](#prerequisites) have to be performed.


To reproduce the case studies from the article, the resulting data can be downloaded from the data repository [for Section 4 (naval testbed)](https://doi.org/10.25835/00p7zgvk) and [for Section 5 (handling robot)](https://doi.org/10.25835/76g8d6a8).

## Reproduce the Results of the Case Studies

This section explains how to generate the results documented in the paper out of existing or newly generated data. Each case study (Section 4 and 5) has a dedicated directory `naval_testbed` and `handling_robot`. However, the scripts are similar.  
The results have to be post-processed for both case studies:
* `eval_figures_pareto.m`: Assemble all Pareto fronts for the individual robots
* `eval_figures_pareto_groups.m`: Group the robots to a smaller set (by combining their base-coupling alignments). This also creates some figures of the paper
* `select_eval_robot_examples.m`: Select the specific robot structures from the Pareto front for the detailed view.

The downloaded data can be copied to any directory. The path to that directory is given by a path function: `naval_testbed/lufi_dimsynth_data_dir.m` and `handling_robot/HandlingRobot_dimsynth_data_dir.m`. These files do not exist in the repository and have to be manually created by copying and renaming the template file in the directory and placing the path to the respective data directory inside it.

### Specific Scripts for the First Case Study (Naval Testbed)

The script `run_evaluation_naval_testbed.m` runs all scripts of this section (takes a few minutes). This can be used as a test to see if the figures are generated like in the paper.

The following scripts are located in `naval_testbed`:
* `lufi_dimsynth_data_dir.m`: Has to be created, see above
* `eval_existing_design.m`: Evaluate the manual design with the synthesis framework (required for Pareto figures)
* `eval_figures_pareto_groups.m`: Creates the following figures of the paper by using the corresponding figure setting from the list in the variable `usr_fig_all`
  * Figure 14: materialstresscolldist_34joints
  * Figure A1: materialstresscolldist_56joints
  * Figure 17: power_vs_mass, power_vs_coll
  * Figure A1: linkdiam_vs_coll
  * Figure A3: motordiagram_prismatic
  * Figure A4: motordiagram_revolute
  * preprocessed data for Figure A6: motordiagram_{revolute/prismatic}_{ga/pso}
* `results_tables_latex.m`: This creates the Tables A1-A4 in the appendix by writing single tex files to `paper/Tables`.
* `markers.m`: This creates the marker image files to be used in the robot figures (based on `select_eval_robot_examples.m`)
* `robot_images.m`: Robot sketches for Figures 15, A7, A8, and A9 (based on `select_eval_robot_examples.m`).

Further scripts are located in `paper/Figures`:
* `lufipkm_alignment_compare_RUS_UPS.m`: Figure 16
* `lufipkm_pareto_3obj_combined.m`: Figure 17 (based on figures created by  `eval_figures_pareto_groups.m`)
* `lufipkm_compare_pso_vs_ga.m`: Figure A5
* `calctime_vs_fval.m`: Figure A6

### Specific Scripts for the Second Case Study (Handling Robot)

The script `run_evaluation_handling_robot.m` runs all scripts of this section (takes a few minutes).

The following scripts are located in `handling_robot`:
* `HandlingRobot_dimsynth_data_dir.m`: Has to be created, see above
* `eval_existing_design.m`: Evaluate the results of the reference paper with the synthesis framework for later comparison (Fig. 5.20a). This creates the images for Figure A16.
* `eval_figures_pareto_groups.m`: different settings in variable `usr_fig_all`
  * Figure 20: default_prismatic, default_revolute
  * Figure A15: onlypoints_prismatic
* `trajectory.m`: Figure 21a
* `markers.m`: This creates the marker image files to be used in the robot figures (based on `select_eval_robot_examples.m`)
* `robot_images.m`: Robot sketches for Figures 19, A10-A14 (based on `select_eval_robot_examples.m`).
* `box_plots_figures.m`: Figure A17
* `results_tables_latex.m`: This creates the Tables A5-A8 in the appendix by writing single tex files to `paper/Tables`.

Further scripts are located in `paper/Figures`:
* `handlingpkm_perfmaps.m`: Figure 21b and A18-A23

The evaluation of functional redundancy (Figure 22) is performed with the scripts in the directory `handling_robot\taskred_eval`:
* `cds_taskred_eval_path_config.m`: For setting the results and data directory for the following scripts
* `cds_taskred_eval_example.m`: Run this script to perform the evaluation for Figure 22
* `cds_taskred_eval_example_cluster_start.m` (optional): To run the previous script on a computing cluster
* `cds_taskred_eval_check_case.m`: Creates Figure 22a from the data generated above
* `cds_taskred_eval_example_diss_figure.m`: Creates Figure 22b,c from the data generated above
* `merge_statistics_tables.m` (optional): For merging results in case the computation was interupted. Has to be adapted manually.



### Run the Dimensional Synthesis for the Case Studies


The scripts to run the dimensional synthesis again are located in the sub-directories `dimsynth` of the case studies.

Naval testbed:
* `config_lufi_pkm.m`: Main script to run the synthesis
* `adapt_settings.m`: Some settings like computation time are defined here
* `lufi_synth_traj.m`: Trajectory
* `lufi_synth_env.m`: Environment (e.g., installation space)

handling robot:
* `dimsynth_PrauseChaCor2015_example.m`

Manual adaptions have to be made to the start script:
* change name of the optimization
* adjust settings, number of repetitions

## Prerequisites <a name="prerequisites">

For the Matlab scripts to work, the following steps have to be performed:

1. Install Matlab (tested with R2024a)
2. Set up the Matlab mex compiler, if not already done
3. Set up the path to the results for this paper by copying `HandlingRobot_dimsynth_data_dir.m.template` and `lufi_dimsynth_data_dir.m.template` without the template suffix to the same location and by changing the content of the file to point to the stored results of the synthesis (see above).
4. Download Matlab dependencies:
    * [Toolbox for geometry functions](https://github.com/SchapplM/robotics-dep-geometry) (tested with rev. c4af53d; 2021-10-26)
    * [Toolbox for trajectory functions](https://github.com/SchapplM/robotics-dep-traj) (tested with rev. 42c8302, 2022-02-09)
    * [Collection of external dependencies](https://github.com/SchapplM/robotics-dep-ext) (tested with rev. f3a14e5, 2024-01-05)
    * [Toolbox for various Matlab functions](https://github.com/SchapplM/matlab_toolbox) (tested with rev. 723498b 2025-03-02)
    * [Robotics toolbox](https://github.com/SchapplM/robotics-toolbox) (tested with rev. 8ca9cf8, 2024-11-01)
    * [Serial robot model database](https://github.com/SchapplM/robsynth-serroblib) (tested with rev. 21d23c5a8, 2024-10-18)
    * [Parallel robot model database](https://github.com/SchapplM/robsynth-parroblib) (tested with rev. 01c8d77, 2025-03-03))
    * [Toolbox for symbolic robot model generation](https://github.com/SchapplM/robsynth-modelgen) (tested with rev. 4128c0d, 2024-07-29)
    * [Toolbox for transferring jobs to a computing cluster](https://github.com/SchapplM/matlab_pbs_transfer) (tested with rev. 66c8fc3, 2025-02-18)
    * [Dimensional synthesis toolbox](https://github.com/SchapplM/robsynth-structdimsynth) (tested with rev. 8986421, 2025-03-03)
    * Update the repositories to the given versions. You should first try the more recent version before downgrading to the given versions. It should still work.
    * Short command to download all repositories (run in Git Bash in Windows or in Linux terminal):
      ```Bash
      git clone git@github.com:SchapplM/robotics-dep-geometry.git
      git clone git@github.com:SchapplM/robotics-dep-traj.git
      git clone git@github.com:SchapplM/robotics-dep-ext.git
      git clone git@github.com:SchapplM/matlab_toolbox.git
      git clone git@github.com:SchapplM/robotics-toolbox.git
      git clone git@github.com:SchapplM/robsynth-serroblib.git
      git clone git@github.com:SchapplM/robsynth-parroblib.git
      git clone git@github.com:SchapplM/robsynth-modelgen.git
      git clone git@github.com:SchapplM/matlab_pbs_transfer.git
      git clone git@github.com:SchapplM/robsynth-structdimsynth.git
      ```
5. Run the path initialization scripts (`..._path_init.m`) of all downloaded software repos in Matlab. Put the run commands in the startup.m file of Matlab if used frequently.
