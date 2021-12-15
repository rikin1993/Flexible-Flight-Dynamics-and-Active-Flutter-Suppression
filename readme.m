Please cite the following when using this code 

1. Gupta, Rikin. "Incorporating Flight Dynamics and Control Criteria in Aircraft Design Optimization." PhD diss., Virginia Tech, 2020.

2. Gupta, Rikin, Wei Zhao, and Rakesh K. Kapania. "Controllability Gramian as Control Design Objective in Aircraft Structural Design Optimization." AIAA Journal 58, no. 7 (2020): 3199-3220.

3. Gupta, Rikin, Wei Zhao, Rakesh K. Kapania, and David K. Schmidt. "Incorporating Flight Dynamics and Control Criteria into MDAO of Composite Aircraft." In AIAA SciTech 2020 Forum, p. 1966. 2020.

4. Gupta, Rikin, Nathan J. Love, Rakesh K. Kapania, and David Schmidt. "Development of Longitudinal Flight Dynamics Analysis Framework with Controllability and Observability Metrics." In 2018 Multidisciplinary Analysis and Optimization Conference, p. 3425. 2018.

% This code is developed with collaboration between Virginia Tech and Schmidt and Associates (DKS)
% There are two main files 
%%
% 1. ASEModel_VLM_maewing1_main.m
% This main function assembles the flight dynamics model using mAEWIng1 FEM
% model for four differnt configurations are stored in AD_models folder
%
% 1. mAEWIng1 - case 1
% 2. SRWCBF (Swpet Rectangular Wing + centerbody and flap) - less panels/more panels - case 2
% 3. SRWCBFW (Swpet Rectangular Wing + centerbody and flap and winglets)-
% less panels - case 3
% 4. SRWCBFW (Swpet Rectangular Wing + centerbody and flap and winglets) -
% more panels - case 4
%
% It uses three different type of FEM Models of mAEWing1 - FEMv2.1/
% FEMv5.1/ FEMV5.2
%
%%%%%%%%%%%%% INPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aerodynamics geometry from TORNADO for longitudinal (results, lattice,
% ref,geo) and lateral - directional (results_LD, lattice, ref, geo)
% 
% Structrual model with FEM and Mode Shapes as two structures 
% FEM - points, rigid mass matrix etc...
% They are stored in FEM_models folder
%
% ModeShape - Modeshapes, natural frequencies 
%
% cases -  [total of 4 cases 
%            1 - mAEWing1 A/D;   
%            2 - SWRWCBF (more panels and less) 
%            3 - SRWCBFW (both longitudinal and lateral directional) 
%            4 - SRWCBFW (refined aeromesh)]
% flutter:   flag if we want to run flutter (only for cases=1)
% trim:      flag if we want to run trim and generate step responses (only for cases=1)
% plot_flag: flag if we want to plot mode shapes, spline, etc...
% plot_lift_dist: flag if we want to plot lift distribution
%
%  OUTPUTS 
% Outputs the folder name with the  - cases (name of the aerodynamics
% model) and structural model being analyzed (FEMv)
%
% example: cases =1 (mAEWing1) FEMv=21 
% output folder name = mAEWing1_FEMv21
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% 2. ASEModel_VLM_RW_SRW_SRWCB_SRWCBF.m
% This main function assembles the flight dynamics model using 3 mass FEM
% model for four differnt configurations are stored in AD_models folder
% 1. RW - FEMV1
% 2. RW with dihedral ( Rectangular ) - FEMv1.1
% 3. SRW  (Swpet Rectangular Wing)- FEMv2
% 4. SRW with dihedral - FEMv2.1 
% 5. SRWCB  (Swpet Rectangular Wing + centerbody) - FEMv3
% 6. SRWCB with dihedral - FEMv3.1
% 
%
%%%%%%%%%%%%% INPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aerodynamics geometry from TORNADO for longitudinal (results, lattice,
% ref,geo) and lateral - directional (results_LD, lattice, ref, geo)
% 
% Structrual model with FEM and Mode Shapes as two structures 
% FEM - points, rigid mass matrix etc...
% ModeShape - Modeshapes, natural frequencies 
%
% FEMv: six cases 
%            1 - RW 
%            1.1 - RW with dihedral 
%            2 - SRW (no dihedral) 
%            2.1 - SRW with dihedral 
%            3 - SRWCB - NO dihedral 
%            3.1 - SRWCB (dihedral)]
% Uses same structural model defined for each of the case separately
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%% AERODYNAMICS MODELS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All the aerodynamics models are stored in AD_models folder and they are
%            1 - mAEWing1 A/D;   
%             'mAEW1_915_morepanels_0-Cx.mat';
%             'mAEW1_915_morepanels_LDv1_0-Cx.mat';

%            2 - SWRWCBF (more panels and less) 
%             'flatwing_demo_swept_dihed0_CB_CBflap-Cx.mat'; %  SWRWCBF (less panels)
%             'flatwing_demo_swept_dihed0_CB_CBflap_morepanels-Cx.mat';  %  SWRWCBF (more panels )

%            3 - SRWCBFW (both longitudinal and lateral directional) 
%            'flatwing_demo_swept_dihed0_CB_CBflap_winglet-Cx.mat'; % 3 - SRWCBFW (lateral directional)
%            'flatwing_demo_swept_dihed0_CB_CBflap_winglet_LD-Cx.mat'; % 3 - SRWCBFW (longitudinal)

%            4 - SRWCBFW (refined aeromesh)]
%            'flatwing_demo_swept_dihed0_CB_CBflap_winglet_morepanels-Cx.mat'; - SRWCBFW (lateral directional
%            'flatwing_demo_swept_dihed0_CB_CBflap_winglet_morepanels_LD-Cx.mat'; - SRWCBFW (longitudinal)

%            5 - RW - with and without dihedral
%            'flatwing_demo_d0-Cx.mat'; % RW no dihedral file 
%            'flatwing_demo_d1-Cx.mat';% RW with 5 dihedral file (flatwing_demo_d1-Cx) 8 degree (flatwing_demo_d2-Cx) 13 degree (flatwing_demo_d3-Cx)  

%            6 - SRW - with and without dihedral
%             'flatwing_demo_swept_s3_ND-Cx.mat'; % SRW no dihedral 
%             'flatwing_demo_swept_dihed-Cx.mat'; % SRW 5 deg dihedral 

%            7 - SRWCB - with and without dihedral
%             'flatwing_demo_swept_dihed0_CB-Cx.mat'; % SRWCB no dihedral 
%             'flatwing_demo_swept_dihed5_CB-Cx.mat'; % SRWCB 5 degree dihedral 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%% STRUCTURAL MODELS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Structrual model with FEM and Mode Shapes as two structures 
% FEM - points, rigid mass matrix etc...
% They are stored in FEM_models folder
% There are three main structural models for mAEWing1 that were used 
%   1. FEMv2.1 -     load('mAEWing1_FEMv2p1_ModeInfo_updated_nodecords.mat');
%   2. FEMv5.1 -     load('mAEWing1_Geri_WS4_FEMv5p1_modeinfo.mat');
%                    load('mAEWing1_Geri_WS4_FEMv5p1_modeinfo_v2.mat');
%   3. FEMv5.2 -     load('mAEWing1_Geri_WS4_FEMv5p2_modeinfo.mat');
%                    load('mAEWing1_Geri_WS4_FEMv5p1_modeinfo_v2.mat');
%   4. Lumped mass models were used for simple examples which are defined
%   in the cases itself 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%%%%%%%%%%%%%%%%%%SUBROUTIENS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All of the codes are stored in the subroutines folder and is kept in
% subfolders inside it based on its type. There are a total of 8 subfolders
% and they contain the following codes:
%%
% 1. aerodynamics subfolder : 
%    aeroinfo.m:  This function is used for generating aerodynamic grid points, collocation 
% points, quater chord points and performing appropriate coordinate 
% transformation for winglets and dihedral to convert into local coordinate
% frame of reference
%    circinfo.m: This function is used for calcauating circulation due to perturbation due 
% to rigid dofs
%    calalpha_dist.m: This function is used for plotting lift distribution
%    for mAEWing1 aircraft ONLY!!!!!
%    exec_AD_model.m: executing cases that need to be executed for selecting the
% aerodynamics geometry
%    geninfo.m: This function is used for calcualting genreal geometry information for aircraft
% using VLM information 
%    geninfono_LD.m: This function is used for calcualting genreal geometry information for aircraft
% using VLM information 
%
%%
% 2. structures subfolder
%    exec_struc_model.m: This function is used for loading strucutural information - strucutral grid, mode shapes,
% FEMversion etc....
% These function are used for generating structural grid points, mode shapes,
% natural frequencies, etc for the speciefic FEM Models
%    strucinfov21.m: FEMv2.1
%    strucinfov51.m: FEMv5.1
%    strucinfov52.m: FEMv5.2
%
%%
% 3. aeroelastic subfolder:
%    aeforces.m: This function is used for calculating aeroelastic panel forces and moments 
% generalized modal forces, and generalized forces due to rigid DOF
%    aeroelast_coeff.m: This function is used for calculating coefficients for aeroelastic panel forces and moments 
% generalized modal forces, and generalized forces due to rigid DOF
%
%%
% 4. flutter subfolder:
%    flutterassebmly.m: This function is used for generating flutter model for given flight
% velocities 
%
%%
% 5. plant subfolder:
%    plantassembly.m:  This function is used for generating plant model for given flight
% velocity and detemrining response to given control input 
%
%%
% 6. trim subfolder:
%    trimanalysis.m:  This function is used for generating trim model for given flight
% velocity and detemrining deformed profile and modal coordinates and static -
% elastically adjusted model 
%    mean2global_transformv2.m: This function is used for transforming
%    mean-axis deformation to global reference frame
% 
%%
% 7. spline subfolder
%    splineinfo.m: This function is used for performing spline based on different cases and 
% different FEMverison being investigated 
% 
%   These are the list of different spline functions to do splining for
%   different aircraft configurations being studied
%    calc_TaSp_Three.m
%    calc_TSps_Two.m
%    spline_grid.m
%    spline_gridv2.m
%    spline_grid_v21.m
%    spline_gridx.m
%    spline_grid_simple_Adi.m
%    spline_grid_simple_Adi_SRW.m
%    spline_grid_simple_Adi_swept_RWv2.m
%    spline_grid_simple_Adi_swept_RWv22.m
%    spline_grid.m
%    spline_grid.m
%   
%%
% 8. Output subfolder
%    gen_plots.m: code for plotting -  aerodynamics grid mode shapes, structural grid mode shapes
% , Aeroidynamics, structural grid, and spline grid , Aerodynamics grid , Structural grids
%
% These codes are storing the forces,moments and their coefficients in the
% table form so that it becomes easy to review them
%    table_aecoeff.m
%    table_aeforces.m
%    table_panel_forces.m
%
%%
% Based on
% Hybrid Panel Method Approach for Aeroservoelastic Modeling via VLM
% Written by 
% Rikin Gupta
% Virginia Tech 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
