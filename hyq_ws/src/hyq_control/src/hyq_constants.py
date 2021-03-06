#!/usr/bin/env python

lf_haa_bound = [-1.22, 0.43]
lf_hfe_bound = [-0.87, 1.22]
lf_kfe_bound = [-2.44, -0.35]
lf_init=[0.43,1.22,-2.44]
lf_init_position=[0.3735,0.207,-0.6]
lf_shoulder=[0.3735,0.207,0]
#lf_shoulder=[0.,0.,0.]
lf_axis_1 = [1,0,0]
lf_axis_2 = [0,-1,0]
lf_axis_3 = [0,-1,0]

lh_haa_bound = [-1.22, 0.43]
lh_hfe_bound = [-1.22, 0.87]
lh_kfe_bound = [0.35, 2.44]
lh_init=[0.43,-1.22,2.44]
lh_init_position=[-0.3735,0.207,-0.6]
lh_shoulder=[-0.3735,0.207,0]
lh_axis_1 = [1,0,0]
lh_axis_2 = [0,-1,0]
lh_axis_3 = [0,-1,0]

rf_haa_bound = [-1.22, 0.43]
rf_hfe_bound = [-0.87, 1.22]
rf_kfe_bound = [-2.44, -0.35]
rf_init=[0.43,1.22,-2.44]
rf_init_position=[0.3735,-0.207,-0.6]
rf_shoulder=[0.3735,-0.207,0]
rf_axis_1 = [-1,0,0]
rf_axis_2 = [0,-1,0]
rf_axis_3 = [0,-1,0]

rh_haa_bound = [-1.22, 0.43]
rh_hfe_bound = [-1.22, 0.87]
rh_kfe_bound = [0.35, 2.44]
rh_init=[0.43,-1.22,2.44]
rh_init_position=[-0.3735,-0.207,-0.6]
rh_shoulder=[-0.3735,-0.207,0]
rh_axis_1 = [-1,0,0]
rh_axis_2 = [0,-1,0]
rh_axis_3 = [0,-1,0]

shoulder={"lf":lf_shoulder,"lh":lh_shoulder,"rf":rf_shoulder,"rh":rh_shoulder}

axis={'lf_1': lf_axis_1, 'lf_2': lf_axis_2, 'lf_3': lf_axis_3,
      'lh_1': lh_axis_1, 'lh_2': lh_axis_2, 'lh_3': lh_axis_3,
      'rf_1': rf_axis_1, 'rf_2': rf_axis_2, 'rf_3': rf_axis_3,
      'rh_1': rh_axis_1, 'rh_2': rh_axis_2, 'rh_3': rh_axis_3
      }

hip_leg  = 0.08
upper_leg = 0.35
lower_leg = 0.346
foot_leg = -0.02175 * 2

test_joint_lf = [-1.22,     1.22,     -2.44]
test_joint_lh = [-1.22,    -1.22,      2.44]
test_joint_rf = [-1.22,     1.22,     -2.44]
test_joint_rh = [-1.22,    -1.22,      2.44]

test_pose = [0.6496, 0.1482, -0.1900]
