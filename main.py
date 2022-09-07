# from asyncio import format_helpers
# import numpy as np
# import yaml
# import matplotlib as mpl
# import matplotlib.pyplot as plt
# mpl.use('WebAgg')

# from double_pendulum.model.symbolic_plant import SymbolicDoublePendulum
# from double_pendulum.utils.csv_trajectory import load_trajectory
# from double_pendulum.model.model_parameters import model_parameters
# from double_pendulum.utils.csv_trajectory import save_trajectory, load_trajectory
# from double_pendulum.utils.plotting import plot_timeseries

# from tvlqr_drake import TVLQRController
# from simulation import DrakeStepSimulator

# # Nominal trajectory csv from ilqr, not undersampled
# csv_path = "nominalTrajectory.csv" 

# ## Controller initialization
# yaml_path = "CMA-ES_design1st.yml"
# par_dict = yaml.safe_load(open(yaml_path, 'r'))
# Q = np.diag([par_dict['q11'], par_dict['q22'], par_dict['q33'], par_dict['q44']])
# R = np.eye(1)*par_dict['r11']
# Qf = np.copy(Q)

# controller = TVLQRController(csv_path, Q, R, Qf, robot_model, context, torque_limit= [0.0, 0.0])
