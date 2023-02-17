# Add motulator into PATH
import os
import sys
current_dir = os.getcwd()
motulator_path = os.path.join(current_dir, 'motulator')
sys.path.append(motulator_path)

import motulator as mt
from im_drive_wo_mech import InductionMotorDriveWoMech
import numpy as np
import matplotlib.pyplot as plt
import copy


def dft(t,x,f0):
    # sampling time
    ts = t[1] - t[0]
    # Lowest frequency
    
    # number of samples
    N = len(x)
    
    w = np.exp(-2j*np.pi*f0*np.arange(N)*ts)
    X0 = (np.dot(w, x))/N
    return f0, X0

base = mt.BaseValues(
    U_nom=400,  # Line-line rms voltage
    I_nom=5,  # Rms current
    f_nom=50,  # Frequency
    tau_nom=14.6,  # Torque
    P_nom=2.2e3,  # Power
    p=2)  

motor = mt.InductionMotorInvGamma(R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, p=2)
mech = mt.Mechanics(J=.025)  # Mechanics model
conv = mt.Inverter(u_dc=540)  # Inverter model
mdl = mt.InductionMotorDrive(motor, mech, conv)

ctrl = mt.InductionMotorVHzCtrl(
    mt.InductionMotorVHzCtrlPars(R_s=0, R_R=0, k_u=0, k_w=0))

# ctrl = mt.InductionMotorVHzObsCtrl(
#     mt.InductionMotorObsVHzCtrlPars(slip_compensation=False))

times = np.array([0, .125, .25])*4
values = np.array([0, 0, 1])*base.w*0.8
ctrl.w_m_ref = mt.Sequence(times, values)


times = np.array([0, .125, .125])*4
values = np.array([0, 0, 1])*base.tau_nom
mdl.mech.tau_L_t = mt.Sequence(times, values)


sim = mt.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=10)

# get motor initial states for signal injection simulations
psi_ss0, psi_rs0, w_M0,_ = copy.deepcopy(sim.mdl.get_initial_values())


times = np.array([0, 1])*5
values = np.array([1, 1])*base.w*0.8
ctrl.w_m_ref = mt.Sequence(times, values)
mdl_wo_mech = InductionMotorDriveWoMech(motor,sim.mdl.conv)
mdl_wo_mech.w_M0 = w_M0
print(sim.mdl.get_initial_values())
sim_wo_mech = mt.Simulation(mdl_wo_mech, copy.deepcopy(sim.ctrl), pwm=False, delay=1)
print(sim_wo_mech.mdl.get_initial_values()+[sim_wo_mech.mdl.w_M0])
print()
sim.ctrl.post_process()
# mt.plot(sim)
sim_wo_mech.simulate(t_stop=4)
sim_wo_mech.ctrl.post_process()
# plt.plot(sim_wo_mech.ctrl.data.t,np.abs(sim_wo_mech.ctrl.data.u_s))
# plt.plot(sim.ctrl.data.t,np.abs(sim.ctrl.data.u_s))
plt.plot(sim_wo_mech.mdl.data.t+sim.mdl.data.t[-1],np.abs(sim_wo_mech.mdl.data.tau_M))
plt.plot(sim.mdl.data.t,np.abs(sim.mdl.data.tau_M))
plt.show()


