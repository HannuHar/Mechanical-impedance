# Add motulator into PATH
import os
import sys
current_dir = os.getcwd()
motulator_path = os.path.join(current_dir, 'motulator')
sys.path.append(motulator_path)


# Motulator
import motulator as mt
# Delete imported modules
del sys, os

from im_drive_wo_mech import InductionMotorDriveWoMech

from time import time
import numpy as np
import matplotlib.pyplot as plt
import copy
import multiprocessing as mp
import scipy.io as io


def plot_injection(sim_wo_mech):
    width = 252
    inches_per_pt = 1 / 72.27
    golden_ratio = (5**.5 - 1) / 2
    fig_width = width * inches_per_pt
    fig_height = fig_width * golden_ratio
    sim.ctrl.post_process()
    sim.mdl.post_process()
    fig, (ax1, ax2) = plt.subplots(2, 1)
    fig.set_size_inches(fig_width,fig_height)
    ax1.plot(sim_wo_mech.mdl.data.t+sim.mdl.data.t[-1]-9.5,(sim_wo_mech.mdl.data.w_M),color="blue") #
    ax1.plot(sim.mdl.data.t-9.5,(sim.mdl.data.w_M),color="blue")
    ax1.set_ylim([np.min(sim_wo_mech.mdl.data.w_M)-2,np.max(sim_wo_mech.mdl.data.w_M)+2])
    ax1.set_ylabel(r"$\omega_\mathrm{M}$ [rad/s]")
    
    ax2.plot(sim_wo_mech.mdl.data.t+sim.mdl.data.t[-1]-9.5,(sim_wo_mech.mdl.data.tau_M),label="Injection",color="blue") #
    ax2.plot(sim.mdl.data.t-9.5,(sim.mdl.data.tau_M),label="Ramp",color="blue")
    ax2.sharex(ax1)
    ax2.set_xlim([0,2.5])
    ax2.set_ylim([np.min(sim_wo_mech.mdl.data.tau_M)-5,np.max(sim_wo_mech.mdl.data.tau_M)+5])
    ax2.set_ylabel(r"$\tau_\mathrm{M}$ [Nm]")
    ax2.set_xlim([0,2.5])
    ax2.set_ylim([np.min(sim_wo_mech.mdl.data.tau_M)-5,np.max(sim_wo_mech.mdl.data.tau_M)+5])
    ax2.set_xlabel(r"$t$ [s]")
    
    plt.tight_layout()
    # plt.savefig("test" + ".pdf", dpi=1000,bbox_inches='tight')
    plt.show()

def dft(t,x,f0):
    n = t.size
    k = np.arange(n)
    w = np.exp(-2j*np.pi*f0*k/n)
    X = np.dot(w,x)/n
    return X

def motormodel_2_2kW(control):
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
    if control == "VHz":
        raise NotImplementedError
    elif control == "open_VHz":
        ctrl = mt.InductionMotorVHzCtrl(
            mt.InductionMotorVHzCtrlPars(R_s=0, R_R=0, k_u=0, k_w=0))
    elif control == "observer_VHz":
        ctrl = mt.InductionMotorVHzObsCtrl(
            mt.InductionMotorObsVHzCtrlPars(slip_compensation=False))
    else:
        raise NameError
    
    return (base,mdl,ctrl)

def motormodel_45kW(control):
    base = mt.BaseValues(
    U_nom=400,  # Line-line rms voltage
    I_nom=81,  # Rms current
    f_nom=50,  # Frequency
    tau_nom=291,  # Torque
    P_nom=45e3,  # Power
    p=2)
    motor = mt.InductionMotorInvGamma(R_s=60e-3, R_R=30e-3, L_sgm=2.2e-3, L_M=24.5e-3, p=2)
    mech = mt.Mechanics(J=.49)  # Mechanics model for initial simulation
    conv = mt.Inverter(u_dc=540)  # Inverter model
    mdl = mt.InductionMotorDrive(motor, mech, conv)
    # Controller
    if control == "VHz":
        raise NotImplementedError
    elif control == "open_VHz":
        ctrl = mt.InductionMotorVHzCtrl(
            mt.InductionMotorVHzCtrlPars(R_s=0, R_R=0, k_u=0, k_w=0))
    elif control == "observer_VHz":
        ctrl = mt.InductionMotorVHzObsCtrl(
            mt.InductionMotorObsVHzCtrlPars(slip_compensation=False,
                                            T_s=250e-6,
                                            R_s=60e-3, R_R=30e-3, L_sgm=2.2e-3, L_M=24.5e-3, p=2,
                                            psi_s_nom=base.psi,i_s_max=3*np.sqrt(2)*81,
                                            k_tau=0.5))
    else:
        raise NameError
    
    return (base,mdl,ctrl)


def identification(i,f_vib:float=0, M_vib:float=0, plot_sim=False):
    # Amout of cycles
    x = 50
    # Simulation stop time
    t_stop = x/f_vib

    # Copy inital states
    motor = copy.deepcopy(sim.mdl.motor)
    conv = copy.deepcopy(sim.mdl.conv)
    ctrl = copy.deepcopy(sim.ctrl)
    # Motor model with signal injection
    mdl = InductionMotorDriveWoMech(motor=motor, conv=conv, 
                                    w_M0=sim.mdl.mech.w_M0, 
                                    f_vib=f_vib,M_vib=M_vib)    
    sim_wo_mech = mt.Simulation(mdl, ctrl, pwm=False, delay=1)
    sim_wo_mech.delay.data =sim.delay.data
    
    sim_wo_mech.simulate(t_stop=t_stop)
    sim_wo_mech.ctrl.post_process()
    sim_wo_mech.mdl.post_process()

    if plot_sim:
        plot_injection(sim_wo_mech=sim_wo_mech)
    
    # Sampling time
    ts = sim_wo_mech.mdl.data.t[1]-sim_wo_mech.mdl.data.t[0]
    # How many samples is one cycle of f_vib
    N = 1/(f_vib*ts)
    # Start and stop index for dft
    t_start_dft = int(np.floor((x-30)*N))
    t_stop_dft = int(np.ceil((x-2)*N))
    # Taking data for dft
    t = sim_wo_mech.mdl.data.t[t_start_dft:t_stop_dft]
    w_M = sim_wo_mech.mdl.data.w_M[t_start_dft:t_stop_dft]
    tau_M = sim_wo_mech.mdl.data.tau_M[t_start_dft:t_stop_dft]
    
    w_M_dft = dft(t,w_M,28)
    tau_M_dft = dft(t,tau_M,28)

    
    return [i,f_vib,w_M_dft,tau_M_dft]

def multi(M_vib=1,asyncmp=True):
    t_start = time()
    
    res = list()

    # Frequency range
    freq = np.linspace(10,100,100)
    
    # data collection function error printing for async multiprocessing
    def collect_result(result):
        res.append(result)
    def custom_error_callback(error):
        print(f'Got error: {error}')
    
    if asyncmp:
        # Multiprocessing
        pool = mp.Pool(mp.cpu_count())
        for i, f_vib in enumerate(freq):
            M = M_vib*f_vib/10
            pool.apply_async(identification, args=[i,f_vib,M], error_callback=custom_error_callback,callback=collect_result) 

        pool.close()
        pool.join()
    
        #sort data
        res.sort(key=lambda x: x[0])
    else:
        for i, f_vib in enumerate(freq):
            # Amplitude is directly proportional to excitation frequency
            M = M_vib*f_vib/10
            result = identification(i=i,f_vib=f_vib,M_vib=M_vib,plot_sim=False)
            collect_result(result=result)

    output = np.vstack(res)
    f = np.real(output[:,1])
    w_M = output[:,2]
    tau_M = output[:,3]
    mdic = {"f":f,"tau_M":tau_M,"w_M":w_M}
    io.savemat(folder+operating_point, mdic) 
    print('\nExecution time: {:.2f} s'.format((time() - t_start)))

    resp = (tau_M)/(w_M)/2
    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.plot(f,np.abs(resp),label="Identified",color="blue")
    ax2.plot(f, np.angle(-resp, deg=True),label="Identified",color="blue")
    plt.show()


control = "open_VHz"
motor_power = "45kW"
folder = "matfiles/" + motor_power+ "/" + control + "/"
operating_point = "w_08_tau_08_test.mat"

# Simulates the motor to operating point which is copied for signal injection
base, mdl, ctrl = motormodel_45kW(control)
sim = mt.Simulation(mdl, ctrl, pwm=False, delay=1)
times = np.array([0, 0.2, 1.8])
values = np.array([0, 0, 1])*base.w*0.8
ctrl.w_m_ref = mt.Sequence(times, values)
times = np.array([0,2, 4])
values = np.array([0,0, 1])*base.tau_nom*0.8
mdl.mech.tau_L_t = mt.Sequence(times, values)
sim.simulate(t_stop=10)

if __name__ == "__main__":
    # identification(i=1,f_vib=10,M_vib=1,plot_sim=True)
    multi(M_vib=1)


