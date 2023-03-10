import scipy.io as io
import matplotlib.pyplot as plt
import numpy as np
width = 426.79
inches_per_pt = 1 / 72.27
golden_ratio = (5**.5 - 1) / 2
fig_width = width * inches_per_pt
fig_height = fig_width * golden_ratio
# my_dpi = 140
# plt.rcParams['text.latex.preamble']=[r"\usepackage{lmodern}"]
#Options
params = {'text.usetex' : True,
          'font.size' : 11,
          'font.family' : 'lmodern',
          'text.latex.unicode': True,
          }
control = "observer_VHz"
motor_power = "45kW"
folder = "matfiles/" + motor_power+ "/" + control + "/"
# analytical = "FRF"
file = "w_025_tau_0"
file2 = "w_08_tau_08"




mat = io.loadmat(folder + file + ".mat")
f = mat["f"].T
tau_M = mat["tau_M"].T
w_M = mat["w_M"].T

resp = (tau_M)/(w_M)/2
mat2 = io.loadmat(folder + file2 + ".mat")
f2 = mat2["f"].T
tau_M2 = mat2["tau_M"].T
w_M2 = mat2["w_M"].T

resp2 = (tau_M2)/(w_M2)/2


# w = io.loadmat(folder + analytical + "_w.mat")["w"].T
# H = io.loadmat(folder + analytical + "_H.mat")["H"].T

data_amp = dict(xlabel=r"$f$ [Hz]", ylabel=r"Amplitude [Nm/(rad/s)]", xlim=[0,2e2],ylim=0)
data_ang = dict(xlabel=r"$f$ [Hz]", ylabel=r"Phase [deg]", xlim=[0,2e2],ylim=[-90,0],yticks=[0,-30,-60,-90, -120])#, xlim=[0,1e2], ylim=-90,yticks=[90,60,30,0,-30,-60,-90]

# data_amp = dict(xlabel=r"$f$ [Hz]", ylabel=r"Amplitude [Nm/(rad/s)]", xlim=[0,2e2])
# data_ang = dict(xlabel=r"$f$ [Hz]", ylabel=r"Phase [deg]", xlim=[0,2e2])
legend_pars = dict(frameon=True,facecolor="white",edgecolor="black",framealpha=1)

# with plt.style.context(['science', 'ieee', 'no-latex']):


fig, (ax1, ax2) = plt.subplots(2, 1)
# fig.set_size_inches(fig_width,fig_height)
# # Comparison plots
# ax1.plot(f,np.abs(resp),label="Identified",color="blue")
# ax1.plot(w[1:]/(2*np.pi),np.abs(H[1:]),label="Analytical",color="red",linestyle="--")
# ax2.plot(f, np.angle(-resp, deg=True),label="Identified",color="blue")
# ax2.plot(w[1:]/(2*np.pi),np.angle(H[1:], deg=True),label="Analytical",color="red",linestyle="--")

# Different operating poits
ax1.plot(f,np.abs(resp),label=r"$\omega_\mathrm{M}=0.25$ [pu], $\tau_\mathrm{M} = 0$ [pu]",color="blue")
ax1.plot(f2,np.abs(resp2),label=r"$\omega_\mathrm{M}=0.8$ [pu], $\tau_\mathrm{M} = 0.8$ [pu]",color="red")
ax2.plot(f, np.angle(-resp, deg=True),label=r"$\omega_\mathrm{M}=0.25$ [pu], $\tau_\mathrm{M} = 0$ [pu]",color="blue")
ax2.plot(f2, np.angle(-resp2, deg=True),label=r"$\omega_\mathrm{M}=0.8$ [pu], $\tau_\mathrm{M} = 0.8$ [pu]",color="red")

ax1.loglog()
# ax1.plot(f,np.imag(resp)*f*2*np.pi,label="from speed",color="green",linestyle="--")
# ax1.plot(f,-np.imag(resp_the)/(2*np.pi*f),label="imag",color="blue",linestyle="--")
# ax1.loglog()
ax2.semilogx()


# stiffness and damping from taum -> w_M (resp), taum -> theta_M (resp_the)
# ax1.plot(f,np.imag(resp)*f,label="from \omega_M",color="blue")
# ax1.plot(f,-np.real(resp_the),label="from \thata_m",color="red",linestyle="--")
# ax2.plot(f, -np.real(resp),label="from \omega_M",color="blue")
# ax2.plot(f,-np.imag(resp_the)/(f),label="from \thata_m",color="red",linestyle="--")
# ax2.plot(f,np.abs(resp),label="from speed",color="green",linestyle="--")


# Styling
ax1.legend(frameon=True,facecolor="white",edgecolor="black",framealpha=1)
ax1.set(**data_amp)
ax1.grid(True)
ax2.legend(frameon=True,facecolor="white",edgecolor="black",framealpha=1) #,fontsize=8
ax2.set(**data_ang)
ax2.grid(True)
# plt.tight_layout()
# plt.savefig("figures/"+ control + "_" + motor_power+ "10xsampling" + ".pdf", dpi=1000,bbox_inches='tight')


# plt.scatter(np.real(-resp),np.imag(-resp))
# plt.grid(True)
plt.show()
