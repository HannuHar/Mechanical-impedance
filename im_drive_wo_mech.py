import numpy as np

from motulator.helpers import abc2complex, Bunch
from motulator.model.im_drive import InductionMotorDrive

class InductionMotorDriveWoMech(InductionMotorDrive):
    """
    Continuous-time model for an induction motor drive.

    This interconnects the subsystems of an induction motor drive and provides
    an interface to the solver. More complicated systems could be modeled using
    a similar template.

    Parameters
    ----------
    motor : InductionMotor | InductionMotorSaturated
        Induction motor model.
    mech : Mechanics
        Mechanics model.
    conv : Inverter
        Inverter model.

    """

    def __init__(self, motor=None, conv=None):
        self.motor = motor
        self.conv = conv
        self.t0 = 0  # Initial time
        # Store the solution in these lists
        self.data = Bunch()  # Stores the solution data
        self.data.t, self.data.q = [], []
        self.data.psi_ss, self.data.psi_rs = [], []
        self.w_M0 = 0
        

    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 4
            Initial values of the state variables.

        """
        x0 = [
            self.motor.psi_ss0,
            self.motor.psi_rs0,
        ]
        return x0

    def set_initial_values(self, t0, x0):
        """
        Set the initial values.

        Parameters
        ----------
        x0 : complex ndarray
            Initial values of the state variables.

        """
        self.t0 = t0
        self.motor.psi_ss0 = x0[0]
        self.motor.psi_rs0 = x0[1]

    def f(self, t, x):
        """
        Compute the complete state derivative list for the solver.

        Parameters
        ----------
        t : float
            Time.
        x : complex ndarray
            State vector.

        Returns
        -------
        complex list
            State derivatives.

        """
        # Unpack the states
        # print("w_M0: " + str(self.w_M0))
        psi_ss, psi_rs = x
        w_M = self.w_M0 + 0*np.sin(2*np.pi*1*t)
        # Interconnections: outputs for computing the state derivatives
        u_ss = self.conv.ac_voltage(self.conv.q, self.conv.u_dc0)
        # State derivatives plus the outputs for interconnections
        motor_f, _, tau_M = self.motor.f(psi_ss, psi_rs, u_ss, w_M)
        
        # List of state derivatives
        return motor_f 

    def save(self, sol):
        """
        Save the solution.

        Parameters
        ----------
        sol : Bunch object
            Solution from the solver.

        """
        self.data.t.extend(sol.t)
        self.data.q.extend(sol.q)
        self.data.psi_ss.extend(sol.y[0])
        self.data.psi_rs.extend(sol.y[1])
        

    def post_process(self):
        """Transform the lists to the ndarray format and post-process them."""
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])

        # Some useful variables
        self.data.i_ss, _, self.data.tau_M = self.motor.magnetic(
            self.data.psi_ss, self.data.psi_rs)
        
        self.data.w_M = self.w_M0 + 0*np.sin(2*np.pi*10*self.data.t)
        
        # self.data.u_ss = self.conv.ac_voltage(self.data.q, self.conv.u_dc0)

        # # Compute the inverse-Γ rotor flux
        # try:
        #     # Saturable stator inductance
        #     L_s = self.motor.L_s(np.abs(self.data.psi_ss))
        # except TypeError:
        #     # Constant stator inductance
        #     L_s = self.motor.L_s
        # gamma = L_s/(L_s + self.motor.L_ell)  # Magnetic coupling factor
        # self.data.psi_Rs = gamma*self.data.psi_rs

