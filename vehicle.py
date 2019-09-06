"""
    Vehicle Behavior
"""


# ===============================================================================
# Imports
# ===============================================================================

from itertools import count
from typing import Optional

# ===============================================================================
# Constants
# ===============================================================================

U_I = 25
W_I = 6.25  # 5  # 6.25
K_X = 0.16  # 0.2  # 0.16
DT = 1 / (W_I * K_X)
T_E = 0.2

A_MAX = 0.5
A_MIN = -0.5

C_1 = 0.5  # Speed difference coefficient
C_2 = 0.5  # Spacing coefficient
C_3 = 0.5  # Tampere coefficient

# ===============================================================================
# Clases
# ===============================================================================


class VehicleTampere(object):
    """ 
        This data implements the Car Following Law. 
        
        To initialize a vehicle
        
        VehicleTampere(x0,v0,c1,c2,c3)
        
    """

    idx = count(0)  # Vehicle ID

    def __init__(
        self,
        init_pos: float,
        init_spd: float,
        c_1: float = C_1,
        c_2: float = C_2,
        c_3: float = C_3,
        veh_lead=None,
    ) -> None:
        """ 
            Initialization of vehicle state
        """
        # Veh info
        self.idx = next(self.__class__.idx)

        # Vehicle parametes
        self.c_1, self.c_2, self.c_3 = c_1, c_2, c_3

        # Veh state description

        # x: position,
        # x_t: past_position
        # v: speed,
        # v_t: past_speed
        # a: acceleration,
        # a_t: past_acceleration

        self.x_t = init_pos
        self.v_t = init_spd
        self.a_t = 0.0

        # Control acceleration (leader only)
        self.a = 0.0

        # Vehicle leader definition
        self._veh_lead = veh_lead

        self.control = 0.0

    @classmethod
    def reset(cls) -> None:
        """
            This is a reset vehicle id.
        """
        cls.idx = count(0)

    @property
    def u(self) -> float:
        """
            Free flow speed
        """
        return U_I

    @property
    def w(self) -> float:
        """
            Shockwave speed
        """
        return W_I

    @property
    def k_x(self) -> float:
        """
            Jam density
        """
        return K_X

    @property
    def s0(self) -> float:
        """
            Minimum spacing
        """
        return 1 / self.k_x

    @property
    def veh_lead(self) -> "VehicleTampere":
        """
            Retrieve the pointer towards this vehicle's leader
        """
        return self._veh_lead

    def set_leader(self, veh_lead) -> None:
        """
            Set the leader of a vehicle 
        """
        self._veh_lead = veh_lead

    @property
    def dv(self) -> float:
        """ 
            Determine current delta of speed
        """
        if self.veh_lead:
            return self.veh_lead.v_t - self.v_t
        return 0

    @property
    def v(self) -> float:
        """
            Dynamic equation speed
        """
        return self.v_t + self.a * DT

    @property
    def x(self) -> float:
        """
            Dynamic equation position 
        """
        return self.x_t + self.v * DT  # Check carefully

    @property
    def s(self) -> float:
        """
            Determine current spacing (X_n-1 - X_n)
        """
        if self.veh_lead:
            return self.veh_lead.x_t - self.x_t
        return 0

    @property
    def s_d(self) -> float:
        """
            Determine desired spacing  (d + gamma * v )
        """
        return self.s0 + 1 / (self.w * self.k_x) * self.v_t

    def cong_acc(self) -> float:
        """
            Breaking term  c_1 (D V) + c_2 (s - s_d)
        """
        return self.c_1 * self.dv + self.c_2 * (self.s - self.s_d)

    def free_acc(self, v_d: float = U_I) -> float:
        """
            Acceleration term (Tampere) c_3 (v_d - v)
        """
        return self.c_3 * (v_d - self.v_t)

    def car_following(self, v_d: float) -> None:
        """ 
            Acceleration car following 
            
            Note: 
                if leader 
                    min(cong_acc, free_acc) -> Tampere
                else 
                    manual acceleration

        """
        if self.idx != 0:
            self.a = min(self.cong_acc(), self.free_acc(v_d))  # Car following
        else:
            self.a = self.control  # Leader vehicle 2nd order

    def shift_state(self) -> None:
        """
            Shift state
        """
        self.x_t = self.x
        self.v_t = self.v
        self.a_t = self.a

    def step_evolution(self, v_d: float, control: float = 0) -> None:
        """
            Use this method to a single step in the simulation
        """
        self.shift_state()  # x_{k-1} = x{k} move info from last time step into current
        self.control = control  # Update control
        self.car_following(v_d)  # Update acceleration
