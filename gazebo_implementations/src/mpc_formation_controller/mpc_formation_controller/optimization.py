import casadi as cas
import numpy as np
from typing import List

class Solver:
    def __init__(self, max_lin_vel: float=0.12, max_ang_vel: float=2.84,
                 max_delta_vel: float=1.5, max_delta_omega: float=np.pi/2,
                 dt: float=0.1, T: int=50):
        """
        Initialize all the symbolic variables needed
        for solving the mpc

        This is a rough python port of the following repo:
        git@github.com:braca51e/mpc_ros.git

        :param max_lin_vel    : Maximum linear velocity
        :param max_ang_vel    : Maximum angular velocity
        :param max_delta_vel  : Maximum change in velocity the robot is able to do
        :param max_delta_omega: Maximum change in angular velocity
        :param dt             : Time delta between two states
        :param T              : for how many future timestaps to optimize
        """
        self.dt = dt
        self.T = T

        self.opti = cas.Opti()

        # define variables
        self.x = self.opti.variable(3, T+1)
        self.u = self.opti.variable(2, T)

        # define parameters
        self.p = self.opti.parameter(3, 1)
        self.x_ref = self.opti.parameter(3, T+1)
        self.u_ref = self.opti.parameter(2, T)
        self.Q = self.opti.parameter(self.x.shape[0], self.x.shape[0])
        self.R = self.opti.parameter(self.u.shape[0], self.u.shape[0])

        # cost
        self.cost = self.cost_function(self.x, self.u, self.x_ref, self.u_ref, self.Q, self.R)

        # tell casadi to minimize cost
        self.opti.minimize(self.cost)

        # define controll horizon
        for t in range(T):
            x_next = self.dynamic_system(self.x[:,t], self.u[:,t], self.dt)
            self.opti.subject_to(self.x[:, t+1] == x_next)

        # add constraints in change of velocity
        for t in range(T-1):
            dvel = self.u[:, t+1] - self.u[:, t]

            # limit maximum delta in velocity
            self.opti.subject_to(
                self.opti.bounded(-max_delta_vel, dvel[0], max_delta_vel)
            )
            # limit maximum delta in angular velocity
            self.opti.subject_to(
                self.opti.bounded(-max_delta_omega, dvel[1], max_delta_omega)
            )

        # add constraints for velocity
        self.opti.subject_to(
            self.opti.bounded(-max_lin_vel, self.u[0], max_lin_vel)
        )

        # add constraints for angular velocity
        self.opti.subject_to(
            self.opti.bounded(-max_ang_vel, self.u[0], max_ang_vel)
        )

        # maximum state values, our arena is theoretically infinite.
        # so these should not be needed

        #self.opti.subject_to(
        #    self.opti.bounded(-2.07, x[0], 2.07)
        #)
        #self.opti.subject_to(
        #    self.opti.bounded(-2.07, x[1], 2.07)
        #)
        #self.opti.subject_to(
        #    self.opti.bounded(-np.pi, x[2], np.pi)
        #)

        # subject to initial position
        self.opti.subject_to(self.x[:,0] == self.p)

        # set paper values
        self.opti.set_value(self.Q, cas.diag(cas.vertcat([1.0, 5.0, 0.5])))
        self.opti.set_value(self.R, cas.diag(cas.vertcat([0.5, 0.25])))

        # initial values for u and i
        self.x_init = cas.repmat([0,0,0], 1, T+1)
        self.u_init = cas.repmat([0,0], 1, T)

        # set the initial guess
        self.opti.set_initial(self.x, self.x_init)
        self.opti.set_initial(self.u, self.u_init)

    def cost_function(self, x, u, x_ref, u_ref, Q, R):
        """
        Compute the cost function.

        Parameters:
            x: CasADi MX vector representing the current state
            u: CasADi MX vector representing the control input
            x_ref: CasADi MX vector representing the reference state
            u_ref: CasADi MX vector representing the reference control input
            Q: CasADi MX matrix representing the state cost matrix
            R: CasADi MX matrix representing the control cost matrix

        Returns:
            cost: CasADi expression representing the cost value
        """
        # Compute the state part of the cost
        state_cost = cas.dot(x - x_ref, cas.mtimes(self.Q, x - x_ref))

        # Compute the control part of the cost
        control_cost = cas.dot(u, cas.mtimes(self.R, u))

        # Total cost
        cost = state_cost + control_cost

        return cost

    def dynamic_system(self, x, u, dt):
        """
        Compute the next position for a robot based on
        its state and a controll signal
        """
        x_pos_next = x[0] + dt * u[0] * cas.cos(x[2])
        y_pos_next = x[1] + dt * u[0] * cas.sin(x[2])
        w_next = x[2] + dt * u[1]

        return cas.vertcat(x_pos_next, y_pos_next, w_next)

    def solve(self, current_state: List[float], goal_state: List[float]) -> List[float]:
        """
        Solves the non linear MPU to achieve the most optimal 
        controll signal for reaching the goal state

        :param current_state : Current state of the robot (x, y, z)
        :param goal_state    : Goal state of the robot (x, y, z)

        :return              : Control signal (v, w)
        """
         # set reference values
        self.opti.set_value(self.x_ref, cas.repmat(goal_state, 1, self.T+1))
        self.opti.set_value(self.u_ref, cas.DM.zeros(2, self.T))

        # set initial state
        self.opti.set_value(self.p, current_state)

        # solve
        solver_opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.opti.solver('ipopt', solver_opts)
        solution = self.opti.solve()

        # warm start for next iteration
        self.x_init = cas.repmat(self.x[:,0], 1, self.T+1)
        self.u_init = cas.repmat(self.u[:,0], 1, self.T)

        # update turtlebot
        control_input = solution.value(self.u)[:,0]

        return control_input
