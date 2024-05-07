import ipdb
import logging
import logging
import casadi as cas
import numpy as np
from typing import List, Tuple


class Solver:
    def __init__(self, n: int, min_dist: float=0.2, max_lin_vel: float=0.12, max_ang_vel: float=2.84,
                 max_delta_vel: float=1.5, max_delta_omega: float=np.pi/2,
                 dt: float=0.1, T: int=50, logger=logging.getLogger(__name__)):
        """
        Initialize all the symbolic variables needed
        for solving the mpc

        This is a rough python port of the following repo:
        git@github.com:braca51e/mpc_ros.git

        :param n              : Number of robots
        :param max_lin_vel    : Maximum linear velocity
        :param max_ang_vel    : Maximum angular velocity
        :param max_delta_vel  : Maximum change in velocity the robot is able to do
        :param max_delta_omega: Maximum change in angular velocity
        :param dt             : Time delta between two states
        :param T              : for how many future timestaps to optimize
        """
        self.logger = logger
        self.n = n
        self.dt = dt
        self.T = T
        self.min_dist = min_dist

        self.opti = cas.Opti()

        # define variables (for each robot)
        self.x = [self.opti.variable(3, T+1) for i in range(n)]
        self.u = [self.opti.variable(2, T) for i in range(n)]


        # define parameters
        #self.distances = [self.opti.variable(n,n) for t in range(T)]
        self.p = [self.opti.parameter(3, 1) for i in range(n)]
        self.x_ref = [self.opti.parameter(3, T+1) for i in range(n)]
        self.u_ref = [self.opti.parameter(2, T) for i in range(n)]
        self.Q = self.opti.parameter(3, 3)
        self.R = self.opti.parameter(2, 2)

        # cost
        self.cost = self.cost_function(self.x, self.u, self.x_ref, self.u_ref, self.Q, self.R)

        # tell casadi to minimize cost
        self.opti.minimize(self.cost)

        # define controll horizon
        for i in range(n):
            for t in range(T):
                x_next = self.dynamic_system(self.x[i][:,t], self.u[i][:,t], self.dt)
                self.opti.subject_to(self.x[i][:, t+1] == x_next)

                # constraint for min distance between two robots
                for j in range(i + 1, n):
                    dist_squared = cas.sumsqr(self.x[i][:2, t] - self.x[j][:2, t])
                    self.opti.subject_to(dist_squared >= self.min_dist ** 2)

            # add constraints in change of velocity
            for t in range(T-1):
                dvel = self.u[i][:, t+1] - self.u[i][:, t]

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
                self.opti.bounded(-max_lin_vel, self.u[i][0], max_lin_vel)
            )

            # add constraints for angular velocity
            self.opti.subject_to(
                self.opti.bounded(-max_ang_vel, self.u[i][0], max_ang_vel)
            )

            # subject to initial position
            self.opti.subject_to(self.x[i][:,0] == self.p[i])

            # initial values for u and i
            self.x_init = [cas.repmat([0,0,0], 1, T+1) for i in range(n)]
            self.u_init = [cas.repmat([0,0], 1, T) for i in range(n)]

            # set the initial guess
            self.opti.set_initial(self.x[i], self.x_init[i])
            self.opti.set_initial(self.u[i], self.u_init[i])

        # set paper values
        self.opti.set_value(self.Q, cas.diag(cas.vertcat([1.0, 5.0, 0.5])))
        self.opti.set_value(self.R, cas.diag(cas.vertcat([0.5, 0.25])))

    def cost_function(self, x, u, x_ref, u_ref, Q, R) -> float:
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
        epsilon = 1e-4  # A small value to prevent division by zero
        total_cost = 0
        for i in range(self.n):
            # Compute the state part of the cost
            state_cost = cas.dot(x[i] - x_ref[i], cas.mtimes(Q, x[i] - x_ref[i]))

            # Compute the control part of the cost
            control_cost = cas.dot(u[i], cas.mtimes(R, u[i]))

            # Total cost
            total_cost += (state_cost + control_cost)

            """
            for j in range(i + 1, self.n):
                for t in range(self.T):
                    pos_i = self.x[i][:2, t]
                    pos_j = self.x[j][:2, t]

                    # Compute the Euclidean distance between them
                    distance = cas.sqrt(cas.dot(pos_i - pos_j, pos_i - pos_j))

                    # Add the penalty term to the cost (e.g., inverse of the distance)
                    total_cost += 1 / (distance + epsilon)
            """

        return total_cost

    def dynamic_system(self, x, u, dt):
        """
        Compute the next position for a robot based on
        its state and a controll signal
        """
        x_pos_next = x[0] + dt * u[0] * cas.cos(x[2])
        y_pos_next = x[1] + dt * u[0] * cas.sin(x[2])
        w_next = x[2] + dt * u[1]

        return cas.vertcat(x_pos_next, y_pos_next, w_next)

    def solve(self, current_states: List[Tuple[float, float, float]],
              goal_states: List[Tuple[float, float, float]]) -> List[Tuple[float, float]]:
        """
        Solves the non linear MPU to achieve the most optimal
        controll signal for each robot

        :param current_states : List of current states of the robots (x, y, z)
        :param goal_states    : List of goal states of the robots (x, y, z)

        :return               : List of control signals (v, w)
        """
        if len(current_states) != len(goal_states):
            raise ValueError("Uneven states and goals!")
        elif len(current_states) != self.n:
            raise ValueError("Lenght of states does not equal number of\
                             registered robots")


        for i in range(self.n):
            # set reference values
            self.opti.set_value(self.x_ref[i], cas.repmat(goal_states[i], 1, self.T+1))
            self.opti.set_value(self.u_ref[i], cas.DM.zeros(2, self.T))

            # set initial state
            self.opti.set_value(self.p[i], current_states[i])

        # solve
        solver_opts = {
            'ipopt.print_level': 1,
            'print_time': 0,
            'ipopt.tol': 1e-3
        }

        self.opti.solver('ipopt', solver_opts)
        try:
            solution = self.opti.solve()
        except Exception as e:
            ipdb.set_trace()
            e = e

        for i in range(self.n):
            # warm start for next iteration
            self.x_init[i] = cas.repmat(self.x[i][:,0], 1, self.T+1)
            self.u_init[i] = cas.repmat(self.u[i][:,0], 1, self.T)

        # extract first controll input for each robot
        control_inputs = [solution.value(self.u[i])[:,0] for i in
                          range(self.n)]

        return control_inputs
