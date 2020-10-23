#!/usr/bin/env python

import numpy as np

class QuinticPolynomial:

    def __init__(self, init_pos, init_vel, init_accel, final_pos, final_vel, final_accel, dist):

        # Derived coefficients
        self.a_0 = init_pos
        self.a_1 = init_vel
        self.a_2 = init_accel / 2.0

        # Solve the linear equation (Ax = B)
        A = np.array([dist ** 3,       dist ** 4,        dist ** 5], 
                     [3 * dist ** 2,   4 * dist ** 3,    5 * dist ** 4],
                     [6 * dist,        12 * dist ** 2,   20 * dist ** 3])

        B = np.array([final_pos - self.a_0 - self.a_1 * dist - self.a_2 * dist ** 2, final_vel - self.a_1 - init_accel * dist, final_accel - init_accel])

        x = np.linalg.solve(A, B)

        self.a_3 = x[0]
        self.a_4 = x[1]
        self.a_5 = x[2]

    def calc_point(self, s):

        xs = self.a0 + self.a1 * s + self.a2 * s ** 2 + self.a3 * s ** 3 + self.a4 * s ** 4 + self.a5 * s ** 5

        return xs

    def calc_first_derivative(self, s):

        xs = self.a1 + 2 * self.a2 * s + 3 * self.a3 * s ** 2 + 4 * self.a4 * s ** 3 + 5 * self.a5 * s ** 4

        return xs

    def calc_second_derivative(self, s):

        xs = 2 * self.a2 + 6 * self.a3 * s + 12 * self.a4 * s ** 2 + 20 * self.a5 * s ** 3

        return xs

    def calc_third_derivative(self, s):

        xs = 6 * self.a3 + 24 * self.a4 * s + 60 * self.a5 * s ** 2

        return xs

class CubicPolynomial:

    def __init__(self, init_pos, init_vel, init_accel):
        
        # Derived coefficients
        self.d = init_pos
        self.c = init_vel
        self.b = init_accel / 2.0

def quintic_polynomial_planner(x_i, y_i, yaw_i, v_i, a_i=0.0, x_f, y_f, yaw_f, v_f, a_f=0.0, ds):
    
    # Calculate the velocity boundary conditions based on vehicle's orientation
    v_xi = v_i * np.cos(yaw_i)
    v_xf = v_f * np.cos(yaw_f)
    v_yi = v_i * np.sin(yaw_i)
    v_yf = v_f * np.sin(yaw_f)

    # Calculate the acceleration boundary conditions based on vehicle's orientation
    a_xi = a_i * np.cos(yaw_i)
    a_xf = a_f * np.cos(yaw_f)
    a_yi = a_i * np.sin(yaw_i)
    a_yf = a_f * np.sin(yaw_f)

    for S in np.arange(0.0, 100.0, 5.0):
        # Initialise the class
        xqp = QuinticPolynomial(x_i, v_xi, a_xi, x_f, v_xf, a_xf, max_accel, max_jerk, S)
        yqp = QuinticPolynomial(y_i, v_yi, a_yi, y_f, v_yf, a_yf, S)

        # Instantiate/clear the arrays
        x = []
        y = []
        v = []
        a = []
        j = []
        yaw = []

        for s in np.arange(0.0, S + ds, ds):
            # Solve for position
            x.append(xqp.calc_point(s))
            y.append(yqp.calc_point(s))

            # Solve for velocity
            v_x = xqp.calc_first_derivative(s)
            v_y = yqp.calc_first_derivative(s)
            v.append(np.hypot(v_x, v_y))

            # Solve for orientation
            yaw.append(np.arctan2(vy, vx))

            # Solve for acceleration
            a_x = xqp.calc_second_derivative(s)
            a_y = yqp.calc_second_derivative(s)

            if len(v) >= 2 and v[-1] - v[-2] < 0.0:
                a.append(-1.0 * np.hypot(a_x, a_y))

            else:
                a.append(np.hypot(a_x, a_y))
        
            # Solve for jerk
            j_x = xqp.calc_third_derivative(s)
            j_y = yqp.calc_third_derivative(s)
            
            if len(a) >= 2 and a[-1] - a[-2] < 0.0:
                j.append(-1 * np.hypot(j_x, j_y))

            else:
                j.append(np.hypot(j_x, j_y))

        if max([abs(i) for i in a]) <= max_accel and max([abs(i) for i in j]) <= max_jerk:
            break

    return x, y, v, a, j, yaw

def cubic_polynomial_planner():
    pass

def main():

    print("This script is a library and is not meant to be executed.")

if __name__ == '__main__':
    main()