#!/usr/bin/env python

import numpy as np

class QuinticPolynomial:

    def __init__(self, init_pos, init_vel, init_accel, final_pos, final_vel, final_accel, dist):

        # Derived coefficients
        self.a_0 = init_pos
        self.a_1 = init_vel
        self.a_2 = init_accel / 2.0

        # Solve the linear equation (Ax = B)
        A = np.array([dist ** 3, dist ** 4, dist ** 5], 
                     [3 * dist ** 2, 4 * dist ** 3, 5 * dist ** 4],
                     [6 * dist, 12 * dist ** 2, 20 * dist ** 3])

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

    def __init__(self):
        pass

def quintic_polynomial_planner(x_i, y_i, yaw_i, v_i, a_i, x_f, y_f, yaw_f, v_f, a_f):
    
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

    for S

def cubic_polynomial_planner():
    pass

def main():

    print("This script is a library and is not meant to be executed.")

if __name__ == '__main__':
    main()