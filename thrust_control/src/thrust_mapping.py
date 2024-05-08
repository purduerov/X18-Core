import numpy as np

SCALE = 0.0254 #converts inches to meters

THRUST_MAX = 3.71 #kg f
THRUST_MIN = -2.92 #kg f


class ThrustMapper:
    def __init__(self):
        self.invert_array = [1, 1, -1, -1, 1, 1, -1, -1] #-1 inverts direction, 1 keeps the same direction
        self.com = np.array([0.0, 0.0, 0.0]) * SCALE
        x_pos = 7.67717 # values in inches
        y_pos = 6.37795 # values in inches
        z_pos = 4.17323 # values in inches
        self.location_frame_absolute = np.matrix([[x_pos, y_pos, z_pos],  # Thruster 1
                                                  [-x_pos, y_pos, z_pos],  # Thruster 2
                                                  [-x_pos, -y_pos, z_pos],  # Thruster 3
                                                  [x_pos, -y_pos, z_pos],  # Thruster 4
                                                  [x_pos, y_pos, -z_pos],  # Thruster 5
                                                  [-x_pos, y_pos, -z_pos],  # Thruster 6
                                                  [-x_pos, -y_pos, -z_pos],  # Thruster 7
                                                  [x_pos, -y_pos, -z_pos]]) * SCALE  # Thruster 8
        alpha = 30 * np.pi / 180.0
        beta = 25 * np.pi / 180.0
        x_comp = np.cos(alpha) * np.cos(beta)
        y_comp = np.sin(alpha) * np.cos(beta)
        z_comp = np.sin(beta)

        self.direction =            [[x_comp, -y_comp, -z_comp],  # Thruster 1
                                    [-x_comp, -y_comp, -z_comp],  # Thruster 2
                                    [-x_comp, y_comp, -z_comp],  # Thruster 3
                                    [x_comp, y_comp, -z_comp],  # Thruster 4
                                    [x_comp, -y_comp, z_comp],  # Thruster 5
                                    [-x_comp, -y_comp, z_comp],  # Thruster 6
                                    [-x_comp, y_comp, z_comp],  # Thruster 7
                                    [x_comp, y_comp, z_comp]]  # Thruster 8
        print(np.matrix(self.direction))
        self.direction = np.matrix([[x * self.invert_array[i] for x in inner]for i, inner in enumerate(self.direction)])
        print()

        print(self.direction)

        self.location = self.change_origin(0, 0, 0)
        self.torque = self.torque_values()
        self.thruster_force_map = self.thruster_force_map_values()

    def change_origin(self, x, y, z):
        return self.location_frame_absolute - self.com + np.array([x, y, z]) * SCALE

    def torque_values(self):
        return np.cross(self.location, self.direction)

    def thruster_force_map_values(self):
        assert len(self.direction) == len(self.torque), 'Initialize direction and torque first!'
        return np.concatenate((np.transpose(self.direction), np.transpose(self.torque)))

    def thruster_output(self, desired_force):
        if not np.array_equal(desired_force, np.zeros(6)):
            
            output_needed = np.transpose(np.array((desired_force,), dtype=np.float))
            psuedo_inv = np.linalg.pinv(self.thruster_force_map)
            force = np.matmul(psuedo_inv, output_needed)

            if max(force) < THRUST_MAX and min(force) > THRUST_MIN:
                pass
            else:
                scale_max = abs(THRUST_MAX / max(force))
                scale_min = abs(THRUST_MIN / min(force))
                force *= min(scale_max, scale_min)

            return np.transpose(force).tolist()[0]
        return np.zeros(8)

    @staticmethod
    def thrust_to_pwm(thrust_val):
        if thrust_val < -.04:
            pwm = 0.018 * (thrust_val ** 3) + 0.117 * (thrust_val ** 2) + 0.4981 * thrust_val - 0.09808
        elif thrust_val > .04:
            pwm = 0.0095 * (thrust_val ** 3) - 0.0783 * (thrust_val ** 2) + 0.4004 * thrust_val + 0.0986
        else:
            pwm = 0.0
        return pwm

    @staticmethod
    def pwm_to_thrust(pwm):
        if pwm < -.1:
            thrust = -.8944 * (pwm ** 3) - 2.971 * (pwm ** 2) + 0.9844 * pwm + .1005
        elif pwm > .1:
            thrust = -1.1095 * (pwm ** 3) + 3.9043 * (pwm ** 2) + 1.1101 * pwm - 0.113
        else:
            thrust = 0.0
        return thrust


if __name__ == '__main__':
    # global oneiteration
    tm = ThrustMapper()  # for i in range(100):
    desired_thrust_final = [1, 0.0, 1, 0.0, 0.0, 0.0]  # X Y Z Ro Pi Ya
    # oneiteration = True
    pwm_values = tm.thruster_output(desired_thrust_final)
    result = np.matmul(tm.thruster_force_map, pwm_values)