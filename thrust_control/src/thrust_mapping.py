import numpy as np

SCALE = 0.0254

THRUST_MAX = 3.71
THRUST_MIN = -2.9

X_COMP = np.sin(7 * np.pi / 18)
Y_COMP = np.cos(7 * np.pi / 18)
is_inverted = [True,False,True,True,False,False,True,False]
class ThrustMapper:
    def __init__(self):
        
        self.com = np.array([0.0, 0.0, 0.2]) * SCALE
        self.location_frame_absolute = np.matrix([
                      [8, 7.25, 4],  # Thruster 1
                      [8, -7.25, -4],  # Thruster 2
                      [8, 7.25, -4],  # Thruster 3
                      [8, -7.25, 4],  # Thruster 4
                      [-8, -7.25, -4],  # Thruster 5
                      [-8, -7.25, 4],  # Thruster 6
                      [-8, 7.25, 4],  # Thruster 7
                      [-8, 7.25, -4]])  * SCALE  # Thruster 8
        alpha = 30 * np.pi / 180.0
        beta = 25 * np.pi / 180.0
        x_comp = np.cos(alpha) * np.cos(beta)
        y_comp = np.sin(alpha) * np.cos(beta)
        z_comp = np.sin(beta)


        self.direction = np.matrix([
                       [ -x_comp, -y_comp, -z_comp],  # Thruster 1
                       [x_comp, y_comp, z_comp],  # Thruster 2
                       [-x_comp,  y_comp, -z_comp],  # Thruster 3
                       [ x_comp,  -y_comp, z_comp],  # Thruster 4
                       [ x_comp, y_comp,  -z_comp],  # Thruster 5
                       [x_comp, -y_comp,  -z_comp],  # Thruster 6
                       [-x_comp,  -y_comp,  z_comp],  # Thruster 7
                       [ -x_comp,  y_comp,  z_comp]  # Thruster 8
                       ])

        self.location = self.change_origin(0, 0, 0)
        self.torque = self.torque_values()
        self.thruster_force_map = self.thruster_force_map_values()

        self.fine = True
        self.multiplier = 1.041
    
    def set_fine(self, nfine):
        self.fine = nfine

    def set_multiplier(self, nmul):
        self.multiplier = nmul

    def change_origin(self, x, y, z):
        return self.location_frame_absolute - self.com + np.array([x, y, z]) * SCALE

    def torque_values(self):
        return np.cross(self.location, self.direction)

    def thruster_force_map_values(self):
        assert len(self.direction) == len(self.torque), 'Initialize direction and torque first!'
        return np.concatenate((np.transpose(self.direction), np.transpose(self.torque)))

    def thruster_output(self, desired_force):
        if not np.array_equal(desired_force, np.zeros(6)):
            if np.linalg.norm(desired_force) > 1.5:
                desired_force /= np.linalg.norm(desired_force)
                desired_force *= 1.5

            output_needed = np.transpose(np.array((desired_force,), dtype=np.float))
            psuedo_inv = np.linalg.pinv(self.thruster_force_map)
            force = np.matmul(psuedo_inv, output_needed)
            if self.fine:
                force *= self.multiplier #1.041  # 3.7657  # Thrust envelop inscribed sphere radius
            else:
                scale_max = abs(THRUST_MAX / max(force))
                scale_min = abs(THRUST_MIN / min(force))
                force *= min(scale_max, scale_min)
            for i in range(0,7):
                if(is_inverted[i]):
                    force[i] *= -1.0
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
    tm = ThrustMapper()    # for i in range(100):
    desired_thrust_final = [0.1, 0, 0.0, 0, 0, 0]  # X Y Z Ro Pi Ya    
    # oneiteration = True
    pwm_values = tm.thruster_output(desired_thrust_final)
    result = np.matmul(tm.thrusterForceMap, pwm_values)
    print(list(np.around(np.array(pwm_values), 2)))
    print(list(np.around(np.array(result), 2)))
