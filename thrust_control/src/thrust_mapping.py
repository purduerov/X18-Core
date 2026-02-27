import numpy as np

SCALE = 0.0254  # converts inches to meters

THRUST_MAX = 3.71  # kg f
THRUST_MIN = -2.92  # kg f

class ThrustMapper:
    def __init__(self):
        # inverts the thrust of each motor
        self.invert_thrust = np.asarray([
            1, # front left
            1, # front right
            1, # back left 
            1, # back right
            1, # top front
            1 # back front
        ])

        # position of each thruster relative to the center of mass
        self.position_vectors = [
            np.asarray([[-2, 2, 0]]),
            np.asarray([[2, 2, 0]]),
            np.asarray([[-2, -2, 0]]),
            np.asarray([[2, -2, 0]]),
            np.asarray([[2, 0, 0]]),
            np.asarray([[-2, 0, 0]])
        ]

        # unit vectors describing the orientation of each thruster
        self.direction_vectors = [
            np.asarray([[1, 1, 0]]),
            np.asarray([[-1, 1, 0]]),
            np.asarray([[1, -1, 0]]),
            np.asarray([[-1, -1, 0]]),
            np.asarray([[0, 0, 1]]),
            np.asarray([[0, 0, 1]]),
        ]

        # matrix that maps commands to thrust
        self.thrust_map = self._setup()

        
        print(self.thrust_map)

    # sets up the thrust map
    def _setup(self):
        # normalize direction vector just in case
        self.direction_vectors = [v / np.linalg.norm(v) for v in self.direction_vectors]

        # create the top three rows of the allocation matrix B
        # these are responsible for lateral motion
        B_top = np.concatenate(self.direction_vectors, axis=0)
        B_top = B_top.T

        # creat the bottom three rows of B
        # these are responsible for torque
        torque_vectors = [np.cross(b, a) for a, b in zip(self.direction_vectors, self.position_vectors)]
        B_bottom = np.concatenate(torque_vectors, axis=0)
        B_bottom = B_bottom.T

        # combine B_top and B_bottom into B
        B = np.concatenate((B_top, B_bottom), axis=0)

        # if the rank is less than 5, there is an issue so we should just return zeros
        if(np.linalg.matrix_rank(B) < 5):
            return np.zeros((6, 6))

        # compute the pseudoinverse of B, which is the thrust mapping matrix
        B_pi = np.linalg.pinv(B)

        return B_pi
    
    # gets the thrust values for a given effort vector
    def _get_thrust(self, effort):
        # find thrust_map * effort
        effort = np.asarray(effort)
        thrust = self.thrust_map @ (effort.T)
        # invert thrusters as desired
        thrust = thrust * self.invert_thrust
        return thrust
    
    # return a pwm value between 0 and 255 for each thruster. 127 is no thrust
    def get_pwm(self, effort):
        thrust = self._get_thrust(effort)
        # clip the thrust to the allowable range
        thrust = np.clip(thrust, a_min=THRUST_MIN, a_max=THRUST_MAX)
        # normalize the thrust to 0 to 255
        norm_thrust = [self._thrust_to_pwm(t)  for t in thrust]
        return norm_thrust
    
    # maps a thrust between THRUST_MIN and THRUST_MAX to a pwm between 0 and 255
    @staticmethod
    def _thrust_to_pwm(thrust):
        return int((thrust / (THRUST_MAX - THRUST_MIN) + 0.5) * 255)


if __name__ == "__main__":
    # global oneiteration
    tm = ThrustMapper()  # for i in range(100):
    desired_thrust_final = [1, 0.0, 1, 0.0, 0.0, 0.0]  # X Y Z Ro Pi Ya
    # oneiteration = True
    pwm_values = tm.get_pwm(desired_thrust_final)

