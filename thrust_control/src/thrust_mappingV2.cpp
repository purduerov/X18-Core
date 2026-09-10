#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>

inline constexpr double SCALE = 0.0254;  // converts inches to meters

inline constexpr double THRUST_MAX = 3.71;   // kg f
inline constexpr double THRUST_MIN = -2.92;  // kg f

inline constexpr double Z_OFFSET = 0.0;

class ThrustMapper {
   public:
    static constexpr int NUM_THRUSTERS = 6;

    using Effort = Eigen::Matrix<double, 6, 1>;
    using Thrust = Eigen::Matrix<double, NUM_THRUSTERS, 1>;
    using ThrustMap = Eigen::Matrix<double, NUM_THRUSTERS, 6>;
    using Pwm = std::array<std::uint8_t, NUM_THRUSTERS>;

    std::array<int, NUM_THRUSTERS> invert_thrust_;
    std::array<Eigen::Vector3d, NUM_THRUSTERS> position_vectors_;
    std::array<Eigen::Vector3d, NUM_THRUSTERS> direction_vectors_;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thrust_map_;

    ThrustMapper()
        : invert_thrust_{1, 1, -1, -1, -1, 1},

          position_vectors_{{{-0.1981, 0.1852, 0.0699 + Z_OFFSET},
                              {0.1981, 0.1852, 0.0699 + Z_OFFSET},
                              {-0.1981, -0.1852, 0.0699 + Z_OFFSET},
                              {0.1981, -0.1852, 0.0699 + Z_OFFSET},
                              {0.2223, 0.0, 0.0},
                              {-0.2223, 0.0, 0.0}}},

          direction_vectors_{{{1.0, 1.0, 0.0},
                               {-1.0, 1.0, 0.0},
                               {1.0, -1.0, 0.0},
                               {-1.0, -1.0, 0.0},
                               {0.0, 0.0, 1.0},
                               {0.0, 0.0, 1.0}}} {
        thrust_map_ = Setup();
    }

    Eigen::Matrix<double, NUM_THRUSTERS, 6> Setup() {
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            // Normalize the direction vector
            direction_vectors_[i].normalize();
        }

        Eigen::Matrix<double, 6, NUM_THRUSTERS> B;
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            B.block<3, 1>(0, i) = direction_vectors_[i];
            B.block<3, 1>(3, i) = position_vectors_[i].cross(direction_vectors_[i]);
        }

        // if the rank is less than 5, there is an issue so we should just return zeros
        // review later when it is 8 thrusters, but for now we will just check for 5
        Eigen::FullPivLU<Eigen::Matrix<double, 6, NUM_THRUSTERS>> lu(B);
        if (lu.rank() < 5) {
            return ThrustMap::Zero();
        }

        return B.completeOrthogonalDecomposition().pseudoInverse();
    }

    Thrust GetThrust(const Effort& effort) const {
        // find thrust_map * effort
        Thrust thrust = thrust_map_ * effort;

        // invert thrusters as desired
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            thrust(i) *= invert_thrust_[i];
        }
        return thrust;
    }

    // return a pwm value between 0 and 255 for each thruster. 127 is no thrust
    Pwm GetPwm(const Effort& effort) const {
        Thrust thrust = GetThrust(effort);

        // clip the thrust to the allowable range
        thrust = thrust.cwiseMax(-THRUST_MAX).cwiseMin(THRUST_MAX);

        // normalize the thrust to 0 to 255
        return ThrustToPwm(thrust);
    }

    // maps a thrust between THRUST_MIN and THRUST_MAX to a pwm between 25 and 230
    static Pwm ThrustToPwm(const Thrust& thrust) {
        Pwm pwm{};
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            double value = std::clamp(thrust(i) / (THRUST_MAX - THRUST_MIN) + 0.5, 0.0, 1.0);
            value = value * 205.0 + 25.0;

            // numpy's int16 cast truncates toward zero, so a plain static_cast matches it
            auto counts = static_cast<std::int16_t>(value);
            counts = std::clamp<std::int16_t>(counts, 25, 230);

            pwm[i] = static_cast<std::uint8_t>(counts);
        }
        return pwm;
    }
};

int main() {
    ThrustMapper tm;

    // suppress scientific notation, matching np.set_printoptions(suppress=True)
    std::cout << std::fixed << std::setprecision(8) << tm.thrust_map_ << '\n';

    ThrustMapper::Effort desired_thrust_final;
    desired_thrust_final << 0.0, 0.0, 10.0, 0.0, 0.0, 0.0;  // X Y Z Ro Pi Ya

    const ThrustMapper::Pwm pwm_values = tm.GetPwm(desired_thrust_final);
    for (const std::uint8_t value : pwm_values) {
        // cast to int, otherwise uint8_t prints as a character
        std::cout << static_cast<int>(value) << ' ';
    }
    std::cout << '\n';

    return 0;
}


        

