import time
import board
import busio
import adafruit_bno055


class BNO055(object):
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self._sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

        self._data = {
            "euler": {
                # Resolution found from a forum post
                "yaw": 0,  # Rotation about z-axis (vertical) +/- 0.01 degree
                "roll": 0,  # Rotation about y-axis (perpendicular to the pins IMU) +/- 0.01 degree
                "pitch": 0,  # Rotation about x-axis (parallel to the pins of IMU) +/- 0.01 degree
            },
            "quat": {"x": 0, "y": 0, "z": 0, "w": 0},
            "gyro": {
                "x": 0,  # 3e-2 degree/sec
                "y": 0,  # 3e-2 degree/sec
                "z": 0,  # 3e-2 degree/sec
            },
            "acceleration": {
                "x": 0,  # +/- 5e-4 g
                "y": 0,  # +/- 5e-4 g
                "z": 0,  # +/- 5e-4 g
            },
            "linear_acceleration": {
                "x": 0,  # +/- 0.25 m/s^2
                "y": 0,  # +/- 0.25 m/s^2
                "z": 0,  # +/- 0.25 m/s^2
            },
            "temp": 0,  # Good enough
        }

    @property
    def data(self):
        return self._data

    def roll(self):
        return self._data["euler"]["roll"]

    def pitch(self):
        return self._data["euler"]["pitch"]

    def yaw(self):
        return self._data["euler"]["yaw"]

    def gyro_x(self):
        return self._data["gyro"]["x"]

    def gyro_y(self):
        return self._data["gyro"]["y"]

    def gyro_z(self):
        return self._data["gyro"]["z"]

    def quat_x(self):
        return self._data["quat"]["x"]

    def quat_y(self):
        return self._data["quat"]["y"]

    def quat_z(self):
        return self._data["quat"]["z"]

    def quat_w(self):
        return self._data["quat"]["w"]

    def quat_arr(self):
        return [self.quat_x(), self.quat_y(), self.quat_z(), self.quat_w()]

    def acceleration_x(self):
        return self._data["acceleration"]["x"]

    def acceleration_y(self):
        return self._data["acceleration"]["y"]

    def acceleration_z(self):
        return self._data["acceleration"]["z"]

    def linear_acceleration_x(self):
        return self._data["linear_acceleration"]["x"]

    def linear_acceleration_y(self):
        return self._data["linear_acceleration"]["y"]

    def linear_acceleration_z(self):
        return self._data["linear_acceleration"]["z"]

    def update(self):
        euler = self._sensor.euler
        if euler[0] is not None:
            self._data["euler"]["yaw"] = euler[0]
        if euler[1] is not None:
            self._data["euler"]["roll"] = euler[1]
        if euler[2] is not None:
            self._data["euler"]["pitch"] = euler[2]

        quat = self._sensor.quaternion
        if quat[0] is not None:
            self._data["quat"]["x"] = quat[0]
        if quat[1] is not None:
            self._data["quat"]["y"] = quat[1]
        if quat[2] is not None:
            self._data["quat"]["z"] = quat[2]
        if quat[3] is not None:
            self._data["quat"]["w"] = quat[3]

        if euler[0] is not None:
            self._data["euler"]["yaw"] = euler[0]
        if euler[1] is not None:
            self._data["euler"]["roll"] = euler[1]
        if euler[2] is not None:
            self._data["euler"]["pitch"] = euler[2]

        gyro = self._sensor.gyro
        if gyro[0] is not None:
            self._data["gyro"]["x"] = gyro[0]
        if gyro[1] is not None:
            self._data["gyro"]["y"] = gyro[1]
        if gyro[2] is not None:
            self._data["gyro"]["z"] = gyro[2]

        acceleration = self._sensor.acceleration
        if acceleration[0] is not None:
            self._data["acceleration"]["x"] = acceleration[0]
        if acceleration[1] is not None:
            self._data["acceleration"]["y"] = acceleration[1]
        if acceleration[2] is not None:
            self._data["acceleration"]["z"] = acceleration[2]

        linear_accel = self._sensor.linear_acceleration
        if linear_accel[0] is not None:
            self._data["linear_acceleration"]["x"] = linear_accel[0]
        if linear_accel[1] is not None:
            self._data["linear_acceleration"]["y"] = linear_accel[1]
        if linear_accel[2] is not None:
            self._data["linear_acceleration"]["z"] = linear_accel[2]

        temp = self._sensor.temperature
        if temp is not None:
            self._data["temp"] = temp

        return True

    def get_calibration(self):
        return self._sensor.calibration_status()

    # def reset_calibration(self):
    #     cal_array_original = self.get_calibration()
    #     self._bno.set_calibration(cal_array_original);
    #     return cal_array_original

    # def set_calibration(self, data):
    #     self._bno.set_calibration(data)


if __name__ == "__main__":

    def main():
        sensor = BNO055()

        # We must initialize the sensor before reading it
        if not sensor:
            print("Sensor could not be initialized")
            exit(1)

        print(
            "Time \tRoll \tPitch \tYaw \tGyro: \tx \ty \tz \tACC: \tx \ty \tz \tLinear: \tx \ty \tz"
        )

        # Spew readings
        while True:
            if sensor.update():
                print(
                    "%s \t%0.2f \t%0.2f \t%0.2f \t\t%0.2f \t%0.2f \t%0.2f \t\t%0.2f \t%0.2f \t%0.2f \t\t%0.2f \t%0.2f \t%0.2f"
                ) % (
                    time.strftime("%H:%M:%S", time.localtime())
                    + ".%d" % (time.time() % 1 * 1000),
                    sensor.roll(),
                    sensor.pitch(),
                    sensor.yaw(),
                    sensor.gyro_x(),
                    sensor.gyro_y(),
                    sensor.gyro_z(),
                    sensor.acceleration_x(),
                    sensor.acceleration_y(),
                    sensor.acceleration_z(),
                    sensor.linear_acceleration_x(),
                    sensor.linear_acceleration_y(),
                    sensor.linear_acceleration_z(),
                )

                time.sleep(0.005)
            else:
                print("Sensor read failed!")
                exit(1)

    main()
