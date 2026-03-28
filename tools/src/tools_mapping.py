# tools_mapping.py

class ToolsMapper:
    """
    Maps surface controller inputs (hat switch + button) to PWM values
    for vertical, horizontal, and claw actuators.
    """

    NEUTRAL = 127  # neutral PWM value

    @staticmethod
    def hat_to_pwm(hat_val: int) -> int:
        """
        Convert hat switch value (-1, 0, 1) to PWM (0-255)
        """
        return ToolsMapper.NEUTRAL + hat_val * 127

    def map_inputs(self, hat_x: int, hat_y: int, button_pressed: bool):
        """
        Map hat switch and button to PWM values.
        
        :param hat_x: horizontal hat value (-1 left, 0 neutral, 1 right)
        :param hat_y: vertical hat value (-1 down, 0 neutral, 1 up)
        :param button_pressed: True if button is pressed (claw)
        :return: list of 4 PWM values [vertical, horizontal, claw, reserved]
        """

        # Vertical actuator (UP/DOWN)
        vertical = self.hat_to_pwm(hat_y)

        # Horizontal actuator (LEFT/RIGHT)
        horizontal = self.hat_to_pwm(hat_x)

        # Claw control
        claw = 255 if button_pressed else 127

        # 4th channel reserved (neutral)
        reserved = 127

        return [vertical, horizontal, claw, reserved]