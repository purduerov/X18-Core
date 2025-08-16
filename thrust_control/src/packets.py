import struct


class BasePacket:
    """Base structure for all packets"""

    def __init__(self, device_id, message_id, data, crc=0, data_length=8):
        """
        :param device_id: 7-bit unique ID
        :param message_id: Unique per message, rolls over
        :param data: List of bytes (length 7 or 8)
        :param crc: 16-bit CRC (default 0, can be computed later)
        :param data_length: Expected length of data field (7 or 8)
        """

        self.device_id = device_id & 0x7F  # Ensure only 7 bits
        self.message_id = message_id & 0xFFFF
        self.data = data
        self.crc = crc & 0xFF
        self.data_length = data_length

        # Define struct format dynamically
        self.format = f">B H {data_length}B B"  # B = uint8, H = uint16, {data_length}B = data bytes

    def pack(self):
        """Converts the object into a fixed-size binary format"""
        return struct.pack(
            self.format, self.device_id, self.message_id, *self.data, self.crc
        )

    @classmethod
    def unpack(cls, byte_data, data_length):
        expected_size = struct.calcsize(f"B H {data_length}B H")

        unpacked_data = struct.unpack(f"B H {data_length}B H", byte_data)
        return cls(
            device_id=unpacked_data[0] & 0x7F,
            message_id=unpacked_data[1],
            data=list(unpacked_data[2 : data_length + 2]),
            crc=unpacked_data[data_length + 2],
        )

    def __repr__(self):
        return f"{self.__class__.__name__}(DeviceID={self.device_id}, MsgID={self.message_id}, Data={self.data}, CRC={self.crc})"


# Specialized Packets
class ThrustPacket(BasePacket):
    """Packet for Thrust Data (8 bytes)"""

    def __init__(self, device_id, message_id, data, crc=0):
        super().__init__(device_id, message_id, data, crc, data_length=8)


class ESCPacket(BasePacket):
    """Packet for ESC Data (8 bytes)"""

    def __init__(self, device_id, message_id, data, crc=0):
        super().__init__(device_id, message_id, data, crc, data_length=8)


class PowerPacket(BasePacket):
    """Packet for Power Data (8 bytes)"""

    def __init__(self, device_id, message_id, data, crc=0):
        super().__init__(device_id, message_id, data, crc, data_length=8)


class ToolsPacket(BasePacket):
    """Packet for Tools Data (7 bytes)"""

    def __init__(self, device_id, message_id, data, crc=0):
        super().__init__(device_id, message_id, data, crc, data_length=7)
