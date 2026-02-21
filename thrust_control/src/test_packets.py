import struct
import time
import serial
from packets import ThrustPacket

# Create a serial connection to simulate the write/read test
ser = serial.Serial(port='/dev/serial0', baudrate=9600, timeout=1)

# Ensure UART is open
if not ser.is_open:
    ser.open()

def test_packet_write():
    # Create a ThrustPacket instance
    test_packet = ThrustPacket(device_id=1, message_id=123, data=[10, 20, 30, 40, 50, 60, 70, 80], crc=9999)

    # Pack the data into binary format
    packed_data = test_packet.pack()

    # Print the packed data for verification
    print("Packed Data to be written:", list(packed_data))

    # Write the packed data to the serial port
    ser.write(packed_data)
    print("Data written to UART successfully")

    # Optionally, add a delay to ensure data has time to be written (especially if reading back is needed)
    time.sleep(0.1)

    # Read back data (if loopback is possible) - simulate reading from UART
    # You might need a loopback cable or an actual receiver to test this
    ser.flushInput()  # Clear any old data
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)  # Read any available data
        print("Data Read from UART:", list(response))
    else:
        print("No data read from UART.")

    # Close the serial port after test
    ser.close()

# Run the test function
if __name__ == "__main__":
    test_packet_write()
