# Reads voltage readings from an Arduino attached to a voltage divider
# Uses pySerial
#
# Works with this Arduino sketch:
# https://create.arduino.cc/editor/pmalmsten/37c15e26-baab-44e9-b115-264694254335/preview

import serial, time

r1_kOhm = 15
r2_kOhm = 5.6

voltage_divider_in_from_out_factor = (r1_kOhm + r2_kOhm) / r2_kOhm
adc_sample_to_volts = 5 / 1023
adc_sample_to_scaled_volts = adc_sample_to_volts * voltage_divider_in_from_out_factor

port = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=0.1)

first_timestamp = None

print("Waiting for Arduino to finish resetting...")
time.sleep(4)
port.reset_input_buffer()
port.readline()  # trash one line to sync up with incoming data

while True:
    try:
        while port.read(1) != '!'.encode("UTF-8"):  # Seek to next start line marker
            pass
        port.read(1)  # Throw away following space
        line = port.readline()  # Read to end of line

        lineStr = line.decode("UTF-8")
        timestamp, adcSample = lineStr.split(" ")

        timestamp = int(timestamp)
        adcSample = int(adcSample)

        if first_timestamp is None:
            first_timestamp = timestamp

        delta_timestamp = timestamp - first_timestamp
        adc_volts = adcSample * adc_sample_to_scaled_volts

        print("{} {}".format(timestamp, adc_volts))
    except KeyboardInterrupt:
        break
    except Exception:
        pass