import struct
import csv

def binary_to_csv(binary_file, csv_file):
    with open(binary_file, 'rb') as bin_file, open(csv_file, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['Time', 'Pressure', 'Temperature'])

        while True:
            entry = bin_file.read(12)  # Read 12 bytes (4 bytes each for time, pressure, temperature)
            if not entry:
                break
            time, pressure, temp = struct.unpack('Iii', entry)
            csv_writer.writerow([time, pressure / 100, temp / 100])

# Usage
binary_to_csv('TEST4.BIN', 'converted_data.csv')