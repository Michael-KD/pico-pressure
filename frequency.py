import csv
import os
import json

def calculate_frequency(file_path):
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        timestamps = [int(row[0]) for row in reader]

    if len(timestamps) < 2:
        raise ValueError("Not enough data to calculate frequency")

    time_diffs = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps) - 1)]
    average_time_diff = sum(time_diffs) / len(time_diffs)
    frequency = 1000 / average_time_diff  # Convert milliseconds to seconds

    # Debug prints
    print(f"Timestamps: {timestamps}")
    print(f"Time differences: {time_diffs}")
    print(f"Average time difference: {average_time_diff}")
    print(f"Calculated frequency: {frequency} Hz")

    return frequency

def calculate_file_size_per_line(file_path):
    file_size = os.path.getsize(file_path)
    with open(file_path, 'r') as file:
        line_count = sum(1 for _ in file) - 1  # Subtract 1 for the header row
    
    if line_count == 0:
        raise ValueError("File is empty")
    
    size_per_line = file_size / (line_count + 1)  # Include header row in file size calculation
    return size_per_line

def save_data_to_json(frequency, size_per_line, notes, current, clock_speed):
    data = {
        "frequency": frequency,
        "size_per_line": size_per_line,
        "notes": notes,
        "current": current,
        "clock_speed": clock_speed
    }

    json_file_path = 'data.json'
    if os.path.exists(json_file_path):
        with open(json_file_path, 'r') as json_file:
            existing_data = json.load(json_file)
    else:
        existing_data = []

    existing_data.append(data)

    with open(json_file_path, 'w') as json_file:
        json.dump(existing_data, json_file, indent=4)

if __name__ == "__main__":
    file_path = 'D:\\TEST2.TXT'
    frequency = calculate_frequency(file_path)
    size_per_line = calculate_file_size_per_line(file_path)
    
    notes = input("Enter any general notes: ")
    current = input("Enter the current: ")
    clock_speed = input("Enter the clock speed: ")

    save_data_to_json(frequency, size_per_line, notes, current, clock_speed)
    print(f"Frequency of sensor readings: {frequency} Hz")
    print(f"File size per line: {size_per_line} bytes")