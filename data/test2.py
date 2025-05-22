import pandas as pd
import matplotlib.pyplot as plt

# Load your CSV file (update the path if needed)
df = pd.read_csv("data.csv", header=None, names=["micros", "pressure", "temperature"])

# Convert pressure column to float
df['pressure'] = pd.to_numeric(df['pressure'], errors='coerce')

# Drop rows where pressure could not be converted
df = df.dropna(subset=['pressure'])

# Extract the pressure column and get the decimal part as a string with two decimals
decimal_places = df['pressure'].apply(lambda x: f"{x:.2f}".split('.')[-1])

# Find unique decimal places used
unique_decimals = sorted(decimal_places.unique())

print(f"Unique decimal places used in pressure column (as .XX):")
for dec in unique_decimals:
    print(f".{dec}")

print(f"\nTotal unique decimal places: {len(unique_decimals)}")

# Count occurrences of each decimal place
decimal_counts = decimal_places.value_counts().sort_index()

# Plot bar chart
plt.figure(figsize=(12,6))
decimal_counts.plot(kind='bar')
plt.xlabel("Decimal Place (.XX)")
plt.ylabel("Count")
plt.title("Usage of Each Decimal Place in Pressure Column")
plt.tight_layout()
plt.show()