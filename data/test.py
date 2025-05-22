import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv("one_temp.csv")
# df = pd.read_csv("data2.csv")

# Remove the first 2 seconds
df = df[df['micros'] > 1_000_000].copy()

# Convert time to ms
df['ms'] = df['micros'] / 1000.0

def plot_limited(df, limit_s):
    # Limit to first N seconds
    df_lim = df[df['micros'] <= (df['micros'].iloc[0] + limit_s * 1_000_000)]
    # Calculate average delay and frequency
    delays = df_lim['micros'].diff().dropna()
    avg_delay = delays.mean() / 1000  # ms
    avg_freq = 1_000_000 / delays.mean() if not delays.empty else 0
    # Calculate RMS and std of pressure
    rms_pressure = (df_lim['pressure']**2).mean()**0.5
    std_pressure = df_lim['pressure'].std()

    # Plot
    plt.figure(figsize=(10,5))
    plt.scatter(df_lim['ms'], df_lim['pressure'])
    plt.xlabel("Time (ms)")
    plt.ylabel("Pressure (mbar)")
    plt.title(f"Pressure vs Time (first {limit_s} seconds)")
    plt.suptitle(
        f"Avg delay: {avg_delay:.2f} ms | Avg freq: {avg_freq:.2f} Hz | RMS: {rms_pressure:.2f} mbar | Std: {std_pressure:.2f} mbar",
        fontsize=10, y=0.92
    )
    plt.tight_layout()
    plt.show()
    return df_lim
# Plot for 10 seconds
plot_limited(df, 10)

# Plot for 3 seconds and save limited data
df_3s = plot_limited(df, 3)

# # Plot for 3 seconds with dotted lines at 1500 and 900 mbar
# plt.figure(figsize=(10,5))
# plt.plot(df_3s['ms'], df_3s['pressure'])
# plt.axhline(1500, color='r', linestyle=':', label='1500 mbar')
# plt.axhline(900, color='g', linestyle=':', label='900 mbar')
# plt.xlabel("Time (ms)")
# plt.ylabel("Pressure (mbar)")
# plt.title("Pressure vs Time (first 3 seconds) with Reference Lines")
# plt.legend()
# plt.tight_layout()
# plt.show()

# # Bar chart (histogram) for pressure distribution in the 10s window
# df_10s = plot_limited(df, 10)  # Ensure df_10s is defined (if not already)
# plt.figure(figsize=(8,5))
# plt.hist(df_10s['pressure'], bins=30, color='skyblue', edgecolor='black')
# plt.xlabel("Pressure (mbar)")
# plt.ylabel("Count")
# plt.title("Pressure Distribution (first 10 seconds)")
# plt.tight_layout()
# plt.show()