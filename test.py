# import numpy as np

# # Input time-domain signal
# x = np.array([3, 2, 1, -1, 0, 0, 0, 0], dtype=complex)
# N = len(x)

# # Direct DFT using the formula (for clarity, not using np.fft)
# def dft(x):
#     N = len(x)
#     X = np.zeros(N, dtype=complex)
#     for k in range(N):
#         for n in range(N):
#             X[k] += x[n] * np.exp(-2j * np.pi * k * n / N)
#     return X

# # Direct IDFT using the formula
# def idft(X):
#     N = len(X)
#     x = np.zeros(N, dtype=complex)
#     for n in range(N):
#         for k in range(N):
#             x[n] += X[k] * np.exp(2j * np.pi * k * n / N)
#         x[n] /= N
#     return x

# # Compute X[k]
# X = dft(x)

# # (a) X1[k] = conjugate of X[k]
# X1 = np.conj(X)
# x1 = idft(X1)

# # (b) X2[k] = Re(X[k])
# X2 = np.real(X)
# x2 = idft(X2)

# # (c) X3[k] = (-j)^k * X[k]
# k_vals = np.arange(N)
# X3 = ((-1j) ** k_vals) * X
# x3 = idft(X3)

# # Display results
# print("Original X[k]:", X)
# print("Inverse DFT of X1[k] (conjugate):", x1)
# print("Inverse DFT of X2[k] (real part):", x2)
# print("Inverse DFT of X3[k] ((-j)^k * X[k]):", x3)


import numpy as np

# Define x[n] from Problem 10.1 (length 4)
x = np.array([3, 2, 1, -1, 0, 0, 0, 0], dtype=complex)

# Define h[n] = [1, -1, 0, 0] (zero-padded to length 4)
h = np.array([1, -1, 0, 0, 0, 0, 0, 0], dtype=complex)

# Compute 4-point DFTs
X = np.fft.fft(x, 8)
H = np.fft.fft(h, 8)

# Multiply in frequency domain: Y[k] = X[k] * H[k]
Y = X * H

# Compute inverse DFT to get y[n]
y = np.fft.ifft(Y)

# Round small imaginary parts and real parts for cleaner output
y = np.round(y.real, decimals=6)

# Print results
print("x[n]:", x)
print("h[n]:", h)
print("X[k]:", np.round(X, 3))
print("H[k]:", np.round(H, 3))
print("Y[k]:", np.round(Y, 3))
print("y[n] (convolution result):", y)
