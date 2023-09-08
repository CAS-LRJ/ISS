import numpy as np
import matplotlib.pyplot as plt

# Define the known points
x = [100, 400]
y = [240, 265]

# Calculate the coefficients. This line answers the initial question. 
coefficients = np.polyfit(x, y, 3)

# Print the findings
print('a =', coefficients[0])
print('b =', coefficients[1])

# Let's compute the values of the line...
polynomial = np.poly1d(coefficients)
x_axis = np.linspace(0,500,100)
y_axis = polynomial(x_axis)

# ...and plot the points and the line
plt.plot(x_axis, y_axis)
plt.plot( x[0], y[0], 'go' )
plt.plot( x[1], y[1], 'go' )
plt.grid('on')
plt.show()