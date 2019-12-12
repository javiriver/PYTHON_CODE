import numpy as np
from scipy import linalg




#m = np.matrix([[1, 2], [3, 4]])

#inversam = linalg.inv(m)
#c = m.dot(inversam)
#t = np.transpose(m)


o = 3 * np.pi/180
f = 42 * np.pi/180
k = 290 * np.pi/180



Rx = np.matrix([[1, 0 , 0], [0, np.cos(o), np.sin(o)], [0, -np.sin(o), np.cos(o)]]) #Matriz de rotacion alrededor del eje X
Ry = np.matrix([[np.cos(f), 0, -np.sin(f)], [0, 1, 0], [np.sin(f), 0, np.cos(f)]])  #Matriz de rotacion alrededor del eje Y
Rz = np.matrix([[np.cos(k), np.sin(k), 0], [-np.sin(k), np.cos(k), 0], [0, 0, 1]])  #Matriz de rotacion alrededor del eje Z


RR = np.matrix([[np.cos(k)*np.cos(f) + np.sin(k)*np.sin(o)*np.sin(f), np.sin(k)*np.cos(o), -np.cos(k)*np.sin(f) + np.sin(k)*np.sin(o)*np.cos(f)],
                [-np.sin(k)*np.cos(f) + np.cos(k)*np.sin(o)*np.sin(f), np.cos(k)*np.cos(o), np.sin(k)*np.sin(f) + np.cos(k)*np.sin(o)*np.cos(f)],
                [np.cos(o)*np.sin(f), -np.sin(o), np.cos(o)*np.cos(f)]])# Matriz de rotacion componiendo las rotaciones en el orden Y X Z

Rv = Rz * Rx * Ry  # Rv tiene que ser igual a RR



num = np.sin(k)*np.cos(o)
den = np.cos(k)*np.cos(o)

print(num, den)


# Extraccion de o,phi,kappa de la matriz de rotacion RR

omega = - np.arcsin(-np.sin(o))
phi = np.arccos((np.cos(o)*np.cos(f)) / (np.cos(o)))
kappa = np.arctan((np.sin(k)*np.cos(o)) / (np.cos(k)*np.cos(o)))



if np.sin(k)*np.cos(o) > 0 and np.cos(k)*np.cos(o) < 0: kappa = kappa + np.pi
if np.sin(k)*np.cos(o) < 0 and np.cos(k)*np.cos(o) < 0: kappa = kappa + np.pi
if np.sin(k)*np.cos(o) < 0 and np.cos(k)*np.cos(o) > 0: kappa = kappa + 2*np.pi




do = omega - o
df = phi - f
dk = kappa - k




print(do,df,dk)

print(df)



