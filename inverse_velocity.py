# -*- coding: utf-8 -*-
"""
Created on Tue Nov 22 14:12:02 2022

@author: AKBAR
"""

import numpy as np
import matplotlib.pyplot as plt

def get_jacobian(th):
    J1 = np.matrix([[np.cos(th), 0.], [np.sin(th), 0.], [0., 1]])
    J2 = np.matrix([[r/2., r/2.], [r/L, -r/L]])
    return J1*J2

#robot
r = 0.05 #jari2 roda robot (meter)
L = 0.15 #luas roda robot (meter)
Ts = 0.1 # Time sampling (detik)
lamda = 1. # gain error /menentukan kecepatan motor


x = np.matrix([[1., 1., np.radians(90.)]]).transpose()
x_des = np.matrix([[6., 4., np.radians(120.)]]).transpose()

print("Posisi Awal = ", x, "Arah Hadap = ", np.degrees(x[2,0]))
# 1. Kontrol Arah Hadap Robot terhadap titik tujuan
traj = x_des - x
th_traj = np.arctan2(traj[1,0], traj[0,0])

# variable untuk merekam posisi setiap sampling
x_plot = [x[0,0]]
y_plot = [x[1,0]]
th_plot = [x[2,0]]
#print(x_plot)

for i in range(0,1000):
    J = get_jacobian(x[2,0]) #menghitung nilai jacobian
    Jinv = np.linalg.pinv(J) #invers nilai jacobian
    E = x_des - x # menghitung error
    E[0,0] = 0 #x dianggap 0
    E[1,0] = 0 #y dianggap 0
    E[2,0] = th_traj - x[2,0] 
    if(np.linalg.norm(E) < 0.1):
        break
    qdot = Jinv * lamda * E
    #print(qdot)
    #hitung forward velocity
    xdot = J * qdot
    #update posisi
    x = x + xdot*Ts
    x_plot.append(x[0,0]) # menambahkan data pada list
    y_plot.append(x[1,0])
    th_plot.append(x[2,0])

# 2. Kontrol Posisi Robot terhadap posisi tujuan
for i in range(0, 1000):
    J = get_jacobian(x[2,0]) # menghitung nilai jacobian
    Jinv = np.linalg.pinv(J) # menghitung invers dari jacobian
    E = x_des - x # menghitung error posisi
    E[2,0] = 0. # dianggap arah hadap robot sudah benar
    if(np.linalg.norm(E) < 0.1):
        break
    qdot = Jinv * lamda * E
    #print(qdot)
    # hitung forward velocity kinematic
    xdot = J * qdot
    #update posisi
    x = x + xdot*Ts
    x_plot.append(x[0,0]) # menambahkan data pada list
    y_plot.append(x[1,0])
    th_plot.append(x[2,0])
    
# 3. Arah hadap robot terhadap arah hadap tujuan
for i in range(0, 1000):
    J = get_jacobian(x[2,0]) # menghitung nilai jacobian
    Jinv = np.linalg.pinv(J) # menghitung invers dari jacobian
    E = x_des - x # menghitung error posisi
    E[0,0] = 0. # error pada sumbu x dianggap 0
    E[1,0] = 0. # error pada sumbu y dianggap 0
    if(np.linalg.norm(E) < np.radians(0.0)):
        break
    qdot = Jinv * lamda * E
    #print(qdot)
    # hitung forward velocity kinematic
    xdot = J * qdot
    #update posisi
    x = x + xdot*Ts
    x_plot.append(x[0,0]) # menambahkan data pada list
    y_plot.append(x[1,0])
    th_plot.append(x[2,0])
    
    min_pwm = 0
    max_pwm = 1023

# Linear relationship between qdot and PWM values
    pwm_left = min_pwm + (max_pwm - min_pwm) * (qdot[0,0] - (-100)) / (100 - (-100))
    pwm_right = min_pwm + (max_pwm - min_pwm) * (qdot[1,0] - (-100)) / (255 - (-100))
    
# Ensure that the PWM values are within the allowed range
    pwm_left = max(min_pwm, min(max_pwm, pwm_left))
    pwm_right = max(min_pwm, min(max_pwm, pwm_right))

print("pwmL=",(pwm_left))
print("pwmR=",(pwm_right))

print("Posisi akhir = ", x, "Arah Hadap = ", np.degrees(x[2,0]))
print("Posisi Akhir =", x)
print("Waktu yang dibutuhkan adalah = ", i*Ts)

# visualisasi
plt.figure(0)
plt.plot(x_plot[0], y_plot[0], marker='X', color='b')
for i in range(0, len(x_plot), 7):
    plt.quiver(x_plot[i], y_plot[i], np.cos(th_plot[i]), np.sin(th_plot[i]), scale=0.9, scale_units='xy', angles = 'xy', color="#228B22")
plt.plot(x_plot[-1], y_plot[-1], marker='*', color='#8B0000')
plt.quiver(x_plot[-1], y_plot[-1], np.cos(th_plot[i]), np.sin(th_plot[i]), scale=0.9, scale_units='xy', angles = 'xy', color="#DAA520")
plt.grid()
plt.xlim(0, 8)
plt.ylim(0, 8)
plt.show()

