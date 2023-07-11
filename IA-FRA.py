import serial
import time
import numpy
import csv
import matplotlib.pyplot as plt

inst=serial.Serial("COM40",115200)
Start_Freq=1    ##kHz unit
Stop_Freq=50    ##kHz unit
Step_Freq=1   ##kHz unit
Ampl=1000       ##mV unit
Wait=0.01       ##sec
Integ=5
Vg=1    ##0=x1, 1=x2, 2=x4, 3=x8
Ig=2    ##0=x10, 1=x100, 2=x1000, 3=x10000

step=int((Stop_Freq-Start_Freq)/Step_Freq)
data=numpy.zeros((step,3))
for i in range(step):
    freq=Start_Freq+i*Step_Freq
    buf=f'{int(freq*1000):08}'+f'{Ampl:04}'+f'{Integ:03}'+f'{Ig:01}'+f'{Vg:01}'
    inst.write(buf.encode())
    time.sleep(Wait)
    buf=inst.read(12)
    re=float(buf)
    buf=inst.read(12)
    im=float(buf)
    ABS=numpy.sqrt(re*re+im*im)
    DEG=numpy.degrees(numpy.arctan2(im,re))
    print('Freq=',freq,'kHz, |Z|=',ABS,'ohm, Ang(Z)=',DEG,'deg.')
    data[i][0]=int(freq*1000)
    data[i][1]=ABS
    data[i][2]=DEG
    
with open('data.csv','w',newline="") as f:
    writer=csv.writer(f)
    writer.writerows(data)

plt.subplot(1,2,1)
plt.plot(data[:,0],data[:,1])
plt.subplot(1,2,2)
plt.plot(data[:,0],data[:,2])
plt.show()
