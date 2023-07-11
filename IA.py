import serial
import numpy

inst=serial.Serial("COM40",115200)
Freq=10  ##kHz unit
Ampl=1000  ##mV unit
Integ=50
Vg=1    ##0=x1, 1=x2, 2=x4, 3=x8
Ig=2    ##0=x10, 1=x100, 2=x1000, 3=x10000

buf=f'{int(Freq*1000):08}'+f'{Ampl:04}'+f'{Integ:03}'+f'{Ig:01}'+f'{Vg:01}'
inst.write(buf.encode())
buf=inst.read(12)
re=float(buf)
buf=inst.read(12)
im=float(buf)
print('Freq=',Freq,'kHz, Ampl=',Ampl,'mV, Integ=',Integ,', Vg=',Vg,', Ig=',Ig)
print('Re(Z)=',re,'ohm, Im(Z)=',im,'ohm\n|Z|=',numpy.sqrt(re*re+im*im),'ohm, Ang(Z)=',numpy.degrees(numpy.arctan2(im,re)),'deg.\n')