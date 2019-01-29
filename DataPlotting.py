import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import matplotlib.gridspec as gridspec

df=pd.read_csv('data.csv')

#print df
time = df['time']
tauR = df['tau_R']
tauL = df['tau_L']

wR=df['wR']
wL=df['wL']

df['PWMR']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df['pwmR']] 
df['PWML']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df['pwmL']]

PWMR=df['PWMR']
PWML=df['PWML']

e1=df['e_x']
e2=df['e_y']
e3=df['e_theta']
xref=df['x_ref']
yref=df['y_ref']
x=df['x']
y=df['y']
thetaref=df['theta_ref']
theta=df['theta']
uR=df['u_R']
uL=df['u_L']
velV=df['vel_v']
velW=df['vel_w']

#VCV=df['VC_v']
#VCW=df['VC_w']
#VCDotV=df['VCDot_v']
#VCDotW=df['VCDot_w']
#print df.head
# plot
plt.figure()
plt.plot(x,y, 'r-')
plt.plot(xref,yref, 'b-')
#plt.plot(time,e3,'r-')

#plt.figure()
#plt.plot(time,tauL, 'bo')
#plt.plot(time,tauR, 'bo')

#plt.figure()
#plt.plot(time,e1, 'r-')
#plt.plot(time,e2, 'b-')
#plt.plot(time,e3, 'g-')
#plt.plot(time,uR, 'r.')
#plt.plot(time,uL, 'b.')

#plt.figure()
#plt.plot(time,velV, 'r-')
#plt.plot(time,velW, 'b-')

###################################
# Plotting PWM and error in angle #
###################################
fig1, ax1 = plt.subplots()

ax1.set_xlabel('time (s)')
ax1.set_ylabel('PWM')
ax1.plot(time,PWMR, 'b-', label='PWM_R')
ax1.plot(time,PWML, 'r-', label='PWM_L')
ax1.tick_params(axis='y')
ax1.legend(loc='upper left')

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

ax2.set_ylabel('Error in theta')  # we already handled the x-label with ax1
ax2.plot(time,e3, 'g-', label='e_theta')
ax2.tick_params(axis='y', color='g')
ax2.legend(loc='upper right')

fig1.tight_layout()  # otherwise the right y-label is slightly clipped

###############################################################
# Plotting PWM vs time and angular velocity of wheel vs time. #
###############################################################
fig2 = plt.figure(tight_layout=True)
gs = gridspec.GridSpec(2, 1)

ax3 = fig2.add_subplot(gs[0, 0])
ax3.set_xlabel('time (s)')
ax3.set_ylabel('PWM')
ax3.plot(time,PWMR, 'b-', label='PWM_R')
ax3.tick_params(axis='y')
ax3.legend(loc='upper left')

ax4 = ax3.twinx()  # instantiate a second axes that shares the same x-axis
ax4.set_ylabel('Angular Velocity')  # we already handled the x-label with ax1
ax4.plot(time,wR, 'r-', label='w_R')
ax4.tick_params(axis='y', color='g')
ax4.legend(loc='upper right')

ax5 = fig2.add_subplot(gs[1, 0])
ax5.set_xlabel('time (s)')
ax5.set_ylabel('PWM')
ax5.plot(time,PWML, 'b-', label='PWM_L')
ax5.tick_params(axis='y')
ax5.legend(loc='upper left')

ax6 = ax5.twinx()  # instantiate a second axes that shares the same x-axis
ax6.set_ylabel('Angular Velocity')  # we already handled the x-label with ax1
ax6.plot(time,wL, 'r-', label='w_L')
ax6.tick_params(axis='y', color='g')
ax6.legend(loc='upper right')

fig2.tight_layout()  # otherwise the right y-label is slightly clipped
#################################################
# Plotting angular and linear vlocities of bot. #
#################################################
fig3 = plt.figure(tight_layout=True)
gs = gridspec.GridSpec(2, 1)

ax7 = fig3.add_subplot(gs[0, 0])
ax7.set_xlabel('time (s)')
ax7.set_ylabel('Linear Velocitie')
ax7.plot(time,velV, 'b-', label='Lin. Vel.')
ax7.tick_params(axis='y')
ax7.legend(loc='upper left')

ax8 = ax7.twinx()  # instantiate a second axes that shares the same x-axis
ax8.set_ylabel('Reference Velocity')  # we already handled the x-label with ax1
ax8.plot(time,0.1*np.ones_like(velV), 'r-', label='Ref. Vel.')
ax8.tick_params(axis='y', color='g')
ax8.legend(loc='upper right')

ax9 = fig3.add_subplot(gs[1, 0])
ax9.set_xlabel('time (s)')
ax9.set_ylabel('Angular Velocity')
ax9.plot(time,velW, 'b-', label='Ang. Vel.')
ax9.tick_params(axis='y')
ax9.legend(loc='upper left')

ax10 = ax9.twinx()  # instantiate a second axes that shares the same x-axis
ax10.set_ylabel('Reference Velocity')  # we already handled the x-label with ax1
ax10.plot(time,0.1*np.ones_like(velW), 'r-', label='Ref. Vel.')
ax10.tick_params(axis='y', color='g')
ax10.legend(loc='upper right')

fig3.tight_layout()  # otherwise the right y-label is slightly clipped

#################################################
# Plotting VC and VCDot of bot. #
#################################################
#fig4 = plt.figure(tight_layout=True)
#gs = gridspec.GridSpec(2, 1)

#ax11 = fig4.add_subplot(gs[0, 0])
#ax11.set_xlabel('time (s)')
#ax11.set_ylabel('Linear Velocitie')
#ax11.plot(time,velV, 'b-', label='Lin. Vel.')
#ax11.plot(time,VCV,'g-', label='VCV')
#ax11.tick_params(axis='y')
#ax11.legend(loc='upper left')

#ax12 = ax11.twinx()  # instantiate a second axes that shares the same x-axis
#ax12.set_ylabel('VCDotV')  # we already handled the x-label with ax1
#ax12.plot(time,VCDotV, 'r-', label='VCDot_V')
#ax12.tick_params(axis='y', color='g')
#ax12.legend(loc='upper right')

#ax13 = fig4.add_subplot(gs[1, 0])
#ax13.set_xlabel('time (s)')
#ax13.set_ylabel('Angular Velocity')
#ax13.plot(time,VCW,'g-', label='VCW')
#ax13.plot(time,velW, 'b-', label='Ang. Vel.')
#ax13.tick_params(axis='y')
#ax13.legend(loc='upper left')

#ax14 = ax13.twinx()  # instantiate a second axes that shares the same x-axis
#ax14.set_ylabel('VCDotW')  # we already handled the x-label with ax1
#ax14.plot(time,VCDotW, 'r-', label='VCDot_W')
#ax14.tick_params(axis='y', color='g')
#ax14.legend(loc='upper right')

#fig4.tight_layout()  # otherwise the right y-label is slightly clipped

###################################
# Plotting PWM and error in angle #
###################################
fig1, ax1 = plt.subplots()

ax1.set_xlabel('time (s)')
ax1.set_ylabel('PWM')
ax1.plot(time,PWMR, 'b-', label='PWM_R')
ax1.plot(time,PWML, 'r-', label='PWM_L')
ax1.tick_params(axis='y')
ax1.legend(loc='upper left')

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

ax2.set_ylabel('Error in position')  # we already handled the x-label with ax1
ax2.plot(time,e1, 'g-', label='e_x')
ax2.plot(time,e2, 'y-', label='e_y')
ax2.tick_params(axis='y', color='g')
ax2.legend(loc='upper right')

fig1.tight_layout()  # otherwise the right y-label is slightly clipped


plt.show()




