import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import matplotlib.gridspec as gridspec

df0=pd.read_csv('VoronoiRuns/data0_2pk_Edge.csv')
df1=pd.read_csv('VoronoiRuns/data1_2pk_Edge.csv')
df2=pd.read_csv('VoronoiRuns/data2_2pk_Edge.csv')
df3=pd.read_csv('VoronoiRuns/data3_2pk_Edge.csv')

#Take time
time0 = df0['time']
time1 = df1['time']
time2 = df2['time']
time3 = df3['time']
#Take Torque
tauR0 = df0['tau_R']
tauL0 = df0['tau_L']
tauR1 = df1['tau_R']
tauL1 = df1['tau_L']
tauR2 = df2['tau_R']
tauL2 = df2['tau_L']
tauR3 = df3['tau_R']
tauL3 = df3['tau_L']
#Take Wheel ang vel
wR0=df0['wR']
wL0=df0['wL']
wR1=df1['wR']
wL1=df1['wL']
wR2=df2['wR']
wL2=df2['wL']
wR3=df3['wR']
wL3=df3['wL']
#Normalize the PWM inputs
df0['PWMR']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df0['pwmR']] 
df0['PWML']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df0['pwmL']]
df1['PWMR']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df1['pwmR']] 
df1['PWML']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df1['pwmL']]
df2['PWMR']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df2['pwmR']] 
df2['PWML']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df2['pwmL']]
df3['PWMR']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df3['pwmR']] 
df3['PWML']=[255 if (x > 255.0) else -255 if	 (x <-255.0) else x.astype(np.int16) for x in df3['pwmL']]
#Take PWMs
PWMR0=df0['PWMR']
PWML0=df0['PWML']
PWMR1=df1['PWMR']
PWML1=df1['PWML']
PWMR2=df2['PWMR']
PWML2=df2['PWML']
PWMR3=df3['PWMR']
PWML3=df3['PWML']
#Take botLocations
x0=df0['x']
y0=df0['y']
x1=df1['x']
y1=df1['y']
x2=df2['x']
y2=df2['y']
x3=df3['x']
y3=df3['y']
#Take orientation
theta0=df0['theta']
theta1=df1['theta']
theta2=df2['theta']
theta3=df3['theta']
#Take velocity
velV0=df0['vel_v']
velW0=df0['vel_w']
velV1=df1['vel_v']
velW1=df1['vel_w']
velV2=df2['vel_v']
velW2=df2['vel_w']
velV3=df3['vel_v']
velW3=df3['vel_w']
#Take Centroid
centroidX0=df0['centroidX']
centroidY0=df0['centroidY']
centroidX1=df1['centroidX']
centroidY1=df1['centroidY']
centroidX2=df2['centroidX']
centroidY2=df2['centroidY']
centroidX3=df3['centroidX']
centroidY3=df3['centroidY']

#Take a_hat
a_hat00=df0['a_hat0']
a_hat01=df0['a_hat1']
a_hat02=df0['a_hat2']
a_hat03=df0['a_hat3']
a_hat04=df0['a_hat4']
a_hat10=df1['a_hat0']
a_hat11=df1['a_hat1']
a_hat12=df1['a_hat2']
a_hat13=df1['a_hat3']
a_hat14=df1['a_hat4']
a_hat20=df2['a_hat0']
a_hat21=df2['a_hat1']
a_hat22=df2['a_hat2']
a_hat23=df2['a_hat3']
a_hat24=df2['a_hat4']
a_hat30=df3['a_hat0']
a_hat31=df3['a_hat1']
a_hat32=df3['a_hat2']
a_hat33=df3['a_hat3']
a_hat34=df3['a_hat4']
#VCV=df['VC_v']
#VCW=df['VC_w']
#VCDotV=df['VCDot_v']
#VCDotW=df['VCDot_w']
#print df.head
# plot
plt.figure()
plt.plot(x0,y0, 'r-', label='Bot0 Motion')
plt.plot(centroidX0,centroidY0, 'r.', label='Bot0 Centroids')
plt.plot(x1,y1, 'b-', label='Bot1 Motion')
plt.plot(centroidX1,centroidY1, 'b.', label='Bot1 Centroids')
plt.plot(x2,y2, 'g-', label='Bot2 Motion')
plt.plot(centroidX2,centroidY2, 'g.', label='Bot2 Centroids')
plt.plot(x3,y3, 'y-', label='Bot3 Motion')
plt.plot(centroidX3,centroidY3, 'y.', label='Bot3 Centroids')
plt.plot(5*0.25,5*0.25, 'mx', label='Peak 0')
plt.plot(-5*0.25,-5*0.25, 'mx', label='Peak 1')
plt.plot(5*0.25,-5*0.25, 'mx', label='Peak 2')
plt.plot(-5*0.25,5*0.25, 'mx', label='Peak 3')
plt.legend(loc='upper left')
plt.xlabel('x-position (in m)')
plt.xlabel('y-position (in m)')
plt.ylim(-2.5,2.5)
plt.xlim(-2.5,2.5)
plt.suptitle('Bots Tracking weighted centroids of Voronoi partition (Known Distribution)')

fig0=plt.figure()
plt.plot(time0,PWMR0, 'b-', label='PWM_R')
plt.plot(time0,PWML0, 'r-', label='PWM_L')
fig0.suptitle('Bot 0', fontsize=20)

fig1=plt.figure()
plt.plot(time1,PWMR1, 'b-', label='PWM_R')
plt.plot(time1,PWML1, 'r-', label='PWM_L')
fig1.suptitle('Bot 1', fontsize=20)

fig2=plt.figure()
plt.plot(time2,PWMR2, 'b-', label='PWM_R')
plt.plot(time2,PWML2, 'r-', label='PWM_L')
fig2.suptitle('Bot 2', fontsize=20)

fig3=plt.figure()
plt.plot(time3,PWMR3, 'b-', label='PWM_R')
plt.plot(time3,PWML3, 'r-', label='PWM_L')
fig3.suptitle('Bot 3', fontsize=20)

#plt.figure()
#plt.plot(time,tauL, 'bo')
#plt.plot(time,tauR, 'bo')

#plt.figure()
#plt.plot(time,e1, 'r-')
#plt.plot(time,e2, 'b-')
#plt.plot(time,e3, 'g-')
#plt.plot(time,uR, 'r.')
#plt.plot(time,uL, 'b.')

# fig4=plt.figure()
# plt.plot(time3,velV3, 'b-', label='velV')
# plt.plot(time3,velW3, 'r-', label='velW')
# fig3.suptitle('Bot 3', fontsize=20)
#plt.figure()
#plt.plot(time,velV, 'r-')
#plt.plot(time,velW, 'b-')
# fig5=plt.figure()
# plt.plot(time0, a_hat00,'.-')
# plt.plot(time0, a_hat01,'.-')
# plt.plot(time0, a_hat02,'.-')
# plt.plot(time0, a_hat03,'.-')
# plt.plot(time0, a_hat04,'.-')
# fig5.suptitle('a_hat0', fontsize=20)

# fig6=plt.figure()
# plt.plot(time1, a_hat10,'-')
# plt.plot(time1, a_hat11,'-')
# plt.plot(time1, a_hat12,'-')
# plt.plot(time1, a_hat13,'-')
# plt.plot(time1, a_hat14,'-')
# fig6.suptitle('a_hat1', fontsize=20)

# fig7=plt.figure()
# plt.plot(time2, a_hat20,'-')
# plt.plot(time2, a_hat21,'-')
# plt.plot(time2, a_hat22,'-')
# plt.plot(time2, a_hat23,'-')
# plt.plot(time2, a_hat24,'-')
# fig5.suptitle('a_hat2', fontsize=20)

# fig8=plt.figure()
# plt.plot(time3, a_hat30,'-')
# plt.plot(time3, a_hat31,'-')
# plt.plot(time3, a_hat32,'-')
# plt.plot(time3, a_hat33,'-')
# plt.plot(time3, a_hat34,'-')
# fig5.suptitle('a_hat3', fontsize=20)
###################################
# Plotting PWM and error in angle #
###################################
# fig1, ax1 = plt.subplots()

# ax1.set_xlabel('time (s)')
# ax1.set_ylabel('PWM')
# ax1.plot(time,PWMR, 'b-', label='PWM_R')
# ax1.plot(time,PWML, 'r-', label='PWM_L')
# ax1.tick_params(axis='y')
# ax1.legend(loc='upper left')

# ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

# ax2.set_ylabel('Error in theta')  # we already handled the x-label with ax1
# ax2.plot(time,e3, 'g-', label='e_theta')
# ax2.tick_params(axis='y', color='g')
#  ax2.legend(loc='upper right')

# fig1.tight_layout()  # otherwise the right y-label is slightly clipped

###############################################################
# Plotting PWM vs time and angular velocity of wheel vs time. #
###############################################################
# fig2 = plt.figure(tight_layout=True)
# gs = gridspec.GridSpec(2, 1)

# ax3 = fig2.add_subplot(gs[0, 0])
# ax3.set_xlabel('time (s)')
# ax3.set_ylabel('PWM')
# ax3.plot(time,PWMR, 'b-', label='PWM_R')
# ax3.tick_params(axis='y')
# ax3.legend(loc='upper left')

# ax4 = ax3.twinx()  # instantiate a second axes that shares the same x-axis
# ax4.set_ylabel('Angular Velocity')  # we already handled the x-label with ax1
# ax4.plot(time,wR, 'r-', label='w_R')
# ax4.tick_params(axis='y', color='g')
# ax4.legend(loc='upper right')

# ax5 = fig2.add_subplot(gs[1, 0])
# ax5.set_xlabel('time (s)')
# ax5.set_ylabel('PWM')
# ax5.plot(time,PWML, 'b-', label='PWM_L')
# ax5.tick_params(axis='y')
# ax5.legend(loc='upper left')

# ax6 = ax5.twinx()  # instantiate a second axes that shares the same x-axis
# ax6.set_ylabel('Angular Velocity')  # we already handled the x-label with ax1
# ax6.plot(time,wL, 'r-', label='w_L')
# ax6.tick_params(axis='y', color='g')
# ax6.legend(loc='upper right')

# fig2.tight_layout()  # otherwise the right y-label is slightly clipped
#################################################
# Plotting angular and linear vlocities of bot. #
#################################################
#fig3 = plt.figure(tight_layout=True)
#gs = gridspec.GridSpec(2, 1)

#ax7 = fig3.add_subplot(gs[0, 0])
#ax7.set_xlabel('time (s)')
#ax7.set_ylabel('Linear Velocitie')
#ax7.plot(time,velV, 'b-', label='Lin. Vel.')
#ax7.tick_params(axis='y')
#ax7.legend(loc='upper left')

#ax8 = ax7.twinx()  # instantiate a second axes that shares the same x-axis
#ax8.set_ylabel('Reference Velocity')  # we already handled the x-label with ax1
#ax8.plot(time,0.1*np.ones_like(velV), 'r-', label='Ref. Vel.')
#ax8.tick_params(axis='y', color='g')
#ax8.legend(loc='upper right')

#ax9 = fig3.add_subplot(gs[1, 0])
#ax9.set_xlabel('time (s)')
#ax9.set_ylabel('Angular Velocity')
#ax9.plot(time,velW, 'b-', label='Ang. Vel.')
#ax9.tick_params(axis='y')
#ax9.legend(loc='upper left')

#ax10 = ax9.twinx()  # instantiate a second axes that shares the same x-axis
#ax10.set_ylabel('Reference Velocity')  # we already handled the x-label with ax1
#ax10.plot(time,0.1*np.ones_like(velW), 'r-', label='Ref. Vel.')
#ax10.tick_params(axis='y', color='g')
#ax10.legend(loc='upper right')

#fig3.tight_layout()  # otherwise the right y-label is slightly clipped

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
#fig1, ax1 = plt.subplots()

#ax1.set_xlabel('time (s)')
#ax1.set_ylabel('PWM')
#ax1.plot(time,PWMR, 'b-', label='PWM_R')
#ax1.plot(time,PWML, 'r-', label='PWM_L')
#ax1.tick_params(axis='y')
#ax1.legend(loc='upper left')

#ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

#ax2.set_ylabel('Error in position')  # we already handled the x-label with ax1
#ax2.plot(time,e1, 'g-', label='e_x')
#ax2.plot(time,e2, 'y-', label='e_y')
#ax2.tick_params(axis='y', color='g')
#ax2.legend(loc='upper right')

fig1.tight_layout()  # otherwise the right y-label is slightly clipped


plt.show()




