import numpy as np
import math
#import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
#from mpl_toolkits.mplot3d import Axes3D


#Computing weighted Voronoi partitions

#First we create the the grid area in which we want the search to be performed.
#The origin of the the grid is set at the orgin of Vicon system. We take a grid
#of 5m by 5m as a few VICON cameras are broken and coverage is restricted.

origin = np.array([0,0])

grid_Size = 5.0 #in meters
grid_Res = 0.02 #in cm

N_Grid_Points = int(grid_Size/grid_Res) #We gonna assume square grids. Else even the Voronoi partition function has to change.

x_grid=np.arange(-grid_Size/2+origin[0], grid_Size/2+origin[0], grid_Res)+grid_Res/2 #+grid_Res/2 gives the centroid
y_grid=np.arange(-grid_Size/2+origin[1], grid_Size/2+origin[1], grid_Res)+grid_Res/2

X_grid, Y_grid = np.meshgrid(x_grid,y_grid)

#pos_grid is a three 250x250 matrix with each point holding the coordinate of the centroid of a given grid aquare. 
#eg. pos_grid[0,0,:] will give the 1st squares centroid as [-2.49,2.49]
pos_grid = np.empty(X_grid.shape + (2,))
pos_grid[:, :, 0] = X_grid; pos_grid[:, :, 1] = Y_grid


#The square will be defined by the bottom-left corner co-ordinate. The first square(0,0) will
#have its corner at (0.0,0.0) and the centroid at (0.01,0.01).

def centroid(i,j,side):
	center=np.array([(i+0.5)*side,(j+0.5)*side]);
	return center

#Creating the phenonmenon (through definition of the basis functions) to be sensed in the environment.
mu0 = np.array([5*0.25, 5*0.25]) 	#The basis is formed by taking two gaussians on each diagnol and a constant function. 
mu1 = np.array([-5*0.25, -5*0.25])
mu2 = np.array([5*0.25, -5*0.25])
mu3 = np.array([-5*0.25, 5*0.25])
Sigma = np.array([[150.0/N_Grid_Points, 0],[0, 150.0/N_Grid_Points]])

rv0 = multivariate_normal(mu0, Sigma)
rv1 = multivariate_normal(mu1, Sigma)
rv2 = multivariate_normal(mu2, Sigma)
rv3 = multivariate_normal(mu3, Sigma)

#Adding all the basis ventors in 1 variable.
K=np.dstack((rv0.pdf(pos_grid), rv1.pdf(pos_grid),rv2.pdf(pos_grid),rv3.pdf(pos_grid),np.ones(pos_grid[:,:,0].shape)))

#Randomly choosen weights to each element in the basis function.
a=np.array([1.0/2,1.0/4,1.0/3,1.0/9,1.0/5]) #np.array([1,1,1,1,1])
phi=a[0]*K[:,:,0]+a[1]*K[:,:,1]+a[2]*K[:,:,2]+a[3]*K[:,:,3]+a[4]*K[:,:,4]

#print phi

#Plotting the basis functions for visualisations.
#fig = plt.figure()
#ax=fig.add_subplot(111, projection='3d')
#ax.plot_surface(X_grid, Y_grid, phi,cmap='viridis',linewidth=0)
#plt.show()


#The number of bots and their locations here will be fed to the system from the master computer later. 

N_bots=4
BotNumber=0  #This will be set correctly (0,1,2...N_bots) so that the bot know which location data is its own. 

bot_loc=np.empty((2,N_bots));

#Temporary shiz
bot_loc[:,0]=(5*0.0,5*0.0);
bot_loc[:,1]=(-5*0.25,5*0.25);
bot_loc[:,2]=(5*0.25,-5*0.25);
bot_loc[:,3]=(-5*0.25,-5*0.25);



#Write the subrcriber here to take all the bot_loc data
def cartesianDist(a,b):
	return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

#print bot_loc

#Writing the Voronoi partition calculator function.
#This function returns the set of points on the pos_grid which lie within the bot's voronoi partition. 
def voronoi(grid, Nbots, BotNo, locations):
	VPartition=[]
	N_Grid_Points = len(grid[:,0,0])
	for i in range(N_Grid_Points):
		for j in range(N_Grid_Points):	#This iterates over all points in the domain.
			inPartition=True 			#This stays one as long as the point is closer to the botIn question than any other point. 
			for N in range(Nbots):
				if N!=BotNo and inPartition:
					inPartition = inPartition and cartesianDist(grid[i,j,:],locations[:,BotNo])<cartesianDist(grid[i,j,:],locations[:,N]) 		
			if(inPartition):
				VPartition.append(grid[i,j,:])
	return VPartition
A=voronoi(pos_grid,N_bots,BotNumber,bot_loc)

print len(A)
#x,y = np.array(A).T


#fig = plt.figure()
#ax = fig.add_subplot(111)
#fig.show()

#ax.scatter(x,y)
#ax.scatter(bot_loc[0][:],bot_loc[1][:], color = 'r', marker = 'x')
