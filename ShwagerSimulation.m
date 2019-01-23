%%
%This is the simulation file for the paper by Shwager et al.

%%
%Creating Random Point locations for the bots and initialization.
N_bots=6;
bot_loc=zeros(2,N_bots);
a = [0.2 0.3 0.15]; a_min=[0.01;0.01;0.01]; 

for i = 1:N_bots
    bot_loc(:,i)=rand(2,1);
    %Initialize the parameters for each bot.
    V(i).Gamma_i=[0 0 0;0 0 0;0 0 0];
    V(i).Lambda_i=[0;0;0]; 
    V(i).a_hat=a_min;
end

%figure;
%scatter(bot_loc(1,:),bot_loc(2,:));

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Creating Voronai partitions using a mesh based approach %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Since our points are in a 1 by 1 square we will divide the entire square
%into a mesh of size n x n. We will try with n=100.
n_mesh=50;
xgrid=1/n_mesh*(1:n_mesh);
ygrid=1/n_mesh*(1:n_mesh);
%%
%Creating the Gaussian distribution. We first create the basis function K.
%We will use 4 gaussians located at (1/4,1/4), (3/4,1/4), (1/4,3/4) and
%(3/4,3/4) loccations on the grid. 

mu1 = [0.25 0.25]; mu2 = [0.75 0.75];
Sigma = [1/n_mesh 0; 0 1/n_mesh];
for i = 1:n_mesh
    for j = 1:n_mesh
    K(1).distribution(i,j)= mvnpdf(centroid(i,j,1/n_mesh),mu1,Sigma);
    K(2).distribution(i,j)= mvnpdf(centroid(i,j,1/n_mesh),mu2,Sigma);
    K(3).distribution(i,j)= 1; %Constant distribution
    end
end
%%
%Creating the distribution to be sensed
phi= a(1)*K(1).distribution+a(2)*K(2).distribution + a(3)*K(3).distribution;
mesh(phi);

%%
%Initialising variables for the control loop.
tfinal=5;
step=0.1;
tspan = linspace(0,tfinal,tfinal/step+1);
KGain=[1 0;0 1];    %Control Gain
gamma=5;          %Adaptive update rate
alpha=1;        %Time constant for w(t). Affacts lambda and gamma updates. 
%%
% Finding nearest bots to help obtain the Voronai partition %
for t = tspan
for BotNum=1:N_bots
%R=1; %Chosen as 5% of maximum dimension of the square. Smaller for better accuracy.
Q=[];  %All space
inCircle=[];
PinCircle=[];
%The loop below finds all the bots in a circle of radius R.It used to do
%this but now it just takes all the points except the one in
%consideration.
for i = 1:N_bots
    if(i~=BotNum)
        PinCircle=[PinCircle,i];
    end
end

%The halfspace are defined as the points closer to central bot
%than to the kth bot.

for k = PinCircle
    Halfspace(k).BotNumber=k;
    Halfspace(k).PointsInHalfspace=[];
end


for i = 1:n_mesh
    for j = 1:n_mesh
        q=centroid(i,j,1/n_mesh);
        %if(sqrt((bot_loc(1,BotNum)-q(1))^2 + (bot_loc(2,BotNum)-q(2))^2)<R)
        %    inCircle=[inCircle,q(:)];                           %All points in the circle of radius R.
        %end
        %Next for each neighbouring point in the circle we create halfspaces.
        for k = PinCircle
            if(sqrt((bot_loc(1,BotNum)-q(1))^2 + (bot_loc(2,BotNum)-q(2))^2)<=sqrt((bot_loc(1,k)-q(1))^2 + (bot_loc(2,k)-q(2))^2))
               Halfspace(k).PointsInHalfspace=[Halfspace(k).PointsInHalfspace,q(:)];
           end
        end
        Q=[Q,q(:)]; %This gives us all points in space.
    end
end
W=Q;

inCircle=Q; %This initially was used for the growing circle but doesnt matter now

for k = PinCircle
    Common=intersect(W',Halfspace(k).PointsInHalfspace','rows');
    W=Common';
end

Common2=intersect(W',inCircle','rows');
W=Common2';
Windex=uint8(W*n_mesh+0.5); %This stores the index values of the points in the voronai region

MPhi=0;                     %This finds the area integral of the voronai partition
LvPhi=[0;0];                %This computes the weighted area integral of the partition.
a_temp=V(BotNum).a_hat;
V(BotNum).phi_hat=a_temp(1)*K(1).distribution+a_temp(2)*K(2).distribution + a_temp(3)*K(3).distribution; %The estimate of the environment 
for k = 1:length(Windex(1,:))   %Iterate over the index of all points in the voronai partition.
    MPhi=MPhi+V(BotNum).phi_hat(Windex(1,k),Windex(2,k));     %Update the area by considering the distribution at the centroid of the kth point in the Vor Region. 
    LvPhi = LvPhi + W(:,k)*V(BotNum).phi_hat(Windex(1,k),Windex(2,k)); %Similarly update the weighted area using centroids.
    Fv=[K(1).distribution(Windex(1,k),Windex(2,k));K(2).distribution(Windex(1,k),Windex(2,k));K(3).distribution(Windex(1,k),Windex(2,k))]*(W(:,k)-bot_loc(:,BotNum))'; %integral(K*(q-p_i) over voronai
end
Cv=LvPhi/MPhi;
%{
while (R<2*maxDistanceInSetToPoint(bot_loc(:,BotNum),W) && R<2*sqrt(2))  %The second inequality ensures that
    R=2*R;                                                                    %The radius doesn't blow up uneccesarily                                                                 
    inCircle=[];
    PinCircle=[];

%The loop below finds all the bots in a circle of radius R.
for i = 1:N_bots
    if(sqrt((bot_loc(1,BotNum)-bot_loc(1,i))^2 + (bot_loc(2,BotNum)-bot_loc(2,i))^2)<R && i~=BotNum)
        PinCircle=[PinCircle,i]; 
    end
end

%The halfspace are defined as the points closer to central bot
%than to the kth bot.
for k = PinCircle
           Halfspace(k).BotNumber=k;
           Halfspace(k).PointsInHalfspace=[];
end


for i = 1:n_mesh
    for j = 1:n_mesh
        q=centroid(i,j,1/n_mesh);
        if(sqrt((bot_loc(1,BotNum)-q(1))^2 + (bot_loc(2,BotNum)-q(2))^2)<R)
            inCircle=[inCircle,q(:)];                           %All points in the circle of radius R.
        end
        %Next for each neighbouring point in the circle we create halfspaces.
        for k = PinCircle
           if(sqrt((bot_loc(1,BotNum)-q(1))^2 + (bot_loc(2,BotNum)-q(2))^2)<=sqrt((bot_loc(1,k)-q(1))^2 + (bot_loc(2,k)-q(2))^2))
               Halfspace(k).PointsInHalfspace=[Halfspace(k).PointsInHalfspace,q(:)];
           end
        end
    end
end
W=Q;

for k = PinCircle
    Common=intersect(W',Halfspace(k).PointsInHalfspace','rows');
    W=Common';
end
Common2=intersect(W',inCircle','rows');
W=Common2';
end
end 
%}
V(BotNum).VoronaiRegion = W;
V(BotNum).VoronaiIndex = Windex;
V(BotNum).VoronaiMass = MPhi;
V(BotNum).VoronaiMoment = LvPhi;
V(BotNum).VoronaiCentroid=Cv;
V(BotNum).VoronaiF=Fv*KGain*Fv'/MPhi;
%%
%For the ith bot we will obtain the lambda and gamma by w(t)*K'(p)K(p) and w(t)*K'(p)*Phi_hat(p) 
%floor(bot_loc(1,BotNum*n_mesh+0.5) this part takes the location of the
%nearest centroid to a bot location.
K_temp=[K(1).distribution(floor(bot_loc(1,BotNum)*n_mesh+0.5),floor(bot_loc(2,BotNum)*n_mesh+0.5)); K(2).distribution(floor(bot_loc(1,BotNum)*n_mesh+0.5),floor(bot_loc(2,BotNum)*n_mesh+0.5)); K(3).distribution(floor(bot_loc(1,BotNum)*n_mesh+0.5),floor(bot_loc(2,BotNum)*n_mesh+0.5))];
Gamma_i_dot=-alpha*V(BotNum).Gamma_i+K_temp*K_temp';
Lambda_i_dot=-alpha*V(BotNum).Lambda_i+K_temp*phi(floor(bot_loc(1,BotNum)*n_mesh+0.5),floor(bot_loc(1,BotNum)*n_mesh+0.5));

%Updating the Gammai and lambdai values.
V(BotNum).Gamma_i=V(BotNum).Gamma_i+step*Gamma_i_dot;
V(BotNum).Lambda_i=V(BotNum).Lambda_i+step*Lambda_i_dot;

%Adaptive update law
a_hat_dot=-V(BotNum).VoronaiF*V(BotNum).a_hat-gamma*(V(BotNum).Gamma_i*V(BotNum).a_hat-V(BotNum).Lambda_i);
if (min(V(BotNum).a_hat)>a_min(1) || (min(V(BotNum).a_hat)==a_min(1) && min(a_hat_dot)>0))
V(BotNum).a_hat=V(BotNum).a_hat+step*a_hat_dot;
end
%Control Law
u=KGain*(V(BotNum).VoronaiCentroid - bot_loc(:,BotNum));
bot_loc(:,BotNum)=bot_loc(:,BotNum)+u*step;

end
end
%%
%Plotting the Voronai Regions
figure;
for BotNum = 1:N_bots
    %plot(V(BotNum).VoronaiRegion(1,:),V(BotNum).VoronaiRegion(2,:),'o');
    %hold on;
    plot(V(BotNum).VoronaiCentroid(1),V(BotNum).VoronaiCentroid(2),'+');
    hold on;
end
plot(bot_loc(1,:),bot_loc(2,:),'x');
hold off;


%%
figure;
voronoi(bot_loc(1,:),bot_loc(2,:));
xlim=([0 1]);
ylim=([0 1]);

%%

