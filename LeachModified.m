clearvars -except walking02AcclerometerData Trial_End_time Trial_Start_time Heel_Strikes walking02
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Network Establishment Parameters %%%%%%%%%%%%%%%%%%%%
%%% Area of Operation %%%

% Field Dimensions in meters %
xm=100;
ym=100;
x=0; % added for better display results of the plot
y=0; % added for better display results of the plot
% Number of Nodes in the field %
n=50;
% Number of Dead Nodes in the beggining %
dead_nodes=0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx=50;
sinky=50;

%%% Energy Values %%%
% Initial Energy of a Node (in Joules) % 
Eo=2; % units in Joules
% Energy required to run circuity (both for transmitter and receiver) %
Eelec=50*10^(-9); % units in Joules/bit
ETx=50*10^(-9); % units in Joules/bit
ERx=50*10^(-9); % units in Joules/bit
% Transmit Amplifier Types %
Eamp=100*10^(-12); % units in Joules/bit/m^2 (amount of energy spent by the amplifier to transmit the bits)
% Data Aggregation Energy %
EDA=5*10^(-9); % units in Joules/bit
% Size of data package %
BitSize=4000; % units in bits
% Suggested percentage of cluster head %
p=0.05; % a 5 percent of the total amount of nodes used in the network is proposed to give good results
% Number of Clusters %
No=p*n; 
% Round of Operation %
rnd=0;
rnd_for_Charging=50;   %%%number of rounds after whcih the CH energy is recharged to some level
EnergyCharged=0.2*Eo;  %%%%  the amount of energy charged to CH that is 20 percent of Eo
temp1=rnd_for_Charging;
% Current Number of operating Nodes %
operating_nodes=n;
transmissions=0;
temp_val=0;
flag1stdead=0;
%%%%%%%%%%%%%%%% parameters for sectoring of the regions%%%%%%%%%
RegionNumber=4;
OffsetRegion=xm/(2*RegionNumber);
a=zeros(1,RegionNumber);
b=zeros(1,RegionNumber);

global Regions
Regions=struct;
global CH
CH=struct;
global SN
SN=struct;
global count_time  %%Value that needs to be updated to keep track of the mobility time of nodes
count_time=0;
global Threshold_Value  %%%value that shows the minimum shortest distance of CH area
Threshold_Value=xm/(2*RegionNumber);
global Velocity
Velocity=0.3;    %%velocity of mobile nodes  : tought to be a constant velocity for all nodes
global newDistCov
Time_Span=0.5; %%%%1s timespan for node to move some distance
newDistCov=2*Velocity*Time_Span; %distance covereed in any direction by the node in 1s 
global energy
%%%%%%%%%%%%%%%%%%%%%%%%%%% End of Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%



            %%% Creation of the Wireless Sensor Network %%%
%%%%%%%%%%%%%%%%%%%%%%SetUp Phase%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 for RegionsX=1: RegionNumber
         a(RegionsX)=(((xm)/RegionNumber)*RegionsX)-OffsetRegion;
         b(RegionsX)=(((ym)/RegionNumber)*RegionsX)-OffsetRegion;        
 end
 
 %%%%%Initilialising Clusters sectors values
 for l=1:(RegionNumber)^2
            CH(l).regionID=0;
            CH(l).x=0;
            CH(l).Xc=0;
            CH(l).y=0;
            CH(l).Yc=0;
            CH(l).electedClusterNode=0;
            CH(l).tNodes=0;
 end
 Rx = repelem(a,RegionNumber);
 Ry = repmat(a,1,RegionNumber);
 voronoi(Rx,Ry);
 Regions.x=Rx';
 Regions.y=Ry';
 
 
%%%%%%%%%%%%%%%Plotting the WSN %%%%%%%%%
plot_Network(RegionNumber,OffsetRegion,x,y,xm,ym,n,Eo,sinkx,sinky)


%%%%%finding optimal location for CH and assiging the closest node as CH%%%%%%%%%%%%%%%%

    for k=1:(RegionNumber)^2
       dtNoptimal=xm;%%maximum is set because the graph extends lot higher value and some value are close to 100
      if (CH(k).tNodes>0)
       CH(k).Yc=(CH(k).y)/(CH(k).tNodes);
       CH(k).Xc=(CH(k).x)/(CH(k).tNodes);
      end
      
       for i=1:n
           if SN(i).region==k
               c=sqrt((CH(k).Xc-SN(i).x)^2 + (CH(k).Yc-SN(i).y)^2);
               SN(i).dtch=c;   %%dtch is the node distance from Centre or optimal location
               if c<dtNoptimal %% Checking The value of the closest node against the standard value
                   dtNoptimal=c;%%%assigning the new closest value to dtNoptimal until new closest distance is found
                   CH(k).electedClusterNode=SN(i).id;%%making the node as cluster head
                end
           end
       end
       if (CH(k).tNodes>0)
       SN(CH(k).electedClusterNode).role=1;
       SN(CH(k).electedClusterNode).dts=sqrt((sinkx-SN(CH(k).electedClusterNode).x)^2 + (sinky-SN(CH(k).electedClusterNode).y)^2); %% distance to the sink or base station
       CH(k).dts= SN(CH(k).electedClusterNode).dts;
       end
    end
    W_Cost_Function(); %%find the cost function for the Cluster head
    temp=0;
    for i=1:1:n        %%find the minimum cost function for each node against every cluster head
       if  SN(i).role==0
           temp=sqrt((CH(SN(i).region).Xc-SN(i).x)^2 + (CH(SN(i).region).Yc-SN(i).y)^2);  %distance of node to CH
            %%above is supposed to be minimum cost of the node to CH
            CH(SN(i).region).W*temp
           [SN(i).Cost,SN(i).region,SN(i).dtch]=Node_Cost_function(CH(SN(i).region).W*temp,SN(i).region,SN(i).x,SN(i).y,temp);
       end
    end
    
    
  CLheads=RegionNumber^2;  %%total number of cluster heads
 
  
  
  
  %%%%%%%%%%%%%%%%%%%%%%%Transmission Phase%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%while operating_nodes>0
  tStart_Transmission=tic;
  tEnd_Transmission=tic;
  tStart=tic;
  while CLheads>0 && rnd<4000
    tEnd=toc(tStart);
    if(rnd==temp1)
    Recharge_CH(EnergyCharged);
    temp1=temp1+rnd_for_Charging;
    end
         if tEnd<Time_Span+0.5 && tEnd>Time_Span-0.5
            [count_time_Temp,dead_nodes,operating_nodes,CLheads]=Mobility_Support(n,ETx,ERx,Eamp,Eelec,EDA,BitSize,dead_nodes,operating_nodes,CLheads);
             count_time=count_time+count_time_Temp;
             tStart=tic;  %%%%%reset the stopwatch
         end
                % time=tic;
      rnd;  % Displays Current Round %           
    % Reseting Previous Amount Of Energy Consumed In the Network on the Previous Round %
    energy=0;
 
% Grouping the Nodes into Clusters & caclulating the distance between node and cluster head %
       for i=1:n
        if  (SN(i).role==0) && (SN(i).E>0) && (CLheads>0) % if node is normal
            
           SN(i).dtch=sqrt((SN(CH(SN(i).region).electedClusterNode).x-SN(i).x)^2 + (SN(CH(SN(i).region).electedClusterNode).y-SN(i).y)^2);
            % we calculate the distance 'd' between the sensor node that is
            % transmitting and the cluster head that is receiving with the following equation+ 
            % d=sqrt((x2-x1)^2 + (y2-y1)^2) where x2 and y2 the coordinates of
            % the cluster head and x1 and y1 the coordinates of the transmitting node
    
        %%SN(i).chid=CL(Col).id; no need of this since we already have the
        %%region number of the Node as cluster ID
        end
       end
                           %%%%%% Steady-State Phase %%%%%%
% Energy Dissipation for normal nodes %   
    for i=1:n
       if (SN(i).cond==1) && (SN(i).role==0) && (CLheads>0)%%if node is active and normal
       	if SN(i).E>0
            ETx= Eelec*BitSize + Eamp * BitSize * SN(i).dtch^2; %%%%%%%%%%%check ths%%%%%%%%%%%
            SN(i).E=SN(i).E - ETx;
            energy=energy+ETx;
            
        % Dissipation for cluster head during reception
        if SN(CH(SN(i).region).electedClusterNode).E>0 && SN(CH(SN(i).region).electedClusterNode).cond==1 && SN(CH(SN(i).region).electedClusterNode).role==1
            ERx=(Eelec+EDA)*BitSize;
            energy=energy+ERx;
            SN(CH(SN(i).region).electedClusterNode).E=SN(CH(SN(i).region).electedClusterNode).E - ERx;
             if SN(CH(SN(i).region).electedClusterNode).E<=0  % if cluster heads energy depletes with reception
                SN(CH(SN(i).region).electedClusterNode).cond=0;
                SN(CH(SN(i).region).electedClusterNode).rop=rnd;
                dead_nodes=dead_nodes +1;
                operating_nodes=operating_nodes - 1;
                CLheads=CLheads-1;
             end
        end
        end
        
        
        if SN(i).E<=0       % if nodes energy depletes with transmission
        dead_nodes=dead_nodes +1;
        operating_nodes=operating_nodes - 1;
        SN(i).cond=0;
        SN(i).rop=rnd;
        end
        
       end
  end            
    
    
    
% Energy Dissipation for cluster head nodes %
   %% because it aggregates the data and sends it to the BS
   for i=1:n
     if (SN(i).cond==1)  && (SN(i).role==1)
         if SN(i).E>0
            ETx= (Eelec+EDA)*BitSize + Eamp * BitSize * SN(i).dts^2;
            SN(i).E=SN(i).E - ETx;
            energy=energy+ETx;
         end
         if  SN(i).E<=0     % if cluster heads energy depletes with transmission
         dead_nodes=dead_nodes +1;
         operating_nodes=operating_nodes - 1;
         SN(i).cond=0;
         SN(i).rop=rnd;
         CLheads=CLheads-1;
         end
     end
   end

   

  
    if operating_nodes<n && temp_val==0
        temp_val=1;
        flag1stdead=rnd
        tStart_Transmission=toc
    end
    % Display Number of Cluster Heads of this round %
    %%CLheads
   
    
    transmissions=transmissions+1;
    if CLheads==0
    transmissions=transmissions-1;
    end
    
 
    % Next Round %
    rnd= rnd +1;
    
    tr(transmissions)=operating_nodes;
    op(rnd)=operating_nodes;
    

    if energy>0
    nrg(transmissions)=energy;
    end
 
 % timeend=toc(time);%%%need to use random way reflection point model to calulate alive nodes in time
 % timeend
  count_time=count_time+1;
   
  end
tEnd_Transmission=toc;
  
  %%%%%%%%%%%%%end of of wirless tranmission of data packets%%%%%%%%%%%%

  
  
  %%%%%%%%%%%%%calculation for simulation output%%%%%%%%%%%%%%%%%%%%
sum=0;
for i=1:flag1stdead
    sum=nrg(i) + sum;
end

temp1=sum/flag1stdead;
temp2=temp1/n;

for i=1:flag1stdead
avg_node(i)=temp2;
end
  
  
  


%%%% Plotting Simulation Results "Operating Nodes per Round" %
    figure(2)
    plot(1:rnd,op(1:rnd),'-r','Linewidth',2);
    title ({'LEACH_M'; 'Operating Nodes per Round';})
    xlabel 'Rounds';
    ylabel 'Operational Nodes';
    hold on;
    
    % Plotting Simulation Results  %
    figure(3)
    plot(1:transmissions,tr(1:transmissions),'-r','Linewidth',2);
    title ({'LEACH_M'; 'Operational Nodes per Transmission';})
    xlabel 'Transmissions';
    ylabel 'Operational Nodes';
    hold on;
    
    % Plotting Simulation Results  %
    figure(4)
    plot(1:flag1stdead,nrg(1:flag1stdead),'-r','Linewidth',2);
    title ({'LEACH_M'; 'Energy consumed per Transmission';})
    xlabel 'Transmission';
    ylabel 'Energy ( J )';
    hold on;
    

    % Plotting Simulation Results  %
    figure(5)
    plot(1:flag1stdead,avg_node(1:flag1stdead),'-r','Linewidth',2);
    title ({'LEACH_M'; 'Average Energy consumed by a Node per Transmission';})
    xlabel 'Transmissions';
    ylabel 'Energy ( J )';
    hold on;
  
  

    
