%% MAE 547 Final Project 2 - Fall 2021
%% Simulation and Manipulation of a RPP Robot using Matlab
%% Team - Aravind Adhith Pandian Saravanakumaran (1222209391)
%%      - Monish Dev Sudhakar  (1220859588)
%%      - Gowtham Dakshnamoorthy (1222598585)



%% Initialization of the Program
clear ; close all; clc

%% The Workspace of Robot (The Extreme Positions the Robot can Reach)

R = -200*sqrt(2);           %The Radius of the Cylinder is taken to be 282 Units

for i = 1:length(R)
theta = linspace(-pi,pi,20);
zt = linspace(0,200,20);    % The Height of Cylinder is taken to be 200 Units
yt = R(i).*sin(theta);
xt = R(i).*cos(theta);
[yt,zt]=meshgrid(yt,zt);

 surf(xt,yt,zt)             % Creating the Surface of the Workspace (i.e) Curved Surface Area of the Cylinder 

 hold on
 alpha 0

end

alpha 0

%% Labelling the Axis in the Graph/Simulation
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
%% Creating Links of the RPP Robot in PeterCoke ToolBox
d1 = 5;
L1 = Revolute('a', 0, 'd', d1, 'qlim', [-pi,pi],'standard');
L2 = Prismatic('a', 0, 'alpha', -pi/2, 'qlim',[5,200],'standard');
L3 = Prismatic('a', 0, 'qlim',[2,abs(R)],'standard');
robot1 = SerialLink( [L1 L2 L3],'name', 'RPP/Cake Robot');
text(0,0,0,' RPP ROBOT')

pf = [];
pl = [];

while(1)                                % Starting Loop to get the end-effector positions
    if length(pf) <1
        prompt = 'Enter Inital position of End Effector [x1,y1,z1] :';
        pfinal = input(prompt);         % Obtaining the Initial Position of the End-Effector in 3-D Space   
        pf = [-pfinal(2),-pfinal(1),pfinal(3)];
        T1 = transl(pf);
        EE_first = inv_kine(T1(1,4),T1(2,4),T1(3,4));
        hold on
    end

    prompt = 'Enter next position of the End Effector or Press Ctrl+C to Terminate Program :';
%     pl = input(prompt);
        p2 = input(prompt);             % Getting the coordinates of next point the End-Effector needs to move 
        pl = [-p2(2),-p2(1),p2(3)];
    hold on

    text(p2(1),p2(2),p2(3),['(' num2str(p2(1)) ',' num2str(p2(2))  ',' num2str(p2(3)) ')'])
    hold on
    text(pfinal(1),pfinal(2),pfinal(3),['(' num2str(pfinal(1)) ',' num2str(pfinal(2))  ',' num2str(pfinal(3)) ')'])
    hold on
    
    T2 = transl(pl);                    % Using the in-built function of Tool Box to Translational Transformation Matrix
    EE_last = inv_kine(T2(1,4),T2(2,4),T2(3,4));    % Calling the Inverse Kinematics functions to compute the joint variables for the next position

    via = [ EE_last];
    q =  mstraj(via, 1, [], EE_first,2, 2); % Funtion used to create a multi-axis trajectory (PeterCorke Toolbox Function)


    for m=1:length(q)
      vars = q(m,:);
      pos = robot1.fkine(vars);
      q1_limit = L1.islimit(vars(1));
      q2_limit = L2.islimit(vars(2));
      q3_limit = L3.islimit(vars(3));

      jval=robot1.jacob0(vars);
      hold on

      if det(jval(1:3,1:3)) == 0            % Condition to check if mechanism reaches singluarity 
          fprintf('\n Singularity reached\n');
          plot3(pos(1,1).t(1),pos(1,1).t(2),pos(1,1).t(3),'.b')
          hold on
      elseif q1_limit ~=0 
           fprintf('\n Joint 1 out of limit\n'); % To check if the entered coordinates are outside the reachabilty of the mechanism
           fprintf('\n Quiting program\n');
            return;
      elseif q2_limit ~=0 
           fprintf('\n Joint 2 out of limit\n'); % To check if the entered coordinates are outside the reachabilty of the mechanism
            return;
      elseif q3_limit ~=0
           fprintf('\n Joint 3 out of limit\n'); % To check if the entered coordinates are outside the reachabilty of the mechanism
            return;    
       else
           plot3(pos(1,1).t(1),pos(1,1).t(2),pos(1,1).t(3),'.r') % If there is no singularity and enteres coordinates are within workspace the trajectory is plotted
           hold on
      end
    hold on
    end
    
    hold on
    t1 = cputime;

    
    robot1.plot(q,'fps',400,'noname','noshadow','notiles','jointdiam',1, 'workspace', [-400,400,-400,400,-0,200]) % Function to run the robot simulation
    fprintf('\nTime Taken(secs) to reach the goal: %f\n', cputime-t1);
    
    hold on;
    
    %% Plotting q(Joint Trajectory), qd(Joint Velocity), qdd(Joint Acceleration) graphs

[x, xd, xdd]=jtraj(EE_first, EE_last, 100);
    figure('Name','q, qd, qdd Plots');

    subplot(2,2,[1 3])
    plot(x);
    title('q (Joint Trajectory)')  
    xlabel('Value of q1, q2, q3'); 
    ylabel('t/sec');
    legend('q1','q2','q3')

    subplot(2,2,2)
    plot(xd);
    title('qd (Joint Velocity)')  
    xlabel('Value of q1d, q2d, q3d'); 
    ylabel('t/sec');
    legend('q1d','q2d','q3d')


    subplot(2,2,4)
    plot(xdd);
    title('qdd (Joint Acceleration)')  
    xlabel('Value of q1dd, q2dd, q3dd'); 
    ylabel('t/sec');
    legend('q1dd','q2dd','q3dd')
    
    %% Replacing the Inital Coordinates with Previous Condition's Final Coordinates
    EE_first =EE_last;
end

robot1.teach('rpy') % Inbuilt Function in PeterCorke Toolbox which is used to drive the Robot

%% Labelling the Axes
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');