clc;
clear all;
close all;
%Defining the intial conditions
target=[46.5 20]; %Final point 
pos=[2 20]; %Initial point
%Defining the position of Obstacle 3
origin_x3 = [35 37 37 35]; %// initial coordinates of vertices
origin_y3 = [28 28 34 34];
destination_x3 = origin_x3+0; %// final coordinates of vertices
destination_y3 = origin_y3-12; 
%Defining the position obstacle 1
origin_x1 = [20 22 22 20]; %// initial coordinates of vertices
origin_y1 = [0 0 4 4];
destination_x1 = origin_x1 +0; %// final coordinates of vertices
destination_y1 = origin_y1 +36;
%Defining the position of Obstacle 2
origin_x2 = [15 17 17 15]; %// initial coordinates of vertices
origin_y2 = [0 0 8 8];
destination_x2 = origin_x2 +0; %// final coordinates of vertices
destination_y2 = origin_y2 +14;
n_steps = 100; %// number of "frames"
t_pause = 0.5; %// seconds between frames
%Properties of the obstacles
h3 = fill(origin_x3, origin_y3, 'y'); %// create object at initial position
hold on
h1 = fill(origin_x1, origin_y1, 'r'); %// create object at initial position
hold on
h2 = fill(origin_x2, origin_y2, 'g'); %// create object at initial position
%Declaring the Area
A=rectangle('position',[0 0 60 40]);
axis equal %// same scale in both axes
axis manual %// prevent axes from auto-scaling
%Empty Matrices for Initialisation and storing the values
X1=[];
da1=[];
X2=[];
da2=[];
X3=[];
da3=[];
axis equal %// same scale in both axes
axis manual %// prevent axes from auto-scaling
%Initial Constants
AFC=0.008;
 d=0.6;
RFC=[0.1 0.3 1.2];
 for t = linspace(0,1,n_steps)
    pre_pos=pos;
    calc=target-pos;
%Defining the initial position 
    initial_position=rectangle('position',[pos(1) pos(2) 1 1],'curvature',[1 1],'facecolor','y');
% Defining the target and the position
    target_location=rectangle('position',[target(1) target(2) 2 2],'curvature',[1 1],'facecolor','b');
%To make Obs 3 to move along y axis
    x3 = destination_x3;  %// update x
    y3 = (1-t)*origin_y3 + t*destination_y3 %// update y
    set(h3, 'Vertices', [x3(:) y3(:)]); %// change object's position
    X3=[x3;y3]';
%To make Obs 1 to move along y axis
    x1 = destination_x1; %// update x
    y1 = (1-t)*origin_y1 + t*destination_y1; %// update y
    set(h1, 'Vertices', [x1(:) y1(:)]); %// change object's position
    X1=[x1;y1]'; 
%To make Obs 2 to move along y axis
    x2 = destination_x2;  %// update x
    y2 =(1-t)*origin_y2 + t*destination_y2 %// update y
    set(h2, 'Vertices', [x2(:) y2(:)]); %// change object's position
    X2=[x2;y2]'; 
%Conditions for Repulsive force  
%Obs 1
for i1=1:length(X1)
     da1=[da1;(([X1(i1,1) X1(i1,2)])-pos)];
    Fa1=-(RFC(1)*(([X1(i1,1) X1(i1,2)])-pos)/norm(([X1(i1,1) X1(i1,2)]-pos))^3);
        Fr1=Fa1;
end
%Obs 2
for i2=1:length(X2)
     da2=[da2;(([X2(i2,1) X2(i2,2)])-pos)];
    Fa2=-(RFC(2)*(([X2(i2,1) X2(i2,2)])-pos)/norm(([X2(i2,1) X2(i2,2)]-pos))^3);
        Fr2=Fa2;
end
%Obs 3
for i3=1:length(X3)
     da3=[da3;(([X3(i3,1) X3(i3,2)])-pos)];
    Fa3=-(RFC(3)*(([X3(i3,1) X3(i3,2)])-pos)/norm(([X3(i3,1) X3(i3,2)]-pos))^3);
        Fr3=Fa3;
end
%Attractive Force
FA=AFC*(calc); 
Fr=Fr1+Fr2+Fr3;
Ft=FA+Fr;
angle = atan(Ft(2)/Ft(1));
pos(1)=pre_pos(1)+(d*cos(angle));
pos(2)=pre_pos(2)+(d*sin(angle));
if pos (1) >=target(1)-0.5
      pos=pre_pos;
      Fr=[0 0];
      Ft=[0 0];
end
pause(t_pause) %// a pause is needed to make movement slower
      end
