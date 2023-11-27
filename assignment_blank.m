clear all
close all
clc

%Define the wall corners
wall_corners(1,:) = [-1,1,1,-1,-1];
wall_corners(2,:) = [1,1,-1,-1,1];

%Define obstacles (square blocks):
obstacles = [];
obstacles(1).center = [-0.6; 0.6];
obstacles(1).side_length_m = 0.1; %10cm
obstacles(1).color = 'r';

obstacles(2).center = [0.5; 0.5];
obstacles(2).side_length_m = 0.2; %20cm
obstacles(2).color = 'r';

%Create a world with a floor image and wall corners defined above
world = RobotWorld(wall_corners,obstacles);

%Draw the floor pattern:
ccenter = [-0.15; -0.1]; %in meters
world.draw_circle(ccenter,0.6,0.02,[0,0,0]);
world.draw_dot(ccenter,0.1,[0,255,0]);

% world.add_dirt();

%Put a robot in the world - Give it the setup and user_program functions
%defined below.  The setup program is called immediately here when the
%robot is "constructed"
world.add_robot(Robot(world,@user_setup,@user_program))

%Start the world's time
world.start();


function user_setup(robot)
    %This function is called only once when the robot "boots up."  Use this
    %function to initialize your program.
    % Use "robot.userdata" as a data structure to store any information you
    % want for your program.  It will be available there each time the
    % "user_program" is called.
    
end
function user_program(robot,dt_sec)
    %The user can read from sensors/encoders and give actuator
    %commands to move or stop the wheels in this function.  This is
    %the primary location for a student to write their navigation
    %code.  This is the driver's seat and is called 20 times per
    %second.
end