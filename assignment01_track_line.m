clear all
close all
clc

%Define the wall corners
wall_corners(1,:) = [-1,1,1,-1,-1];
wall_corners(2,:) = [1,1,-1,-1,1];

%Define obstacles (square blocks):
obstacles = [];


%Create a world with a floor image and wall corners defined above
world = RobotWorld(wall_corners,obstacles);

%Draw a circle on the floor:
ccenter = [-0.15; -0.1];
world.draw_circle(ccenter,0.6,0.02,[0,0,0]);
% world.draw_dot(ccenter,0.05,[0,255,0]);
% world.draw_square(ccenter,1,0.02,[0,0,0]);



%Put a robot in the world
world.add_robot(Robot(world,@user_setup,@user_program))
%Start the world's time
world.start();


function user_setup(robot)
    robot.userdata.state = 0;
    robot.set_track_history(true);
    robot.set_draw_icc(true);
    robot.userdata.mytime = 0;
end
function user_program(robot,dt_sec)
    %The user can read from sensors/encoders and give actuator
    %commands to move or stop the wheels in this function.  This is
    %the primary location for a student to write their navigation
    %code.  This is the driver's seat and is called 10 times per
    %second.
    
    color = robot.read_floor_color();
    intensity = mean(color);
    robot.userdata.mytime = robot.userdata.mytime + dt_sec;
    
    switch robot.userdata.state
        case 0
            %Find the line first
            robot.set_wheel_speed_dps(900,900);
            if(intensity == 0), robot.userdata.state = 1; end
        case 1
            %Waggle around one edge of the line
            if intensity == 0
                robot.set_wheel_speed_dps(300,-100);
            else
                robot.set_wheel_speed_dps(0,500);
            end
            if (robot.userdata.mytime > 4)
                robot.userdata.state = 2;
            end
        case 2
            if intensity == 0
                robot.set_wheel_speed_dps(500,0);
                % robot.set_wheel_speed_dps(1080,1080);
            else
                robot.set_wheel_speed_dps(0,500);
                % robot.set_wheel_speed_dps(880,1080);
            end
    end
    
end