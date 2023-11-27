clear all
close all
clc

%Define the wall corners
wall_corners(1,:) = [-1,1,1,-1,-1];
wall_corners(2,:) = [1,1,-1,-1, 1];

%Define obstacles (square blocks):
obstacles = [];

%Create a world with a floor image and wall corners defined above
world = RobotWorld(wall_corners,obstacles);

%Draw the floor pattern:
ccenter = [-0.15; -0.1]; %in meters
world.draw_circle(ccenter,0.6,0.02,[0,0,0]);

world.add_dirt();
world.add_dirt();

%Put a robot in the world - Give it the setup and user_program functions
%defined below.  The setup program is called immediately here when the
%robot is "constructed"
world.add_robot(Robot(world,@user_setup,@user_program))

%Start the world's time
world.start();


function user_setup(robot)
    robot.vacuum(true);
    robot.userdata.state = 0;
    robot.userdata.angle_sum = 0;
    robot.userdata.time = 0;
    % robot.set_max_dirt(200);
end
function user_program(robot,dt_sec)

    robot.userdata.time = robot.userdata.time + dt_sec;

    intensity = mean(robot.read_floor_color());
    xlabel(sprintf('%d dirt',robot.count_dirt()))
    switch robot.userdata.state
        case 0 %In the white space
            if intensity == 0 %Found black edge, turn
                robot.userdata.state = 1;
                robot.set_wheel_speed_dps(-400,400);
                robot.userdata.angle_sum = 0;
                robot.userdata.turn_target = 90 + rand()*90;
            else
                %drive across the circle vacuuming
                robot.set_wheel_speed_dps(700,700);
            end
        case 1
            %On the black line
            %Keep track of the relative turn that is made
            robot.userdata.angle_sum = robot.userdata.angle_sum + robot.read_gyro_dps()*dt_sec;
            %turn 110 degrees left plus a random tweak
            if robot.userdata.angle_sum > robot.userdata.turn_target
                robot.userdata.state = 0; %Return to cleaning state
            end

    end
    
end