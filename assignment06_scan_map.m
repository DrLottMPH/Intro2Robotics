clear all
close all
clc

%Define the wall corners
wall_corners(1,:) = [-1,1,1,-1,-1];
wall_corners(2,:) = [1,1,-1,-1,1];


%Define obstacles (square columns):
obstacles = [];
obstacles(1).center = [-0.6; 0.6];
obstacles(1).side_length_m = 0.1; %10cm
obstacles(1).color = 'r';

obstacles(2).center = [0.5; 0.5];
obstacles(2).side_length_m = 0.2; %20cm
obstacles(2).color = 'r';

%Create a world with a floor image and wall corners defined above
world = RobotWorld(wall_corners,obstacles);

ccenter = [-0.15; -0.1];
world.draw_circle(ccenter,0.6,0.02,[0,0,0]);
world.draw_dot(ccenter,0.1,[0,255,0]);

% world.add_dirt();



%Put a robot in the world
world.add_robot(Robot(world,@user_setup,@user_program))
%Start the world's time
world.start();



function user_setup(robot)
    %Containers to store the map data
    % robot.userdata.range_calibration = 0.04; %From design
    robot.userdata.angle_track = 0;
    robot.userdata.range = [];
    robot.userdata.theta = [];
    robot.userdata.position = [];
    robot.userdata.range_cal = 0.04;
    robot.set_wheel_speed_dps(-100,100); %Spin about the center of the axle
    robot.userdata.state = 0;

    %Visualize it live:
    robot.userdata.fig = figure;
    robot.userdata.h_plot = line(nan,nan,'linestyle','none','marker','.','markersize',10,'color','b');
    robot.userdata.h_ax = gca;
    set(robot.userdata.h_ax,'ylim',[0 2],'xlim',[0 360])
    robot.userdata.h_fill = line(nan,nan,'linestyle','none','marker','.','markersize',10,'color','r');
end
function user_program(robot,dt_sec)
    %The user can read from sensors/encoders and give actuator
    %commands to move or stop the wheels in this function.  This is
    %the primary location for a student to write their navigation
    %code.  This is the driver's seat and is called 20 times per
    %second.

    %Accumulate the relative angle that i've traversed.
    robot.userdata.angle_track = robot.userdata.angle_track + robot.read_gyro_dps()*dt_sec;
    
    switch robot.userdata.state
        case 0
            %Building initial coarse map from rotating in the middle
            robot.userdata.range(end+1) = robot.read_range() + robot.userdata.range_cal;
            robot.userdata.theta(end+1) = robot.read_compass_degrees();
            robot.userdata.position(:,end+1) = robot.read_gps_meters();

            set(robot.userdata.h_plot,'xdata',robot.userdata.theta,'ydata',robot.userdata.range)
            if(robot.userdata.angle_track > 360)
                robot.userdata.state = 1;
            end
        case 1
            %Visualize the 2D map in x/y
            robot.set_wheel_speed_dps(0,0);
            range = robot.userdata.range;
            theta = robot.userdata.theta;

            xdata = range.*cos(theta*pi/180) + robot.userdata.position(1,:);
            ydata = range.*sin(theta*pi/180) + robot.userdata.position(2,:);
            
            set(robot.userdata.h_plot,'xdata',xdata,'ydata',ydata)
            set(robot.userdata.h_ax,'xlim',[-1.5, 1.5],'ylim',[-1.5, 1.5])
            
            robot.userdata.state = 2;
            %Allow for fill in (in red)
            robot.userdata.range = [];
            robot.userdata.theta = [];
            robot.userdata.position = [];
        case 2
            %Fill in map as user drives around
            robot.userdata.range(end+1) = robot.read_range() + robot.userdata.range_cal;
            robot.userdata.theta(end+1) = robot.read_compass_degrees();
            robot.userdata.position(:,end+1) = robot.read_gps_meters();

            xdata = robot.userdata.range.*cos(robot.userdata.theta*pi/180) + robot.userdata.position(1,:);
            ydata = robot.userdata.range.*sin(robot.userdata.theta*pi/180) + robot.userdata.position(2,:);
            
            set(robot.userdata.h_fill,'xdata',xdata,'ydata',ydata)
            set(robot.userdata.h_ax,'xlim',[-1.5, 1.5],'ylim',[-1.5, 1.5])
            
    end

    
    
    % - What happens if you spin around the wheel instead of the center?  
    % - What if you take a larger radius of curvature in your spin?
    % - What happens if you spin faster?
    % - How can we turn this into a useable map of the world and what would we use it for?
    % - Can we add floor color to our map?  How would we do that?
    % - Lets talk about building a navigation algorithm using our map and our wheel encoder and gyroscope
    % - How to create a plan to get from point A to point B? "Path planning"
    % - How to create a log file to store all this behavior?
    % - Can we add to this map from a different location in the world to fill
    % in behind the object? (why we need navigation)
    % - Get into dead reckoning (walk with eyes closed through desks) -
    % Feel your uncertainty grow - talk about navigation Kalman Filters
    % - Can we fit a piecewise line to the walls and tell if future
    % datapoints correspond to these walls or new walls?
    % - Next assignment needs to just be about developing a navigation
    % solution.
    
end