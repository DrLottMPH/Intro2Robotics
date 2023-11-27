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

%Draw the floor pattern:
ccenter = [-0.15; -0.1]; %in meters
world.draw_circle(ccenter,0.6,0.02,[0,0,0]);


dump_center = [0.75;0];
world.add_dirt();
world.add_dirt();

%Put a robot in the world - Give it the setup and user_program functions
%defined below.  The setup program is called immediately here when the
%robot is "constructed"
world.add_robot(Robot(world,@user_setup,@user_program))
world.add_dump_station(dump_center);

%Start the world's time
world.start();


function user_setup(robot)
    robot.userdata.state = 0;
    robot.userdata.angle_sum = 0;
    robot.userdata.time = 0;

    robot.vacuum(true);
    robot.set_max_dirt(200);
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
                if robot.count_dirt() == robot.set_max_dirt()
                    %Go to dump if full
                    robot.userdata.state = 2;
                    robot.userdata.return_waypoint = robot.read_gps_meters();
                    robot.userdata.waypoint_w = robot.find_dump_station();
                    robot.vacuum(false);
                end
            end
        case 1
            %On the black line
            %Keep track of the relative turn that is made
            robot.userdata.angle_sum = robot.userdata.angle_sum + robot.read_gyro_dps()*dt_sec;
            %turn 110 degrees left plus a random tweak
            if robot.userdata.angle_sum > robot.userdata.turn_target
                robot.userdata.state = 0; %Return to cleaning state
            end
        case 2
            %Seek a waypoint
            pos_w = robot.read_gps_meters();
            waypoint_b = robot.userdata.waypoint_w - pos_w;
            distance = norm(waypoint_b);
        
            %Compute robot heading vector:
            theta_deg = robot.read_compass_degrees();
            heading_dir = [cosd(theta_deg); sind(theta_deg)];

            %Compare the waypoint direction to the heading
            waypoint_b = waypoint_b/distance;
            
            cos_angle = heading_dir(1)*waypoint_b(1) + heading_dir(2)*waypoint_b(2);
            sin_angle = heading_dir(1)*waypoint_b(2) - heading_dir(2)*waypoint_b(1);
            diff_angle_deg = atan2(sin_angle,cos_angle)*180/pi;

            if(abs(diff_angle_deg) < 5)
                if(distance > 0.1)
                    robot.set_wheel_speed_dps(1080,1080);
                else
                    robot.set_wheel_speed_dps(400,400);
                end
            else
                if (diff_angle_deg < 0)
                    robot.set_wheel_speed_dps(200,-200);
                else
                    robot.set_wheel_speed_dps(-200,200);
                end
            end

            if distance < 0.02
                %Made it to waypoint
                robot.set_wheel_speed_dps(0,0);
                if robot.count_dirt() ~= 0
                    %Dump
                    robot.dump_dirt();
                    robot.userdata.waypoint_w = robot.userdata.return_waypoint;
                else
                    %start vacuuming again
                    robot.userdata.state = 0;
                    robot.vacuum(true);
                end
            end

    end
    
end