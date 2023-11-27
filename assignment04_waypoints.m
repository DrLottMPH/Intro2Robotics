clear all
close all
clc

%Define the wall corners
wall_corners(1,:) = [-1,1,1,-1,-1];
wall_corners(2,:) = [1,1,-1,-1,1];

obstacles = [];

%Create a world with a floor image and wall corners defined above
world = RobotWorld(wall_corners,obstacles);


%Waypoints
waypoints(:,1) = [-0.5; -0.7];
waypoints(:,2) = [0.5; 0.65];
waypoints(:,3) = [-0.45; 0.1];
waypoints(:,4) = [0;0];

world.draw_dot(waypoints(:,1),0.05,[255,0,0]);
world.draw_dot(waypoints(:,2),0.05,[0,255,0]);
world.draw_dot(waypoints(:,3),0.05,[0,0,255]);



%Put a robot in the world
world.add_robot(Robot(world,@user_setup,@user_program))

%Give it the waypoints
world.robot.userdata.waypoints = waypoints;

%Start the world's time
world.start();



function user_setup(robot)
   robot.userdata.way_index = 1;
   robot.userdata.state = 0;
   robot.set_track_history(true);
end
function user_program(robot,dt_sec)
    %Goal is to seek waypoints and avoid obstacles
    % Build a map and then use the map to avoid obstacles


    windex = robot.userdata.way_index;
    waypoint_w = robot.userdata.waypoints(:,windex);
    pos_w = robot.read_gps_meters();
    waypoint_b = waypoint_w - pos_w;
    distance = norm(waypoint_b);

    %Compute robot heading vector:
    theta_deg = robot.read_compass_degrees();
    heading_dir = [cosd(theta_deg); sind(theta_deg)];
    
    %Compare the waypoint direction to the heading
    waypoint_b = waypoint_b/norm(waypoint_b);

    %The dot product gives cos(θ) and the cross product gives sin(θ)
    cos_angle = heading_dir(1)*waypoint_b(1) + heading_dir(2)*waypoint_b(2);
    sin_angle = heading_dir(1)*waypoint_b(2) - heading_dir(2)*waypoint_b(1);
    
    diff_angle_deg = atan2(sin_angle,cos_angle)*180/pi;


    switch robot.userdata.state
        case 0 %Travel to waypoint
            if(abs(diff_angle_deg) > 5)
                %If still not pointing roughly at the target, go back to
                %seeking the correct heading
                if (diff_angle_deg < 0)
                    robot.set_wheel_speed_dps(200,-200);
                else
                    robot.set_wheel_speed_dps(-200,200);
                end
            else %Aligned with the target, go to it
                if distance > 0.1
                    %Go fast when far away
                    robot.set_wheel_speed_dps(1080,1080);
                elseif distance > 0.005
                    %Slow down so we don't overshoot
                    robot.set_wheel_speed_dps(400,400);
                else %Made it to the target
                    robot.set_wheel_speed_dps(0,0);
                    robot.userdata.state = 1;
                end
            end
        case 1
            %At target, turn on LED and move to next waypoint
            if windex < size(robot.userdata.waypoints,2)
                %Turn on the associated LED:
                color = robot.read_floor_color();
                led_vals = robot.led_state();
                if color(1) > 0
                    robot.led_state(true,led_vals(2),led_vals(3));
                elseif color(2) > 0
                    robot.led_state(led_vals(1),true,led_vals(3));
                elseif color(3) > 0
                    robot.led_state(led_vals(1),led_vals(2),true);
                end
                %move to next waypoint
                robot.userdata.way_index = robot.userdata.way_index + 1;
                robot.userdata.state = 0;
            else %made it to the last waypoint
                robot.set_wheel_speed_dps(0,0);
                robot.userdata.state = 2; %move to final state
            end
        case 2 %Seeking north (90 degrees) at last step

            %Seek the angle to this but don't go to it
            % waypoint_b = [0;1]; %Point north

            %The dot product gives cos(θ) and the cross product gives sin(θ)
            cos_angle = heading_dir(2);
            sin_angle = heading_dir(1);
            diff_angle_deg = atan2(sin_angle,cos_angle)*180/pi;

            if(abs(diff_angle_deg) < 3)
                robot.set_wheel_speed_dps(0,0);
                robot.userdata.state = 3;
            else
                if (diff_angle_deg < 0)
                    robot.set_wheel_speed_dps(200,-200);
                else
                    robot.set_wheel_speed_dps(-200,200);
                end
            end
        case 3 %done
            %Do nothing, finished
    end



    
end