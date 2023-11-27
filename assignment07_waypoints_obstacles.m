clear all
close all
clc

%Define the wall corners
wall_corners(1,:) = [-1,1,1,-1,-1];
wall_corners(2,:) = [1,1,-1,-1,1];

obstacles = [];
obstacles(1).center = [-0.3; -0.3];
obstacles(1).side_length_m = 0.1; %10cm
obstacles(1).color = 'r';

obstacles(2).center = [0.3; 0.4];
obstacles(2).side_length_m = 0.1; %10cm
obstacles(2).color = 'r';

obstacles(3).center = [-0.35; 0.4];
obstacles(3).side_length_m = 0.1; %10cm
obstacles(3).color = 'r';


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

   %Start tracking map
   robot.userdata.angle_track = 0;
   robot.userdata.map = zeros(2,0);
   robot.userdata.range_cal = 0.04;
   robot.userdata.pre_angle = 0;

   robot.userdata.obstacle_passing_threshold = 0.15;
   robot.set_track_history(true);
   robot.userdata.h_map = line(nan,nan,'marker','.','linestyle','none','color','k','markersize',10);
end
function user_program(robot,dt_sec)
    % Track to waypoints, avoiding obstacles on the way

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

    %The sin/cos of the angle between them for full quadrant correct
    %reconstruction of the direction.
    cos_angle = heading_dir(1)*waypoint_b(1) + heading_dir(2)*waypoint_b(2);
    sin_angle = heading_dir(1)*waypoint_b(2) - heading_dir(2)*waypoint_b(1);
    
    diff_angle_deg = atan2(sin_angle,cos_angle)*180/pi;

    switch robot.userdata.state
        case 0 %Build map
            %Accumulate orientation for a total 360 spin
            robot.userdata.angle_track = robot.userdata.angle_track + robot.read_gyro_dps()*dt_sec;
            robot.set_wheel_speed_dps(-100,100);
            
            %Compute range and heading to obstacles
            range = robot.read_range() + robot.userdata.range_cal;
            theta_deg = robot.read_compass_degrees();
            pos_w = robot.read_gps_meters();
            
            %Convert this point to the world frame
            x = range.*cosd(theta_deg) + pos_w(1);
            y = range.*sind(theta_deg) + pos_w(2);
            
            %Accumulate the map of the world (a collection of x/y points)
            robot.userdata.map(:,end+1) = [x;y];

            set(robot.userdata.h_map,'xdata',robot.userdata.map(1,:),'ydata',robot.userdata.map(2,:))

            if(robot.userdata.angle_track > 360)
                %Initial coarse map complete, seek first waypoint
                robot.set_wheel_speed_dps(0,0);
                robot.userdata.state = 1;
            end
        case 1 %Detect obstacles and make intermediate plans
            %Does my path intersect (come close to) an object between here
            %and the target waypoint.  The line between myself and the 
            windex = robot.userdata.way_index;
            waypoint_w = robot.userdata.waypoints(:,windex);
            pos_w = robot.read_gps_meters();

            %No obstacles detected, goto seek waypoint
            %This may change if one is detected below
            robot.userdata.state = 2;

            robot.userdata.obstacle = intersect_map(robot.userdata.map,pos_w,waypoint_w,robot.userdata.obstacle_passing_threshold);
            if ~isempty(robot.userdata.obstacle)
                robot.userdata.state = 4; %Plan the new waypoint around it
            end
            

        case 2 %Seeking heading to waypoint
            
            if(abs(diff_angle_deg) < 5)
                %If roughly pointed, travel directly to waypoint
                if(distance > 0.1)
                    robot.set_wheel_speed_dps(1000,1000);
                else
                    robot.set_wheel_speed_dps(200,200);
                end
            else
                %Seek heading to waypoint
                if diff_angle_deg < 0
                    robot.set_wheel_speed_dps(200,-200);
                else
                    robot.set_wheel_speed_dps(-200,200);
                end
            end

            if distance < 0.005 %Have arrived
                robot.userdata.state = 3;
            end

        case 3 %At target
            %Turn on the associated LED:
            color = robot.read_floor_color();
            led_vals = robot.led_state();
            if color(1) > 0 && color(2) == 0 && color(3) == 0

                robot.led_state(true,led_vals(2),led_vals(3));
                
            elseif color(1) == 0 && color(2) > 0 && color(3) == 0

                robot.led_state(led_vals(1),true,led_vals(3));

            elseif color(1) == 0 && color(2) == 0 && color(3) > 0

                robot.led_state(led_vals(1),led_vals(2),true);
            end

            %Next waypoint?
            if robot.userdata.way_index ~= size(robot.userdata.waypoints,2)
                robot.userdata.way_index = robot.userdata.way_index + 1;
                %Seek next waypoint
                robot.userdata.state = 1; 
            else
                robot.set_wheel_speed_dps(0,0);
                robot.userdata.state = 5; %move to done state
            end

        case 4 
            %Plan an extra waypoint to go around obstacles
    
            %Find a point in the world that can act as an intermediate that
            %goes around the obstacle.
            obstacle = robot.userdata.obstacle;

            % Look along the normal of the line from the point of
            % intersection on the line.  This is a line search for a point
            % that is not within the threshold

            %Start from the line and work out in both directions.
            dx = linspace(-2,2,20);
            % if obstacle.s > 0
            % 
            % else
            %     dx = linspace(-0.2,-2,5);
            % end
            
            nl = obstacle.l_path(1:2)/norm(obstacle.l_path(1:2));
            mindist = inf;
            way = [];

            for i=1:length(dx) 
                % pl = map(:,i) - s*l_path(1:2);
                new_way = obstacle.on_line_point - dx(i)*nl;
                temp_obstacle = intersect_map(robot.userdata.map,pos_w,new_way,robot.userdata.obstacle_passing_threshold);

                if isempty(temp_obstacle)
                    %This is the new waypoint with the min dist from the
                    %existing line.
                    if(abs(dx(i)) < mindist )
                        way = new_way;
                        mindist = abs(dx(i));
                    end
                end
            end
            %Draw the new waypoint
            line(way(1),way(2),'color','r','marker','x')
            
            robot.userdata.waypoints = ...
                [robot.userdata.waypoints(:,1:(windex-1)),...
                way,...
                robot.userdata.waypoints(:,windex:end)];

            robot.userdata.state = 2; %Seek waypoint
        case 5 %Done state
            %Do nothing, finished
    end %Switch machine
end

function obstacle = intersect_map(map,pos_w,waypoint_w,threshold)
%Seek to see if some map points are within a threshold distance of the line
%between the two points specified.

    obstacle = [];
    l_path = cross([pos_w;1],[waypoint_w;1]);
    % l_path = [a,b,c] such that a*x + b*y + c = 0 for the line
    l_path = l_path/sqrt(l_path(1)^2 + l_path(2)^2); %make a^2 + b^2 = 1

    %The point on the line given the shortest distance is
    %If we find that a point in the map passes within a threshold
    %distance of our line, we have an obstacle in the way.
    
    %Distance from a point to the line is d = abs(a*x + b*y + c) if
    % a^2 + b^2 = 1. 
    %Scan through every point in the map
    min_dist = inf; %Find the nearest obstacle to the robot
    for i=1:size(map,2)
        %The signed distance from the line to the point is:
        s = l_path(1:2)'*map(:,i) + l_path(3);

        %If |s| is below some threshold based on body geometry, then
        %check to see if the closest point is on the line segment
        %between the waypoint and the current location
        if abs(s) < threshold %Specified in boot function above
            %The closest point on the line is p = [x;y] - s*[a;b]
            pl = map(:,i) - s*l_path(1:2);
            %Find the point closest to the line segment
            dist = abs(s);
            if(dist > min_dist) 
                continue; 
            end

            pt_a = waypoint_w - pl;
            pt_b = pos_w - pl;
            if (pt_a'*pt_b < 0)
                %The point is near the line segment (it is in the
                %way of the motion)

                obstacle.ref_point = map(:,i);
                obstacle.on_line_point = pl;
                obstacle.l_path = l_path;
                obstacle.pos_w = pos_w;
                obstacle.waypoint_w = waypoint_w;
                obstacle.s = s;

                min_dist = dist;
            end
        end
    end
end
