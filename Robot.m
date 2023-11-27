classdef Robot < handle
    %ROBOT - a simulation of a TI Innovation Rover
    %   This robot has a pair of independently controllable wheels each 
    %   with a wheel position encoder, a downward facing color sensor, a 
    %   forward facing range sensor, a gyroscope measuring angular rate,
    %   and three LEDs (rgb) which are user programmable

    % TODO: add start/stop/step buttons to  figure 
    %  - Add battery level and reading.  The robot will stop if the battery
    %  runs out.  Need to have a charging station in the world or just
    %  making it a ticking clock to complete a task.
    % - Add a sensor that gives a location of the charging port somehow

    
    properties
        userdata %Place for user to track parameters
    end
    
    methods
        %Below are functions that let the user control the robot or read
        %from its sensors - this is the robot's API (Application
        %Programming Interface)

        %Functions to read from vehicle sensors
        function color = read_floor_color(robot)
            color = robot.floor_color;
        end
        function range = read_range(robot)
            range = robot.range;
        end
        function [left_angle_deg, right_angle_deg] = read_wheel_encoders(robot)
            left_angle_deg = robot.left_angle_deg;
            right_angle_deg = robot.right_angle_deg;
        end
        function angular_rate_dps = read_gyro_dps(robot)
            angular_rate_dps = robot.omega_gyro * 180/pi;
        end
        function [left_dps, right_dps] = read_wheel_speed_dps(robot)
            left_dps = robot.left_dps;
            right_dps = robot.right_dps;
        end

        %Functions for handling dirt and batteries
        function n_dirt = count_dirt(robot)
            n_dirt = robot.n_dirt;
        end
        function vac_state = vacuum(robot,on_off)
            %Turn on/off the vacuum - on = true, off = false
            %Also read the vacuum state
            if(nargin == 2)
                robot.dirt_on_off = on_off; %true or false
            end
            vac_state = robot.dirt_on_off; %optional output
        end
        function dump_dirt(robot)
            %Cause the robot to dump its dirt on the ground below it
            location = robot.read_gps_meters();
            extra_dirt = randn(2,robot.n_dirt)*robot.width/4 + repmat(location,[1,robot.n_dirt]);
            robot.world.dirt = [robot.world.dirt,extra_dirt];
            robot.n_dirt = 0;
        end
        function max = set_max_dirt(robot,max)
            %A user configuration that limits the amount of dirt the robot
            %can handle to simulate a real robot vacuum
            %Also reads out the max amoung of dirt
            if(nargin == 2)
                robot.max_dirt = max;
            end
            if(nargout == 1)
                max = robot.max_dirt;
            end
        end
        function dump_waypoint = find_dump_station(robot)
            % "Some technique" gives the location of the dump center in the
            % world frame.
            dump_waypoint = robot.world.dump_center;
        end

        %Sensors for students to "install" on the robot
        function xy_position_meters = read_gps_meters(robot)
            % Method functions like GPS - report position
            % Bonus: also report velocity like a GPS system does
            
            %My Solution: return the midpoint of the axle
            xy_position_meters = (robot.left_center + robot.right_center)/2;
        end
        function heading_deg = read_compass_degrees(robot)
            %return the forward direction in degrees (0 being to the right)
            % Acts like a compass

            %This is the "worlds magnetic field" seen by the robot
            mag_body_nT = robot.read_magnetic_field_nT();

            %Student Project: How to determine the heading of the robot?
            %You know where north is from the perspective of the robot, now
            %how do you use this to tell how the robot is pointed in the
            %world.  North is in the positive y-direction in the world.

            %My Solution
            mag_b = mag_body_nT/norm(mag_body_nT);
            %mag_w = [0;1];

            %This treats "north" (along the y-axis) as 90 degrees
            % The normal navigation convention is that it is that north is 
            % zero, but in cartesian/polar coordinates, 0 degrees is 
            % usually pointed along the positive x-axis.
            heading_deg = atan2(mag_b(2),mag_b(1)) * 180/pi;
            if(heading_deg < 0)
                heading_deg = heading_deg + 360;
            end

        end
        

        %Functions to set actuators
        function set_wheel_speed_dps(robot,left_dps,right_dps)
            robot.left_dps = left_dps;
            robot.right_dps = right_dps;
        end
        
        %Input/Output functions - set or read
        function vals = led_state(robot,red_on, green_on, blue_on)
            %The inputs are boolean values like robot.set_leds(true,false,true)
            if(nargin == 4)
                robot.led_values = [red_on,green_on,blue_on];
            end
            vals = robot.led_values; %optionally read out RGB values
        end

        %Visualization functions
        function set_track_history(robot,on_off)
            %Draw green history track behind the vehicle
            robot.track_history = on_off; %True for on and False for off
            if(robot.track_history)
                robot.center_history(:,end+1) = zeros(2,1)*nan; %add a gap
            end
        end
        function set_draw_icc(robot,on_off)
            %Draw the instantaneous center of curvature of the motion
            robot.draw_icc = on_off;
        end
        
    end

%-----------------------------------------------------------------------%
%Below this are the internals of the robot that are not necessary for
%programming it to act in the world
    properties (Access = private)
        %Dynamics
        %Wheel Speed in degrees per second (dps):
        %Actuators:
        left_dps
        right_dps
        leds

        %Sensors
        left_angle_deg
        right_angle_deg
        omega_gyro
        floor_color
        range

        world %The world that contains it

        %Body part coordinates
        body
        wheel_left
        wheel_right

        center
        left_center
        right_center
        led_r
        led_g
        led_b
        icc
        
        %other Properties
        width 
        len
        wheel_radius
        wheel_base
        color_sensor
        range_sensor 
        time_s
        led_values

        %Dirt details
        n_dirt
        dirt_on_off
        max_dirt

        %data
        center_history

        %User methods
        user_setup_function
        user_navigation_function

        %Graphics elements
        track_history
        draw_icc
        graphics
    end

    %Function the User doesn't need to look at are here and below.  These
    %are the internal workings of the robot
    methods
        %constructor
        function robot = Robot(world,setupfcn,navfcn)
            %ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            robot.world = world;
            robot.user_setup_function = setupfcn;
            robot.user_navigation_function = navfcn;

            robot.width = 0.10; %meters
            robot.len = 0.19; %meters
            robot.wheel_radius = 0.02; %meters
            
            robot.left_dps = 0; %degrees per second
            robot.right_dps = 0; %degrees per second
            robot.left_angle_deg = 0; %Absolute angular position for encoder
            robot.right_angle_deg = 0;
            robot.time_s = 0;

            robot.body(1,:) = [0,  0, robot.width, robot.width, 0]; %x values of body (meters)
            robot.body(2,:) = [0,robot.len, robot.len, 0,    0]; %Y values of body edges (meters)
            robot.wheel_right(1,:) = [.10, .10, .12, .12, .10];
            robot.wheel_right(2,:) = [.13, .17, .17, .13, .13];
            robot.wheel_left(1,:) = [-.02, -0.02, 0, 0, -0.02];
            robot.wheel_left(2,:) = [.13, .17, .17, .13, .13];
            
            robot.center = mean(robot.body(:,1:4),2);
            robot.right_center = mean(robot.wheel_right(:,1:4),2);
            robot.left_center = mean(robot.wheel_left(:,1:4),2);
            robot.color_sensor = [robot.width/2; robot.len - 0.02];
            robot.range_sensor = [robot.width/2; robot.len];


            robot.led_r(1,:) = [-1, -1, 1, 1, -1]*0.015;
            robot.led_r(2,:) = [-1, 1, 1, -1, -1]*0.015;
            robot.led_b = robot.led_r + repmat([robot.width/2;0.11],[1,5]);
            robot.led_g = robot.led_r + repmat([robot.width/2;0.07],[1,5]);
            robot.led_r = robot.led_r + repmat([robot.width/2;0.03],[1,5]);
            robot.led_values = [false,false,false];

            robot.wheel_base = norm(robot.left_center - robot.right_center);
            robot.center_history = zeros([2,0]);
            robot.icc = zeros(2,1)*nan;
            
            robot.n_dirt = 0;
            robot.dirt_on_off = false;
            robot.max_dirt = inf;

            robot.read_color();

            %Draw the robot initially in the axis
            robot.track_history = false;
            robot.draw_icc = false;
            robot.graphics.h_history = line(robot.world.ax,nan,nan);
            robot.graphics.h_icc = line(robot.world.ax,nan,nan,'marker','+','color','k','markersize',20);

            robot.graphics.h_body = line(robot.world.ax,robot.body(1,:),robot.body(2,:));
            robot.graphics.h_wheel_right = patch(robot.world.ax,robot.wheel_right(1,:),robot.wheel_right(2,:),'r','edgecolor','none');
            robot.graphics.h_wheel_left = patch(robot.world.ax,robot.wheel_left(1,:),robot.wheel_left(2,:),'r','edgecolor','none');
            robot.graphics.h_color_sensor = line(robot.world.ax,robot.color_sensor(1),robot.color_sensor(2),'marker','+','color','k');
            robot.graphics.h_range_sensor = line(robot.world.ax,robot.range_sensor(1),robot.range_sensor(2),'marker','*','color','k');
            
            robot.graphics.h_led_r = patch(robot.world.ax,robot.led_r(1,:),robot.led_r(2,:),'k','edgecolor','none');
            robot.graphics.h_led_g = patch(robot.world.ax,robot.led_g(1,:),robot.led_g(2,:),'k','edgecolor','none');
            robot.graphics.h_led_b = patch(robot.world.ax,robot.led_b(1,:),robot.led_b(2,:),'k','edgecolor','none');

            set(robot.graphics.h_body,'marker','none','linewidth',2);
            set(robot.graphics.h_history,'linewidth',1,'color','g');
            
            %Call the user's setup functions once everything else is set
            robot.user_setup_function(robot);
            drawnow
        end

        function remote_control(robot,key)
            %The UI can send letter commands to the robot (e.g. from the
            %keyboard) to cause it to change behaviors.
            velocity_step_dps = 100; %degrees per second on little wheels
            switch key
                case 'uparrow'
                    robot.set_wheel_speed_dps(...
                        robot.left_dps + velocity_step_dps,...
                        robot.right_dps + velocity_step_dps);
                case 'downarrow'
                    robot.set_wheel_speed_dps(...
                        robot.left_dps - velocity_step_dps,...
                        robot.right_dps - velocity_step_dps);
                case 'rightarrow'
                    robot.set_wheel_speed_dps(...
                        robot.left_dps + velocity_step_dps,...
                        robot.right_dps - velocity_step_dps);
                case 'leftarrow'
                    robot.set_wheel_speed_dps(...
                        robot.left_dps - velocity_step_dps,...
                        robot.right_dps + velocity_step_dps);
                case 'space'
                    robot.set_wheel_speed_dps(0,0);
                case 'r' %Toggle Red LED
                    robot.led_values(1) = ~robot.led_values(1);
                case 'g' %Toggle green LED
                    robot.led_values(2) = ~robot.led_values(2);
                case 'b' %Toggle Blue LED
                    robot.led_values(3) = ~robot.led_values(3);
                case 'm'
                    %Activate mapping?
            end
        end
    
        %Main timer function runs at the timer period
        function update(robot,dt)
            %Primary time evolution function for the robot.
            %Update all the rigid body coordinates of the robot
            robot.time_s = robot.time_s + dt;

            %Limit speed
            max_speed = 360*3;
            if(robot.left_dps > max_speed), robot.left_dps = max_speed; end
            if(robot.right_dps > max_speed), robot.right_dps = max_speed; end

            %Differential drive kinematics
            % See: https://www.cs.columbia.edu/~allen/F19/NOTES/icckinematics.pdf
            %Velocity of left wheel:
            omega_left = robot.left_dps * (pi/180);
            Vl =  omega_left * robot.wheel_radius; %Linear wheel velocity (left)
            %Velocity of right wheel:
            omega_right = robot.right_dps * (pi/180);
            Vr =  omega_right * robot.wheel_radius; %Linear wheel velocity (right)
            
            %The body will rotate if there is a differential rotation
            omega_rad = (Vr-Vl)/robot.wheel_base;
            velocity = (Vl+Vr)/2; %linear velocity component
            axle_center = (robot.left_center + robot.right_center)/2;

            %compute the motion, it is either a turn or a translation
            % Don't apply the motion until we search for collisions
            dx = [0;0];
            dtheta = 0;
            
            
            if(omega_rad ~= 0)
                %Some rotation
                %t = location of the "instantaneous center of curvature" (icc)
                t = (robot.wheel_base/2)*(Vl+Vr)/(Vr-Vl);
                %Calculate center of rotation
                axle_dir = (robot.left_center - robot.right_center);
                axle_dir = axle_dir/norm(axle_dir);
                %"icc" = Instantaneous Center of Curvature
                robot.icc = axle_center + axle_dir * t;
                dtheta = omega_rad*dt;
            else
                forward = axle_center - robot.center;
                forward = forward/norm(forward); %Unit length vector
                robot.icc = [nan;nan]; %icc is at infinity for pure translation
                dist =  dt * velocity;
                dx = forward * dist;
            end

            
            %Check collisions with obstacles and walls
            % - The obstacles can be smaller than the body, so we must
            % check to see if they intersect the lines of the body instead
            % of see if the corners cross into the obstacle. The robot could be going so fast that it jumps
            % past the entire obstacle so that it's still inside the body?

            effective_dt = dt;
            obstacles = robot.world.obstacles;
            %This trick just treats the walls of the world as another
            %obstacle to be avoided.  Don't allow body lines to intersect
            %with wall lines.  If this happens, cancel the step
            obstacles(end+1).corners = robot.world.wall_corners;
            %For each wall of the body
            bc = robot.body;
            %Calculate the next step
            if(omega_rad ~= 0)
                R = robot.rotation_matrix(dtheta);
                bcdx = R*(bc - repmat(robot.icc,[1,size(bc,2)]));
                bcdx = bcdx + repmat(robot.icc,[1,size(bc,2)]);
            else
                bcdx = bc + repmat(dx,[1,size(bc,2)]);
            end

            %For each obstacle (includes walls)
            for k=1:length(obstacles)
                %Extract the points for the obstacle's corners
                oc = obstacles(k).corners;
                %For each wall of each obstacle (each pair of points)
                found = false;
                for i = 1:(size(oc,2)-1)
                    %Extract the points at either end of the wall segment
                    pt0 = oc(:,i);
                    pt1 = oc(:,i+1);
                    pt10 = pt1 - pt0;
                    %Parameterize the points on the wall line as:
                    %   pt = pt0 + s * pt10
                    % For this parameterization, a point with s between
                    % 0 and 1 is on the line.  Any other value of s is off 
                    % the wall segment.  
                    
                    for j=1:(size(bcdx,2)-1)
                        %The updated body perimeter wall segment of the
                        %robot itself.
                        pb0 = bcdx(:,j);
                        pb1 = bcdx(:,j+1);
                        pb10 = pb1 - pb0;
                        %parameterized wall segment as pb = pb0 + d * pb10
                        % d = 0 for one corner and d = 1 for the other
                        
                        %Solve for the intersection of the two wall
                        %segments (intersect two lines and find the point).
                        A = [pt10, -pb10]; b = pb0 - pt0;
                        if(abs(det(A))<1e-15), continue; end %lines are parallel
                        sd = A\b;
                        s = sd(1);
                        d = sd(2);

                        %If the intersection point is on both walls
                        %(between 0 and 1 for s and d), then the two walls
                        %intersect.
                        if(s>=0)&&(s<=1)&&(d>=0)&&(d<=1)
                            %If this conditional is met, After the 
                            % motion, body lines DO intersect with object
                            % lines (wall or obstacles).  TODO: If the
                            % robot is going so fast that it jumps entirely
                            % past the object, this will miss, but that
                            % shouldn't be possible with speed limiting.

                            %NAIVE: Just terminate the step:
                            dx = dx*0;
                            dtheta = 0;
                            omega_rad = 0;
                            effective_dt = 0;
                            found = true;
                            break;

                            %The more complex approach to this is to solve
                            %for the sub-motion that brings the robot
                            %exactly up to the wall.  This is a complex
                            %problem involving solving many linear systems
                            %as well as circle-line intersections.  I have
                            %derived it in the bottom of the Intro to
                            %Robotics class 12 notes, but when I
                            %implemented it, it seems to need some real
                            %careful handling and it's a ton of code.
                            %This naive solution is actually pretty nice
                            %even though it stops you a distance from the
                            %wall proportional to your speed and your
                            %starting point.
                            
                        end
                    end
                    if(found), break; end
                end
                if(found), break; end
            end
            

            %Finally apply the collision corrected motion
            if(omega_rad ~= 0) %If turning
                robot.translate(-robot.icc);
                robot.rotate(dtheta);
                robot.translate(robot.icc);
            else
                robot.translate(dx); %If pure translation
            end

            
            %Track motor angles over time for the wheel encoders
            robot.left_angle_deg = robot.left_angle_deg + robot.left_dps*effective_dt;
            robot.right_angle_deg = robot.right_angle_deg + robot.right_dps*effective_dt;

            %Draw history trail if requested
            if(robot.track_history)
                robot.center_history = [robot.center_history,robot.center];
            end
            
            robot.read_color(); %Read out the floor color under the robot
            robot.calc_range(); %Compute range to nearest object
            robot.omega_gyro = omega_rad; %Update gyro value
            
            %Vacuum dirt
            if(robot.dirt_on_off)
                robot.vacuum_dirt();
            end
            
            %Update graphics
            robot.draw_robot();

            %Call the user's navigation program
            robot.user_navigation_function(robot,dt);
            drawnow
        end

    end

    methods (Access = private)

        function calc_range(robot)
            %Compute a range measurement to an obstacle or wall

            %Move range sensor to origin along with 
            % robot center, and walls locally (not on the actual robot)
            range_pt = robot.range_sensor - robot.range_sensor;%[0,0]
            center_pt = robot.center - robot.range_sensor;
            w_corners = robot.world.wall_corners;
            obstacles = robot.world.obstacles;
            %Tack wall lines together with obstacle wall lines
            for i=1:length(obstacles) 
                w_corners = [w_corners,[nan;nan],obstacles(i).corners];
            end
            
            for i=1:size(w_corners,2)
                w_corners(:,i) = w_corners(:,i) - robot.range_sensor;
            end

            %The unit vector pointing out of the body from the center to
            %the range sensor center.  This could be tweaked with a
            %mounting error such that this angle is off a bit.
            dir = range_pt - center_pt;
            dir = dir/norm(dir); %unit vector

            %For all pairs of points in the world and obstacle borders
            %Find an intersection
            dist = inf;
            for i=1:(size(w_corners,2)-1)
                if isnan(w_corners(1,i))||isnan(w_corners(1,i+1))
                    continue
                end

                pt0 = w_corners(:,i);
                pt1 = w_corners(:,i+1);
                pt10 = pt1 - pt0;

                %Parameterize the range as p_r = d*dir
                %Parameterize the wall as p_w = p0 + s*p10
                %Solve for s and d in this 2x2 linear system where p_r = p_w.  
                % If s is between [0,1] and d > 0 then it's an intersection
                A = [pt10, -dir]; b = -pt0;
                % if(det(A) == 0), continue; end %If line and ray are parallel
                if(abs(det(A))<1e-14), continue; end
                sd = A\b;
                s = sd(1); 
                d = sd(2);
                
                if (s>=0)&&(s<=1) %Between the wall corners?
                    if(d>=0) %In front of robot?
                        if(d<dist) %Closer than previous distance
                            dist = d; %Accept it
                        end
                    end
                end
            end
            robot.range = dist;
        end

        function read_color(robot)
            %Find the nearest discrete index into the image at the color
            %sensor and lookup the pixel value there.  Much faster than
            %interp2 calls
            mx = abs(robot.world.floor.x-robot.color_sensor(1));
            indx = find(mx == min(mx));
            my = abs(robot.world.floor.y-robot.color_sensor(2));
            indy = find(my == min(my));
            for i=1:3
                robot.floor_color(i,1) = robot.world.floor.C(indy,indx,i);
            end
        end

        function vacuum_dirt(robot)
            %Remove any dirt within the body frame from the world
            in = inpolygon(robot.world.dirt(1,:),robot.world.dirt(2,:),...
                robot.body(1,:),robot.body(2,:));

            %"Pick up" the dirt that's within the perimeter of the
            % robot's body.
            in_dirt = robot.world.dirt(:,in);
            robot.world.dirt = robot.world.dirt(:,~in);

            %Count the number of dirt specs picked up?
            % Make sure it doesn't exceed max capacity
            new_dirt_count = robot.n_dirt + sum(in);
            if(new_dirt_count > robot.max_dirt)
                delta = new_dirt_count - robot.max_dirt;
                %Put some of the dirt back
                robot.world.dirt = [robot.world.dirt, in_dirt(:,1:delta)];
                robot.n_dirt = robot.max_dirt;
            else
                robot.n_dirt = new_dirt_count;
            end

        end

        function mag_b_nT = read_magnetic_field_nT(robot)
            
            mag_b_nT = robot.R_body_from_world() * robot.world.magnetic_field_nT;

        end

        function R_bw = R_body_from_world(robot)
            %Calculate body_from_world rotation
            %This should map world points into body frame points
            % it is just one angle:

            %Forward in the world frame
            axle_center = (robot.left_center + robot.right_center)/2;
            body_center = robot.center;
            direction_w = axle_center - body_center;
            direction_w = direction_w/norm(direction_w);

            %Forward in the body frame (y-direction)
            direction_b = [0 1]';

            %Both these "forward" vectors have amplitude of 1 so:
            % dot product gives cos(θ)
            costh = direction_b'*direction_w;
            % cross product gives sin(θ)
            b_x_w = cross([direction_b;0],[direction_w;0]);
            sinth = b_x_w(3); %will always be in "z-direction"
            
            R_bw = [costh, -sinth; 
                    sinth, costh];
        end

        function translate(robot,dx)
            %Shift all body points by dx
            for i=1:size(robot.body,2)
                robot.body(:,i) = robot.body(:,i) + dx;
            end
            for i=1:size(robot.wheel_left,2)
                robot.wheel_left(:,i) = robot.wheel_left(:,i) + dx;
            end
            for i=1:size(robot.wheel_right,2)
                robot.wheel_right(:,i) = robot.wheel_right(:,i) + dx;
            end
            robot.center = robot.center + dx;
            robot.left_center = robot.left_center + dx;
            robot.right_center = robot.right_center + dx;
            robot.color_sensor = robot.color_sensor + dx;
            robot.range_sensor = robot.range_sensor + dx;
            robot.led_r = robot.led_r + dx;
            robot.led_g = robot.led_g + dx;
            robot.led_b = robot.led_b + dx;
        end
        function rotate(robot,theta_rad)
            %Rotate all points around the origin by theta
            R = robot.rotation_matrix(theta_rad);
    
            robot.body = R*robot.body;
            robot.wheel_left = R*robot.wheel_left;
            robot.wheel_right = R*robot.wheel_right;
            robot.center = R*robot.center;
            robot.left_center = R*robot.left_center;
            robot.right_center = R*robot.right_center;
            robot.color_sensor = R*robot.color_sensor;
            robot.range_sensor = R*robot.range_sensor;
            robot.led_r = R*robot.led_r;
            robot.led_g = R*robot.led_g;
            robot.led_b = R*robot.led_b;
        end
        function R = rotation_matrix(~,theta_rad)
            R = [cos(theta_rad),-sin(theta_rad);
                 sin(theta_rad), cos(theta_rad)];
        end

        function draw_robot(robot)
            set(robot.graphics.h_body,'xdata',robot.body(1,:),'ydata',robot.body(2,:));
            set(robot.graphics.h_wheel_right,'xdata',robot.wheel_right(1,:),'ydata',robot.wheel_right(2,:));
            set(robot.graphics.h_wheel_left,'xdata',robot.wheel_left(1,:),'ydata',robot.wheel_left(2,:));
            if robot.track_history
                set(robot.graphics.h_history,'xdata',robot.center_history(1,:),'ydata',robot.center_history(2,:));
            end
            if robot.draw_icc
                set(robot.graphics.h_icc,'xdata',robot.icc(1),'ydata',robot.icc(2))
            end

            set(robot.graphics.h_color_sensor,'xdata',robot.color_sensor(1),'ydata',robot.color_sensor(2));
            set(robot.graphics.h_range_sensor,'xdata',robot.range_sensor(1),'ydata',robot.range_sensor(2));

            set(robot.graphics.h_led_r,'xdata',robot.led_r(1,:),'ydata',robot.led_r(2,:));
            set(robot.graphics.h_led_g,'xdata',robot.led_g(1,:),'ydata',robot.led_g(2,:));
            set(robot.graphics.h_led_b,'xdata',robot.led_b(1,:),'ydata',robot.led_b(2,:));

            set(robot.world.h_dirt,'xdata',robot.world.dirt(1,:),'ydata',robot.world.dirt(2,:));
            
            if robot.right_dps == 0
                set(robot.graphics.h_wheel_right,'facecolor','r')
            else
                set(robot.graphics.h_wheel_right,'facecolor','g')
            end
            if robot.left_dps == 0
                set(robot.graphics.h_wheel_left,'facecolor','r')
            else
                set(robot.graphics.h_wheel_left,'facecolor','g')
            end

            set(robot.world.h_title,'string',sprintf('%.1f seconds',robot.time_s));
            
            if(robot.led_values(1))
                set(robot.graphics.h_led_r,'facecolor','r');
            else
                set(robot.graphics.h_led_r,'facecolor','k');
            end
            if(robot.led_values(2))
                set(robot.graphics.h_led_g,'facecolor','g');
            else
                set(robot.graphics.h_led_g,'facecolor','k');
            end
            if(robot.led_values(3))
                set(robot.graphics.h_led_b,'facecolor',[0.2,0.2,1]);
            else
                set(robot.graphics.h_led_b,'facecolor','k');
            end
        end
    end
end

