classdef RobotWorld < handle
    %WORLD in which the robot romes
    %   This world contains walls, obstacles, a robot, and a floor with
    %   colors
    %
    %   TODO: Add the ability to record video of the world adding one frame
    %   per timer call
    
    properties
        %Wall properties
        wall_corners
        %Obstacles - cylinders
        obstacles

        %data structure containing the floor image
        floor
        dirt
        dump_center

        %Magnetic field vector for the world
        % Units are nanotesla
        magnetic_field_nT

        %Robots
        robot

        %Global time system
        timer
        
        %Graphics
        fig
        ax
        h_title
        h_wall
        h_floor
        h_obstacles
        h_dirt
    end
    
    methods
        function add_dirt(world)
            %Add specs of dirt all over the floor that the robot will
            %cleanup.
            
            %Compute the space the walls enclose
            minx = min(world.wall_corners(1,:));
            maxx = max(world.wall_corners(1,:));
            miny = min(world.wall_corners(2,:));
            maxy = max(world.wall_corners(2,:));
            meanval(1) = (minx + maxx)/2;
            meanval(2) = (miny + maxy)/2;
            amp(1) = maxx - meanval(1);
            amp(2) = maxy - meanval(2);

            %Create random values in this space
            npoints = 1000;
            data = (rand([2,npoints])-0.5)*2; %random values between [-1 1]
            data(1,:) = data(1,:) * amp(1)*.98 + meanval(1);
            data(2,:) = data(2,:) * amp(2)*.98 + meanval(2);

            %Eliminate dirt outside the walls
            in = inpolygon(data(1,:),data(2,:),world.wall_corners(1,:),world.wall_corners(2,:));
            data = data(:,in);
            
            %Eliminate dirt inside obstacles
            for i=1:length(world.obstacles)
                in = inpolygon(data(1,:),data(2,:),...
                    world.obstacles(i).corners(1,:),world.obstacles(i).corners(2,:));
                data = data(:,~in);
            end

            world.dirt = [world.dirt,data];
            set(world.h_dirt,'xdata',world.dirt(1,:),'ydata',world.dirt(2,:))
        end
        function add_dump_station(world,dump_center)
            world.dump_center = dump_center;
            world.draw_circle(dump_center,0.1,0.02,[0,0,0]);
        end

        function draw_circle(world,center,radius,thickness,color)
            %All measurements in meters
            C = world.floor.C;
            x = world.floor.x;
            y = world.floor.y;

            for i=1:size(C,2)
                for j=1:size(C,1)
                    d = sqrt((center(1) - x(i))^2 + (center(2)-y(j))^2);
                    if (d>=(radius-thickness/2)) && (d<=(radius+thickness/2))
                        C(j,i,:) = color;
                    end
                end
            end

            world.floor.C = C;
            set(world.h_floor,'cdata',C);
        end

        function draw_dot(world,center,radius,color)
            %Measurements in meters, color is 3-vector [r,g,b] (0-255)
            C = world.floor.C;
            x = world.floor.x;
            y = world.floor.y;

            for i=1:size(C,2)
                for j=1:size(C,1)
                    d = sqrt((center(1) - x(i))^2 + (center(2)-y(j))^2);
                    if (d<=radius)
                        C(j,i,:) = color;
                    end
                end
            end
            world.floor.C = C;
            set(world.h_floor,'cdata',C);
        end
        function draw_square(world,center,side_length,thickness,color)
            pt1 = center + [-side_length/2;  side_length/2];
            pt2 = center + [ side_length/2;  side_length/2];
            pt3 = center + [ side_length/2; -side_length/2];
            pt4 = center + [-side_length/2; -side_length/2];
            world.draw_line(pt1,pt2,thickness,color);
            world.draw_line(pt2,pt3,thickness,color);
            world.draw_line(pt3,pt4,thickness,color);
            world.draw_line(pt4,pt1,thickness,color);
        end

        function draw_line(world,point_a,point_b,thickness,color)
            %Draw a straight line from pointa to pointb
            %Calculate the line:
            lab = cross([point_a;1],[point_b;1]);
            nab = [lab(1:2);0]/norm(lab(1:2));

            C = world.floor.C;
            x = world.floor.x;
            y = world.floor.y;

            for i=1:size(C,2)
                for j=1:size(C,1)
                    pt = [x(i);y(j);1];
                    %Find the line from the point to the line
                    lpt = cross(pt,pt+nab);
                    %Find the intersection (orthogonal distance)
                    ptx = cross(lpt,lab);
                    ptx = ptx(1:2)/ptx(3);
                    %if this point is between the two points on the line
                    %and within a certain distance from the line
                    %(thickness/2) then make it the color
                    ptax = point_a - ptx;
                    ptbx = point_b - ptx;
                    if ptax'*ptbx > 0
                        continue;
                    end
                    d = sqrt(sum((ptx - pt(1:2)).^2));

                    if (d<=thickness/2)
                        C(j,i,:) = color;
                    end
                end
            end
            world.floor.C = C;
            set(world.h_floor,'cdata',C);
            
        end

        function world = RobotWorld(wall_points,obstacles)

            world.floor.C = ones(1000,1000,3)*255; %All white floor
            %TODO: keep a certain "resolution" (pixels/m)
            xrange = [min(wall_points(1,:)), max(wall_points(1,:))];
            yrange = [min(wall_points(2,:)), max(wall_points(2,:))];
            world.floor.x = linspace(xrange(1),xrange(2),size(world.floor.C,2));
            world.floor.y = linspace(yrange(1),yrange(2),size(world.floor.C,1));
            %Draw the image on the floor
            hold on
            world.h_floor = image(world.floor.x,world.floor.y,world.floor.C);
            

            %Boundary walls and obstacles
            world.wall_corners = wall_points;
            
            %Dirt
            world.dirt = zeros([2,0]);
            world.h_dirt = line(nan,nan,'color',[0.8,0.6,0.6],'marker','.','linestyle','none');
            world.dump_center = [0;0];

            %Magnetic Field points "north" (positive y-direction)
            % units are nanoTesla (roughly what it is over New York State)
            world.magnetic_field_nT = [0,52000]';

            %Draw the walls and frame the world
            world.fig = gcf;
            world.h_title = title('0 sec'); 
            world.ax = gca;
            world.h_wall = line(world.wall_corners(1,:),world.wall_corners(2,:),'linewidth',2,'color','r');
            axis equal
            set(world.ax,'xgrid','on','ygrid','on')
            %TODO: make this scale to 20% more than the wall bounding box
            mx = mean(xrange);
            my = mean(yrange);
            xrange = (xrange - mx)*1.1 + mx;
            yrange = (yrange - my)*1.1 + my;
            set(world.ax,'xlim',xrange,'ylim',yrange)

            %Draw square obstacles in the world
            world.obstacles = obstacles;
            for i=1:length(world.obstacles)
                d = world.obstacles(i).side_length_m;
                pts(1,:) = [-1, 1, 1,-1,-1]*d/2 + world.obstacles(i).center(1);
                pts(2,:) = [ 1, 1,-1,-1, 1]*d/2 + world.obstacles(i).center(2);
                world.obstacles(i).corners = pts;
                
                world.h_obstacles(i) = line(...
                    pts(1,:),pts(2,:),...
                    'color',world.obstacles(i).color,'linewidth',2);
            end

            %Setup world time system
            world.timer = timer;
            world.timer.TimerFcn = @robot_timer;
            world.timer.Period = 0.05; %tick every 50ms (20Hz)
            world.timer.TasksToExecute = inf;
            world.timer.ExecutionMode = 'fixedRate';
            world.timer.BusyMode = 'queue';

            world.timer.TimerFcn = @(~,~)world.updateWorld();
            set(world.fig,'DeleteFcn', @(~,~)delete(world))

            set(world.fig,'KeyPressFcn',@(~,event)world.figureKeypress(event));

            %The user must add a robot with the apprpriate control
            %functions using the "world.add_robot(...)" method
            world.robot = Robot.empty(0,0);
        end
        function delete(world)
            %Finish the video
            world.timer.stop();
            delete(world.timer);
        end

        function add_robot(world,robot)
            world.robot(end+1) = robot;
        end

        function start(world)
            world.timer.start();
        end
        function stop(world)
            world.timer.stop();
        end
        function figureKeypress(world,event)
            for i=1:length(world.robot)
                world.robot(i).remote_control(event.Key);
            end
        end
        
        function updateWorld(world)
            %This is the main time function that updates the world at the
            %given timer interval
            
            %If recording video, do that here

            dt = world.timer.Period;
            for i = 1:length(world.robot)
                world.robot(i).update(dt);
            end
        end
    end
end

