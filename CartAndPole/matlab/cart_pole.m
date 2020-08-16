classdef cart_pole < handle
    properties
        M % mass of cart
        m % mass of inverted pendulum
        I % moment of inertia for inverted pendulum
        L % length of iverted pendulum
        x % position of cart
        x_dot % velocity of cart
        theta % angular position of pendulum
        theta_dot % angular velocity
        cart_dim = struct('w',5,'h',2.5,'xs',[],'ys',[]);
        pole_dim = struct('xb',0,'yb',2.5,'theta',pi/2);
        cart_color;
        plt_cart;
        plt_pole;
        g = 9.18;
        dt = 0.1;
        t = 0.0;
        rec_gif = true;
        gif_file = struct("name","inverted_pend.gif");
    end
    methods
        function init_params(obj)
            obj.M = 10;
            obj.m = 1;
            obj.I = 1;
            obj.L = 5;
            obj.x = 0;
            obj.x_dot;
            obj.x_dot = 0;
            obj.theta = 0;
            obj.theta_dot = 0;
            w = obj.cart_dim.w;
            h = obj.cart_dim.h;
            obj.cart_dim.xs = [-w/2 w/2 w/2 -w/2];
            obj.cart_dim.ys = [0 0 h h];
            obj.cart_color = [0.5 0.5 0.5];
            obj.pole_dim.x = [];
        end
        function plot(obj)
            hold on;
            obj.plt_cart = patch(obj.cart_dim.xs,obj.cart_dim.ys,obj.cart_color);
            obj.plt_pole = plot([obj.pole_dim.xb (obj.L*cos(obj.theta+pi/2) + obj.pole_dim.xb)],...
                [obj.pole_dim.yb (obj.L*sin(obj.theta+pi/2) + obj.pole_dim.yb)],"LineWidth",3);
            axis equal;
            xlim([-20,20]);
            ylim([0,8]);
        end
        function update_plot(obj)
           obj.plt_cart.XData = obj.cart_dim.xs;
           obj.plt_cart.YData = obj.cart_dim.ys;
           obj.plt_pole.XData = [obj.pole_dim.xb (obj.L*cos(obj.theta+pi/2)+obj.pole_dim.xb)];
           obj.plt_pole.YData = [obj.pole_dim.yb (obj.L*sin(obj.theta+pi/2)+obj.pole_dim.yb)];
        end
        function dxdt = EOM(obj,~,x,u,d)
            % x, theta, x_dot, theta_dot
            % u = corrective force
            % d = disturbance
            m = obj.m;
            M = obj.M;
            L = obj.L;
            theta = x(2);
            x_dot = x(3);
            theta_dot = x(4);
            g = obj.g;
            
            x_ddot = (u + (3*m*g)/2*sin(theta)*cos(theta) - m*L/2*theta_dot^2*sin(theta))/...
                (M + m*(1-3/4*cos(theta)^2));
            
            theta_ddot = d + 3*(g*sin(theta)+x_ddot*cos(theta))/(2*L);
            
            dxdt = [x_dot;...
                theta_dot;...
                x_ddot;...
                theta_ddot];
        end
        function step(obj,u,d)
            x_state = [obj.x;
                obj.theta;
                obj.x_dot;
                obj.theta_dot];
            [t,x_state] = ode45(@(t,x) obj.EOM(t,x,u,d),[obj.t obj.t+obj.dt],x_state);
            obj.t = t(end);
            obj.x = x_state(end,1);
            obj.theta = x_state(end,2);
            obj.x_dot = x_state(end,3);
            obj.theta_dot = x_state(end,4);
            obj.cart_dim.xs = obj.cart_dim.w/2*[-1 1 1 -1] + obj.x*ones(1,4);
            obj.pole_dim.xb = obj.x;
            obj.pole_dim.theta = obj.theta;
            obj.update_plot();
            if obj.rec_gif
               obj.rec_frame(gcf); 
            end
        end
        function rec_frame(obj,fh)
            persistent n;
            if isempty(n)
               n = 1; 
            end
            frame = getframe(fh);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            if n==1
                imwrite(imind,cm,obj.gif_file.name,'gif', 'Loopcount',inf); 
            else
                imwrite(imind,cm,obj.gif_file.name,'gif','DelayTime',0.1,'WriteMode','append');
            end
            n = n + 1;
        end
    end
end