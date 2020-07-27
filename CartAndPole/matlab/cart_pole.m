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
        pole_dim = struct('xb',0,'yb',2.5,'x',0,'y',2.5);
        cart_color;
        plt_cart;
        plt_pole;
    end
    methods
        function init_params(obj)
            obj.M = 10;
            obj.m = 1;
            obj.I = 1;
            obj.L = 1;
            obj.x = 0;
            obj.x_dot = 0;
            obj.theta = pi/2;
            obj.theta_dot = 0;
            w = obj.cart_dim.w;
            h = obj.cart_dim.h;
            obj.cart_dim.xs = [-w/2 w/2 w/2 -w/2];
            obj.cart_dim.ys = [0 0 h h];
            obj.cart_color = [0.5 0.5 0.5];
%             obj.pole_dim.x = 
        end
        function plot(obj)
            hold on;
            obj.plt_cart = patch(obj.cart_dim.xs,obj.cart_dim.ys,obj.cart_color);
            obj.plt_pole = plot([obj.pole_dim.xb obj.]);
            
            axis equal;
            xlim([-10,10]);
            ylim([0,8]);
            
        end
    end
end