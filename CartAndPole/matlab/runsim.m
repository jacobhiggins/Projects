close all;
%% Initialize params
ip = cart_pole();
ip.init_params();

%% Plot figure
figure(1);
ip.plot();

%% Simulation
ip.t = 0;
d = 0;
while ip.t < 20
    if ip.t < 0.5
       d = -1;
       u = 1;
    else
       d = 0;
       u = -1;
    end
    
    ip.step(u,d); 
%     disp(ip.t)
    pause(0.05);
end