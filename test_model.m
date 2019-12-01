% Simple test program to check the water_tank_model class
% Create an instance of the class
A = 0.008;
a = 0.0021;
gamma = 0.001;
wt = water_tank_model(A,a,gamma);
wt_euler = water_tank_model(A,a,gamma);

% Peform 100 simulation steps
nb_steps = 100;
dt = 0.1;

% These are only used for plotting the results
u_vec = zeros(1,nb_steps);
y_vec = zeros(1,nb_steps);
dy_vec = zeros(1,nb_steps);

y_euler_vec = zeros(1,nb_steps);

u = 0;

wt_controller = PID_Controller(20,6,0.4,5,dt);
%wt_euler_controller = PID_Controller(20,6,0.4,5,dt);

for (i = 1:nb_steps)
    u = wt_controller.pid(wt.getWaterLevel);
    u = wt.controlLimits(u);

    y_vec(i) = wt.getWaterLevel();
    dy_vec(i) = wt.changeInWaterLevel(y_vec(i), u);

    y_euler_vec(i) = wt_euler.getWaterLevel();

    % Integrate the two instances using different integrators
    wt.integrateControlRK4(u, dt);
    wt_euler.integrateControlEuler(u, dt);

    u_vec(i) = u;

end

figure
plot(1:nb_steps,u_vec, 'r-');
hold on
plot(1:nb_steps,y_vec, 'b-');
plot(1:nb_steps,dy_vec,'g-');
plot(1:nb_steps,y_euler_vec, 'k-');
%plot(1:nb_steps,dy_euler_vec,'p.-');
