clear;
font_size=18;
add_cross=false;
add_xline=false;
add_switch_point=true;

eae_data = readmatrix('example4_euler.csv');
mj_data = readmatrix('example4_mujoco.csv');
i_data = readmatrix('example4_incremental.csv');

sz = size(eae_data);
last_point_mj=size(mj_data);
last_point_mj=last_point_mj(1);
x_line_point=0.362;%the moment that twist limit is reached

%computed corrected gamma
[corrected_gamma, switch_points] = correctTwistAngle(eae_data);

t = tiledlayout(3,2); % Create a 3x2 grid layout
t.TileSpacing = 'compact'; % Adjust the space between the tiles
t.Padding = 'compact'; % Adjust the space around the edges
%subplot(3,2,1);
nexttile;
plot(mj_data(:,1),mj_data(:,2),'-o','MarkerIndices',1:200:length(mj_data(:,2)),LineWidth=2);
hold on;
plot(i_data(:,1),i_data(:,2),'-^','MarkerIndices',1:240:length(i_data(:,2)),LineWidth=2);
hold on;
plot(eae_data(:,1),eae_data(:,2),LineWidth=2);
if add_cross
    hold on;
    plot(mj_data(last_point_mj,1),mj_data(last_point_mj,2),'xr','MarkerSize',20, 'LineWidth', 2);
end
if add_xline
    xline(x_line_point,'--','LineWidth', 2);
end
if add_switch_point
    k=1;
    while switch_points(k)~=0
         xline(switch_points(k),'--r','LineWidth', 2);
         k=k+1;
    end
end
%ylim([-1 3]);
xlabel('t','FontSize',font_size);
ylabel('x','FontSize',font_size);

nexttile;
plot(mj_data(:,1),mj_data(:,5),'-o','MarkerIndices',1:200:length(mj_data(:,5)),LineWidth=2);
hold on;
plot(i_data(:,1),i_data(:,5),'-^','MarkerIndices',1:240:length(mj_data(:,5)),LineWidth=2);
hold on;
plot(eae_data(:,1),eae_data(:,5),LineWidth=2);
if add_cross
    hold on;
    plot(mj_data(last_point_mj,1),mj_data(last_point_mj,5),'xr','MarkerSize',20,'LineWidth',2);
end
if add_xline
    xline(x_line_point,'--','LineWidth', 2);
end
if add_switch_point
    k=1;
    while switch_points(k)~=0
         xline(switch_points(k),'--r','LineWidth', 2);
         k=k+1;
    end
end
xlabel('t','FontSize',font_size);
ylabel('$\alpha$','Interpreter', 'latex','FontSize',font_size);
legend('MuJoCo','Incremental','Euler','FontSize',10);

nexttile;
plot(mj_data(:,1),mj_data(:,3),'-o','MarkerIndices',1:200:length(mj_data(:,3)),LineWidth=2);
hold on;
plot(i_data(:,1),i_data(:,3),'-^','MarkerIndices',1:240:length(i_data(:,3)),LineWidth=2);
hold on;
plot(eae_data(:,1),eae_data(:,3),LineWidth=2);
if add_cross
    hold on;
    plot(mj_data(last_point_mj,1),mj_data(last_point_mj,3),'xr','MarkerSize',20,'LineWidth',2);
end
if add_xline
    xline(x_line_point,'--','LineWidth', 2);
end
if add_switch_point
    k=1;
    while switch_points(k)~=0
         xline(switch_points(k),'--r','LineWidth', 2);
         k=k+1;
    end
end
xlabel('t','FontSize',font_size);
ylabel('y','FontSize',font_size);

nexttile;
plot(mj_data(:,1),mj_data(:,6),'-o','MarkerIndices',1:200:length(mj_data(:,6)),LineWidth=2);
hold on;
plot(i_data(:,1),i_data(:,6),'-^','MarkerIndices',1:240:length(i_data(:,6)),LineWidth=2);
hold on;
plot(eae_data(:,1),eae_data(:,6),LineWidth=2);
if add_cross
    hold on;
    plot(mj_data(last_point_mj,1),mj_data(last_point_mj,6),'xr','MarkerSize',20,'LineWidth',2);
end
if add_xline
    xline(x_line_point,'--','LineWidth', 2);
end
if add_switch_point
    k=1;
    while switch_points(k)~=0
         xline(switch_points(k),'--r','LineWidth', 2);
         k=k+1;
    end
end
ylim([-pi pi]);
xlabel('t','FontSize',font_size);
ylabel('beta','FontSize',font_size);

nexttile;
plot(mj_data(:,1),mj_data(:,4),'-o','MarkerIndices',1:200:length(mj_data(:,4)),LineWidth=2);
hold on;
plot(i_data(:,1),i_data(:,4),'-^','MarkerIndices',1:240:length(i_data(:,4)),LineWidth=2);
hold on;
plot(eae_data(:,1),eae_data(:,4),LineWidth=2);
if add_cross
    hold on;
    plot(mj_data(last_point_mj,1),mj_data(last_point_mj,4),'xr','MarkerSize',20,'LineWidth',2);
end
if add_xline
    xline(x_line_point,'--','LineWidth', 2);
end
if add_switch_point
    k=1;
    while switch_points(k)~=0
         xline(switch_points(k),'--r','LineWidth', 2);
         k=k+1;
    end
end
ylim([-1 1]);
xlabel('t','FontSize',font_size);
ylabel('z','FontSize',font_size);

nexttile;
plot(mj_data(:,1),mj_data(:,7),'-o','MarkerIndices',1:200:length(mj_data(:,5)),LineWidth=2);
hold on;
plot(i_data(:,1),i_data(:,7),'-^','MarkerIndices',1:240:length(i_data(:,7)),LineWidth=2);
hold on;
plot(eae_data(:,1),corrected_gamma,LineWidth=2);
if add_cross
    hold on;
    plot(mj_data(last_point_mj,1),mj_data(last_point_mj,7),'xr','MarkerSize',20,'LineWidth',2);
end
if add_xline
    xline(x_line_point,'--','LineWidth', 2);
end
if add_switch_point
    k=1;
    while switch_points(k)~=0
         xline(switch_points(k),'--r','LineWidth', 2);
         k=k+1;
    end
end
ylim([-pi pi]);
xlabel('t','FontSize',font_size);
ylabel('gamma','FontSize',font_size);
% legend('switched','no switch');
%% 
clear;
sim0 = readmatrix('sim_data0.csv');
sim1 = readmatrix('sim_data1.csv');
sim2 = readmatrix('sim_data2.csv');

sim3 = readmatrix('sim_data0-4.csv');
sim4 = readmatrix('sim_data1-4.csv');
sim5 = readmatrix('sim_data2-4.csv');

% Define different MarkerIndices for each curve
markerIdx1 = 1:10:length(sim0); % Every 10th point for the first group
markerIdx2 = 1:15:length(sim1); % Every 15th point
markerIdx3 = 1:20:length(sim2); % Every 20th point
markerIdx4 = 1:25:length(sim3); % Every 25th point
markerIdx5 = 1:30:length(sim4); % Every 30th point
markerIdx6 = 1:35:length(sim5); % Every 35th point

figure;
hold on;

% Plot first group (dt=1e-2)
plot(sim0(:,1), sim0(:,2), '-o', 'LineWidth', 2, 'MarkerIndices', markerIdx1, 'DisplayName', 'Omega dt=1e-2');
plot(sim1(:,1), sim1(:,2), '-s', 'LineWidth', 2, 'MarkerIndices', markerIdx2, 'DisplayName', 'Euler dt=1e-2');
plot(sim2(:,1), sim2(:,2), '-d', 'LineWidth', 2, 'MarkerIndices', markerIdx3, 'DisplayName', 'Direct dt=1e-2');

% Plot second group (dt=1e-4)
plot(sim3(:,1), sim3(:,2), '-^', 'LineWidth', 2, 'MarkerIndices', markerIdx4, 'DisplayName', 'Omega dt=1e-4');
plot(sim4(:,1), sim4(:,2), '-v', 'LineWidth', 2, 'MarkerIndices', markerIdx5, 'DisplayName', 'Euler dt=1e-4');
plot(sim5(:,1), sim5(:,2), '-x', 'LineWidth', 2, 'MarkerIndices', markerIdx6, 'DisplayName', 'Direct dt=1e-4');

xlabel('t');
ylabel('Twisting angle');
legend;
hold off;
%% 
clear;
sim0 = readmatrix('5_0.csv');
sim1 = readmatrix('5_1.csv');
sim2 = readmatrix('5_2.csv');
sim3 = readmatrix('5_3.csv');
sim4 = readmatrix('5_4.csv');

% Define different MarkerIndices for each curve
markerIdx1 = 1:10:length(sim0); % Every 10th point for the first group
markerIdx2 = 1:15:length(sim1); % Every 15th point
markerIdx3 = 1:20:length(sim2); % Every 20th point
markerIdx4 = 1:25:length(sim3); % Every 25th point
markerIdx5 = 1:30:length(sim4); % Every 30th point

figure;
hold on;

correct0 = mapToPi(sim0(:,2));
plot(sim0(:,1), correct0, '-o', 'LineWidth', 2, 'MarkerIndices', markerIdx1, 'DisplayName', 'omega dt');
correct1 = mapToPi(sim1(:,2));
plot(sim1(:,1), correct1, '-s', 'LineWidth', 2, 'MarkerIndices', markerIdx2, 'DisplayName', 'incremental Euler');
correct2 = mapToPi(sim2(:,2));
plot(sim2(:,1), correct2, '-d', 'LineWidth', 2, 'MarkerIndices', markerIdx3, 'DisplayName', 'incremental direct');
plot(sim3(:,1), sim3(:,2), '-^', 'LineWidth', 2, 'MarkerIndices', markerIdx4, 'DisplayName', 'Euler');
plot(sim4(:,1), sim4(:,2), '-v', 'LineWidth', 2, 'MarkerIndices', markerIdx5, 'DisplayName', 'direct');

xlabel('t');
ylabel('Twist');
legend;
hold off;

function [corrected_result, switch_points] = correctTwistAngle(raw_data)
    sz = size(raw_data);
    corrected_result=zeros(1,sz(1));
    switch_points=zeros(5,1);
    k = 1;
    for i=1:sz(1)
        if raw_data(i,7) >= 3.14
            raw_data(i,7)=raw_data(i,7)-2*pi;
        end
        if i~=1
            if i+1<=sz(1) & raw_data(i,8)~=raw_data(i+1,8)
                if raw_data(i,7)<0
                    corrected_result(i)=raw_data(i,7)+pi-raw_data(i-1,7)+corrected_result(i-1);
                else
                    corrected_result(i)=raw_data(i,7)-pi-raw_data(i-1,7)+corrected_result(i-1);
                end
                switch_points(k)=raw_data(i,1);
                k=k+1;
            else
                corrected_result(i)=raw_data(i,7)-raw_data(i-1,7)+corrected_result(i-1);
            end
        else
           corrected_result(1)=raw_data(1,7);
        end  
    end
end

function result = mapToPi(input)
     sz = size(input);
     result = zeros(sz(1),1);
     result(1) = input(1);
     for i = 2:sz(1)
         result(i) = result(i-1) + input(i) - input(i-1);
         if result(i) > pi
            result(i) = result(i) - 2 * pi;
         end
         if result(i) < -pi
            result(i) = result(i) + 2 * pi;
         end
     end
end
