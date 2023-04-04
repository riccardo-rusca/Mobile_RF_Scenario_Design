%% Clean up the workspace
clear all
close all
clc

%% Final name for the .mat file with the channel matrix
FINAL_FILENAME="SAMARCANDA_scenario_1GHz_duplex.mat"; % INSERT HERE THE FINAL FILE NAME

%% Selected communication frequency for the scenario
COMM_FREQUENCY=1e9; % Should be specified in [Hz]

%% Parse data and define area for Colosseum
filename = 'SAMARCANDA_dataset.csv'; % INSERT HERE THE FILE NAME

format long g
SAMARCANDA = readtable(filename);

% Retrieve some useful quantitities
num_agents=max(SAMARCANDA.agent_id); % Number of vehicles (herein referred as agents, following the SAMARCANDA therminology
SAMARCANDA_agents=cell(1,num_agents);
SAMARCANDA_base_ts=cell(1,num_agents);
SAMARCANDA_num_points_per_agent=zeros(1,num_agents);

for i=1:num_agents
    SAMARCANDA_agents{i}=SAMARCANDA(SAMARCANDA.agent_id==i,:); % Each cell of this cell array contains a sub-table related to a specific vehicle
    SAMARCANDA_base_ts{i}=SAMARCANDA_agents{i}(1,:).timeStamp_posix; % Cell array of initial timestamps (to compute relative timestampts)
    SAMARCANDA_agents{i}.diffTstamp=SAMARCANDA_agents{i}.timeStamp_posix-SAMARCANDA_base_ts{i}; % Thanks to cell arrays, we can quickly add the relative timestamps to SAMARCANDA
    SAMARCANDA_num_points_per_agent(i)=length(SAMARCANDA_agents{i}.diffTstamp); % Each value in this array contains the number of trace points (one every around 100 ms) for each vehicle
end
SAMARCANDA_nppa_min = min(SAMARCANDA_num_points_per_agent); % Minimum number of points available for each vehicle/agent

% Fixed BS/RSU coordinates
antenna_lat = 44.88338;
antenna_long = 7.33152;

% 1 km square coordinates (Colosseum requires the vehicles to be limited in
% a 1 km^2 area)
square_lat_tl = 44.88809;
square_long_tl = 7.32425;

square_lat_tr = 44.88809;
square_long_tr = 7.33690;

square_lat_bl = 44.87910;
square_long_bl = 7.32409;

square_lat_br = 44.87910;
square_long_br = 7.33674;

% Check that is around 1Km each distance
disp("All these distance values should be around 1000 (meters):");
dist_and_check(square_lat_tl,square_long_tl,square_lat_tr,square_long_tr,999,1001,true);
dist_and_check(square_lat_bl, square_long_bl, square_lat_br, square_long_br,999,1001,true);
dist_and_check(square_lat_tl, square_long_tl, square_lat_bl, square_long_bl,999,1001,true);
dist_and_check(square_lat_tr, square_long_tr, square_lat_br, square_long_br,999,1001,true);

% Add an extra field containing "True" if each trace point of each vehicle
% is located inside the 1 km^2 square defined by "square", "False"
% otherwise
min_lat = min([square_lat_tl square_lat_tr square_lat_bl square_lat_br square_lat_tl]);
max_lat = max([square_lat_tl square_lat_tr square_lat_bl square_lat_br square_lat_tl]);
min_lon = min([square_long_tl square_long_tr square_long_bl square_long_br square_long_tl]);
max_lon = max([square_long_tl square_long_tr square_long_bl square_long_br square_long_tl]);

for i=1:num_agents
    SAMARCANDA_agents{i}.isinside = SAMARCANDA_agents{i}.latitude_deg >= min_lat & SAMARCANDA_agents{i}.latitude_deg <= max_lat & SAMARCANDA_agents{i}.longitude_deg >= min_lon & SAMARCANDA_agents{i}.longitude_deg <= max_lon;
end

% Check how many times a car cross the area of interest
real_num_agents = 0;
real_agents = [];
for i=1:num_agents
    sum = 0;
    val = SAMARCANDA_agents{i}.isinside;
    for x=1:length(val)
        if val(x)==1
            sum = sum + 1;
        end
    end
    if sum > 0
        real_num_agents = real_num_agents + 1;
        real_agents(real_num_agents) = i;
    end
end
real_agents = real_agents';

%% Create the chMatrix variable
num_antennas=1;
total_nodes=real_num_agents+num_antennas;

%For pruning timestamps we decided to use only the one from 2500 to 7700
%instead of all
new_matrix_dimension = 7700 - 2500;
chMatrix = cell(total_nodes,total_nodes,new_matrix_dimension);

for i=1:total_nodes
    for j=1:total_nodes
        for k=1:new_matrix_dimension
            if j==total_nodes
                chMatrix{i,j,k}.delay=[1e-9, 0, 0, 0];
                chMatrix{i,j,k}.iq=[1+0i, 0, 0, 0];
            else
                chMatrix{i,j,k}.delay=[0, 0, 0, 0];
                chMatrix{i,j,k}.iq=[0, 0, 0, 0];
            end
        end
    end
end

%% Perform ray tracing
inside = zeros(length(real_agents)+1,1);
inside_time = zeros(new_matrix_dimension,1);
if ~isfile(FINAL_FILENAME)
    time_discard = 0;
    for i=1:SAMARCANDA_nppa_min
        if i >= 2500 && i < 7700
            disp(i);
            noutside = 0;
            index=i-2499;
            for x=1:length(real_agents)
                ag=real_agents(x);
                if(SAMARCANDA_agents{ag}(i,:).isinside == false)
                    noutside = noutside + 1;
                else
                    curr_lat=SAMARCANDA_agents{ag}(i,:).latitude_deg;
                    curr_lon=SAMARCANDA_agents{ag}(i,:).longitude_deg;
                    [tap_gains,tap_delays]=raytrace_fcn(curr_lat,curr_lon,1.5,23,199.53*1e-3,COMM_FREQUENCY,44.88338,7.33152,30);
       
                    chMatrix{x,total_nodes,index}.iq=tap_gains(tap_gains~=0)';
                    chMatrix{x,total_nodes,index}.delay=tap_delays(find(tap_gains~=0))';

                    % The channel matrix should be duplex (i.e.,
                    % bi-directional, vehicles->antenna and
                    % antenna->vehicles)
                    chMatrix{total_nodes,x,index}.iq=chMatrix{x,total_nodes,index}.iq;
                    chMatrix{total_nodes,x,index}.delay=chMatrix{x,total_nodes,index}.delay;
                end
            end

            in = length(real_agents)-noutside;
            inside_time(index)=in;
            inside(in+1)=inside(in+1)+1;
            if noutside == length(real_agents)
                time_discard = time_discard + 1;
            end
        end
    end

    plot(1:new_matrix_dimension, inside_time)
    for i = 1:(length(inside))
        fprintf('There are %d times, %d vehicles\n', inside(i), i-1)
    end

    save("chMatrix_data.mat","chMatrix")
else
    load chMatrix_data.mat
end

%% Compute the remaining variables
timestamps = zeros(1,new_matrix_dimension);

% How to manage different timestamps for different vehicles?
% Workaround: we average them between all the agents/vehicles at each step
for ts=1:SAMARCANDA_nppa_min
    if ts >= 2500 && ts < 7700
        timestamps_agents=zeros(1,real_num_agents);
        for x=1:real_num_agents
            nag=real_agents(x);
            timestamps_agents(x)=SAMARCANDA_agents{nag}(ts,:).diffTstamp;
        end
        timestamps(ts-2499)=mean(timestamps_agents).*1e3; % The multiplication with 1e3 is to convert the timestamps from s to ms
    end
end

t = timestamps(1);
for ts=1:new_matrix_dimension
    timestamps(ts) = timestamps(ts) - t;
end

coordinates = cell(total_nodes,3,new_matrix_dimension);
FIXED_ALTITUDE_METERS_ASL=376; % Workaround as the current version of SAMARCANDA does not contain the altitude data; setting the average altitude of Pinerolo

for ts=1:SAMARCANDA_nppa_min
    if ts >= 2500 && ts < 7700
        for x=1:real_num_agents
            ag=real_agents(x);
            curr_lat=SAMARCANDA_agents{ag}(ts,:).latitude_deg;
            curr_lon=SAMARCANDA_agents{ag}(ts,:).longitude_deg;
        
            % Convert to cartesian coordinates
            % Which tranformation/projection? ECEF?
            % x, y, z in meters
            [curr_x,curr_y,curr_z]=geodetic2ecef(wgs84Ellipsoid('meter'),curr_lat,curr_lon,FIXED_ALTITUDE_METERS_ASL);
        
            index = ts-2499;
            coordinates{x,1,index}=curr_x;
            coordinates{x,2,index}=curr_y;
            coordinates{x,3,index}=curr_z;
        end
    end
end


[curr_x,curr_y,curr_z]=geodetic2ecef(wgs84Ellipsoid('meter'),antenna_lat,antenna_long,FIXED_ALTITUDE_METERS_ASL);
for ts=1:SAMARCANDA_nppa_min
    if ts >= 2500 && ts < 7700
        index = ts-2499;
        coordinates{total_nodes,1,index}=curr_x;
        coordinates{total_nodes,2,index}=curr_y;
        coordinates{total_nodes,3,index}=curr_z;
    end
end


% Setting as origin the center of the 1 km^2 area
origin.lat=44.883595;
origin.lon=7.330495;

if ~isfile(FINAL_FILENAME)
    save(FINAL_FILENAME,"chMatrix","timestamps","coordinates","origin");
else
    warning("No MAT file created. File already exists.");
end

%% Function to compute the distance between two points on the WGS84 ellipsoid and to automatically check if is it within given limits
function distval=dist_and_check(lat_a,lon_a,lat_b,lon_b,limit_low,limit_high,display)
    % Using distance() from the Mapping Toolbox
    % vdist() from MATLAB Exchange can also be used (https://it.mathworks.com/matlabcentral/fileexchange/5379-geodetic-distance-on-wgs84-earth-ellipsoid)
    [dist,~]=distance(lat_a,lon_a,lat_b,lon_b,...
                referenceEllipsoid("WGS84"));

    if display == true
        disp("dist_and_check() - current distance value [m]: " + num2str(dist));
    end
    
    if dist < limit_low || dist > limit_high
        error("Error: the specified distance is outside the specified limits: " + num2str(limit_low) + "," + num2str(limit_high));
    end
end