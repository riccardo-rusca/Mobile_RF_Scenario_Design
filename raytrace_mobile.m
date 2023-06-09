%% Clean up the workspace
clear all
close all
clc

%%% PRE-PROCESSING

%% Final name for the .mat file with the channel matrix
FINAL_FILENAME="RF_scenario.mat"; % {INPUT} Insert here the final file name

%% Selected communication frequency for the scenario
COMM_FREQUENCY=1e9; % {INPUT} Should be specified in [Hz]

%% Parse data and define area of interest
filename = 'dataset.csv'; % {INPUT} Insert here the dataset file name
% {INPUT} Define here and filter here the data to keep only traces inside the area of interest

% {INPUT} Convert here the dataset to match the required format (see
% README.md)

format long g
DATASET = readtable(filename);

%% Retrieve some useful quantitities
num_agents=max(DATASET.agent_id); % Number of vehicles (herein referred as agents)
real_agents = [1:num_agents]'; % Array with the ids of the vehicles
DATASET_agents=cell(1,num_agents);
DATASET_base_ts=cell(1,num_agents);
DATASET_num_points_per_agent=zeros(1,num_agents);

for i=1:num_agents
    DATASET_agents{i}=DATASET(DATASET.agent_id==i,:); % Each cell of this cell array contains a sub-table related to a specific vehicle
    DATASET_base_ts{i}=DATASET_agents{i}(1,:).timeStamp_posix; % Cell array of initial timestamps (to compute relative timestampts)
    DATASET_agents{i}.diffTstamp=DATASET_agents{i}.timeStamp_posix-DATASET_base_ts{i}; % Thanks to cell arrays, we can quickly add the relative timestamps to DATASET
    DATASET_num_points_per_agent(i)=length(DATASET_agents{i}.diffTstamp); % Each value in this array contains the number of trace points (one every around x ms, depending on the dataset) for each vehicle
end
DATASET_nppa_min = min(DATASET_num_points_per_agent); % Minimum number of points available for each vehicle/agent

%% Fixed BS/RSU coordinates
antenna_lat = 44.88338; % {INPUT} Insert here the latitude of your RSU/BS
antenna_long = 7.33152; % {INPUT} Insert here the longitude of your RSU/BS


%% Initialization of the chMatrix variable
num_antennas=1; % This code considers for the time being one centralized RSU/BS
total_nodes=num_agents+num_antennas;

new_matrix_dimension = 5000; % {INPUT} Insert here the total number of timestamps to be included in the scenario
chMatrix = cell(total_nodes,total_nodes,new_matrix_dimension);

% Initializing the channel matrix with empty values
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

%%% RAY-TRACING and CLUSTERING
if ~isfile(FINAL_FILENAME)
    for i=1:DATASET_nppa_min
		for x=1:length(real_agents)
			ag=real_agents(x);
            % Get the latitude and longitude of the vehicle at
            % the current timestamp
			curr_lat=DATASET_agents{ag}(i,:).latitude_deg;
			curr_lon=DATASET_agents{ag}(i,:).longitude_deg;
			% Call the raytrace function inside raytrace_fcn.m
			[tap_gains,tap_delays]=raytrace_fcn(curr_lat,curr_lon,1.5,23,199.53*1e-3,COMM_FREQUENCY,antenna_lat,antenna_long,30);

			chMatrix{x,total_nodes,i}.iq=tap_gains(tap_gains~=0)';
			chMatrix{x,total_nodes,i}.delay=tap_delays(find(tap_gains~=0))';

			% The channel matrix should be duplex (i.e., bi-directional, vehicles->antenna and antenna->vehicles)
			chMatrix{total_nodes,x,i}.iq=chMatrix{x,total_nodes,i}.iq;
			chMatrix{total_nodes,x,i}.delay=chMatrix{x,total_nodes,i}.delay;
		end
    end
end

    save("chMatrix_data.mat","chMatrix")
else
    load chMatrix_data.mat
end


%%% Computation of others needed variables

%% Array of timestamps
timestamps = zeros(1,new_matrix_dimension); 

% How to manage slightly different timestamps for different vehicles?
% Workaround: we average them between all the agents/vehicles at each step
for ts=1:SAMARCANDA_nppa_min
    timestamps_agents=zeros(1,num_agents);
	for x=1:num_agents
		nag=real_agents(x);
		timestamps_agents(x)=SAMARCANDA_agents{nag}(ts,:).diffTstamp;
	end
	timestamps(ts)=mean(timestamps_agents).*1e3; % The multiplication with 1e3 is to convert the timestamps from s to ms
end

t = timestamps(1);
for ts=1:new_matrix_dimension
    timestamps(ts) = timestamps(ts) - t;
end

%% Array of coordinates
coordinates = cell(total_nodes,3,new_matrix_dimension);
FIXED_ALTITUDE_METERS_ASL=376; % {INPUT} Insert here the altitude (we suppose here a fixed altitude, for the time being), in [m]

for ts=1:SAMARCANDA_nppa_min
	for x=1:num_agents
		ag=real_agents(x);
		curr_lat=SAMARCANDA_agents{ag}(ts,:).latitude_deg;
		curr_lon=SAMARCANDA_agents{ag}(ts,:).longitude_deg;
	
		% Convert from geodetic to cartesian ECEF coordinates
		[curr_x,curr_y,curr_z]=geodetic2ecef(wgs84Ellipsoid('meter'),curr_lat,curr_lon,FIXED_ALTITUDE_METERS_ASL);
	
		coordinates{x,1,ts}=curr_x;
		coordinates{x,2,ts}=curr_y;
		coordinates{x,3,ts}=curr_z;
	end
end


[curr_x,curr_y,curr_z]=geodetic2ecef(wgs84Ellipsoid('meter'),antenna_lat,antenna_long,FIXED_ALTITUDE_METERS_ASL);
for ts=1:SAMARCANDA_nppa_min
	coordinates{total_nodes,1,ts}=curr_x;
	coordinates{total_nodes,2,ts}=curr_y;
	coordinates{total_nodes,3,ts}=curr_z;
end


%% Setting as origin the center of the area of interest
origin.lat=44.883595; % {INPUT} Insert here the latitude of the center of the area
origin.lon=7.330495; % {INPUT} Insert here the longitude of the center of the area

if ~isfile(FINAL_FILENAME)
    save(FINAL_FILENAME,"chMatrix","timestamps","coordinates","origin");
else
    warning("No MAT file created. File already exists.");
end