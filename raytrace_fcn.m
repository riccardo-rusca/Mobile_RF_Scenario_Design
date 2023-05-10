function [tap_gains,tap_delays]=raytrace_fcn(lat, lon, veh_antenna_h, veh_txpwr_dBm, veh_txpwr_watt, frequency, infra_lat, infra_lon, infra_h)
    
    TXPOWER=veh_txpwr_dBm; % In dBm
    TXPOWER_WATT=veh_txpwr_watt; % In W
    % Tx power set to 23 dBm according to: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9252895
    
    tx = txsite("Name", "Car", ...
        "Latitude", lat, ...
        "Longitude", lon, ...
        "TransmitterPower",TXPOWER_WATT, ...
        "TransmitterFrequency",frequency,...
        "AntennaHeight",veh_antenna_h); % Insert here the height of the vehicle
    rx = rxsite("Name", "Antenna", ...
        "Latitude", infra_lat, ...
        "Longitude",infra_lon, ...
        "AntennaHeight",infra_h);
    
    % We consider up to "3rd order reflections"
    pm = propagationModel("raytracing", ...
        "Method","sbr", ...
        "MaxNumReflections",3, ...
        "BuildingsMaterial","concrete", ...
        "TerrainMaterial","concrete");
		
	%%% RAY-TRACING
    raysPerfect = raytrace(tx,rx,pm,"Type","power");

    plPerfect = [raysPerfect{1}.PathLoss];
    phPerfect = [raysPerfect{1}.PhaseShift];
    
    % Computing Rx power using Friis Equation (reference:
    % https://www.mathworks.com/matlabcentral/answers/530603-how-can-i-get-the-signal-strength-for-individual-rays-when-i-do-the-ray-tracing)
    fq = tx.TransmitterFrequency;
    Ptx_dBm = 10 * log10(1000*tx.TransmitterPower);
    Gtx_db = 0; % Transmitter antenna gain
    Grx_db = 0; % Receiver antenna gain
    Prx_dBm = Ptx_dBm + Gtx_db + Grx_db - plPerfect - tx.SystemLoss - rx.SystemLoss;
    
    hi=zeros(1,length(plPerfect));
    for ipl=1:length(plPerfect)
        hi(ipl)=(10^((Prx_dBm(ipl)-TXPOWER)/20))*exp((1i)*raysPerfect{1}(ipl).PhaseShift);
    end
    
    %%% CLUSTERING
    K=4; % Insert here the number of taps
    
    %% K-means with MCD 
    X_table=table();
    X_table.PropagationDelay=[raysPerfect{1}.PropagationDelay].';
    X_table.PathLoss=[raysPerfect{1}.PathLoss].';
    AOAs=[raysPerfect{1}.AngleOfArrival];
    AODs=[raysPerfect{1}.AngleOfDeparture];
    X_table.AngleOfArrival_az=[AOAs(1,:)].';
    X_table.AngleOfArrival_el=[AOAs(2,:)].';
    X_table.AngleOfDeparture_az=[AODs(1,:)].';
    X_table.AngleOfDeparture_el=[AODs(2,:)].';
    X_table.hi(:)=hi';
    P_th = -89.1;

    if height(X_table)>K
        [~,C2,X_l]=mcdkmeans(X_table,K,P_th,TXPOWER,0,0,3,200,0,tx,rx);

        % Reconstruct approximated taps: Hck = sum  (x app. k)
        % abs(Hx)*e^(jfix), con k=1..K
        taps = zeros(K,2);
        Hck = zeros(K,1);
        for k=1:K
            Hck(k) = sum(X_l(X_l.assignment==k,:).hi);
        end
        taps(:,1)=C2(:,2);
        taps(:,2)=Hck;
        colNames = {'delay','h'};
        taps = array2table(taps,'VariableNames',colNames);
    else
        K=height(X_table);
        X_l=X_table;
        X_l.assignment=[1:K]';
        
        taps = zeros(K,2);
        taps(:,1)=X_l.PropagationDelay;
        taps(:,2)=X_l.hi;

        colNames = {'delay','h'};
        taps = array2table(taps,'VariableNames',colNames);
    end
    
    %% CIR re-sampling algorithm
    ds = 10e-9; % Sampling interval in s 
    fs = 1/ds; % FIR filters sampling frequency in Hz
    N = 512; % Number of FIR filters
    
    tap_delays = zeros(N,1);
    tap_gains = zeros(N,1);
    for n=1:N
        tap_delays(n) = n*ds;
        tap_gains(n) = 0 + 0i;
    end
    for k=1:height(taps)
        i=round((taps(k,:).delay)/ds);
        tap_gains(i) = tap_gains(i) + taps(k,:).h;
    end
    
    %% Function for MCD K-means algorithm
    function [idx,c_ik,X_l]=mcdkmeans(X,K,P_th,Ptx_dB,Gtx_db,Grx_db,zeta,Nrep,tshold,tx,rx)
        Prx_dBm_MCD = Ptx_dB + Gtx_db + Grx_db - X.PathLoss - tx.SystemLoss - rx.SystemLoss;
        X.rxpwr = Prx_dBm_MCD;
        X_l=X(X.rxpwr>=P_th,:);
        X_l.assignment=ones(1,height(X_l))';
        
        c_ik=zeros(K,6);
        c_ik_idx=randperm(height(X_l),K);
        c_ik(:,1)=X_l(c_ik_idx,:).PathLoss;
        c_ik(:,2)=X_l(c_ik_idx,:).PropagationDelay;
        c_ik(:,3)=X_l(c_ik_idx,:).AngleOfArrival_az;
        c_ik(:,4)=X_l(c_ik_idx,:).AngleOfArrival_el;
        c_ik(:,5)=X_l(c_ik_idx,:).AngleOfDeparture_az;
        c_ik(:,6)=X_l(c_ik_idx,:).AngleOfDeparture_el;
    
        for nr=1:Nrep
            tau_std=std(X.PropagationDelay);
            tau_pairs=nchoosek(X.PropagationDelay,2);
            tau_max=max(abs(tau_pairs(:,1)-tau_pairs(:,2)));
            for xli=1:height(X_l)
                MCD_tau=(zeta.*(X_l(xli,:).PropagationDelay-c_ik(1:K,2)).*tau_std./(tau_max).^2).';
    
                theta_AOA_i=X_l(xli,:).AngleOfArrival_az;
                phi_AOA_i=X_l(xli,:).AngleOfArrival_el;
                theta_AOD_i=X_l(xli,:).AngleOfDeparture_az;
                phi_AOD_i=X_l(xli,:).AngleOfDeparture_el;
                
                MCD_AOA=zeros(1,K);
                MCD_AOD=zeros(1,K);
    
                for ki=1:K
                    theta_AOA_j=c_ik(ki,3);
                    phi_AOA_j=c_ik(ki,4);
                    theta_AOD_j=c_ik(ki,5);
                    phi_AOD_j=c_ik(ki,6);
    
                    MCD_AOA(ki)=norm(0.5*abs([sin(theta_AOA_i)*cos(phi_AOA_i);sin(theta_AOA_i)*sin(phi_AOA_i);cos(theta_AOA_i)]-[sin(theta_AOA_j)*cos(phi_AOA_j);sin(theta_AOA_j)*sin(phi_AOA_j);cos(theta_AOA_i)]));
                    MCD_AOD(ki)=norm(0.5*abs([sin(theta_AOD_i)*cos(phi_AOD_i);sin(theta_AOD_i)*sin(phi_AOD_i);cos(theta_AOD_i)]-[sin(theta_AOD_j)*cos(phi_AOD_j);sin(theta_AOD_j)*sin(phi_AOD_j);cos(theta_AOD_i)]));
                end
    
                MCD=sqrt(MCD_AOA.^2+MCD_AOD.^2+MCD_tau.^2);
    
                [idx,indx]=min(MCD);
                X_l(xli,:).assignment=indx;
            end
            
            delta_cik_pl=zeros(1,K);
            delta_cik_pd=zeros(1,K);
            delta_cik_aoa_1=zeros(1,K);
            delta_cik_aoa_2=zeros(1,K);
            delta_cik_aod_1=zeros(1,K);
            delta_cik_aod_2=zeros(1,K);
            for ki=1:K
                X_k=X_l(X_l.assignment==ki,:);
                if height(X_k) ~= 0
                    mean_pl=mean(X_k.PathLoss);
                    mean_pd=mean(X_k.PropagationDelay);
                    mean_aoa_1=mean(X_k.AngleOfArrival_az);
                    mean_aoa_2=mean(X_k.AngleOfArrival_el);
                    mean_aod_1=mean(X_k.AngleOfDeparture_az);
                    mean_aod_2=mean(X_k.AngleOfDeparture_el);
                    delta_cik_pl(ki)=abs(mean_pl-c_ik(ki,1));
                    delta_cik_pd(ki)=abs(mean_pd-c_ik(ki,2));
                    delta_cik_aoa_1(ki)=abs(mean_aoa_1-c_ik(ki,3));
                    delta_cik_aoa_2(ki)=abs(mean_aoa_2-c_ik(ki,4));
                    delta_cik_aod_1(ki)=abs(mean_aod_1-c_ik(ki,5));
                    delta_cik_aod_2(ki)=abs(mean_aod_2-c_ik(ki,6));
                    c_ik(ki,1)=mean_pl;
                    c_ik(ki,2)=mean_pd;
                    c_ik(ki,3)=mean_aoa_1;
                    c_ik(ki,4)=mean_aoa_2;
                    c_ik(ki,5)=mean_aod_1;
                    c_ik(ki,6)=mean_aod_2;
                else
                    c_ik_idx=randperm(height(X_l),1);
                    c_ik(ki,1)=X_l(c_ik_idx,:).PathLoss;
                    c_ik(ki,2)=X_l(c_ik_idx,:).PropagationDelay;
                    c_ik(ki,3)=X_l(c_ik_idx,:).AngleOfArrival_az;
                    c_ik(ki,4)=X_l(c_ik_idx,:).AngleOfArrival_el;
                    c_ik(ki,5)=X_l(c_ik_idx,:).AngleOfDeparture_az;
                    c_ik(ki,6)=X_l(c_ik_idx,:).AngleOfDeparture_el;
                end
            end
    
            if(mean(delta_cik_pl(ki))<tshold && mean(delta_cik_pd(ki))<tshold)
                break;
            end
        end
    
        idx=X_l.assignment;
    end
end
