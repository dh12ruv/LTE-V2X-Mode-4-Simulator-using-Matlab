%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% Guide to use the Simulator:
% 1. Input the vehicle density ranging from 2 to 20.
%    Total number of vehicles = vehicle density * number of directions
%    i.e., 6
% 2. Input the MCS index from 0 to 20. 


% The code block integrates two simulators: (a) Spatial-Environment Simulator 
% and (b) LTE-V2X-Simulator (mode 4)
% (a) Spatial Environment Simulator: 
%   - A city road environment with 3 road-ways, each rodway has two
%     directions, each direction has two lanes resulting in a total of 6
%     directions, namely North-South, South-North, East-West1, West-East1,
%     East-West2, and West-East2
%   - There are 6 gray boxes representing the buildings
%   - The green square boxes represent road side units (RSUs)
%   - The black circular ring around each RSU represent the range of the
%     RSU.
%   - The pink colored moving dots are the vehicles
%   - The black rectangular moving boxes are the vehicles
%   - The blue lines represent the real-time connected links
%   - The red lines represent the real-time blocked links

% (b) LTE-V2X-Simulator:
%   - Integrates the UMi-Street Canyon Pathloss Model, simplified version
%   of Semi-Persistent-Scheduling, a simplified
%   version of a Congestion Control Mechanism with CBR and CR calculations, 
%   and Modulation and Coding Schemes. 
%   

% The connected and blocked states are recorded as 1 and 0 respectively for
% each vehicle at each position and at each time instant. The total 
% simulation run time is set to 200 time slots as default. The connected/
% blocked status are fed into the LTE-V2X-Simulator for PDR and Latency 
% calculations. 


% MCS index list with coding rate and modulation technique
% reference: 
% { Bazzi, A., Cecchini, G., Menarini, M., Masini, B.M., & Zanella, A. (2019). 
% Survey and Perspectives of Vehicular Wi-Fi versus Sidelink Cellular-V2X 
% in the 5G Era. Future Internet, 11, 122.}


% The simulator currently focuses on comparing MCS indices 7 and 11. However, 
% users have the flexibility to visualize Packet Delivery Ratio (PDR) and 
% Latency plots for any specific MCS index from the provided list. 
% The simulator includes a basic MCS implementation, and ongoing updates are 
% being made to improve the accuracy associated with MCS.

% MCS0 = struct('EbNo',SNR,'CR',0.13,'Modulation',"QPSK");
% MCS1 = struct('EbNo',SNR,'CR',0.17,'Modulation',"QPSK");
% MCS2 = struct('EbNo',SNR,'CR',0.21,'Modulation',"QPSK");
% MCS3 = struct('EbNo',SNR,'CR',0.27,'Modulation',"QPSK");
% MCS4 = struct('EbNo',SNR,'CR',0.33,'Modulation',"QPSK");
% MCS5 = struct('EbNo',SNR,'CR',0.41,'Modulation',"QPSK");
% MCS6 = struct('EbNo',SNR,'CR',0.48,'Modulation',"QPSK");
% MCS7 = struct('EbNo',SNR,'CR',0.57,'Modulation',"QPSK");
% MCS8 = struct('EbNo',SNR,'CR',0.65,'Modulation',"QPSK");
% MCS9 = struct('EbNo',SNR,'CR',0.73,'Modulation',"QPSK");
% MCS10 = struct('EbNo',SNR,'CR',0.82,'Modulation',"QPSK");
% 
% MCS11 = struct('EbNo',SNR,'CR',0.41,'Modulation',"QAM");
% MCS12 = struct('EbNo',SNR,'CR',0.46,'Modulation',"QAM");
% MCS13 = struct('EbNo',SNR,'CR',0.52,'Modulation',"QAM");
% MCS14 = struct('EbNo',SNR,'CR',0.59,'Modulation',"QAM");
% MCS15 = struct('EbNo',SNR,'CR',0.67,'Modulation',"QAM");
% MCS16 = struct('EbNo',SNR,'CR',0.72,'Modulation',"QAM");
% MCS17 = struct('EbNo',SNR,'CR',0.75,'Modulation',"QAM");
% MCS18 = struct('EbNo',SNR,'CR',0.84,'Modulation',"QAM");
% MCS10 = struct('EbNo',SNR,'CR',0.92,'Modulation',"QAM");
% MCS20 = struct('EbNo',SNR,'CR',1,'Modulation',"QAM");


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%
clear all;
close all;
clc;

fprintf("Total vehicles = vehicle density * number of lanes (i.e., 6)\n");
fprintf("Example: vehicle density = 2 is equal to a total of 2*6 = 12 vehicles\n\n");
fprintf("Recommended vehicle density = 5, 10, 15, 20\n")
lambda_input = input("Enter vehicle density = ");
fprintf("\n\n")
% 
% fprintf("The maximum number of RSUs is 3\n")
% RSU_N_input = input("Enter the number of RSUs = ");
% fprintf("\n\n")

fprintf("To view MCS 7 and MCS 11 together enter [7 11]\n")
fprintf("To view results for individual MCS index, enter a single integer value (0 to 20)\n")
MCS_input = input("Enter MCS index: ");

tic;
msec_per_tt = 100; %msec RSU transmits messages every tt_msec. Tx rate is 10 Hz i.e., 10 msgs/sec = 1 msg/100 msec
resolution = (1/1000)*msec_per_tt; %controls the granularity of the simulation/vehicle movement; 1 unit = 1sec = 1000 msec


%Vehicles and Trucks Densities:
% Vehicles
lambda = lambda_input;                         % Vehicle density per road segment (i.e., # vehs per 2 lanes) <--------------------------

%Trucks
theta = 1;                          % Truck density per road segment. Should be smaller than or eqaul to the vehicle density
if theta>lambda
    error('Truck density is greater than vehicle density!! Truck density should be smaller than or equal to vehicle denisty');
end


RSU_N = 1; %Number of RSUs <--------------------------
if RSU_N>3
    error('Number of RSU can not exceed 3!!');
end

%MCS
MCS_set = MCS_input;
PDR_below_90  = [];
x_mcs = [];
y_mcs = [];

for mcs = 1:length(MCS_set)

    %For PDR Calculation:
    PDR_range = 10;
    RSU_range = 150;
    veh_RSU_dist = 0:PDR_range:RSU_range;
    % PDR_fr_dist = [];
    ll = length(veh_RSU_dist);
    i_PDR = 1; %iteration for PDR and and 2junction
    PDR_fr_dist_all = zeros(i_PDR,ll);
    PDR_data_avg = [];
    Latency_data_avg = [];

    iterate = 1; %Iteration for the 2junction only
    % PDR_all = [];
    PDR_data = zeros(iterate,lambda*6,i_PDR); %there are 6 road segments, there are 2 RSUs
    Latency_data = zeros(iterate,lambda*6,i_PDR);

    for a = 1: i_PDR

        for i_iterate = 1:iterate
            fprintf('iteration = %d\n\n',i_iterate);
            % Random seed for different iterations
            rng_iterate = randperm(110011,iterate);
            rng(rng_iterate(i_iterate));
            tMax = 100;                      % Number of time slots; 1 slot = 0.5msec <---------------------------


            %---------- Transmitted and Received Power ----------%
            power_tx = 23-30 ; % transmitted signal power in dB i.e. (23dBm = 23-30 dB)
            power_rx_threshold = -97.28; %dBm % Minimum receiver power at the receiver to be detectable according to the standard is -90.4 dBm
            fc = 5.9; %GHz (fc Normalized to 1GHz for the UMi-street canyon pathloss modal)

            %Initialization for pathloss in each lane
            pathloss_all = [];
            pathloss_SN = zeros(lambda,tMax);
            pathloss_NS = zeros(lambda,tMax);
            pathloss_EW1 = zeros(lambda,tMax);
            pathloss_WE1 = zeros(lambda,tMax);
            pathloss_EW2 = zeros(lambda,tMax);
            pathloss_WE2 = zeros(lambda,tMax);

            %Initialization for rx power of each vehicle in each lanes
            power_rx_SN = zeros(lambda,tMax);
            power_rx_NS = zeros(lambda,tMax);
            power_rx_EW1 = zeros(lambda,tMax);
            power_rx_WE1 = zeros(lambda,tMax);
            power_rx_EW2 = zeros(lambda,tMax);
            power_rx_WE2 = zeros(lambda,tMax);

            %initilization for the matrix of co-ordinates of all the lines of the dges of
            %each truck in each lane
            truckCoSN_M = [];
            truckCoNS_M = [];
            truckCoEW1_M = [];
            truckCoWE1_M = [];
            truckCoEW2_M = [];
            truckCoWE2_M = [];
            %Matrix of co-ordinates of all the lines of all the vehicles of all the
            %lanes
            truckCo_All = [];

            truckSpeed = 10;                    % Mean truck speed (m/s) <-----------------------
            truckW = 8; % units                        % truck width(along with X)
            truckL = 20; % units                       % truck height (along with Y)

            % Vehicle
            vehSpeed = 12;                      % Mean vehicle speed (m/s) <----------------------
            vehW = 1;                           % Vehicle width (along with X)
            vehH = 1;                           % Vehicle height (along with Y)

            %Distances between RSU and the vehicles in each tt
            Dis_SN = zeros(lambda,tMax);
            Dis_NS = zeros(lambda,tMax);
            Dis_EW1 = zeros(lambda,tMax);
            Dis_WE1 = zeros(lambda,tMax);
            Dis_EW2 = zeros(lambda,tMax);
            Dis_WE2 = zeros(lambda,tMax);

            %D: Counter for blocked lines
            flagBlocked = 0;
            truckFlagBlocked = 0;

            %D------------Modulation and Coding Scheme (MCS) parameters--------------
            %gnerating random binary Basic Saftey Message (BSM)
            msgLen = 2432; %BSM size: 300 bytes = 2400bits + 32 bits (CRC)
            BSM_tx = randi([0,1],msgLen, 1); %Random binary BSM

            %calculating Noise Power
            k = 1.380649e-23; %Boltamann constant
            To = 295; % average atmospheric temperature in Kelvin
            BW = 20e6; %20 MHz channel Bandwidth
            Noise_power = k*To*BW; %in watts
            Pn_dBm = 10*log10(Noise_power)+30; % Noise power in dBm

            %Coding rates for MCS0 to MCS20
            coding_rate_QPSK = [0.13 0.17 0.21 0.27 0.33 0.41 0.48 0.57 0.65 0.73 0.82];

            coding_rate_QAM = [ 0.41 0.46 0.52 0.59 0.67 0.72 0.75 0.84 0.92 1];
            %-----------------------------------------------------------------------------


            %D: Initialization for calculation of total number of successful/blocked connections
            connections_all = [];
            connections_total = 0;
            disconnections_total = 0;
            connections_total_matrix = [];
            disconnections_total_matrix = [];

            %D: Sting for each lanes
            lanes = [ "NS" "SN" "EW1" "WE1" "EW2" "WE2" ] ;

            %%D: Initialization for Latency calculation:
            %Latency of all vehicles in all the lanes:
            Latency_all = [];
            %--Latency for SN lanes--%
            linked_SN_v = zeros(1,lambda) ;      %Connection identifier; 0 = blocked
            connections_SN_v = zeros(lambda,tMax); %matrix of Latency of a vehicle
            %Connection identifier; 1 = connected


            %--Latency for NS lanes--%
            linked_NS_v = zeros(1,lambda);
            connections_NS_v = zeros(lambda,tMax);


            %--Latency for EW1 lanes--%
            linked_EW1_v = zeros(1,lambda);
            connections_EW1_v = zeros(lambda,tMax);


            %--Latency for WE1 lanes--%
            linked_WE1_v = zeros(1,lambda);
            connections_WE1_v = zeros(lambda,tMax);


            %--Latency for EW2 lanes--%
            linked_EW2_v = zeros(1,lambda);
            connections_EW2_v = zeros(lambda,tMax);

            %--Latency for WE2 lanes--%
            linked_WE2_v = zeros(1,lambda);
            connections_WE2_v = zeros(lambda,tMax);


            %D: Rays Counters Initializaiton:
            SuccessRaysCtr = 0; %Transmitted rays counter
            BlockedRaysCtr = 0; %Blocked rays counter

            %D: Scussess Counters For Individual Vehicles:
            SC_SN = zeros(lambda,1);
            SC_NS = zeros(lambda,1);
            SC_EW1 = zeros(lambda,1);
            SC_WE1 = zeros(lambda,1);
            SC_EW2 = zeros(lambda,1);
            SC_WE2= zeros(lambda,1);

            %D: Blocked Counters For Individual Vehicles:
            BC_SN = zeros(lambda,1);
            BC_NS = zeros(lambda,1);
            BC_EW1 = zeros(lambda,1);
            BC_WE1 = zeros(lambda,1);
            BC_EW2 = zeros(lambda,1);
            BC_WE2= zeros(lambda,1);

            %D: Message Delivery rate initialization for each vehicles:
            MDR_SN = zeros(lambda,1);
            MDR_NS = zeros(lambda,1);
            MDR_EW1 = zeros(lambda,1);
            MDR_WE1 = zeros(lambda,1);
            MDR_EW2 = zeros(lambda,1);
            MDR_WE2 = zeros(lambda,1);
            %MDR of all vehicles:
            PDR_all = [];

            %S: Road coordinates
            R = 100;                                                        % Range of vehicle distribution space
            laneWidth = 10;                                                 % Lane width
            northWall = 6*laneWidth + 2*R;                                  % Y coordinate for north wall
            southWall = -2*laneWidth - R;                                   % Y coordinate for south wall
            eastWall = 2*laneWidth + R;                                     % X coordinate for east wall
            westWall = -2*laneWidth - R;                                    % X coordinate for west wall
            SN_X = [laneWidth - laneWidth/2 laneWidth + laneWidth/2];       % X coordinate for South-North lane
            NS_X = -SN_X;                                                   % X coordinate for North-South lane
            EW1_Y = [laneWidth - laneWidth/2 laneWidth + laneWidth/2];      % Y coordinate for East-West lane
            WE1_Y = -EW1_Y;                                                 % Y coordinate for West-East lane
            EW2_Y = [6*laneWidth+R - laneWidth/2 6*laneWidth+R - 3*laneWidth/2];      % Y coordinate for East-West lane
            WE2_Y = [2*laneWidth+R + 3*laneWidth/2 2*laneWidth+R + laneWidth/2];      % Y coordinate for West-East lane


            %D: Road Side Unit location
            RSU1 = complex(-2*laneWidth, 2*laneWidth+R);
            x_RSU1 = real(RSU1)+0.5;
            y_RSU1 = imag(RSU1)+0.5;
            TxRange = 300/2;    %Range (in meter) of the Transmitted rays

            RSU2 = complex(-2*laneWidth, -2*laneWidth);
            x2_RSU2 = real(RSU2)+0.5;
            y2_RSU2 = imag(RSU2)+0.5;
            TxRange = 300/2;    %Range (in meter) of the Transmitted rays

            RSU3 = complex(-2*laneWidth, 6*laneWidth+2*R);
            x3_RSU3 = real(RSU3)+0.5;
            y3_RSU3 = imag(RSU3);

            if RSU_N == 1
                RSU = RSU1;
                x_RSU = x_RSU1;
                y_RSU = y_RSU1;
            elseif RSU_N == 2
                RSU = [RSU1 RSU2];
                x_RSU = [x_RSU1 x2_RSU2];
                y_RSU = [y_RSU1 y2_RSU2];
            elseif RSU_N == 3
                RSU = [RSU1 RSU2 RSU3];
                x_RSU = [x_RSU1 x2_RSU2 x3_RSU3];
                y_RSU = [y_RSU1 y2_RSU2 y3_RSU3];
            end

            [~,n_RSU] = size(RSU);
            %D: Coordinates of the lines of the edges of all the buildings
            % --- Co-ordinates of each building --- %
            B1 = BldLinesCo(2*laneWidth,southWall,R,R);
            B2 = BldLinesCo(-2*laneWidth-R,southWall,R,R);
            B3 = BldLinesCo(2*laneWidth,2*laneWidth,R,R);
            B4 = BldLinesCo(-2*laneWidth-R,2*laneWidth,R,R);
            B5 = BldLinesCo(2*laneWidth,6*laneWidth+R,R,R);
            B6 = BldLinesCo(-2*laneWidth-R,6*laneWidth+R,R,R);

            BldLinesCoordinates = [B1; B2; B3; B4; B5; B6]; %of all buildings


            vehObj = []; %Initialization of matrix for the position of all vehicles
            %% Vehicles distribution

            % Palettes for plots
            for kk = 1 : lambda
                vehColorSN(kk,:) = [1 0 1];
                vehColorNS(kk,:) = [1 0 1];
                vehColorEW1(kk,:) = [1 0 1];
                vehColorWE1(kk,:) = [1 0 1];
                vehColorEW2(kk,:) = [1 0 1];
                vehColorWE2(kk,:) = [1 0 1];
            end

            % Vehicles movement
            for tt = 1 : tMax
                disp(['tt = ' num2str(tt) '/' num2str(tMax)]);
                fprintf('iteration = %d\n\n',i_iterate);


                % Initialization for wallTouched
                wallTouchedSN = 0;
                wallTouchedNS = 0;
                wallTouchedEW = 0;
                wallTouchedWE = 0;

                for ii = 1 : lambda
                    %--- Vehicle parameters ---%
                    %D: To reduce the granule from 1 to 20 , multiplying the speed
                    %value by (20/1000)

                    if tt == 1
                        % South to North
                        SN(ii).speed = resolution*randi([1,vehSpeed]);     % (m/s) Speed of vehicle (1/50)*
                        % North to South
                        NS(ii).speed = resolution*randi([1,vehSpeed]);     % (m/s) Speed of vehicle
                        % East to West Junction 1
                        EW1(ii).speed = resolution*randi([1,vehSpeed]);    % (m/s) Speed of vehicle
                        % West to East Junction 1
                        WE1(ii).speed = resolution*randi([1,vehSpeed]);    % (m/s) Speed of vehicle
                        % East to West Junction 2
                        EW2(ii).speed = resolution*randi([1,vehSpeed]);    % (m/s) Speed of vehicle
                        % West to East Junction 2
                        WE2(ii).speed = resolution*randi([1,vehSpeed]);    % (m/s) Speed of vehicle
                    end

                    %--- Vehicle position ---%
                    % Initial position
                    if tt == 1
                        % South to North
                        SN(ii).vehXinit = SN_X(randi([1 2]));
                        SN(ii).vehYinit = randi([-R R]);
                        SN(ii).vehX(tt) = SN(ii).vehXinit;
                        SN(ii).vehY(tt) = SN(ii).vehYinit;
                        % North to South
                        NS(ii).vehXinit = NS_X(randi([1 2]));
                        NS(ii).vehYinit = randi([-R R]);
                        NS(ii).vehX(tt) = NS(ii).vehXinit;
                        NS(ii).vehY(tt) = NS(ii).vehYinit;
                        % East to West Junction 1
                        EW1(ii).vehXinit = randi([-R R]);
                        EW1(ii).vehYinit = EW1_Y(randi([1 2]));
                        EW1(ii).vehX(tt) = EW1(ii).vehXinit;
                        EW1(ii).vehY(tt) = EW1(ii).vehYinit;
                        % West to East Junction 1
                        WE1(ii).vehXinit = randi([-R R]);
                        WE1(ii).vehYinit = WE1_Y(randi([1 2]));
                        WE1(ii).vehX(tt) = WE1(ii).vehXinit;
                        WE1(ii).vehY(tt) = WE1(ii).vehYinit;
                        % East to West Junction 2
                        EW2(ii).vehXinit = randi([-R R]);
                        EW2(ii).vehYinit = EW2_Y(randi([1 2]));
                        EW2(ii).vehX(tt) = EW2(ii).vehXinit;
                        EW2(ii).vehY(tt) = EW2(ii).vehYinit;
                        % West to East Junction 2
                        WE2(ii).vehXinit = randi([-R R]);
                        WE2(ii).vehYinit = WE2_Y(randi([1 2]));
                        WE2(ii).vehX(tt) = WE2(ii).vehXinit;
                        WE2(ii).vehY(tt) = WE2(ii).vehYinit;
                        % After t > 1
                    else
                        %             BC_SN(ii) = [];
                        % South to North
                        SN(ii).vehX(tt) = SN(ii).vehX(tt-1);
                        SN(ii).vehY(tt) = SN(ii).vehY(tt-1) + SN(ii).speed;
                        % North to South
                        NS(ii).vehX(tt) = NS(ii).vehX(tt-1);
                        NS(ii).vehY(tt) = NS(ii).vehY(tt-1) - NS(ii).speed;
                        % East to West Junction 1
                        EW1(ii).vehX(tt) = EW1(ii).vehX(tt-1) - EW1(ii).speed;
                        EW1(ii).vehY(tt) = EW1(ii).vehY(tt-1);
                        % West to East Junction 1
                        WE1(ii).vehX(tt) = WE1(ii).vehX(tt-1) + WE1(ii).speed;
                        WE1(ii).vehY(tt) = WE1(ii).vehY(tt-1);
                        % East to West Junction 2
                        EW2(ii).vehX(tt) = EW2(ii).vehX(tt-1) - EW2(ii).speed;
                        EW2(ii).vehY(tt) = EW2(ii).vehY(tt-1);
                        % West to East Junction 2
                        WE2(ii).vehX(tt) = WE2(ii).vehX(tt-1) + WE2(ii).speed;
                        WE2(ii).vehY(tt) = WE2(ii).vehY(tt-1);
                    end

                    %--- Vehicle position when touching wall ---%
                    % South to North
                    if SN(ii).vehY(tt) >= northWall
                        wallTouchedSN = 1;
                    end
                    if wallTouchedSN == 1
                        wallTouchedSN = 0;              % When vehicle hits wall
                        SN(ii).vehX(tt) = SN(ii).vehX(tt-1);
                        SN(ii).vehY(tt) = southWall;
                    end
                    % North to South
                    if NS(ii).vehY(tt) <= southWall
                        wallTouchedNS = 1;
                    end
                    if wallTouchedNS == 1
                        wallTouchedNS = 0;              % When vehicle hits wall
                        NS(ii).vehX(tt) = NS(ii).vehX(tt-1);
                        NS(ii).vehY(tt) = northWall;
                    end
                    % East to West Junction 1
                    if EW1(ii).vehX(tt) <= westWall
                        wallTouchedEW = 1;
                    end
                    if wallTouchedEW == 1
                        wallTouchedEW = 0;              % When vehicle hits wall
                        EW1(ii).vehX(tt) = eastWall;
                        EW1(ii).vehY(tt) = EW1(ii).vehY(tt-1);
                    end
                    % West to East Junction 1
                    if WE1(ii).vehX(tt) >= eastWall
                        wallTouchedWE = 1;
                    end
                    if wallTouchedWE == 1
                        wallTouchedWE = 0;              % When vehicle hits wall
                        WE1(ii).vehX(tt) = westWall;
                        WE1(ii).vehY(tt) = WE1(ii).vehY(tt-1);
                    end
                    % East to West Junction 2
                    if EW2(ii).vehX(tt) <= westWall
                        wallTouchedEW = 1;
                    end
                    if wallTouchedEW == 1
                        wallTouchedEW = 0;              % When vehicle hits wall
                        EW2(ii).vehX(tt) = eastWall;
                        EW2(ii).vehY(tt) = EW2(ii).vehY(tt-1);
                    end
                    % West to East Junction 2
                    if WE2(ii).vehX(tt) >= eastWall
                        wallTouchedWE = 1;
                    end
                    if wallTouchedWE == 1
                        wallTouchedWE = 0;              % When vehicle hits wall
                        WE2(ii).vehX(tt) = westWall;
                        WE2(ii).vehY(tt) = WE2(ii).vehY(tt-1);
                    end

                    %--- Individual vehicle ---%
                    vehObjSN(tt,ii) = complex(SN(ii).vehX(tt), SN(ii).vehY(tt));
                    vehObjNS(tt,ii) = complex(NS(ii).vehX(tt), NS(ii).vehY(tt));
                    vehObjEW1(tt,ii) = complex(EW1(ii).vehX(tt), EW1(ii).vehY(tt));
                    vehObjWE1(tt,ii) = complex(WE1(ii).vehX(tt), WE1(ii).vehY(tt));
                    vehObjEW2(tt,ii) = complex(EW2(ii).vehX(tt), EW2(ii).vehY(tt));
                    vehObjWE2(tt,ii) = complex(WE2(ii).vehX(tt), WE2(ii).vehY(tt));

                    %D--- Matrix of positions of all the vehicles ----D%
                    vehObj = [vehObjSN(tt,ii),vehObjNS(tt,ii),vehObjEW1(tt,ii), vehObjWE1(tt,ii),vehObjEW2(tt,ii),vehObjWE2(tt,ii)];

                end

                %%  ----------------For Trucks--------------------  %%
                %     rng(17);
                % Trucks movement
                if theta>=1 %runs only when truck density value if a positive integer
                    NOtruck = 0;
                    for ii = 1 : theta
                        %--- truck parameters ---%


                        if tt == 1
                            % South to North
                            SN_T(ii).speed = resolution*randi([1,truckSpeed]);     % (m/s) Speed of truck (1/50)*
                            % North to South
                            NS_T(ii).speed = resolution*randi([1,truckSpeed]);     % (m/s) Speed of truck
                            % East to West Junction 1
                            EW1_T(ii).speed = resolution*randi([1,truckSpeed]);    % (m/s) Speed of truck
                            % West to East Junction 1
                            WE1_T(ii).speed = resolution*randi([1,truckSpeed]);    % (m/s) Speed of truck
                            % East to West Junction 2
                            EW2_T(ii).speed = resolution*randi([1,truckSpeed]);    % (m/s) Speed of truck
                            % West to East Junction 2
                            WE2_T(ii).speed = resolution*randi([1,truckSpeed]);    % (m/s) Speed of truck
                        end

                        %--- truck position ---%
                        % Initial position
                        if tt == 1
                            % South to North
                            SN_T(ii).vehXinit = SN_X(randi([1 2]));
                            SN_T(ii).vehYinit = randi([-R R]);
                            SN_T(ii).vehX(tt) = SN_T(ii).vehXinit;
                            SN_T(ii).vehY(tt) = SN_T(ii).vehYinit;
                            % North to South
                            NS_T(ii).vehXinit = NS_X(randi([1 2]));
                            NS_T(ii).vehYinit = randi([-R R]);
                            NS_T(ii).vehX(tt) = NS_T(ii).vehXinit;
                            NS_T(ii).vehY(tt) = NS_T(ii).vehYinit;
                            % East to West Junction 1
                            EW1_T(ii).vehXinit = randi([-R R]);
                            EW1_T(ii).vehYinit = EW1_Y(randi([1 2]));
                            EW1_T(ii).vehX(tt) = EW1_T(ii).vehXinit;
                            EW1_T(ii).vehY(tt) = EW1_T(ii).vehYinit;
                            % West to East Junction 1
                            WE1_T(ii).vehXinit = randi([-R R]);
                            WE1_T(ii).vehYinit = WE1_Y(randi([1 2]));
                            WE1_T(ii).vehX(tt) = WE1_T(ii).vehXinit;
                            WE1_T(ii).vehY(tt) = WE1_T(ii).vehYinit;
                            % East to West Junction 2
                            EW2_T(ii).vehXinit = randi([-R R]);
                            EW2_T(ii).vehYinit = EW2_Y(randi([1 2]));
                            EW2_T(ii).vehX(tt) = EW2_T(ii).vehXinit;
                            EW2_T(ii).vehY(tt) = EW2_T(ii).vehYinit;
                            % West to East Junction 2
                            WE2_T(ii).vehXinit = randi([-R R]);
                            WE2_T(ii).vehYinit = WE2_Y(randi([1 2]));
                            WE2_T(ii).vehX(tt) = WE2_T(ii).vehXinit;
                            WE2_T(ii).vehY(tt) = WE2_T(ii).vehYinit;
                            % After t > 1
                        else
                            %             BC_SN(ii) = [];
                            % South to North
                            SN_T(ii).vehX(tt) = SN_T(ii).vehX(tt-1);
                            SN_T(ii).vehY(tt) = SN_T(ii).vehY(tt-1) + SN_T(ii).speed;
                            % North to South
                            NS_T(ii).vehX(tt) = NS_T(ii).vehX(tt-1);
                            NS_T(ii).vehY(tt) = NS_T(ii).vehY(tt-1) - NS_T(ii).speed;
                            % East to West Junction 1
                            EW1_T(ii).vehX(tt) = EW1_T(ii).vehX(tt-1) - EW1_T(ii).speed;
                            EW1_T(ii).vehY(tt) = EW1_T(ii).vehY(tt-1);
                            % West to East Junction 1
                            WE1_T(ii).vehX(tt) = WE1_T(ii).vehX(tt-1) + WE1_T(ii).speed;
                            WE1_T(ii).vehY(tt) = WE1_T(ii).vehY(tt-1);
                            % East to West Junction 2
                            EW2_T(ii).vehX(tt) = EW2_T(ii).vehX(tt-1) - EW2_T(ii).speed;
                            EW2_T(ii).vehY(tt) = EW2_T(ii).vehY(tt-1);
                            % West to East Junction 2
                            WE2_T(ii).vehX(tt) = WE2_T(ii).vehX(tt-1) + WE2_T(ii).speed;
                            WE2_T(ii).vehY(tt) = WE2_T(ii).vehY(tt-1);
                        end

                        %--- Truck position when touching wall ---%
                        % South to North
                        if SN_T(ii).vehY(tt) >= northWall
                            wallTouchedSN = 1;
                        end
                        if wallTouchedSN == 1
                            wallTouchedSN = 0;              % When truck hits wall
                            SN_T(ii).vehX(tt) = SN_T(ii).vehX(tt-1);
                            SN_T(ii).vehY(tt) = southWall;
                        end
                        % North to South
                        if NS_T(ii).vehY(tt) <= southWall
                            wallTouchedNS = 1;
                        end
                        if wallTouchedNS == 1
                            wallTouchedNS = 0;              % When truck hits wall
                            NS_T(ii).vehX(tt) = NS_T(ii).vehX(tt-1);
                            NS_T(ii).vehY(tt) = northWall;
                        end
                        % East to West Junction 1
                        if EW1_T(ii).vehX(tt) <= westWall
                            wallTouchedEW = 1;
                        end
                        if wallTouchedEW == 1
                            wallTouchedEW = 0;              % When truck hits wall
                            EW1_T(ii).vehX(tt) = eastWall;
                            EW1_T(ii).vehY(tt) = EW1_T(ii).vehY(tt-1);
                        end
                        % West to East Junction 1
                        if WE1_T(ii).vehX(tt) >= eastWall
                            wallTouchedWE = 1;
                        end
                        if wallTouchedWE == 1
                            wallTouchedWE = 0;              % When truck hits wall
                            WE1_T(ii).vehX(tt) = westWall;
                            WE1_T(ii).vehY(tt) = WE1_T(ii).vehY(tt-1);
                        end
                        % East to West Junction 2
                        if EW2_T(ii).vehX(tt) <= westWall
                            wallTouchedEW = 1;
                        end
                        if wallTouchedEW == 1
                            wallTouchedEW = 0;              % When truck hits wall
                            EW2_T(ii).vehX(tt) = eastWall;
                            EW2_T(ii).vehY(tt) = EW2_T(ii).vehY(tt-1);
                        end
                        % West to East Junction 2
                        if WE2_T(ii).vehX(tt) >= eastWall
                            wallTouchedWE = 1;
                        end
                        if wallTouchedWE == 1
                            wallTouchedWE = 0;              % When truck hits wall
                            WE2_T(ii).vehX(tt) = westWall;
                            WE2_T(ii).vehY(tt) = WE2_T(ii).vehY(tt-1);
                        end

                        %--- Individual trucks ---%
                        truckObjSN(tt,ii) = complex(SN_T(ii).vehX(tt), SN_T(ii).vehY(tt));
                        truckObjNS(tt,ii) = complex(NS_T(ii).vehX(tt), NS_T(ii).vehY(tt));
                        truckObjEW1(tt,ii) = complex(EW1_T(ii).vehX(tt), EW1_T(ii).vehY(tt));
                        truckObjWE1(tt,ii) = complex(WE1_T(ii).vehX(tt), WE1_T(ii).vehY(tt));
                        truckObjEW2(tt,ii) = complex(EW2_T(ii).vehX(tt), EW2_T(ii).vehY(tt));
                        truckObjWE2(tt,ii) = complex(WE2_T(ii).vehX(tt), WE2_T(ii).vehY(tt));

                        %D--- Matrix of positions of all the trucks ----D%
                        truckObj = [truckObjSN(tt,ii),truckObjNS(tt,ii),truckObjEW1(tt,ii), truckObjWE1(tt,ii),truckObjEW2(tt,ii),truckObjWE2(tt,ii)];

                        %D: Co-ordinates of the lines of the edges of each truck in SN lane

                        %truckCoSN.set is a struct with a field 'set'
                        %truckCoSN_M is a matrix of all the co-ordinates of the lines of
                        %the edges of each truck in SN lane
                        truckCoSN(ii).set = BldLinesCo(real(truckObjSN(tt,ii))-truckW/2, imag(truckObjSN(tt,ii))-truckL/2, truckW, truckL);
                        truckCoSN_M = [ truckCoSN_M; truckCoSN(ii).set];

                        %For NS lane
                        truckCoNS(ii).set = BldLinesCo(real(truckObjNS(tt,ii))-truckW/2, imag(truckObjNS(tt,ii))-truckL/2, truckW, truckL);
                        truckCoNS_M = [ truckCoNS_M; truckCoNS(ii).set];
                        %For EW1 lane
                        truckCoEW1(ii).set = BldLinesCo(real(truckObjEW1(tt,ii))-truckL/2, imag(truckObjEW1(tt,ii))-truckW/2, truckL, truckW);
                        truckCoEW1_M = [ truckCoEW1_M; truckCoEW1(ii).set];
                        %For WE1 lane
                        truckCoWE1(ii).set = BldLinesCo(real(truckObjWE1(tt,ii))-truckL/2, imag(truckObjWE1(tt,ii))-truckW/2, truckL, truckW);
                        truckCoWE1_M = [ truckCoWE1_M; truckCoWE1(ii).set];
                        %For EW2 lane
                        truckCoEW2(ii).set = BldLinesCo(real(truckObjEW2(tt,ii))-truckL/2, imag(truckObjEW2(tt,ii))-truckW/2, truckL, truckW);
                        truckCoEW2_M = [ truckCoEW2_M; truckCoEW2(ii).set];
                        %For WE2 lane
                        truckCoWE2(ii).set = BldLinesCo(real(truckObjWE2(tt,ii))-truckL/2, imag(truckObjWE2(tt,ii))-truckW/2, truckL, truckW);
                        truckCoWE2_M = [ truckCoWE2_M; truckCoWE2(ii).set];


                    end

                    %D: %Single matrix of the co-ordinates of all the lines of all the
                    %vehicles in all lanes
                    truckCo_All = [ truckCoSN_M; truckCoNS_M; truckCoEW1_M; truckCoWE1_M; truckCoEW2_M; truckCoWE2_M];

                else
                    NOtruck = 1; %if the truck value if zero then there wont be any plots for trucks
                end


                %%

                % Plot lane
                bldgFaceColor = [0.6 0.6 0.6];
                figure(1);

                %   To adjust display size according to screen resolution
                set (0,'unit','inches');
                inchs_screen = get(0,'screensize');
                screenSmall = [ 0 0 13 8];
                screenLarge = [ 0 0 17 10];

                if floor(inchs_screen) == screenSmall
                    % set(gcf, 'Position', [0,0,480,680]);
                    set(gcf, 'Position', [0,0,480,680]);
                elseif floor(inchs_screen) == screenLarge
                    set(gcf, 'Position', [0,0,650,1100]);
                else
                    set(gcf, 'Position', [ 0,0,650,1100]);
                end
                rectanglePlot(laneWidth,southWall,R,bldgFaceColor,RSU_N);

                for rr = 1:n_RSU
                    for jj = 1 : lambda


                        %%  ---------Plots for for SN lane--------%

                        %vehicles plot
                        plot(real(vehObjSN(tt,jj)),imag(vehObjSN(tt,jj)), 'o', 'MarkerEdgeColor', vehColorSN(jj,:), 'MarkerFaceColor', vehColorSN(jj,:), 'LineWidth', 5);
                        x2 = real(vehObjSN(tt,jj));
                        y2 = imag(vehObjSN(tt,jj));

                        %To check if the building blocks the signal between vehicles and RSU
                        for mm = 1:length(BldLinesCoordinates)
                            lineSN = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], BldLinesCoordinates(mm,:));
                            if lineSN.intMatrixX ~= 0 || lineSN.intMatrixY ~= 0
                                flagBlocked = 1;
                                break;
                            end
                        end
                        Dis_SN(jj,tt) = sqrt((x2-x_RSU(rr))^2+(y2-y_RSU(rr))^2); %distance between vehicle and RSU
                        if flagBlocked == 0


                            %To check if the vehicle is within the Range of RSU
                            if Dis_SN(jj,tt) > TxRange %condition for not within the Range of RSU
                                line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                %counting total number of blocked rays
                                BlockedRaysCtr = BlockedRaysCtr + 1;

                                %counting number of blocked rays for individual
                                %vehicle for MDR calculation
                                BC_SN(jj,1) = BC_SN(jj,1) + 1;

                                %Latency calculation: connections_SN_v contains tMax number of elments.
                                %Elements are only 1's and 0's; 1 = connected, 0 = disconnected

                                linked_SN_v(jj) = 0;
                                connections_SN_v(jj,tt,rr)= linked_SN_v(jj);
                                %For connections among vehicles:
                                connections_all = [ connections_all linked_SN_v(jj) ];


                            else %condition for within the range of RSU
                                %For loop to check if the connection is blocked by any of
                                %the trucks
                                for mm = 1:length(truckCo_All)
                                    lineSN = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], truckCo_All(mm,:));
                                    if lineSN.intMatrixX ~= 0 || lineSN.intMatrixY ~= 0
                                        truckFlagBlocked = 1;
                                        break;
                                    end
                                end
                                if truckFlagBlocked == 0

                                    %Calculation of Pathloss
                                    pathloss_SN(jj,tt) = Pathloss_UMi(Dis_SN(jj,tt), fc);
                                    pathloss_all  = [pathloss_all pathloss_SN(jj,tt)];
                                    % pow2db(db2pow(power_tx)/db2pow(pathloss_SN(jj,tt)));
                                    power_rx_SN(jj,tt) = pow2db(db2pow(power_tx)/db2pow(pathloss_SN(jj,tt)))+30; %Received Signal Power
                                    if power_rx_SN(jj,tt) >= power_rx_threshold

                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 0 0 1]);%Blue Line
                                        %counting total number of connected rays
                                        SuccessRaysCtr = SuccessRaysCtr + 1;

                                        %counting number of connected links for
                                        %individual vehicle  for MDR calculation
                                        SC_SN(jj,1) = SC_SN(jj,1) + 1;

                                        %Latency calculation: connections_SN_v1 contains tMax number of elments.
                                        %Elements are only 1's and 0's; 1 = connected, 0 = disconnected
                                        linked_SN_v(jj) = 1;
                                        connections_SN_v(jj,tt,rr)= linked_SN_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_SN_v(jj) ];

                                        %counter stores for how many tt-slots the
                                        %connection continues
                                        counter_SN(jj,tt) = randi(15);

                                        %for re-transmission:
                                        for re= 1:counter_SN
                                            RT_SN(jj,tt) = counter_SN(jj,tt)-1;
                                        end

                                    else
                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                        %counting total number of blocked rays
                                        BlockedRaysCtr = BlockedRaysCtr + 1;

                                        %counting number of blocked rays for individual vehicle for MDR calculation
                                        BC_SN(jj,1) = BC_SN(jj,1) + 1;

                                        %Latency calculation: connections_SN_v1 contains tMax number of elments.
                                        %Elements are only 1's and 0's; 1 = connected, 0 = disconnected
                                        linked_SN_v(jj) = 0;
                                        connections_SN_v(jj,tt,rr)= linked_SN_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_SN_v(jj) ];
                                    end


                                else
                                    line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                    %counting total number of blocked rays
                                    BlockedRaysCtr = BlockedRaysCtr + 1;

                                    %counting number of blocked rays for individual vehicle for MDR calculation
                                    BC_SN(jj,1) = BC_SN(jj,1) + 1;

                                    %Latency calculation: connections_SN_v1 contains tMax number of elments.
                                    %Elements are only 1's and 0's; 1 = connected, 0 = disconnected
                                    linked_SN_v(jj) = 0;
                                    connections_SN_v(jj,tt,rr)= linked_SN_v(jj);
                                    %For connections among vehicles:
                                    connections_all = [ connections_all linked_SN_v(jj) ];
                                end

                            end
                        else
                            line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                            %counting total number of blocked rays
                            BlockedRaysCtr = BlockedRaysCtr + 1;

                            %counting number of blocked rays for individual vehicle for MDR calculation
                            BC_SN(jj,1) = BC_SN(jj,1) + 1;

                            %Latency calculation: connections_SN_v1 contains tMax number of elments.
                            %Elements are only 1's and 0's; 1 = connected, 0 = disconnected
                            linked_SN_v(jj) = 0;
                            connections_SN_v(jj,tt,rr)= linked_SN_v(jj);
                            %For connections among vehicles:
                            connections_all = [ connections_all linked_SN_v(jj) ];

                        end
                        flagBlocked = 0 ;
                        truckFlagBlocked = 0;

                        grid on
                        box on
                        title('Urban: 2 junctions')
                        xlim([westWall eastWall])
                        ylim([southWall northWall])
                        xticks(westWall : laneWidth : eastWall)
                        yticks(southWall : laneWidth : northWall)
                        rectanglePlot(laneWidth,southWall,R,bldgFaceColor,RSU_N);


                        %trucks plot:
                        if NOtruck==0
                            for dd = 1: theta
                                posTruck_SN = [real(truckObjSN(tt,dd))-truckW/2 imag(truckObjSN(tt,dd))-truckL/2 truckW truckL];
                                rectangle('Position', posTruck_SN, 'FaceColor', 'k');
                            end
                        end

                        hold on;

                        %%  ---------Plots for for NS lane--------%

                        %trucks plot:
                        if NOtruck==0
                            for dd = 1: theta
                                posTruck_NS = [real(truckObjNS(tt,dd))-truckW/2 imag(truckObjNS(tt,dd))-truckL/2 truckW truckL];
                                rectangle('Position', posTruck_NS, 'FaceColor', 'k');
                            end
                        end

                        %vehicles plot
                        plot(real(vehObjNS(tt,jj)),imag(vehObjNS(tt,jj)), 'o', 'MarkerEdgeColor', vehColorNS(jj,:), 'MarkerFaceColor', vehColorNS(jj,:), 'LineWidth', 5);
                        x2 = real(vehObjNS(tt,jj));
                        y2 = imag(vehObjNS(tt,jj));
                        %To check if the building blocks the signal between vehicles and RSU
                        for mm = 1:length(BldLinesCoordinates)
                            lineNS = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], BldLinesCoordinates(mm,:));
                            if lineNS.intMatrixX ~= 0 || lineNS.intMatrixY ~= 0
                                flagBlocked = 1;
                                break;
                            end
                        end
                        Dis_NS(jj,tt) = sqrt((x2-x_RSU(rr))^2+(y2-y_RSU(rr))^2); %distance between vehicle and RSU
                        if flagBlocked == 0

                            %To check if the vehicle is within the Range of RSU
                            if  Dis_NS(jj,tt) > TxRange
                                line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                BlockedRaysCtr = BlockedRaysCtr + 1;
                                BC_NS(jj,1) = BC_NS(jj,1) + 1;
                                %Latency Calculation:
                                linked_NS_v(jj) = 0;
                                connections_NS_v(jj,tt,rr)= linked_NS_v(jj);
                                %For connections among vehicles:
                                connections_all = [ connections_all linked_NS_v(jj) ];
                            else
                                for mm = 1:length(truckCo_All)
                                    lineSN = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], truckCo_All(mm,:));
                                    if lineSN.intMatrixX ~= 0 || lineSN.intMatrixY ~= 0
                                        truckFlagBlocked = 1;
                                        break;
                                    end
                                end
                                if truckFlagBlocked == 0
                                    %Calculation of Pathloss
                                    pathloss_NS(jj,tt) = Pathloss_UMi(Dis_NS(jj,tt), fc);
                                    pathloss_all  = [pathloss_all pathloss_NS(jj,tt)];
                                    power_rx_NS(jj,tt) = pow2db(db2pow(power_tx)/db2pow(pathloss_NS(jj,tt)))+30;
                                    if power_rx_NS(jj,tt) >= power_rx_threshold

                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 0 0 1]);%Blue Line
                                        SuccessRaysCtr = SuccessRaysCtr + 1;
                                        SC_NS(jj,1) = SC_NS(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_NS_v(jj) = 1;
                                        connections_NS_v(jj,tt,rr)= linked_NS_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_NS_v(jj) ];

                                    else
                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                        BlockedRaysCtr = BlockedRaysCtr + 1;
                                        BC_NS(jj,1) = BC_NS(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_NS_v(jj) = 0;
                                        connections_NS_v(jj,tt,rr)= linked_NS_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_NS_v(jj) ];
                                    end

                                else
                                    line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                    BlockedRaysCtr = BlockedRaysCtr + 1;
                                    BC_NS(jj,1) = BC_NS(jj,1) + 1;
                                    %Latency Calculation:
                                    linked_NS_v(jj) = 0;
                                    connections_NS_v(jj,tt,rr)= linked_NS_v(jj);
                                    %For connections among vehicles:
                                    connections_all = [ connections_all linked_NS_v(jj) ];
                                end
                            end
                        else
                            line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                            BlockedRaysCtr = BlockedRaysCtr + 1;
                            BC_NS(jj,1) = BC_NS(jj,1) + 1;
                            %Latency Calculation:
                            linked_NS_v(jj) = 0;
                            connections_NS_v(jj,tt,rr)= linked_NS_v(jj);
                            %For connections among vehicles:
                            connections_all = [ connections_all linked_NS_v(jj) ];

                        end
                        flagBlocked = 0;
                        truckFlagBlocked = 0;

                        %%  ---------Plots for for EW1 lane--------%

                        %trucks plot:
                        if NOtruck==0
                            for dd = 1: theta
                                posTruck_EW1 = [real(truckObjEW1(tt,dd))-truckL/2 imag(truckObjEW1(tt,dd))-truckW/2 truckL truckW];
                                rectangle('Position', posTruck_EW1, 'FaceColor', 'k');
                            end
                        end

                        %vehicles plot
                        plot(real(vehObjEW1(tt,jj)),imag(vehObjEW1(tt,jj)), 'o', 'MarkerEdgeColor', vehColorEW1(jj,:), 'MarkerFaceColor', vehColorEW1(jj,:), 'LineWidth', 5);
                        x2 = real(vehObjEW1(tt,jj));
                        y2 = imag(vehObjEW1(tt,jj));
                        for mm = 1:length(BldLinesCoordinates)
                            %To check if the building blocks the signal between vehicles and RSU
                            lineEW1 = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], BldLinesCoordinates(mm,:));
                            if lineEW1.intMatrixX ~= 0 || lineEW1.intMatrixY ~= 0
                                flagBlocked = 1;
                                break;
                            end
                        end
                        Dis_EW1(jj,tt) = sqrt((x2-x_RSU(rr))^2+(y2-y_RSU(rr))^2);
                        if flagBlocked == 0

                            %To check if the vehicle is within the Range of RSU
                            if Dis_EW1(jj,tt) > TxRange
                                line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                BlockedRaysCtr = BlockedRaysCtr + 1;
                                BC_EW1(jj,1) = BC_EW1(jj,1) + 1;
                                %Latency Calculation:
                                linked_EW1_v(jj) = 0;
                                connections_EW1_v(jj,tt,rr)= linked_EW1_v(jj);
                                %For connections among vehicles:
                                connections_all = [ connections_all linked_EW1_v(jj) ];
                            else
                                for mm = 1:length(truckCo_All)
                                    lineSN = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], truckCo_All(mm,:));
                                    if lineSN.intMatrixX ~= 0 || lineSN.intMatrixY ~= 0
                                        truckFlagBlocked = 1;
                                        break;
                                    end
                                end
                                if truckFlagBlocked == 0
                                    %Calculation of Pathloss
                                    pathloss_EW1(jj,tt) = Pathloss_UMi(Dis_EW1(jj,tt), fc);
                                    pathloss_all  = [pathloss_all pathloss_EW1(jj,tt)];
                                    power_rx_EW1(jj,tt) = pow2db(db2pow(power_tx)/db2pow(pathloss_EW1(jj,tt)))+30;
                                    if power_rx_EW1(jj,tt) >= power_rx_threshold

                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 0 0 1]);%Blue Line
                                        SuccessRaysCtr = SuccessRaysCtr + 1;
                                        SC_EW1(jj,1) = SC_EW1(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_EW1_v(jj) = 1;
                                        connections_EW1_v(jj,tt,rr)= linked_EW1_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_EW1_v(jj) ];
                                    else
                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                        BlockedRaysCtr = BlockedRaysCtr + 1;
                                        BC_EW1(jj,1) = BC_EW1(jj,1) + 1;

                                        %Latency Calculation:
                                        linked_EW1_v(jj) = 0;
                                        connections_EW1_v(jj,tt,rr)= linked_EW1_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_EW1_v(jj) ];
                                    end
                                else
                                    line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                    BlockedRaysCtr = BlockedRaysCtr + 1;
                                    BC_EW1(jj,1) = BC_EW1(jj,1) + 1;

                                    %Latency Calculation:
                                    linked_EW1_v(jj) = 0;
                                    connections_EW1_v(jj,tt,rr)= linked_EW1_v(jj);
                                    %For connections among vehicles:
                                    connections_all = [ connections_all linked_EW1_v(jj) ];
                                end
                            end
                        else
                            line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                            BlockedRaysCtr = BlockedRaysCtr + 1;
                            BC_EW1(jj,1) = BC_EW1(jj,1) + 1;

                            %Latency Calculation:
                            linked_EW1_v(jj) = 0;
                            connections_EW1_v(jj,tt,rr)= linked_EW1_v(jj);
                            %For connections among vehicles:
                            connections_all = [ connections_all linked_EW1_v(jj) ];

                        end
                        flagBlocked = 0;
                        truckFlagBlocked = 0;


                        %%  ---------Plots for for WE1 lane--------%

                        %trucks plot:
                        if NOtruck==0
                            for dd = 1: theta
                                posTruck_WE1 = [real(truckObjWE1(tt,dd))-truckL/2 imag(truckObjWE1(tt,dd))-truckW/2 truckL truckW];
                                rectangle('Position', posTruck_WE1, 'FaceColor', 'k');
                            end
                        end

                        %vehicles plot
                        plot(real(vehObjWE1(tt,jj)),imag(vehObjWE1(tt,jj)), 'o', 'MarkerEdgeColor', vehColorWE1(jj,:), 'MarkerFaceColor', vehColorWE1(jj,:), 'LineWidth', 5);
                        x2 = real(vehObjWE1(tt,jj));
                        y2 = imag(vehObjWE1(tt,jj));
                        for mm = 1:length(BldLinesCoordinates)
                            lineWE1 = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], BldLinesCoordinates(mm,:));
                            if lineWE1.intMatrixX ~= 0 || lineWE1.intMatrixY ~= 0
                                flagBlocked = 1;
                                break;
                            end
                        end
                        Dis_WE1(jj,tt) = sqrt((x2-x_RSU(rr))^2+(y2-y_RSU(rr))^2);
                        if flagBlocked == 0

                            if Dis_WE1(jj,tt) > TxRange
                                line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                BlockedRaysCtr = BlockedRaysCtr + 1;
                                BC_WE1(jj,1) = BC_WE1(jj,1) + 1;
                                %Latency Calculation:
                                linked_WE1_v(jj) = 0;
                                connections_WE1_v(jj,tt,rr)= linked_WE1_v(jj);
                                %For connections among vehicles:
                                connections_all = [ connections_all linked_WE1_v(jj) ];
                            else
                                for mm = 1:length(truckCo_All)
                                    lineSN = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], truckCo_All(mm,:));
                                    if lineSN.intMatrixX ~= 0 || lineSN.intMatrixY ~= 0
                                        truckFlagBlocked = 1;
                                        break;
                                    end
                                end
                                if truckFlagBlocked == 0
                                    %Calculation of Pathloss
                                    pathloss_WE1(jj,tt) = Pathloss_UMi(Dis_WE1(jj,tt), fc);
                                    pathloss_all  = [pathloss_all pathloss_WE1(jj,tt)];
                                    power_rx_WE1(jj,tt) = pow2db(db2pow(power_tx)/db2pow(pathloss_WE1(jj,tt)))+30;
                                    if power_rx_WE1(jj,tt) >= power_rx_threshold

                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 0 0 1]);%Blue Line
                                        SuccessRaysCtr = SuccessRaysCtr + 1;
                                        SC_WE1(jj,1) = SC_WE1(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_WE1_v(jj) = 1;
                                        connections_WE1_v(jj,tt,rr)= linked_WE1_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_WE1_v(jj) ];
                                    else
                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                        BlockedRaysCtr = BlockedRaysCtr + 1;
                                        BC_WE1(jj,1) = BC_WE1(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_WE1_v(jj) = 0;
                                        connections_WE1_v(jj,tt,rr)= linked_WE1_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_WE1_v(jj) ];
                                    end
                                else
                                    line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                    BlockedRaysCtr = BlockedRaysCtr + 1;
                                    BC_WE1(jj,1) = BC_WE1(jj,1) + 1;
                                    %Latency Calculation:
                                    linked_WE1_v(jj) = 0;
                                    connections_WE1_v(jj,tt,rr)= linked_WE1_v(jj);
                                    %For connections among vehicles:
                                    connections_all = [ connections_all linked_WE1_v(jj) ];
                                end
                            end
                        else
                            line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                            BlockedRaysCtr = BlockedRaysCtr + 1;
                            BC_WE1(jj,1) = BC_WE1(jj,1) + 1;
                            %Latency Calculation:
                            linked_WE1_v(jj) = 0;
                            connections_WE1_v(jj,tt,rr)= linked_WE1_v(jj);
                            %For connections among vehicles:
                            connections_all = [ connections_all linked_WE1_v(jj) ];
                        end
                        flagBlocked = 0;
                        truckFlagBlocked = 0;


                        %%  ---------Plots for for EW2 lane--------%

                        %trucks plot:
                        if NOtruck==0
                            for dd = 1: theta
                                posTruck_EW2 = [real(truckObjEW2(tt,dd))-truckL/2 imag(truckObjEW2(tt,dd))-truckW/2 truckL truckW];
                                rectangle('Position', posTruck_EW2, 'FaceColor', 'k');
                            end
                        end

                        %vehicles plot
                        plot(real(vehObjEW2(tt,jj)),imag(vehObjEW2(tt,jj)), 'o', 'MarkerEdgeColor', vehColorEW2(jj,:), 'MarkerFaceColor', vehColorEW2(jj,:), 'LineWidth', 5);
                        x2 = real(vehObjEW2(tt,jj));
                        y2 = imag(vehObjEW2(tt,jj));
                        for mm = 1:length(BldLinesCoordinates)
                            lineEW2 = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], BldLinesCoordinates(mm,:));
                            if lineEW2.intMatrixX ~= 0 || lineEW2.intMatrixY ~= 0
                                flagBlocked = 1;
                                break;
                            end
                        end
                        Dis_EW2(jj,tt) = sqrt((x2-x_RSU(rr))^2+(y2-y_RSU(rr))^2);
                        if flagBlocked == 0

                            if Dis_EW2(jj,tt) > TxRange
                                line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                BlockedRaysCtr = BlockedRaysCtr + 1;
                                BC_EW2(jj,1) = BC_EW2(jj,1) + 1;
                                %Latency Calculation:
                                linked_EW2_v(jj) = 0;
                                connections_EW2_v(jj,tt,rr)= linked_EW2_v(jj);
                                %For connections among vehicles:
                                connections_all = [ connections_all linked_EW2_v(jj) ];
                            else
                                for mm = 1:length(truckCo_All)
                                    lineSN = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], truckCo_All(mm,:));
                                    if lineSN.intMatrixX ~= 0 || lineSN.intMatrixY ~= 0
                                        truckFlagBlocked = 1;
                                        break;
                                    end
                                end
                                if truckFlagBlocked == 0
                                    %Calculation of Pathloss
                                    pathloss_EW2(jj,tt) = Pathloss_UMi(Dis_EW2(jj,tt), fc);
                                    pathloss_all  = [pathloss_all pathloss_EW2(jj,tt)];
                                    power_rx_EW2(jj,tt) = pow2db(db2pow(power_tx)/db2pow(pathloss_EW2(jj,tt)))+30;
                                    if power_rx_EW2(jj,tt) >= power_rx_threshold

                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 0 0 1]);%Blue Line
                                        SuccessRaysCtr = SuccessRaysCtr + 1;
                                        SC_EW2(jj,1) = SC_EW2(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_EW2_v(jj) = 1;
                                        connections_EW2_v(jj,tt,rr)= linked_EW2_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_EW2_v(jj) ];
                                    else
                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                        BlockedRaysCtr = BlockedRaysCtr + 1;
                                        BC_EW2(jj,1) = BC_EW2(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_EW2_v(jj) = 0;
                                        connections_EW2_v(jj,tt,rr)= linked_EW2_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_EW2_v(jj) ];
                                    end
                                else
                                    line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                    BlockedRaysCtr = BlockedRaysCtr + 1;
                                    BC_EW2(jj,1) = BC_EW2(jj,1) + 1;
                                    %Latency Calculation:
                                    linked_EW2_v(jj) = 0;
                                    connections_EW2_v(jj,tt,rr)= linked_EW2_v(jj);
                                    %For connections among vehicles:
                                    connections_all = [ connections_all linked_EW2_v(jj) ];
                                end
                            end
                        else
                            line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                            BlockedRaysCtr = BlockedRaysCtr + 1;
                            BC_EW2(jj,1) = BC_EW2(jj,1) + 1;
                            %Latency Calculation:
                            linked_EW2_v(jj) = 0;
                            connections_EW2_v(jj,tt,rr)= linked_EW2_v(jj);
                            %For connections among vehicles:
                            connections_all = [ connections_all linked_EW2_v(jj) ];

                        end
                        flagBlocked = 0;
                        truckFlagBlocked = 0;

                        %%  ---------Plots for for WE2 lane--------%

                        %trucks plot:
                        if NOtruck==0
                            for dd = 1: theta
                                posTruck_WE2 = [real(truckObjWE2(tt,dd))-truckL/2 imag(truckObjWE2(tt,dd))-truckW/2 truckL truckW];
                                rectangle('Position', posTruck_WE2, 'FaceColor', 'k');
                            end
                        end

                        %vehicles plot
                        plot(real(vehObjWE2(tt,jj)),imag(vehObjWE2(tt,jj)), 'o', 'MarkerEdgeColor', vehColorWE2(jj,:), 'MarkerFaceColor', vehColorWE2(jj,:), 'LineWidth', 5);
                        x2 = real(vehObjWE2(tt,jj));
                        y2 = imag(vehObjWE2(tt,jj));
                        for mm = 1:length(BldLinesCoordinates)
                            lineWE2 = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], BldLinesCoordinates(mm,:));
                            if lineWE2.intMatrixX ~= 0 || lineWE2.intMatrixY ~= 0
                                flagBlocked = 1;
                                break;
                            end
                        end
                        Dis_WE2(jj,tt) = sqrt((x2-x_RSU(rr))^2+(y2-y_RSU(rr))^2);
                        if flagBlocked == 0

                            if Dis_WE2(jj,tt) > TxRange
                                line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                BlockedRaysCtr = BlockedRaysCtr + 1;
                                BC_WE2(jj,1) = BC_WE2(jj,1) + 1;
                                %Latency Calculation:
                                linked_WE2_v(jj) = 0;
                                connections_WE2_v(jj,tt,rr)= linked_WE2_v(jj);
                                %For connections among vehicles:
                                connections_all = [ connections_all linked_WE2_v(jj) ];
                            else
                                for mm = 1:length(truckCo_All)
                                    lineSN = lineSegmentIntersect([ x_RSU(rr) y_RSU(rr) x2 y2], truckCo_All(mm,:));
                                    if lineSN.intMatrixX ~= 0 || lineSN.intMatrixY ~= 0
                                        truckFlagBlocked = 1;
                                        break;
                                    end
                                end
                                if truckFlagBlocked == 0
                                    %Calculation of Pathloss
                                    pathloss_WE2(jj,tt) = Pathloss_UMi(Dis_WE2(jj,tt), fc);
                                    pathloss_all  = [pathloss_all pathloss_WE2(jj,tt)];
                                    power_rx_WE2(jj,tt) = pow2db(db2pow(power_tx)/db2pow(pathloss_WE2(jj,tt)))+30;
                                    if power_rx_WE2(jj,tt) >= power_rx_threshold

                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 0 0 1]);%Blue Line
                                        SuccessRaysCtr = SuccessRaysCtr + 1;
                                        SC_WE2(jj,1) = SC_WE2(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_WE2_v(jj) = 1;
                                        connections_WE2_v(jj,tt,rr)= linked_WE2_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_WE2_v(jj) ];
                                    else
                                        line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                        BlockedRaysCtr = BlockedRaysCtr + 1;
                                        BC_WE2(jj,1) = BC_WE2(jj,1) + 1;
                                        %Latency Calculation:
                                        linked_WE2_v(jj) = 0;
                                        connections_WE2_v(jj,tt,rr)= linked_WE2_v(jj);
                                        %For connections among vehicles:
                                        connections_all = [ connections_all linked_WE2_v(jj) ];
                                    end
                                else
                                    line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                                    BlockedRaysCtr = BlockedRaysCtr + 1;
                                    BC_WE2(jj,1) = BC_WE2(jj,1) + 1;
                                    %Latency Calculation:
                                    linked_WE2_v(jj) = 0;
                                    connections_WE2_v(jj,tt,rr)= linked_WE2_v(jj);
                                    %For connections among vehicles:
                                    connections_all = [ connections_all linked_WE2_v(jj) ];
                                end

                            end
                        else
                            line([real(RSU(rr)) x2], [imag(RSU(rr)) y2], 'Color', [ 1 0 0]);%Red Line
                            BlockedRaysCtr = BlockedRaysCtr + 1;
                            BC_WE2(jj,1) = BC_WE2(jj,1) + 1;
                            %Latency Calculation:
                            linked_WE2_v(jj) = 0;
                            connections_WE2_v(jj,tt,rr)= linked_WE2_v(jj);
                            %For connections among vehicles:
                            connections_all = [ connections_all linked_WE2_v(jj) ];

                        end
                        flagBlocked = 0;
                        truckFlagBlocked = 0;

                    end
                end

                %Resetting truck coordinates for next iteration
                truckCoSN_M = [];
                truckCoNS_M = [];
                truckCoEW1_M = [];
                truckCoWE1_M = [];
                truckCoEW2_M = [];
                truckCoWE2_M = [];
                connections_all = [];%Resetting the value for the next tt iteration
                hold off;

            end

            % D: Printing total number of Successful and Blocked connections with RSU :
            fprintf('The number of successful Connections with RSU is: %d\n',SuccessRaysCtr);
            fprintf('The number of blocked Connections with RSU is: %d\n\n',BlockedRaysCtr);

            %D:%% combining the connections values from RSUs by using logical-OR
            %SN-lane:
            if RSU_N==1
                connections_SN_RSU = connections_SN_v;
            elseif RSU_N==2
                RSU_1_SN = connections_SN_v(:,:,1);
                RSU_2_SN = connections_SN_v(:,:,2);
                connections_SN_RSU = double(RSU_1_SN | RSU_2_SN);
            elseif RSU_N==3
                RSU_1_SN = connections_SN_v(:,:,1);
                RSU_2_SN = connections_SN_v(:,:,2);
                RSU_3_SN = connections_SN_v(:,:,3);
                connections_SN_RSU = double(RSU_1_SN | RSU_2_SN | RSU_3_SN);
            end

            %NS-lane:
            if RSU_N==1
                connections_NS_RSU = connections_NS_v;
            elseif RSU_N==2
                RSU_1_NS = connections_NS_v(:,:,1);
                RSU_2_NS = connections_NS_v(:,:,2);
                connections_NS_RSU = double(RSU_1_NS | RSU_2_NS);
            elseif RSU_N==3
                RSU_1_NS = connections_NS_v(:,:,1);
                RSU_2_NS = connections_NS_v(:,:,2);
                RSU_3_NS = connections_NS_v(:,:,3);
                connections_NS_RSU = double(RSU_1_NS | RSU_2_NS | RSU_3_NS);
            end

            %EW1-lane:
            if RSU_N==1
                connections_EW1_RSU = connections_EW1_v;
            elseif RSU_N==2
                RSU_1_EW1 = connections_EW1_v(:,:,1);
                RSU_2_EW1 = connections_EW1_v(:,:,2);
                connections_EW1_RSU = double(RSU_1_EW1 | RSU_2_EW1);
            elseif RSU_N==3
                RSU_1_EW1 = connections_EW1_v(:,:,1);
                RSU_2_EW1 = connections_EW1_v(:,:,2);
                RSU_3_EW1 = connections_EW1_v(:,:,3);
                connections_EW1_RSU = double(RSU_1_EW1 | RSU_2_EW1 | RSU_3_EW1);
            end

            %WE1-lane:
            if RSU_N==1
                connections_WE1_RSU = connections_WE1_v;
            elseif RSU_N==2
                RSU_1_WE1 = connections_WE1_v(:,:,1);
                RSU_2_WE1 = connections_WE1_v(:,:,2);
                connections_WE1_RSU = double(RSU_1_WE1 | RSU_2_WE1);
            elseif RSU_N==3
                RSU_1_WE1 = connections_WE1_v(:,:,1);
                RSU_2_WE1 = connections_WE1_v(:,:,2);
                RSU_3_WE1 = connections_WE1_v(:,:,3);
                connections_WE1_RSU = double(RSU_1_WE1 | RSU_2_WE1 | RSU_3_WE1);
            end

            %EW2-lane:
            if RSU_N==1
                connections_EW2_RSU = connections_EW2_v;
            elseif RSU_N==2
                RSU_1_EW2 = connections_EW2_v(:,:,1);
                RSU_2_EW2 = connections_EW2_v(:,:,2);
                connections_EW2_RSU = double(RSU_1_EW2 | RSU_2_EW2);
            elseif RSU_N==3
                RSU_1_EW2 = connections_EW2_v(:,:,1);
                RSU_2_EW2 = connections_EW2_v(:,:,2);
                RSU_3_EW2 = connections_EW2_v(:,:,3);
                connections_EW2_RSU = double(RSU_1_EW2 | RSU_2_EW2 | RSU_3_EW2);
            end

            %WE2-lane:
            if RSU_N==1
                connections_WE2_RSU = connections_WE2_v;
            elseif RSU_N==2
                RSU_1_WE2 = connections_WE2_v(:,:,1);
                RSU_2_WE2 = connections_WE2_v(:,:,2);
                connections_WE2_RSU = double(RSU_1_WE2 | RSU_2_WE2);
            elseif RSU_N==3
                RSU_1_WE2 = connections_WE2_v(:,:,1);
                RSU_2_WE2 = connections_WE2_v(:,:,2);
                RSU_3_WE2 = connections_WE2_v(:,:,3);
                connections_WE2_RSU = double(RSU_1_WE2 | RSU_2_WE2 | RSU_3_WE2);
            end


            %FOR SINR calculation:

            %SNR parameters:
            veh_all_per_tt = [connections_SN_RSU; connections_NS_RSU; connections_EW1_RSU; connections_WE1_RSU; connections_EW2_RSU; connections_WE2_RSU];

            SNR_distances = [vehObjSN'; vehObjNS'; vehObjEW1'; vehObjWE1'; vehObjEW2'; vehObjWE2'];
            SNR_distances = SNR_distances.*(veh_all_per_tt~=0);
            % SNR_pathloss = [ pathloss_SN; pathloss_NS; pathloss_EW1; pathloss_WE1; pathloss_EW2; pathloss_WE2];
            Tx_veh_power = power_tx; %same as that of an RSU
            SNR_Rx_power = [];

        end


       %% Method 2
        %%  Calculating Latency and PDR of all the vehicles for every iterations:
        veh_all_per_tt = [connections_SN_RSU; connections_NS_RSU; connections_EW1_RSU; connections_WE1_RSU; connections_EW2_RSU; connections_WE2_RSU];
        veh_all_dis = [Dis_SN; Dis_NS; Dis_EW1; Dis_WE1; Dis_EW2; Dis_WE2];
        if MCS_set(mcs) == 2
            power_rx_all_MCS2 = [power_rx_SN; power_rx_NS; power_rx_EW1; power_rx_WE1; power_rx_EW2; power_rx_WE2];
        elseif MCS_set(mcs) == 7
            power_rx_all_MCS7 = [power_rx_SN; power_rx_NS; power_rx_EW1; power_rx_WE1; power_rx_EW2; power_rx_WE2];
        elseif MCS_set(mcs) == 11
            power_rx_all_MCS11 = [power_rx_SN; power_rx_NS; power_rx_EW1; power_rx_WE1; power_rx_EW2; power_rx_WE2];
        elseif MCS_set(mcs) == 15
            power_rx_all_MCS15 = [power_rx_SN; power_rx_NS; power_rx_EW1; power_rx_WE1; power_rx_EW2; power_rx_WE2];
        end
        power_rx_all = [power_rx_SN; power_rx_NS; power_rx_EW1; power_rx_WE1; power_rx_EW2; power_rx_WE2];

        x_mcs = [];
        y_mcs = [];

        payload = [300]; %size of BSM

        %Calculating Noise Power:
        msgLen = payload*8+32; %BSM size: 300 bytes = 2400bits + 32 bits (CRC)
        k = 1.380649e-23; %Boltamann constant
        To = 295; % average atmospheric temperature in Kelvin
        BW = 20e6; %20 MHz channel Bandwidth
        Noise_power = k*To*BW; %in watts
        Pn_dBm = 10*log10(Noise_power)+30; % Noise power in dBm

        %initializing pathloss vector for each vehicle at each tt:
        BER_V = zeros(lambda*6,tMax);
        SINR_V = zeros(lambda*6,tMax);
        MCS = MCS_set(mcs);
        if MCS == 0
            BER_threshold = 0.1;
            MCS_struct = struct('Index',0,'CR',0.13,'Modulation',"QPSK",'M',4,"min_SINR",-2.83);
        elseif MCS == 1
            BER_threshold = 0.1;
            MCS_struct = struct('Index',1,'CR',0.17,'Modulation',"QPSK",'M',4,"min_SINR",-1.38);
        elseif MCS == 2
            BER_threshold = 0.1;
            MCS_struct = struct('Index',2,'CR',0.21,'Modulation',"QPSK",'M',4,"min_SINR",-0.22);
        elseif MCS == 3
            BER_threshold = 0.1;
            MCS_struct = struct('Index',3,'CR',0.27,'Modulation',"QPSK",'M',4,"min_SINR",1.49);
        elseif MCS == 4
            BER_threshold = 0.1;
            MCS_struct = struct('Index',4,'CR',0.33,'Modulation',"QPSK",'M',4,"min_SINR",2.76);
        elseif MCS == 5
            BER_threshold = 0.1;
            MCS_struct = struct('Index',5,'CR',0.41,'Modulation',"QPSK",'M',4,"min_SINR",4.4);
        elseif MCS == 6
            BER_threshold = 0.1;
            MCS_struct = struct('Index',6,'CR',0.48,'Modulation',"QPSK",'M',4,"min_SINR",5.79);
        elseif MCS == 7
            BER_threshold = 0.1;
            MCS_struct = struct('Index',7,'CR',0.57,'Modulation',"QPSK",'M',4,"min_SINR",7.3);
        elseif MCS == 8
            BER_threshold = 0.1;
            MCS_struct = struct('Index',8,'CR',0.65,'Modulation',"QPSK",'M',4,"min_SINR",8.6);
        elseif MCS == 9
            BER_threshold = 0.1;
            MCS_struct = struct('Index',9,'CR',0.73,'Modulation',"QPSK",'M',4,"min_SINR",9.88);
        elseif MCS == 10
            BER_threshold = 0.1;
            MCS_struct = struct('Index',10,'CR',0.82,'Modulation',"QPSK",'M',4,"min_SINR",11.16);
        elseif MCS == 11
            BER_threshold = 0.1;
            MCS_struct = struct('Index',11,'CR',0.41,'Modulation',"QAM",'M',16,"min_SINR",11.16);
        elseif MCS == 12
            BER_threshold = 0.1;
            MCS_struct = struct('Index',12,'CR',0.46,'Modulation',"QAM",'M',16,"min_SINR",12.83);
        elseif MCS == 13
            BER_threshold = 0.1;
            MCS_struct = struct('Index',13,'CR',0.52,'Modulation',"QAM",'M',16,"min_SINR",14.46);
        elseif MCS == 14
            BER_threshold = 0.1;
            MCS_struct = struct('Index',14,'CR',0.59,'Modulation',"QAM",'M',16,"min_SINR",16.39);
        elseif MCS == 15
            BER_threshold = 0.1;
            MCS_struct = struct('Index',15,'CR',0.67,'Modulation',"QAM",'M',16,"min_SINR",18.73);
        elseif MCS == 16
            BER_threshold = 0.1;
            MCS_struct = struct('Index',16,'CR',0.72,'Modulation',"QAM",'M',16,"min_SINR",20.05);
        elseif MCS == 17
            BER_threshold = 0.1;
            MCS_struct = struct('Index',17,'CR',0.75,'Modulation',"QAM",'M',16,"min_SINR",21.10);
        elseif MCS == 18
            BER_threshold = 0.1;
            MCS_struct = struct('Index',18,'CR',0.84,'Modulation',"QAM",'M',16,"min_SINR",23.54);
        elseif MCS == 19
            BER_threshold = 0.1;
            MCS_struct = struct('Index',19,'CR',0.92,'Modulation',"QAM",'M',16,"min_SINR",25.93);
        elseif MCS == 20
            BER_threshold = 0.1;
            MCS_struct = struct('Index',20,'CR',1,'Modulation',"QAM",'M',16,"min_SINR",28.10);
        end

        % %Turbo Coding with QPSK:
        % MCS_2 = struct('Index',2,'CR',0.21,'Modulation',"QPSK",'M',4);
        % MCS_7 = struct('Index',7,'CR',0.57,'Modulation',"QPSK",'M',4);
        % MCS_9 = struct('Index',9,'CR',0.73,'Modulation',"QPSK",'M',4);
        % MCS_11 = struct('Index',11,'CR',0.41,'Modulation',"QAM",'M',16);
        % MCS_15 = struct('Index',15,'CR',0.67,'Modulation',"QAM",'M',16);


        %Initializing matrix variables for PDR calculations
        %1. Distances at which the vehicles are in LOS of RSU and PDR are calcuated
        V_RSU_dis = zeros(size(veh_all_dis));
        %2. packets_Txed:
        packets_txed = zeros(size(veh_all_dis));
        %3. packets_txed_AND_rxed:
        packets_txed_AND_rxed = zeros(size(veh_all_dis));
        %4. packets_dropped:
        packets_dropped = zeros(size(veh_all_dis));
        %5. PDR at every instant of distance:
        PDR_dis = zeros(size(veh_all_dis));

        % rng(17211)
        RT = 1; %Number of retransmissions:

        %Bandwidth
        BW = 20; %MHz



        RRI = 100; %Resource Reservation Interval
        MCS = MCS_set(mcs);
        % CCM = 0; %Congestion Control Mechanism Enabled = 1/ Disabled = 0
        CCM = 1;

        % Resource Reselection Counter (RC) range depending upon the RRI
        if RRI==20
            RC_Low = 25; %5
            RC_High = 75; %15
        elseif RRI ==50
            RC_Low = 10; %5
            RC_High = 30; %15
        elseif RRI == 100
            RC_Low = 5; %5
            RC_High = 15; %15
        else
            RC_Low = 4;
            RC_High = 6;
        end

        % For 20MHz Bandwidth 1 subframe is equal to 1 slot
        %Number of subchannel per subframe is determined by the chosen MCS
        %index
        %MCS 7: QPSK, (Coding Rate = 0.57, RBs per subchannel = 20,
        %subchannels per subframe = RRI/20 = 100/20 = 5.
        if MCS >=0 && MCS<5
            subchannel_N = 2;
            slot_delay = 1; %one subframe has two slots. one subframe = 1 msec, one slot = 0.5msec
        elseif MCS >=5 && MCS<10
            subchannel_N = 5;
            slot_delay = 1; %one subframe has two slots. one subframe = 1 msec, one slot = 0.5msec
        elseif MCS >=10 && MCS<15
            subchannel_N = 7;
            slot_delay = 1; %one subframe has two slots. one subframe = 1 msec, one slot = 0.5msec
        elseif MCS>=15
            subchannel_N = 10;
            slot_delay = 1; %one subframe has two slots. one subframe = 1 msec, one slot = 0.5msec
        end

        %string variables for each vehicle in the simulation:
        for i=1:lambda
            SN_v(i) = strcat("SN_v",string(i));
            NS_v(i) = strcat("NS_v",string(i));
            EW1_v(i) = strcat("EW1_v",string(i));
            WE1_v(i) = strcat("WE1_v",string(i));
            EW2_v(i) = strcat("EW2_v",string(i));
            WE2_v(i) = strcat("WE2_v",string(i));
        end
        % veh_names_1row = [SN_v, NS_v, EW1_v, WE1_v, EW2_v, WE2_v];
        veh_names = [SN_v, NS_v, EW1_v, WE1_v, EW2_v, WE2_v];

        %creating a structure named "slot" which contains fields:
        % RRI  %msec is the Resource Rervation Interval after which the resources are reselected;
        %RRR = Resource Reselection Interval
        if CCM == 1
            slots_N= RRI*2;% Total number of slots in each RRI (one slot for one packet transmission)
        elseif CCM == 0
            slots_N = RRI;
        end

        %Number of slots occupied including retransmission:
        % RT_Slots = floor(slots_N/(RT+1));
        % RT_Slot_Num = 0;
        rt_flag = 1; %flag to indicate the completion of retransmission window/RRI period
        ss_RT = 0;
        %---------------------------------------------------------------------------------------------------

        % creating a structure for slots
        for ww= 1: RSU_N*subchannel_N
            s.subChannel(ww) = "";
            s.RC(ww) = 0; %Reselection Counter
            s.retransmitCounter(ww) = 0; %Retransmission Counter
            s.reserved(ww) = 0;
            s.RT_Completed (ww)  = 0;
            s.RRI_ON(ww) = 0;
            s.RRI_delay(ww) = 0;
            s.RSSI(ww) = 0;

        end

        %creating a structure named "Latency" to calculate latency for each vehicle
        l.vehicle = "";
        l.message_received  = 0;
        l.alloted = 0;
        l.packets_dropped = 0;
        l.RT_Counter = 0;
        l.RRI_delay = 0;
        % l.RRI_ON = 0;
        % l.RT_delay = 0;
        l.total = 0;
        l.slot_position.slot = 0;
        l.slot_position.subChannel = 0;
        l.slot_RT_Completed = 0;
        l.packets_TXed  = 0;
        l.packets_TXed_AND_RXed = 0;
        l.PDR = 0;
        l.subChannel_Occupied = 0;
        flag_1st_txed = zeros(1,length(veh_all_per_tt));



        %Each block of 50RBs = 1subchannel, 1 subchannel accomodates one vehicle
        %for data transmission; for [10, 20, 30]MHz, there are [1,2,3] vehicles
        %being accomodated per slot (there are two slots in a subframe).
        slot = repmat(s,1,slots_N); %total number of slots available for every tt transmission is RRI*2
        Latency = repmat(l,1,length(veh_names));


        RRI_delay = RRI;%


        for xx = 1:slots_N%assignment of delays for each slot
            slot(xx).slot_delay =slot_delay*xx;
            %     disp(xx);
            %     disp(slot(xx).slot_delay);
        end


        %% Since the RC of slots are randomly selected: Averaging the PDR plots for dist over "a" number of iterations

        % RC_holder = 0; %To hold the value of RC for one vehicle until it reaches to zero.
        %setting up random counter from RC_Low to RC_High which gives the number of time-frames
        %(tt's) a connected vehicle can communicate (i.e. can be stay connected)
        for i = 1:numel(slot)
            for mm = 1:RSU_N*subchannel_N
                slot(i).RC(mm) = randi([RC_Low,RC_High]); %Random Reselection Counter
                slot(i).retransmitCounter(mm) = 1+RT;

            end
        end
        slot_occupied = 0;
        RC_holder = 0;
        RT_tracker = 0;
        RT_holder = 0;

        if RC_Low<=RT
            error('RC_Low should be greater than RT!!!')
        end
        %allocating slots: vehicles with 1s (connected) are assigned slots and are stored in linked. vehicles with 0s are not assigned slots and are
        %stored in not_linked
        linked = [];

        queue = []; %to store vehicles waiting for their turn


        %%Channel Busy Ratio/Rate:
        %RSSI threshold:
        RSSI_th = -94; %dBm
        exceeded_RSSI_th = 0;

        CBR = 0;
        CBR_all = []; %Channel Busy Ratio of each vehicle
        C_R = zeros(1,floor(tMax/10)); %Channel Occupancy Ratio of each vehicle
        C_R_all = [];

        for tt = 1:tMax
            rt_flag = 1;
            veh_status =  veh_all_per_tt(:,tt); %status of vehicles in each time frame (tt=1)
            veh_RSSI = power_rx_all(:,tt);
            Num_veh = numel(veh_status); %total number of vehicles in time frame (tt); includes both conntected (0s) and blocked (1s)


            %     if isempty(queue) %if the queue is empty
            for pp = 1:Num_veh
                fprintf('tt = %d, pp = %d\n',tt,pp);
                if (veh_status(pp)==1) %if vehicle is connected (1)
                    oo_flag = 0;
                    rt_flag = 1;
                    % pp_flag = 0; %to break off of the loop RT_Completed~=1

                    % RT_tracker = 0;
                    if  Latency(pp).RT_Counter  == 0
                        ss = 1;
                        if slot_occupied ~= slots_N
                            while ss<=slots_N
                                for oo = 1:RSU_N*subchannel_N %for each RSU
                                     fprintf(' oo = %d, ss = %d, tt = %d\n',oo,ss,tt);
                                    if slot(ss).reserved(oo)==0 %is the slot available? (slot is available)
                                        %                         if period_counter == 0
                                        fprintf(' oo = %d, ss = %d, tt = %d\n',oo,ss,tt);
                                        if isempty(queue) %is the queue empty? (the queue is empty)
                                            if isempty(linked) %when there is no vehicle connected (this if-block is executed for the first time only)
                                                slot(ss).subChannel(oo) = veh_names(pp);
                                                linked = unique([linked veh_names(pp)],'stable'); % not implemented yet----------remove after RC is zero
                                                slot(ss).reserved(oo) = 1;
                                                slot(ss).RT_Completed(oo) = 0;
                                                RC_holder = slot(ss).RC(oo);
                                                Latency(pp).vehicle = slot(ss).subChannel(oo);
                                                Latency(pp).packets_TXed   = Latency(pp).packets_TXed   + 1; %Calculate BER; BER<BER_threshold is considered as succesfully decoded --- to be implemented ----
                                                flag_1st_txed(pp) = flag_1st_txed(pp) + 1 ;
                                                Latency(pp).alloted = Latency(pp).alloted + 1;
                                                slot_occupied = slot_occupied + 1;
                                                Latency(pp).slot_position.slot(ss) = ss;
                                                Latency(pp).slot_position.subChannel(ss) = oo;

                                                %PDR Calculation:
                                                V_RSU_dis(pp,tt) = veh_all_dis(pp,tt);
                                                packets_txed(pp,tt) = packets_txed(pp,tt)+1;

                                                % %RSSI
                                                slot(ss).RSSI(oo) = veh_RSSI(pp);
                                                 if slot(ss).retransmitCounter(oo)>1

                                                    RT_tracker = 1; %Tracks retransmission until retransmitCounter decreases to zero; 1 = continues retransmission; 0 = stops retransmission
                                                    RT_holder = slot(ss).retransmitCounter(oo);
                                                    Latency(pp).RT_Counter = RT_holder;

                                                else
                                                    RT_tracker = 0;
                                                    rt_flag = 0;
                                                    slot(ss).RT_Completed  (oo) = 1;
                                                    Latency(pp).message_received =  Latency(pp).message_received + 1;
                                                    Latency(pp).total = Latency(pp).total + slot(ss).slot_delay;
                                                    Latency(pp).alloted = 0 ;
                                                    Latency(pp).RRI_delay = 0;
                                                    Latency(pp).RT_Counter = 1;
                                                    slot(ss).RRI_delay(oo) = 0;
                                                    slot(ss).RRI_ON(oo) = 0;
                                                    for c=1:ss
                                                        for cc=1:RSU_N*subchannel_N
                                                            % for c=1:ss

                                                            if slot(c).subChannel(cc) == slot(ss).subChannel(oo)
                                                                slot(c).RT_Completed  (cc) = 1;
                                                            end
                                                        end
                                                    end
                                                end

                                            else%is the vehicle already connected? if it is connected it will be in the "linked" vector
                                                r = find(linked==veh_names(pp));
                                                if isempty(r)%the vehicle is not connected (assign a new slot)

                                                    slot(ss).subChannel(oo) = veh_names(pp);
                                                    linked = unique([linked veh_names(pp)],'stable');
                                                    slot(ss).reserved(oo) = 1;
                                                    slot(ss).RT_Completed  (oo) = 0;
                                                    RC_holder = slot(ss).RC(oo);
                                                    slot_occupied = slot_occupied + 1;
                                                    Latency(pp).vehicle = slot(ss).subChannel(oo);
                                                    Latency(pp).packets_TXed   = Latency(pp).packets_TXed   + 1;
                                                    flag_1st_txed(pp) = flag_1st_txed(pp) + 1;
                                                    Latency(pp).alloted = Latency(pp).alloted + 1;
                                                    Latency(pp).slot_position.slot(ss) = ss;
                                                    Latency(pp).slot_position.subChannel(ss) = oo;

                                                    %PDR Calculation:
                                                    V_RSU_dis(pp,tt) = veh_all_dis(pp,tt);
                                                    packets_txed(pp,tt) = packets_txed(pp,tt)+1;

                                                    % %RSSI
                                                    slot(ss).RSSI(oo) = veh_RSSI(pp);

                                                    if slot(ss).retransmitCounter(oo)>1
                                                        RT_tracker = 1; %Tracks retransmission until retransmitCounter decreases to zero; 1 = continues retransmission; 0 = stops retransmission
                                                        RT_holder = slot(ss).retransmitCounter(oo);
                                                        Latency(pp).RT_Counter = RT_holder;
                                                        % RC_holder = slot(ss).RC(oo);
                                                    else
                                                        RT_tracker = 0;
                                                        rt_flag = 0;
                                                        slot(ss).RT_Completed  (oo) = 1;
                                                        Latency(pp).message_received =  Latency(pp).message_received + 1;
                                                        Latency(pp).total = Latency(pp).total + slot(ss).slot_delay;
                                                        Latency(pp).alloted = 0 ;
                                                        Latency(pp).RRI_delay = 0;
                                                        % Latency(pp).RT_delay = 0;
                                                        Latency(pp).RT_Counter = 1;
                                                        slot(ss).RRI_delay(oo) = 0;
                                                        slot(ss).RRI_ON(oo) = 0;
                                                        for c=1:ss
                                                            for cc=1:RSU_N*subchannel_N
                                                                % for c=1:ss

                                                                if slot(c).subChannel(cc) == slot(ss).subChannel(oo)
                                                                    slot(c).RT_Completed  (cc) = 1;
                                                                end
                                                            end
                                                        end
                                                    end

                                                else %the vehicle is in the "linked" list
                                                    if RT_tracker~=0 %if Vehicle is still retransmitting

                                                        slot(ss).subChannel(oo) = veh_names(pp);
                                                        linked = unique([linked veh_names(pp)],'stable');
                                                        slot(ss).reserved(oo) = 1;
                                                        slot(ss).RT_Completed  (oo) = 0;
                                                        slot(ss).retransmitCounter(oo) = RT_holder-1;
                                                        slot(ss).RC(oo) = RC_holder;
                                                        RT_holder = RT_holder - 1;
                                                        Latency(pp).RT_Counter = RT_holder;
                                                        Latency(pp).vehicle = slot(ss).subChannel(oo);
                                                        Latency(pp).packets_TXed   = Latency(pp).packets_TXed   + 1;
                                                        flag_1st_txed(pp) =  flag_1st_txed(pp) + 1;
                                                        Latency(pp).alloted = Latency(pp).alloted + 1;
                                                        Latency(pp).slot_position.slot = ss;
                                                        Latency(pp).slot_position.subChannel = oo;
                                                        slot_occupied = slot_occupied + 1;

                                                        %PDR Calculation:
                                                        V_RSU_dis(pp,tt) = veh_all_dis(pp,tt);
                                                        packets_txed(pp,tt) = packets_txed(pp,tt)+1;

                                                        % %RSSI
                                                        slot(ss).RSSI(oo) = veh_RSSI(pp);
 
                                                        if RT_holder == 1 %if it is the last retransmission
                                                            RT_tracker = 0;
                                                            rt_flag = 0; %the retransmission is completed
                                                            slot(ss).RT_Completed(oo) = 1;
                                                            Latency(pp).message_received =  Latency(pp).message_received + 1;
                                                            Latency(pp).total = Latency(pp).total + slot(ss).slot_delay;
                                                            Latency(pp).alloted = 0 ;
                                                            Latency(pp).RRI_delay = 0;
                                                            Latency(pp).RT_Counter = 1;
                                                            slot(ss).RRI_delay(oo) = 0;
                                                            slot(ss).RRI_ON(oo) = 0;
                                                            for c=1:ss
                                                                for cc=1:RSU_N*subchannel_N
                                                                    % for c=1:ss

                                                                    if slot(c).subChannel(cc) == slot(ss).subChannel(oo)
                                                                        slot(c).RT_Completed  (cc) = 1;
                                                                    end
                                                                end
                                                            end

                                                            break; %breaks from the current ss-loop (slots)
                                                        end

                                                    end
                                                end
                                            end
                                        else %queue is not empty

                                            if isempty(queue) %To put vehicle(pp) at the end of the queue
                                                queue = ([queue veh_names(pp)]);
                                            else
                                                f_1 = find(queue==veh_names(pp));
                                                if isempty(f_1)
                                                    queue = ([queue veh_names(pp)])
                                                end
                                            end

                                            queue_RT_holder = 1+RT;
                                            queue_RC_holder = randi([RC_Low,RC_High]); %to set RC value for the first vehicle in the queue
                                            queue_rt_flag = 0;
                                            % bb=1;
                                            % while bb<=RSU_N*subchannel_N
                                            % while aa<=slots_N
                                            while slot_occupied ~= slots_N*RSU_N
                                                for aa = 1:slots_N
                                                    for bb = 1:RSU_N*subchannel_N
                                                        fprintf('aa = %d, bb = %d\n',aa,bb);
                                                        if slot(aa).reserved(bb) == 0 && veh_all_per_tt(f_0,tt)

                                                            if slot(aa).retransmitCounter(bb)>1
                                                                RT_tracker = 1; %Tracks retransmission until retransmitCounter decreases to zero; 1 = continues retransmission; 0 = stops retransmission
                                                                slot(aa).subChannel(bb) = veh_names(f_0);
                                                                slot(aa).reserved(bb) = 1;
                                                                slot(aa).RC(bb) = queue_RC_holder; %%%%%--------might need correction RT_holder
                                                                slot_occupied(bb) = slot_occupied(bb) + 1;
                                                                slot(aa).retransmitCounter(bb) = queue_RT_holder;
                                                                Latency(f_0).RT_Counter = queue_RT_holder;
                                                                Latency(f_0).packets_TXed   = Latency(f_0).packets_TXed   + 1;
                                                                flag_1st_txed(f_0) =  flag_1st_txed(f_0) + 1;
                                                                Latency(f_0).alloted = Latency(f_0).alloted + 1;
                                                                linked = unique([linked veh_names(f_0)],'stable');
                                                                V_RSU_dis(f_0,tt) = veh_all_dis(f_0,tt);

                                                                %PDR Calculation:
                                                                V_RSU_dis(f_0,tt) = veh_all_dis(f_0,tt);
                                                                packets_txed(f_0,tt) = packets_txed(f_0,tt)+1;
                                                                %
                                                                % %RSSI
                                                                slot(aa).RSSI(bb) = veh_RSSI(f_0);
                                                                % if veh_RSSI(f_0)>RSSI_th
                                                                %     exceeded_RSSI_th = exceeded_RSSI_th + 1;
                                                                % end

                                                                if isempty(queue)
                                                                    queue = [];
                                                                else
                                                                    queue = queue(queue~=veh_names(f_0));
                                                                end
                                                                if queue_RT_holder == 1 %retransmission completed
                                                                    RT_tracker = 0;
                                                                    queue_rt_flag = 0;
                                                                    slot(aa).RT_Completed  (bb) = 1;
                                                                    Latency(f_0).message_received =  Latency(f_0).message_received + 1;
                                                                    Latency(f_0).total = Latency(f_0).total + slot(aa).slot_delay;
                                                                    % Latency(pp).total = Latency(pp).total + Latency(pp).alloted*slot_delay + Latency(pp).RT_delay +Latency(pp).RRI_delay;
                                                                    Latency(f_0).alloted = 0 ;
                                                                    Latency(f_0).RT_Counter = 1;
                                                                    Latency(f_0).alloted = 0;
                                                                    Latency(f_0).RRI_delay = 0;
                                                                    % Latency(f_0).RT_delay = 0;
                                                                    slot(aa).RRI_delay(bb) = 0;
                                                                    slot(aa).RRI_ON(bb) = 0;
                                                                    for c=1:aa
                                                                        for cc=1:RSU_N*subchannel_N
                                                                            % for c=1:aa

                                                                            if slot(c).subChannel(cc) == slot(aa).subChannel(cc)
                                                                                slot(c).RT_Completed (cc) = 1;
                                                                            end
                                                                        end
                                                                    end
                                                                    % break;

                                                                end
                                                                if queue_RT_holder~=1
                                                                    queue_RT_holder = queue_RT_holder-1 ;
                                                                    Latency(f_0).RT_Counter = queue_RT_holder;
                                                                    % Latency(f_0).alloted = Latency(f_0).alloted + 1;
                                                                end
                                                                if slot_occupied(bb)==slots_N
                                                                    % aa_flag= 1;
                                                                    break;
                                                                end
                                                            end

                                                        end
                                                        % aa = aa + 1;
                                                        if queue_rt_flag==1 %if the retransmission is conpleted, reset the queue_RT_holder for the current RSU
                                                            if isempty(queue)
                                                                queue = [];
                                                                break;
                                                            else
                                                                f_0 = find(veh_names==queue(1));
                                                                queue_RT_holder = 1+RT;
                                                                queue_RC_holder = randi([RC_Low,RC_High]); %to set RC value for the first vehicle in the queue
                                                                queue_rt_flag = 0;
                                                            end
                                                        end

                                                    end
                                                end

                                                % bb = bb+1;
                                            end

                                        end

                                        % ss = ss+1;
                                    else
                                        % ss = ss+1;
                                    end
                                end
                                if ss>=slots_N
                                    break;
                                else
                                    if rt_flag == 0
                                        rt_flag = 1;
                                        break; %breaks from the oo loop (RSU loop)
                                    end
                                    ss = ss+1;
                                end
                            end
                        end
                        if sum(slot_occupied) == slots_N * RSU_N*subchannel_N %if all the slots in all the RSUs are full
                            if ~isempty(linked)
                                vv = find(linked == Latency(pp).vehicle);
                            end
                            if isempty(vv)
                                if isempty(queue)
                                    queue = ([queue veh_names(pp)])
                                else
                                    f_1 = find(queue==veh_names(pp));
                                    if isempty(f_1)
                                        queue = ([queue veh_names(pp)])
                                    end
                                end

                            end
                        end
                    elseif Latency(pp).RT_Counter>1 %------------ executed only if RT>1 -----------------------%
                        for sss = 1:slots_N
                            for ooo = 1:RSU_N*subchannel_N
                                % for sss = 1:slots_N

                                if Latency(pp).vehicle == slot(sss).subChannel(ooo)
                                    vv = find(veh_names == slot(sss).subChannel(ooo));
                                    if Latency(vv).RT_Counter==1
                                        slot(sss).retransmitCounter(ooo) = Latency(vv).RT_Counter;
                                    else
                                        slot(sss).retransmitCounter(ooo) = Latency(vv).RT_Counter -1;
                                        Latency(vv).packets_TXed   = Latency(vv).packets_TXed   + 1;
                                        flag_1st_txed(vv) =  flag_1st_txed(vv) + 1;
                                    end
                                    Latency(vv).RT_Counter = slot(sss).retransmitCounter(ooo);
                                    if Latency(vv).slot_RT_Completed==0
                                        Latency(vv).packets_TXed   = Latency(vv).packets_TXed   + 1;
                                        flag_1st_txed(vv) =  flag_1st_txed(vv) + 1;
                                        Latency(vv).alloted = Latency(vv).alloted + 1;
                                    end

                                    %PDR Calculation:
                                    V_RSU_dis(vv,tt) = veh_all_dis(vv,tt);
                                    packets_txed(vv,tt) = packets_txed(vv,tt)+1;

                                    if slot(sss).retransmitCounter(ooo) == 1 %if it is the last retransmission/retransmission is completed

                                        if Latency(vv).slot_RT_Completed~=1
                                            slot(sss).RT_Completed(ooo) = 1;
                                            Latency(vv).message_received =  Latency(vv).message_received + 1;
                                            Latency(vv).total = Latency(vv).total + slot(sss).slot_delay;
                                            Latency(vv).alloted = 0 ;
                                            Latency(vv).RRI_delay = 0;
                                            % Latency(vv).RT_delay = 0;
                                            Latency(vv).RT_Counter = 1;
                                            slot(sss).RRI_delay(ooo) = 0;
                                            % slot(vv).RRI_ON(oo) = 1;
                                            Latency(vv).slot_RT_Completed = 1;
                                            for c=1:sss
                                                for cc=1:RSU_N*subchannel_N
                                                    % for c=1:sss

                                                    if slot(c).subChannel(cc) == slot(sss).subChannel(ooo)
                                                        slot(c).RT_Completed  (cc) = 1;
                                                    end
                                                end
                                            end
                                        else
                                            for c=1:sss
                                                for cc=1:RSU_N*subchannel_N
                                                    % for c=1:sss

                                                    if slot(c).subChannel(cc) == slot(sss).subChannel(ooo)
                                                        slot(c).RT_Completed  (cc) = 1;
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end

                    end
                end
                if Latency(pp).vehicle == ""
                    Latency(pp).vehicle = veh_names(pp);
                end

            end

            %To reduce the RC by 1, to count the packets TXed_AND_RXed, and to
            %count the packets_dropped

            for sc = 1:numel(slot) % to reduce the RC by 1 at the end of each RRI
                for oo = 1:RSU_N*subchannel_N
                    fprintf('slot = %d, subchannel = %d\n',sc,oo);
                    v = find(veh_names == slot(sc).subChannel(oo)); %Finds the vehicle name that is occupying the current slot.

                    if ~isempty(v)
                        % Calculate BER and SINR:
                        [BER_V(v,tt), SINR_V(v,tt)] = BER_SNR(power_rx_all(v,tt), SNR_distances, Pn_dBm, tt, v, MCS_struct,fc);
                    end

                    %Checking the slot whose RC == 1
                    if slot(sc).RC(oo)==1 && slot(sc).reserved(oo)==1 %empty the slot and reset when the RC is 1 and vehicle will no longer be connected to the RSU

                        if ~isempty(v)
                            Latency(v).RT_Counter = 0;
                            Latency(v).alloted = 0;
                            Latency(v).slot_RT_Completed = 0;
                            flag_1st_txed(v) = 0;
                            if isempty(linked)
                                linked = [];
                            else
                                linked = linked(linked~=Latency(v).vehicle);
                            end
                        end
                        if slot(sc).subChannel(oo) == Latency(v).vehicle
                            slot_occupied = slot_occupied - 1;
                        end
                        slot(sc).subChannel(oo) = "";
                        slot(sc).reserved(oo) = 0;
                        slot(sc).RC(oo) = randi([RC_Low,RC_High]);
                        slot(sc).RT_Completed  (oo) = 0;
                        slot(sc).retransmitCounter(oo) = 1+RT;
                        slot(sc).RRI_ON(oo) = 0;
                        slot(sc).RSSI(oo) = 0;

                    elseif slot(sc).RC(oo)>1 && slot(sc).reserved(oo)==1%The RC of a vehicles is still greater than 0.

                        slot(sc).RC(oo) = slot(sc).RC(oo) - 1;

                        if slot(sc).RSSI(oo)>RSSI_th
                            exceeded_RSSI_th = exceeded_RSSI_th + 1;% Counts the total number of subchannels where RSSI>RSSI_th
                        end



                        if slot(sc).retransmitCounter(oo)>1
                            slot(sc).retransmitCounter(oo) = Latency(v).RT_Counter;
                        end

                        %Drop the packets if PDR is bellow threshold and transmitted messages are blocked due to non line of sight
                        if veh_all_per_tt(v,tt) == 1 %The slot-allocated vehicle is still in line of sight of the RSU
                            if ~isempty(v)
                                % Latency(v).RT_Counter = 0;
                                if flag_1st_txed(v) == 0
                                    Latency(v).packets_TXed = Latency(v).packets_TXed + 1;
                                    packets_txed(v,tt) = packets_txed(v,tt) + 1;
                                    V_RSU_dis(v,tt) = veh_all_dis(v,tt);
                                end
                                if flag_1st_txed(v)>0
                                    flag_1st_txed(v) = flag_1st_txed(v)-1;
                                end
                                % end
                                if BER_V(v,tt)<=BER_threshold || SINR_V(v,tt)<=MCS_struct.min_SINR
                                    Latency(v).packets_TXed_AND_RXed = Latency(v).packets_TXed_AND_RXed + 1;
                                    packets_txed_AND_rxed(v,tt) = packets_txed_AND_rxed(v,tt) + 1;
                                    V_RSU_dis(v,tt) = veh_all_dis(v,tt);
                                else
                                    packets_dropped(v,tt) = packets_dropped(v,tt) + 1;
                                end
                            end
                        elseif veh_all_per_tt(v,tt) == 0
                            Latency(v).packets_dropped = Latency(v).packets_dropped + 1;
                            packets_dropped(v,tt) = packets_dropped(v,tt) + 1;
                            if flag_1st_txed(v) == 0
                                Latency(v).packets_TXed = Latency(v).packets_TXed + 1;
                                packets_txed(v,tt) = packets_txed(v,tt) + 1;
                                V_RSU_dis(v,tt) = veh_all_dis(v,tt);
                            end
                            if flag_1st_txed(v)>0
                                flag_1st_txed(v) = flag_1st_txed(v)-1;
                            end
                        end


                    end

                end
                fprintf('For  subchannel =  %d\n',oo);


            end


            fprintf('ss = %d, oo = %d, tt = %d\n',ss,oo,tt);
            disp(queue);
            queue_update = 0;
            for qq = 1: length(queue)
                qv = find(veh_names == queue(qq));
                if veh_all_per_tt(qv,tt)==0 %if the vehicle is not linked in the current tt, it is removed from the queue
                    queue_new = queue(queue~=Latency(qv).vehicle);
                    queue_update = 1;
                else
                    %if the vehicle does not get a slot although in the line-of-sight, (RT+1) packets are dropped
                    Latency(qv).packets_dropped = Latency(qv).packets_dropped + (RT+1);
                end
            end
            if queue_update==1
                queue = queue_new;
            end

            % Calculating Channel Busy Rate/Ratio (CBR) after every 100 msec
            % if rem(tt,10) == 0
            CBR = exceeded_RSSI_th/(slots_N*subchannel_N);
            CBR_all = [CBR_all CBR] %100 = subframes per RRI
            exceeded_RSSI_th = 0;
            % end

            % Calculating Channel Occupancy Rate/Ratio (C_R) afte reery 10*100 =
            % 1000 msec:
            for pp = 1:length(Latency)
                if Latency(pp).RT_Counter>0
                    Latency(pp).subChannel_Occupied = Latency(pp).subChannel_Occupied + 2;
                end

                if rem(tt,10) == 0
                    C_R(pp) = Latency(pp).subChannel_Occupied/(100*2*10); %100 subframes, 2 subchannels, 10 tt
                    C_R_all = [C_R_all C_R];
                    Latency(pp).subChannel_Occupied = 0;

                    if CCM == 1
                        %Congestion Control Mechanism:
                        %If C_R is above the predefiend CR_limits, drop the transmitted and retransmitted packets for that vehicle
                        % for PPPP = 3 to 5,
                        for sc = 1:numel(slot)
                            for oo = 1:RSU_N*subchannel_N
                                % for sc = 1:numel(slot)
                                if Latency(pp).vehicle == slot(sc).subChannel(oo)
                                    if  (CBR<=0.65 && C_R(pp)>=(0.03))
                                        %do not consider the packets as rxed/re-rxed
                                        Latency(pp).packets_TXed_AND_RXed = Latency(pp).packets_TXed_AND_RXed - 1;
                                        packets_txed_AND_rxed(pp,tt) = packets_txed_AND_rxed(pp,tt) - 1;

                                        %Drop the tx and re-tx packets
                                        Latency(pp).packets_dropped = Latency(pp).packets_dropped + 1;
                                        packets_dropped(pp,tt) = packets_dropped(pp,tt) + 1;
                                    elseif (CBR>0.65 && C_R(pp)>=(0.006)) && (CBR<=0.8 && C_R(pp)>=(0.006))
                                        %do not consider the packets as rxed/re-rxed
                                        Latency(pp).packets_TXed_AND_RXed = Latency(pp).packets_TXed_AND_RXed - 1;
                                        packets_txed_AND_rxed(pp,tt) = packets_txed_AND_rxed(pp,tt) - 1;

                                        %Drop the tx and re-tx packets
                                        Latency(pp).packets_dropped = Latency(pp).packets_dropped + 1;
                                        packets_dropped(pp,tt) = packets_dropped(pp,tt) + 1;
                                    elseif (CBR>=0.8 && C_R(pp)>=(0.003))
                                        %do not consider the packets as rxed/re-rxed
                                        Latency(pp).packets_TXed_AND_RXed = Latency(pp).packets_TXed_AND_RXed - 1;
                                        packets_txed_AND_rxed(pp,tt) = packets_txed_AND_rxed(pp,tt) - 1;

                                        %Drop the tx and re-tx packets
                                        Latency(pp).packets_dropped = Latency(pp).packets_dropped + 1;
                                        packets_dropped(pp,tt) = packets_dropped(pp,tt) + 1;
                                    end
                                end
                            end
                        end
                        %Reset the C_R value
                        C_R = zeros(1,floor(tMax/10));
                    end
                end

            end




        end

        Latency_linked= [];
        PDR_linked = [];
        %total latency is the sum of RRI_delay and slot_delay
        for ll = 1:numel(Latency)
            Latency_linked = [Latency_linked Latency(ll).total/Latency(ll).message_received];
            Latency(ll).PDR = (Latency(ll).packets_TXed_AND_RXed)/(Latency(ll).packets_TXed+Latency(ll).packets_dropped);
            PDR_linked = [PDR_linked Latency(ll).PDR]
        end


        PDR_data(i_iterate,:,a) = PDR_linked;
        Latency_data(i_iterate,:,a) = Latency_linked;
        PDR_data_avg = [PDR_data_avg; PDR_data(i_iterate,:,a)];
        Latency_data_avg = [Latency_data_avg; Latency_data(i_iterate,:,a)];

        %
        if iterate == 1
            avg_PDR = PDR_data;
            avg_Latency = Latency_data;
        else
            avg_PDR = mean(PDR_data);
            avg_Latency = mean(Latency_data);
        end
        mean_PDR = mean(avg_PDR(avg_PDR>0));



        %%%% Distance vs PDR calculation: only for the distances at which vehicles were alloted slots
        %%11_18_2023

        PDR_fr_dist = [];
        ll = length(veh_RSU_dist);
        TX_pkt = zeros(ll,tMax);
        TX_AND_RX_pkt = zeros(ll,tMax);
        dropped_pkt = zeros(ll,tMax);
        for t = 1:tMax
            for p = 1: (length(veh_names))
                if V_RSU_dis(p,t)>0
                    for q = 1:ll
                        r = veh_RSU_dist(q);
                        if r==0
                            % pkt_TX = 0;
                            % PDR_fr_dist(q,t) = 0;
                            continue;
                        end
                        if ((r-PDR_range)<=V_RSU_dis(p,t)) && (V_RSU_dis(p,t)<r)
                            TX_pkt(q,t) = TX_pkt(q,t) + packets_txed(p,t);
                            TX_AND_RX_pkt(q,t) = TX_AND_RX_pkt(q,t) + packets_txed_AND_rxed(p,t);
                            dropped_pkt(q,t) = dropped_pkt(q,t) + packets_dropped(p,t);
                        end
                    end

                end
            end
        end
        PDR_fr_dist = sum(TX_AND_RX_pkt,2)./(sum(TX_pkt,2)+(sum(dropped_pkt,2)));
        PDR_fr_dist_all(a,:) = PDR_fr_dist';
    end%------------------ %end of i_PDR

    %% Averaging the PDR_fr_dist for "a" iterations:
    [ra,ca] = size(PDR_fr_dist_all);

    for c = 1:ca
        fr_each_a = PDR_fr_dist_all(:,c);
        PDR_fr_dist_avg(c) = mean(fr_each_a(~isnan(fr_each_a)));
    end

    [r_PDR,c_PDR] = size(veh_names);
    for c = 1:c_PDR
        PDR_fr_each_a = PDR_data_avg(:,c);
        PDR_avg(c) = mean(PDR_fr_each_a (~isnan(PDR_fr_each_a )));
        Latency_fr_each_a = Latency_data_avg(:,c);
        Latency_avg(c) = mean(Latency_fr_each_a (~isnan(Latency_fr_each_a )));
    end

    % %Compounding PDR vs distance:
    % if length(MCS_set) ==2
    %     figure(21);
    %     dis_PDR = 0;
    %     sum_PDR = 0;
    %     for d=2:length(veh_RSU_dist)
    %         if isnan(PDR_fr_dist_all(a,d))
    %             PDR_fr_dist_all(a,d) = 1;
    %         end
    %         dis_PDR(d) = mean(PDR_fr_dist_all(a,2:d))
    %     end
    %     if MCS_struct.Modulation == "QPSK"
    %         plot(veh_RSU_dist(2:end),dis_PDR(2:end),'LineWidth',2);
    %     elseif MCS_struct.Modulation == "QAM"
    %         plot(veh_RSU_dist(2:end),dis_PDR(2:end),'--','LineWidth',2);
    %     end
    %     ylim([0 1])
    %     xlim([min(veh_RSU_dist) max(veh_RSU_dist)])
    %     yticks(0:0.1:1);
    % 
    %     grid on;
    %     hold on;
    % end




    % %% BER vs SINR Plots after packets dropped
    % %To sort the SINR_V and BER_V in a single row with ascending order
    % a = SINR_V;
    % b = BER_V;
    % x = [];
    % x_0 = 0;
    % y_0 = 0;
    % y = [];
    % [i1,j1] = size(SINR_V)
    % figure(20);
    % for i = 1:i1
    %     for j = 1:j1
    %         x_0 = a(i,j);
    %         x = [x x_0];
    %         y_0 = b(i,j);
    %         y = [y y_0];
    %         x_0 = 0; x_y = 0;
    %         [~,x_i] = sort(x);
    %         y_sorted = y(x_i);
    %         x_sorted = x(x_i);
    % 
    %         % plot(x_sorted(i,j),y_sorted(i,j),'-*');
    %         % hold on;
    %     end
    % end
    % [m,n] = unique(x_sorted,'stable');
    % grid on;
    % xlabel('SINR');
    % ylabel('BER');
    % x = x_sorted(n);
    % y = y_sorted(n);
    % plot(x(x~=0),y(y~=0),'-*');
    % disp(a); disp(b);
    % disp(x);
    % disp(y);
    % disp(x_sorted);
    % disp(y_sorted);
    % hold on;
    % % set(gca, 'YScale', 'log')


    %% Average of PDR, Latency, PDR_vs_distance plots
    % figure();
    % subplot(2,2,1);
    % % set(gcf, 'Position', [680,80,650,500]);
    % Latency_all_msec = avg_Latency;
    % Latency_all_nonzero = Latency_all_msec(Latency_all_msec>0&Latency_all_msec~=Inf);
    % HistLatency = histogram(Latency_all_nonzero,'Normalization','probability'); %Message Delivery Rates of all vehicles
    % HistLatency.BinWidth = 5;
    % xlim([0, 100]);
    % ylim([0 1])
    % xlabel('x (msec)');
    % ylabel('f_{x}(x)');
    % title("Distribution of Latency (#vehicles = "+ lambda*6+")"+" #RT = "+RT);
    % pbaspect([2 1 1])
    % grid on;
    % 
    % %
    figure();
    % subplot(2,2,2);
    % set(gcf, 'Position', [10,80,650,500]);
    HistMDR = histogram((PDR_avg(PDR_avg>0&PDR_avg~=Inf)),'Normalization','probability','FaceColor','red');
    HistMDR.BinWidth = 0.05;
    % end
    xlim([0 1]);
    ylim([0 1])
    xlabel('avg PDR');
    ylabel('CDF');
    title("avg PDR vs vehicle frequency (#vehicles = "+ lambda*6+")"+" #RT = "+RT+" MCS = "+ MCS);
    grid on;
    pbaspect([2 1 1])
    % 
    % % figure(6); %PDR vs Distance plots
    % subplot(2,2,3);
    % x = veh_RSU_dist((PDR_fr_dist_avg>0));%&&(~isnan(PDR_fr_dist_avg)));
    % p = plot(veh_RSU_dist(PDR_fr_dist_avg>0),PDR_fr_dist_avg(PDR_fr_dist_avg>0),'-*','Color','magenta','LineWidth',1);
    % xlabel('distance from the RSU (meters)');
    % ylabel('PDR');
    % title(" (avg PDR Vs Distance from the RSU) (#vehicles = "+ lambda*6+")"+" #RT = "+RT+" subchannel = "+subchannel_N);
    % x_tick = x(1):10:RSU_range;
    % xticks(x_tick);
    % ylim([0 1])
    % grid on;
    % % p(2).Marker = '*';
    % pbaspect([2 1 1])
    % 
    % % figure(7);
    % subplot(2,2,4);
    % % set(gcf, 'Position', [10,80,650,500]);
    % HistMDR = histogram((PDR_fr_dist_avg(PDR_fr_dist_avg>0&PDR_fr_dist_avg~=Inf)),'Normalization','probability','FaceColor','magenta');
    % % if length(PDR_fr_dist_avg(PDR_fr_dist_avg>0&PDR_fr_dist_avg~=Inf))<15
    % HistMDR.BinWidth = 0.05;% 1/length(PDR_fr_dist_avg(PDR_fr_dist_avg>0&PDR_fr_dist_avg~=Inf));
    % % end
    % xlim([0 1]);
    % ylim([0 1])
    % xlabel('PDR');
    % ylabel('CDF');
    % title("avg PDR (#vehicles = "+ lambda*6+")"+" #RT = "+RT+"  CCM ="+CCM+" (1=ON, 0=OFF)");
    % grid on;
    % pbaspect([2 1 1]);
    % PDR_below_90 = [PDR_below_90 length(PDR_fr_dist_avg(PDR_fr_dist_avg>0.95))];
    % PDR_th = PDR_fr_dist_avg(~isnan(PDR_fr_dist_avg));
    % PDR_th = PDR_th(PDR_th>0);
    % PDR_90 = length(PDR_th(PDR_th>=0.9))/length(PDR_th);
    % xline(0.9,'g','linewidth',2);
    % text(0.64, 0.7, sprintf('(PDR > 0.9) = %2.4f',PDR_90))
    % % fprintf('Number of instances witnessing PDR below 95 percent = %d, MCS = %d\n',length(PDR_fr_dist_avg(PDR_fr_dist_avg<0.95)),MCS);

    %% Individual plots:

    figure();
    % subplot(2,2,1);
    % set(gcf, 'Position', [680,80,650,500]);
    Latency_all_msec = avg_Latency;
    Latency_all_nonzero = Latency_all_msec(Latency_all_msec>0&Latency_all_msec~=Inf);
    Latency_20 = length(Latency_all_nonzero(Latency_all_nonzero<=20))/length(Latency_all_nonzero);
    HistLatency = histogram(Latency_all_nonzero,'Normalization','probability','FaceColor','b'); %Message Delivery Rates of all vehicles
    HistLatency.BinWidth = 5;
    xlim([0, 60]);
    ylim([0 1])
    xlabel('x (msec)');
    ylabel('f_{x}(x)');
    line([20 20],[0 1],'Color','red','LineWidth',2);
    text(21, 0.85, sprintf('(Latency <= 20 msec) = %2.2f %%',Latency_20*100))
    title("Distribution of Latency (#vehicles = "+ lambda*6 +")"+" MCS = "+ MCS);
    pbaspect([2 2 2])
    xticks(0:5:60)
    grid on;

    %
    % figure(); %PDR vs Distance plots
    % % subplot(2,2,3);
    if MCS == 0
        x_mcs_0 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_0 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 1
        x_mcs_1 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_1 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 2
        x_mcs_2 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_2 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 3
        x_mcs_3 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_3 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 4
        x_mcs_4 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_4 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 5
        x_mcs_5 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_5 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 6
        x_mcs_6 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_6 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 7
        x_mcs_7 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_7 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 8
        x_mcs_8 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_8 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 9
        x_mcs_9 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_9 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 10
        x_mcs_10 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_10= [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 11
        x_mcs_11 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_11 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 12
        x_mcs_12 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_12 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 13
        x_mcs_13 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_13= [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 14
        x_mcs_14 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_14 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 15
        x_mcs_15 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_15 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 16
        x_mcs_16 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_16 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 17
        x_mcs_17 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_17 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 18
        x_mcs_18 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_18= [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 19
        x_mcs_19 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_19 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    elseif MCS == 20
        x_mcs_20 = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_20 = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    end

        x_mcs_individual = [0 veh_RSU_dist((PDR_fr_dist_avg>0))];%&&(~isnan(PDR_fr_dist_avg)));
        y_mcs_individual = [1 PDR_fr_dist_avg(PDR_fr_dist_avg>0)];
    % 
    % 
    figure();
    HistMDR = histogram((PDR_fr_dist_avg(PDR_fr_dist_avg>0&PDR_fr_dist_avg~=Inf)),'Normalization','probability','FaceColor','magenta');
    HistMDR.BinWidth = 0.05;% 1/length(PDR_fr_dist_avg(PDR_fr_dist_avg>0&PDR_fr_dist_avg~=Inf));
    % end
    xlim([0 1]);
    ylim([0 1])
    xticks([0:0.1:1])
    ylabel('CDF');
    xlabel('avg PDR wrt distance');
    title("avg PDR (#vehicles = "+ lambda*6+")"+" MCS = "+ MCS);
    grid on;
    pbaspect([2 2 2])
    % PDR_below_90 = [PDR_below_90 length(PDR_fr_dist_avg(PDR_fr_dist_avg>0.95))];
    PDR_th = PDR_fr_dist_avg(~isnan(PDR_fr_dist_avg));
    PDR_th = PDR_th(PDR_th>0);
    PDR_90 = length(PDR_th(PDR_th>=0.9))/length(PDR_th);
    xline(0.9,'g','linewidth',2);
    text(0.5, 0.9, sprintf('(PDR > 0.9) = %2.2f %%',PDR_90*100))

    fprintf('Number of instances witnessing PDR below 95 percent = %d, MCS = %d\n',length(PDR_fr_dist_avg(PDR_fr_dist_avg<0.95)),MCS);
    hold on;

end

figure(); %PDR vs Distance plots
if length(MCS_set) == 1

    plot(x_mcs_individual,y_mcs_individual,'-','Color','magenta','LineWidth',1); %MCS 7
    legend(sprintfc('MCS = %d',MCS_set),'Location','southwest');
elseif length(MCS_set)==2
    if MCS_set(1) == 7 && MCS_set(2) == 11
        plot(x_mcs_7,y_mcs_7,'-','Color','green','LineWidth',2); %MCS 7
        hold on;
        plot(x_mcs_11,y_mcs_11,'-','Color','blue','LineWidth',1); %MCS 11
        legend('MCS = 7', 'MCS = 11','Location','southwest');
    end
elseif length(MCS_set)==3
    if   MCS_set(1) == 7 && MCS_set(2) == 9 && MCS_set(3) == 11
        plot(x_mcs_7,y_mcs_7,'-','Color','magenta','LineWidth',1); %MCS 7
        hold on;
        plot(x_mcs_9,y_mcs_9,'-','Color','green','LineWidth',2); %MCS 9
        plot(x_mcs_11,y_mcs_11,'-','Color','blue','LineWidth',1); %MCS 11
        legend('MCS = 7', 'MCS = 9', 'MCS = 11','Location','southwest');
    end
else
    plot(x_mcs_2,y_mcs_2,'-','Color','green','LineWidth',1); %MCS 2
    hold on;
    plot(x_mcs_7,y_mcs_7,'-','Color','magenta','LineWidth',1); %MCS 7
    % plot(x_mcs_9,y_mcs_9,'-','Color','green','LineWidth',2); %MCS 9
    plot(x_mcs_11,y_mcs_11,'-','Color','blue','LineWidth',1); %MCS 11
    plot(x_mcs_15,y_mcs_15,'-','Color','red','LineWidth',1); %MCS 15
    legend('MCS = 2', 'MCS = 7', 'MCS = 11', 'MCS = 15','Location','southwest');

end

xlabel('distance from the RSU (meters)');
ylabel('PDR');
title(" (avg PDR Vs Distance from the RSU) (#vehicles = "+ lambda*6+")");
x_tick = 0:10:150;
xticks(x_tick);
ylim([0 1])
grid on;
% legend('MCS = 7', 'MCS = 9', 'MCS = 11','Location','southwest');
% p(2).Marker = '*';
pbaspect([2 1 1])

% figure(20);
% % legend('MCS = 2', 'MCS = 7', 'MCS = 11', 'MCS = 15','Location','northeast');
% legend('MCS = 7', 'MCS = 11','Location','northeast');
% 
% if length(MCS_set)==2
%     figure(21);
%     % legend('MCS = 2', 'MCS = 7', 'MCS = 11', 'MCS = 15','Location','southwest');
%     legend('MCS = 7', 'MCS = 11','Location','southwest');
%     x_tick = 0:10:150;
%     xticks(x_tick);
%     ylabel('PDR');
%     xlabel('distance (m)');
% end
% 




%% Functions to determine if there is intersection between the lines
function out = lineSegmentIntersect(XY1,XY2)

%LINESEGMENTINTERSECT Intersections of line segments.
%   OUT = LINESEGMENTINTERSECT(XY1,XY2) finds the 2D Cartesian Coordinates of
%   intersection points between the set of line segments given in XY1 and XY2.
%
%   XY1 and XY2 are N1    x4 and N2x4 matrices. Rows correspond to line segments.
%   Each row is of the form [x1 y1 x2 y2] where (x1,y1) is the start point and
%   (x2,y2) is the end point of a line segment:
%
%                  Line Segment
%       o--------------------------------o
%       ^                                ^
%    (x1,y1)                          (x2,y2)
%
%   OUT is a structure with fields:
%
%   'intAdjacencyMatrix' : N1xN2 indicator matrix where the entry (i,j) is 1 if
%       line segments XY1(i,:) and XY2(j,:) intersect.
%
%   'intMatrixX' : N1xN2 matrix where the entry (i,j) is the X coordinate of the
%       intersection point between line segments XY1(i,:) and XY2(j,:).
%
%   'intMatrixY' : N1xN2 matrix where the entry (i,j) is the Y coordinate of the
%       intersection point between line segments XY1(i,:) and XY2(j,:).
%
%   'intNormalizedDistance1To2' : N1xN2 matrix where the (i,j) entry is the
%       normalized distance from the start point of line segment XY1(i,:) to the
%       intersection point with XY2(j,:).
%
%   'intNormalizedDistance2To1' : N1xN2 matrix where the (i,j) entry is the
%       normalized distance from the start point of line segment XY1(j,:) to the
%       intersection point with XY2(i,:).
%
%   'parAdjacencyMatrix' : N1xN2 indicator matrix where the (i,j) entry is 1 if
%       line segments XY1(i,:) and XY2(j,:) are parallel.
%
%   'coincAdjacencyMatrix' : N1xN2 indicator matrix where the (i,j) entry is 1
%       if line segments XY1(i,:) and XY2(j,:) are coincident.
% Version: 1.00, April 03, 2010
% Version: 1.10, April 10, 2010
% Author:  U. Murat Erdem
% CHANGELOG:
%
% Ver. 1.00:
%   -Initial release.
%
% Ver. 1.10:
%   - Changed the input parameters. Now the function accepts two sets of line
%   segments. The intersection analysis is done between these sets and not in
%   the same set.
%   - Changed and added fields of the output. Now the analysis provides more
%   information about the intersections and line segments.
%   - Performance tweaks.
% I opted not to call this 'curve intersect' because it would be misleading
% unless you accept that curves are pairwise linear constructs.
% I tried to put emphasis on speed by vectorizing the code as much as possible.
% There should still be enough room to optimize the code but I left those out
% for the sake of clarity.
% The math behind is given in:
%   http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
% If you really are interested in squeezing as much horse power as possible out
% of this code I would advise to remove the argument checks and tweak the
% creation of the OUT a little bit.

%%% Argument check.
%-------------------------------------------------------------------------------
validateattributes(XY1,{'numeric'},{'2d','finite'});
validateattributes(XY2,{'numeric'},{'2d','finite'});
[n_rows_1,n_cols_1] = size(XY1);
[n_rows_2,n_cols_2] = size(XY2);
if n_cols_1 ~= 4 || n_cols_2 ~= 4
    error('Arguments must be a Nx4 matrices.');
end

%%% Prepare matrices for vectorized computation of line intersection points.
%-------------------------------------------------------------------------------
X1 = repmat(XY1(:,1),1,n_rows_2);
X2 = repmat(XY1(:,3),1,n_rows_2);
Y1 = repmat(XY1(:,2),1,n_rows_2);
Y2 = repmat(XY1(:,4),1,n_rows_2);
XY2 = XY2';
X3 = repmat(XY2(1,:),n_rows_1,1);
X4 = repmat(XY2(3,:),n_rows_1,1);
Y3 = repmat(XY2(2,:),n_rows_1,1);
Y4 = repmat(XY2(4,:),n_rows_1,1);
X4_X3 = (X4-X3);
Y1_Y3 = (Y1-Y3);
Y4_Y3 = (Y4-Y3);
X1_X3 = (X1-X3);
X2_X1 = (X2-X1);
Y2_Y1 = (Y2-Y1);
numerator_a = X4_X3 .* Y1_Y3 - Y4_Y3 .* X1_X3;
numerator_b = X2_X1 .* Y1_Y3 - Y2_Y1 .* X1_X3;

if Y4_Y3==0 && Y2_Y1==0 || X2_X1==0 && X4_X3==0 || Y4_Y3==0 && X4_X3==0

    % Output
    out.intAdjacencyMatrix = 0;
    out.intMatrixX = 0;
    out.intMatrixY = 0;
    out.intNormalizedDistance1To2 = 0;  % May not be correct. Not used for now.
    out.intNormalizedDistance2To1 = 0;  % May not be correct. Not used for now.
    out.parAdjacencyMatrix = 0;         % May not be correct. Not used for now.
    out.coincAdjacencyMatrix= 0;        % May not be correct. Not used for now.

else
    denominator = Y4_Y3 .* X2_X1 - X4_X3 .* Y2_Y1;
    u_a = numerator_a ./ denominator;
    u_b = numerator_b ./ denominator;

    % Find the adjacency matrix A of intersecting lines.
    INT_X = X1+X2_X1.*u_a;
    INT_Y = Y1+Y2_Y1.*u_a;
    INT_B = (u_a >= 0) & (u_a <= 1) & (u_b >= 0) & (u_b <= 1);
    PAR_B = denominator == 0;
    COINC_B = (numerator_a == 0 & numerator_b == 0 & PAR_B);

    % Output
    out.intAdjacencyMatrix = INT_B;
    out.intMatrixX = INT_X .* INT_B;
    out.intMatrixY = INT_Y .* INT_B;
    out.intNormalizedDistance1To2 = u_a;
    out.intNormalizedDistance2To1 = u_b;
    out.parAdjacencyMatrix = PAR_B;
    out.coincAdjacencyMatrix= COINC_B;
end

end

%%  Function to get the coordinates of building lines:

%Function BldLines(C1,C2) gives the four coordinates point of a each line of a building
%(C1,C2) is the starting vertex(Bottom Left Vertex) of a rectangle.

function out = BldLinesCo(C1, C2, R1, R2)
%v1,v2,v3,v4 are the vertices of a rectangle
v1 = [C1 C2];
v2 = [C1 C2+R2];
v3 = [C1+R1 C2+R2];
v4 = [C1+R1 C2];
out = [ [v1 v2]; [v2 v3]; [v3 v4]; [v4 v1]];
end

%%  D: Function to plot the buildings and the circle(range of RSU)
function rectanglePlot(laneWidth,southWall,R,bldgFaceColor,RSU_N)

rectangle('Position', [2*laneWidth southWall R R], 'FaceColor', bldgFaceColor);
rectangle('Position', [-2*laneWidth-R southWall R R], 'FaceColor', bldgFaceColor);
rectangle('Position', [2*laneWidth 2*laneWidth R R], 'FaceColor', bldgFaceColor);
rectangle('Position', [-2*laneWidth-R 2*laneWidth R R], 'FaceColor', bldgFaceColor);
rectangle('Position', [2*laneWidth 6*laneWidth+R R R], 'FaceColor', bldgFaceColor);
rectangle('Position', [-2*laneWidth-R 6*laneWidth+R R R], 'FaceColor', bldgFaceColor);


if RSU_N == 1
    % Circle of diameter 300m and RSU plotting
    %RSU: Green Rectangle
    pos = [(-2*laneWidth-0.8*laneWidth) (2*laneWidth+R-0.8*laneWidth) (1.6*laneWidth) (1.6*laneWidth)];
    rectangle('Position', pos, 'FaceColor', 'g');

    ax = gca;
    ax.Clipping = 'off';
    centers = [-2*laneWidth, 2*laneWidth+R];
    radii = 150;
    viscircles(centers, radii, 'Color','k');

elseif RSU_N == 2

    %RSU1: Green Rectangle
    pos = [(-2*laneWidth-0.8*laneWidth) (2*laneWidth+R-0.8*laneWidth) (1.6*laneWidth) (1.6*laneWidth)];
    rectangle('Position', pos, 'FaceColor', 'g');

    %RSU2: Green Rectangle
    pos = [(-2*laneWidth-0.8*laneWidth) (-2*laneWidth-0.8*laneWidth) (1.6*laneWidth) (1.6*laneWidth)];
    rectangle('Position', pos, 'FaceColor', 'g');

    %RSU1: Circle d = 300m
    ax = gca;
    ax.Clipping = 'off';
    centers = [-2*laneWidth, 2*laneWidth+R];
    radii = 150;
    viscircles(centers, radii, 'Color','k');

    %RSU12: Circle d = 300m
    ax = gca;
    ax.Clipping = 'off';
    centers = [-2*laneWidth, -2*laneWidth];
    radii = 150;
    viscircles(centers, radii, 'Color','k');

elseif RSU_N ==3
    %RSU1: Green Rectangle
    pos = [(-2*laneWidth-0.8*laneWidth) (2*laneWidth+R-0.8*laneWidth) (1.6*laneWidth) (1.6*laneWidth)];
    rectangle('Position', pos, 'FaceColor', 'g');

    %RSU2: Green Rectangle
    pos = [(-2*laneWidth-0.8*laneWidth) (-2*laneWidth-0.8*laneWidth) (1.6*laneWidth) (1.6*laneWidth)];
    rectangle('Position', pos, 'FaceColor', 'g');

    %RSU2: Green Rectangle
    pos = [(-2*laneWidth-0.8*laneWidth) (6*laneWidth+2*R-0.8*laneWidth) (1.6*laneWidth) (1.6*laneWidth)];
    rectangle('Position', pos, 'FaceColor', 'g');

    %RSU1: Circle d = 300m
    ax = gca;
    ax.Clipping = 'off';
    centers = [-2*laneWidth, 2*laneWidth+R];
    radii = 150;
    viscircles(centers, radii, 'Color','k');

    %RSU2: Circle d = 300m
    ax = gca;
    ax.Clipping = 'off';
    centers = [-2*laneWidth, -2*laneWidth];
    radii = 150;
    viscircles(centers, radii, 'Color','k');

    %RSU3: Circle d = 300m
    ax = gca;
    ax.Clipping = 'off';
    centers = [-2*laneWidth, 6*laneWidth+2*R];
    radii = 150;
    viscircles(centers, radii, 'Color','k');

end
end


%%  ****************************************************************************************%
%Implementation of Urban Micro Street Canyon mdoel (UMi-Street Canyon) to calculate Pathloss
%All distances and heights are in meter %Returns Pathloss value in dBm
%Central frequency(fc) is normalized to 1GHz (for 2 junction modal: fc is 5.9)
%Dis_2D is the distance between the base of the RSU and the base of the vehicle
%Dis_3D is the actual distance between the antenna tip of RSU and vehicle's antenna tip
%Written by: Dhruba Sunuwar/Department of Electrical and Computer Engineering
%Georgia Southern University
%*******************************************************************************************%

function [Pathloss]  = Pathloss_UMi(Dis_2D, fc)
dis_LOS_prob =18; %distaance within which the probability of LOS is always 1
h_RSU = 10; %RSU antenna height - meter;
c = 3e8; %propagation velocity in free space - m/s
% c= 20e-3;
% Dis_BP_all = [];
%to generate random vehicle height: normal vehicles on the road have
%heights ranging from 1.5 m to 1.7m
height_random = 1.5:0.05:1.7;
index_random = randperm(numel(height_random),1); %to get random index of element of height_random
h_veh = height_random(index_random); %to get random vehicle height

%effective heights:
h_E = 1; %Effective environment height for UMi is 1m
h_veh_E = h_veh - h_E; %effective vehicle antenna height from the ground
h_RSU_E = h_RSU - h_E; %effective RSU antenna height from the ground

%Calculation of Dis_3D:
Dis_3D = sqrt((h_RSU-h_veh)^2 + Dis_2D^2);
Dis_BP = 4 * (h_RSU_E) * (h_veh_E) * (fc/c); %Breakpoint distance
% Dis_BP_all = [Dis_BP_all Dis_BP];

%% Trial Section:
%UMi-Street Canyon (Line-of-sight(LOS))
if Dis_2D < 10
    Pathloss_LOS = 0;
elseif (10 <= Dis_2D) && (Dis_2D <= Dis_BP)
    Pathloss_LOS = 32.4 + 21*log10(Dis_3D) + 20*log10(fc);
elseif (Dis_BP <= Dis_2D) && (Dis_2D <= 5000)
    Pathloss_LOS = (32.4 + 40*log10(Dis_3D) + 20*log10(fc) - 9.5*log10((Dis_BP)^2 + (h_RSU - h_veh)^2));
end

%UMi-Street Canyon (Non-Line-of-sight(NLOS))
Pathloss_NLOS_prime = 35.3*log10(Dis_3D) + 22.4 + 21.3*log10(fc)- 0.3*(h_veh-1.5);
Pathloss_NLOS = max(Pathloss_LOS, Pathloss_NLOS_prime); %Pathloss value returned by the function for Dis_2D>10

%LOS Probability:
if Dis_2D < 10
    Probability_LOS = 1;
    Pathloss = 0; %in dBm    %Pathloss = 0, because Pathloss_Los=0 for Dis_2D<10
elseif (10<=Dis_2D) && (Dis_2D<=dis_LOS_prob)
    Probability_LOS = 1;
    Pathloss = Probability_LOS*Pathloss_LOS;%in dB
elseif dis_LOS_prob < Dis_2D
    Probability_LOS = (dis_LOS_prob/Dis_2D) + exp(-Dis_2D/36)*(1-(dis_LOS_prob/Dis_2D));
    %     Pathloss = (1-Probability_LOS)*Pathloss_NLOS-30; %in dBm
    % Pathloss = (Probability_LOS)*(Pathloss_LOS)  + (1-Probability_LOS)*(Pathloss_NLOS); %in dB
    Pathloss = pow2db((db2pow(Pathloss_LOS)*Probability_LOS)+(db2pow(Pathloss_NLOS)*(1-Probability_LOS)));
end
% fprintf('Probability of LOS is: %1.4f\n', Probability_LOS);
% % disp(Dis_BP_all);
end

%% Function to calculate BER and SINR:

function [BER, SINR] = BER_SNR(Pr_V, SNR_distances, N, tt_V, pp_V, MCS, fc)
%Pr_V is the received power of the vehicle in concern, N is the noise
%power, (tt,V, pp_V) is the time slot and vehicle in concern
payload = 300; %size of BSM
msgLen = payload*8+32; %BSM size: 300 bytes = 2400bits + 32 bits (CRC)

BSM_tx = randi([0,1],msgLen, 1);

M = MCS.M; %Modulation order

% First step: Calculate Interference at a vehicle due to the presence of other vehicles within the range of the RSU:
sum_I = 0;
I = 0;
[vehicles,~] = size(SNR_distances);
x1 = real(SNR_distances(pp_V,tt_V));
y1 = imag(SNR_distances(pp_V,tt_V));
for jj = 1:vehicles
    if SNR_distances(jj,tt_V)~=0
        x2 = real(SNR_distances(jj,tt_V));
        y2 = imag(SNR_distances(jj,tt_V));
        Dis_2D = sqrt((x2-x1)^2+(y2-y1)^2);
        PL = Pathloss_UMi(Dis_2D, fc);
        I = (23-30) - PL; %in dB
        sum_I = sum_I + I;
    end
end

% Second step: Calculate SINR:
I_N = pow2db(db2pow(sum_I) + db2pow(N)); %Interference + noise in dB
SINR = Pr_V - I_N; %dB

% Third step: Encoding, Modulating, transmission, De-modulating, and Decoding the signal:
%  %length of turbo-coded message
cdLen = msgLen/MCS.CR;

%Turbo Encoding
encoded = lteTurboEncode(BSM_tx);
encoded_rm = lteRateMatchTurbo(encoded, cdLen, 0);
encoded_rx = lteRateRecoverTurbo(encoded_rm, msgLen-24, 0); % Subtract the number of CRC bits

%Modulation:

%Modulation
if MCS.Modulation == "QPSK"
    txSig = pskmod(double(encoded_rx{1,1}),M,pi/4);
elseif MCS.Modulation == "QAM"
    txSig = qammod(double(encoded_rx{1,1}),M,'InputType','bit');
end

%Pass the signal through an AWGN channel.
rxSig = awgn(txSig,SINR);

%Demodulation
if  MCS.Modulation == "QPSK"
    demod_BSM = pskdemod(rxSig,M,pi/4);
elseif MCS.Modulation == "QAM"
    demod_BSM = qamdemod(rxSig,M,'OutputType','bit');
end

%Turbo Decoding
BSM_rx = double(lteTurboDecode(int8(demod_BSM)));
% size(decoded{1}) % This provides the length as input passed

%Bit Error:
numErrs = symerr(BSM_tx,BSM_rx);

%Bit Error Rate:

BER = numErrs/msgLen;
if MCS.Modulation == "QPSK"
    if BER == 0
        BER = 1e-3;
    end
elseif MCS.Modulation == "QAM"
    if BER == 0
        BER = 24e-3;
    end
end


sprintf('MCS = %s, Modulation = %s \n',num2str(MCS.Index),MCS.Modulation)


%% Function to calculate Pathloss:
    function [Pathloss]  = Pathloss_UMi(Dis_2D, fc)
        dis_LOS_prob =18; %distaance within which the probability of LOS is always 1
        h_RSU = 10; %RSU antenna height - meter;
        c = 3e8; %propagation velocity in free space - m/s
        % c= 20e-3;
        % Dis_BP_all = [];
        %to generate random vehicle height: normal vehicles on the road have
        %heights ranging from 1.5 m to 1.7m
        height_random = 1.5:0.05:1.7;
        index_random = randperm(numel(height_random),1); %to get random index of element of height_random
        h_veh = height_random(index_random); %to get random vehicle height

        %effective heights:
        h_E = 1; %Effective environment height for UMi is 1m
        h_veh_E = h_veh - h_E; %effective vehicle antenna height from the ground
        h_RSU_E = h_RSU - h_E; %effective RSU antenna height from the ground

        %Calculation of Dis_3D:
        Dis_3D = sqrt((h_RSU-h_veh)^2 + Dis_2D^2);
        Dis_BP = 4 * (h_RSU_E) * (h_veh_E) * (fc/c); %Breakpoint distance
        % Dis_BP_all = [Dis_BP_all Dis_BP];

        %% Trial Section:
        %UMi-Street Canyon (Line-of-sight(LOS))
        if Dis_2D < 10
            Pathloss_LOS = 0;
        elseif (10 <= Dis_2D) && (Dis_2D <= Dis_BP)
            Pathloss_LOS = 32.4 + 21*log10(Dis_3D) + 20*log10(fc);
        elseif (Dis_BP <= Dis_2D) && (Dis_2D <= 5000)
            Pathloss_LOS = (32.4 + 40*log10(Dis_3D) + 20*log10(fc) - 9.5*log10((Dis_BP)^2 + (h_RSU - h_veh)^2));
        end

        %UMi-Street Canyon (Non-Line-of-sight(NLOS))
        Pathloss_NLOS_prime = 35.3*log10(Dis_3D) + 22.4 + 21.3*log10(fc)- 0.3*(h_veh-1.5);
        Pathloss_NLOS = max(Pathloss_LOS, Pathloss_NLOS_prime); %Pathloss value returned by the function for Dis_2D>10

        %LOS Probability:
        if Dis_2D < 10
            Probability_LOS = 1;
            Pathloss = 0; %in dBm    %Pathloss = 0, because Pathloss_Los=0 for Dis_2D<10
        elseif (10<=Dis_2D) && (Dis_2D<=dis_LOS_prob)
            Probability_LOS = 1;
            Pathloss = Probability_LOS*Pathloss_LOS;%in dB
        elseif dis_LOS_prob < Dis_2D
            Probability_LOS = (dis_LOS_prob/Dis_2D) + exp(-Dis_2D/36)*(1-(dis_LOS_prob/Dis_2D));
            Pathloss = pow2db((db2pow(Pathloss_LOS)*Probability_LOS)+(db2pow(Pathloss_NLOS)*(1-Probability_LOS)));
        end

    end
end

