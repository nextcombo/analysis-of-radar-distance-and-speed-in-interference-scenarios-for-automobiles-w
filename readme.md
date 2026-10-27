# Analysis of Radar Distance and Speed in Interference Scenarios for Automobiles [with MATLAB Code]

Key Target Impact Analysis: Study how FMCW radar jamming affects target detection precision, especially how it affects the ranging and speed measurement of nearby objects.
Data-Driven Insights: Analyze how jamming interferes with radar's ability to precisely detect and track targets using collected data and visualize this interference.
Filtering Implementation: Test and demonstrate the effectiveness of various filtering techniques aimed at mitigating the negative impact of radar jamming on performance and reliability, enhancing radar capabilities.
Dataset Description: The dataset includes parameters for both the victim radar system and the jamming radar system:
Victim Radar Parameters:
EIRP: -5 dBW
Antenna Gain: 10 dBi
Start Frequency: 76.5 GHz
Bandwidth: 500 MHz
Scanning Time: 50 µs
Pulse Repetition Interval: 60 µs
Coherent Processing Interval: 10 ms
Low Pass Filter Cutoff Frequency: 3.5 MHz
Signal Modulation: FMCW
Antenna Configuration: Single Node (Initial Stage), TDMA-MIMO (Advanced Stage)
Speed: (0, 0) m/s
Jamming Radar Parameters:
EIRP: -5 dBW
Start Frequency: 76.45 GHz
Bandwidth: 900 MHz
Scanning Time: 25 µs
Pulse Repetition Interval: 30 µs
Signal Modulation: FMCW
Target:
Target 1: Size (4 m x 1.9 m x 1.5 m), Speed (10, 0) m/s, Position (40, 3.6) m
Target 2: Size (4.5 m x 1.9 m x 1.5 m), Speed (-5, 0) m/s, Position (20, 7.2) m
MATLAB code: clear all; \nclc; \n% Victim radar parameters\nfc = 76.5e9; % Start frequency\nbw_sv = 500e6; % Bandwidth\ntm_sv = 50e-6; % Sweep time\nfs_sv = bw_sv; % Sampling frequency\nN_sv = 25000; % Number of samples in each chirp\nNd_sv = 135; % Number of chirps\n% Initialize variables for later use\nN=N_sv; \nNd=Nd_sv; \nfs=fs_sv; \nc=3e8; \nslope=bw_sv/tm_sv; % Frequency slope\nlambda = c/fc; % Wavelength (m)\ntm=tm_sv; \n% Interferer radar parameters\nfc_int = 76.45e9; % Start frequency\nbw_int = 900e6; % Bandwidth\ntm_int = 25e-6; % Sweep time\nfs_int = bw_int; % Sampling frequency (Hz)\n% Time vectors\nt_sv = 0:1/fs_sv:(tm_sv*Nd_sv)-1/fs_sv; % Victim radar\n% Transmitted signal (Victim radar)\n% --- Target Parameters ---\nt1_vel = 10; % Velocity of target 1 (m/s)\nt2_vel = -5; % Velocity of target 2 (m/s)\nt1_pos = [40, 3.6]; % Initial position of target 1 (m)\nt2_pos = [20, 7.2]; % Initial position of target 2 (m)\nvictim_pos = [0, 0]; % Position of victim radar (m)\n% Calculate initial distances from radar to targets\nr1 = pdist([victim_pos; t1_pos], 'euclidean'); \nr2 = pdist([victim_pos; t2_pos ], 'euclidean'); \nrange1 = r1 + t1_vel * t_sv; %change of range with respect to time\ndelta_t1 = 2 * range1 / 3e8; %change in delay time\nt_chirp = 0:1/fs_sv:(tm_sv)-1/fs_sv; %time vectors for single chirp\nt_int1=0:1/fs_int:tm_int-1/fs_int; % time vectors for interference radar\nt_int= repmat(t_int1,[1,150]); \n% --- Power and Amplitude Calculations ---\nL = 4; % Length of the longest side of target (m)\nL_int = 4.5; % Length of the longest side of interferer (m)\nEIRP_dBW = -5; % Effective Isotropic Radiated Power (dBW)\nEIRP_dBW_int = -5; % EIRP for interferer (dBW)\nG_dBi = 10; % Antenna gain (dBi)\nlambda_val = 0.0039; % Wavelength (m)\n% Calculate Radar Cross Section (RCS)\nrcs = 4 * pi * (L^4) / (3 * (lambda^2)); % RCS for target (m^2)\nrcs_int = 4 * pi * (L_int^4) / (3 * (lambda^2)); % RCS for interferer (m^2)\n% Convert EIRP and Gr from dB to linear scale\nEIRP_linear = 10^(EIRP_dBW/10); % in Watts\nEIRP_linear_int = 10^(EIRP_dBW_int/10); % in Watts\nG_r_linear = 10^(G_dBi/10); % Linear scale\n% Radar equation to compute Pr\nPr = (EIRP_linear * G_r_linear * lambda_val^2 * rcs) / ((4*pi)^3 * r1^4); \nA = sqrt(2*Pr); % Amplitude for victim radar\n% Radar equation to compute Pr of interference\nPr_int = (EIRP_linear * G_r_linear * lambda_val^2*rcs_int ) / ((4*pi)^2 * r2^2); \nA_int = sqrt (2*Pr_int); % Amplitude for recieved radar\n% --- Signal Generation for Each Chirp ---\nfor chirp_idx = 1:Nd_sv-1\n % Update the range for target 1 for each chirp\n r1(chirp_idx+1) = r1(chirp_idx) + t1_vel * tm_sv; \nend\nfor chirp_idx = 1:Nd_sv\n % Calculate the time offset for the current chirp\n time_offset = (chirp_idx - 1) * tm_sv; \n \n % Calculate the time delay for target 1\n delta_t1 = 2 * r1(chirp_idx) / c; \n % Generate the transmitted signal for one chirp\n st_chirp = exp(1j * 2 * pi * (fc * t_chirp + (bw_sv / tm_sv) * (t_chirp.^2)/2)); \n % Generate the received signal for one chirp considering target 1\n sr_chirp = A * exp(1j * 2 * pi * (fc * (t_chirp - delta_t1) + (bw_sv / tm_sv)*(t_chirp - delta_t1).^2 / 2 - 2 * fc * t1_vel / c)); \n % Add noise to the received signal\n noise_level = A * randn(size(sr_chirp)); \n sr_chirp1 = sr_chirp + noise_level; \n % Store the transmitted and received signals for this chirp\n st_all_chirps(:, chirp_idx) = st_chirp; \n sr_all_chirps(:, chirp_idx) = sr_chirp1; \nend\n% --- Generate Interference Signal ---\nslope_int = bw_int / tm_int; % Frequency slope for interferer\n% Generate the interference signal\nsr_interference = A_int * exp(1j * 2 * pi * (fc_int * t_int + (slope_int / 2) * (t_int.^2) - 2 * fc_int * t2_vel / c)); \n% Reshape the interference signal to match dimensions\nsr_int = reshape(sr_interference, [N, Nd]); \n% Add the interference to the received signal\nsr = sr_all_chirps + sr_int; MATLAB simulation results

## Images

![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](01.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](02.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](03.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](04.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](05.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](06.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](07.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](08.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](09.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](10.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](11.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](12.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](13.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](14.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](15.jpg)
![Analysis of Radar Distance and Speed Measurement in Interference Scenes [with MATLAB Code]](16.jpg)

<!-- PAYMENT_BLOCK -->

## Get this dataset

- Payment link (Stripe): [Buy now](https://buy.stripe.com/3cs8yP7sY87d0vu9AB)
- Payment QR code: ![Payment QR code](pay.png)

<!-- /PAYMENT_BLOCK -->
