
%%  Energy Efficiency Evaluation of Bluetooth 5 (BLE 5)Technology %%

clc
clear all
close all
%% PACKET LENGTH CALCULATION%%
dho =1; % Rreference distance
n = 2.1;    % path loss component
f=2.4*10^9;
c=3*10^8;
% No=-114; %dBm/1mHz
No=10.^(-114/10); %Thermal noise power in linear value
% N0 = 10*log10(1.38e-23 * 290 * 1e3) + 10*log10(1e6); %dBm, HK
% N0_lin =10.^(N0./10);%HK 
% No = N0_lin;
a= 1;% Preamble
b=4;% access address
h=2;% Header 
d=[12 14 16 18 20 22 24 26 28];% Payload
e=4;% MIC
g=3;% CRC
k= a+b+h+e+g;
x = plus(k,d); %x is packet length
%% REFERENCE PATHLOSS & PATHLOSS CALCULATION FOR BLE 4 (mode 1) AND BLE 5 (mode 3) %%
Dm3= [50 100 200 300 350 400 450 480 500];% Measurements distance for mode 3
Dm1=[50 100 120 150 170 200 210 220 230];% Measurements distance for mode 1
Imp_loss = 12; %implementation losses in dB

pldm11 = 20*log10((4*pi*dho*f)/c) + 18; %Reference path loss for mode 1 in dB
pldm1 =  pldm11+10*n*log10(Dm1/dho); %Total path loss for mode  1 in dB
pldmdBm1=pldm1;% Total path loss in dBm for mode 1, HK modified
pldmL1= 10.^(pldmdBm1/10);%Total pathloss in linear value (mw) for mode 1

pldm33 = 20*log10((4*pi*dho*f)/c) + Imp_loss; %Reference path loss for mode 3 in dB
pldm3 =  pldm33+10*n*log10(Dm3/dho); %Total path loss for mode 3 in dB
pldmdBm3 = pldm3;% Total path loss in dBm for mode 3, HK modified
pldmL3= 10.^(pldmdBm3/10);%Total pathloss in linear value (mw) for mode 3

%% REFERENCE PATH LOSS PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3)  %%
figure(1)
%plot(Dm1,10*log10(pldm11),'-*');
subplot (2,1,1);
plot(Dm1,pldm11,'-*');
xlabel('Distance in m'); 
ylabel('Reference Path loss in dB'); 
title('Reference Path loss for mode 1 in different distance'); 

subplot (2,1,2);
% plot(Dm3,10*log10(pldm33),'-*');
plot(Dm3,pldm33,'-*');
xlabel('Distance in m'); 
ylabel('Reference Path loss in dB'); 
title('Reference Path loss for mode 3 in different distance'); 
%% PATHLOSS PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3) %%
figure(2)
subplot (2,1,1);
plot(Dm1,pldm1,'-*');
xlabel('Distance in m'); 
ylabel('Path loss in dBm'); 
title('Path loss for mode 1 in different distance'); 

subplot (2,1,2);
plot(Dm3,pldm3,'-*');
xlabel('Distance in m'); 
ylabel('Path loss in dBm'); 
title('Path loss for mode 3 in different distance'); 

%% SNR & RECEIVED SIGNAL POWER CALCULATION FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%
tb=1e-6; %Time of transmit bit in micro second
%  ptx= 1e-3; %transmit power in miliwatt
ptxdBm=0;%dBm
ptx=10.^(0/10);% Transmit power in linear value in mw

prx1= (ptxdBm-pldmdBm1);% prx is received signal power and ptx is transmit power for mode 1 in dBm
prxL1=10.^(prx1/10);%Linear value of received signal power for mode 1
prx3=(ptxdBm-pldmdBm3);% prx is received signal power and ptx is transmit power for mode 3 in dBm
prxL3=10.^(prx3/10);%Linear value of received signal power for mode 3
Es1= prxL1.*tb;    % Energy per symbol and tb transmit bit time for mode 1 in linear
% EsL1= 10.^(Es1/10);  
Es3= prxL3.*tb;    % Energy per symbol and tb transmit bit time for mode 3 in linear
% EsL3= 10.^(Es3/10); 
SNR1=prxL1./No;          % SNR for mode 1 with linear value in mw for mode 1
SNR3=prxL3./No;          % SNR for mode 1 with linear value in mw for mode 1
%% SNR PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%
figure(3)
subplot (2,1,1);
plot(Dm1,10*log10(SNR1),'-*')
title('SNR in different distance for mode 1')
xlabel('Distance in m'); 
ylabel('SNR in dB')

subplot (2,1,2);
plot(Dm3,10*log10(SNR3),'-*')
title('SNR in different distance for mode 1')
xlabel('Distance in m'); 
ylabel('SNR in dB')
%% RECEIVED SIGNAL POWER PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%
figure(4)
subplot (2,1,1);
plot(Dm1,prx1,'-*')
title('Received power in different distance for mode 1')
xlabel('Distance in m'); 
ylabel('Received power in dBm')

subplot (2,1,2);
plot(Dm3,prx3,'-*')
title('Received power in different distance for mode 3')
xlabel('Distance in m'); 
ylabel('Received power in dBm')
%% SYMBOL ERROR RATE CALCULATION & PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%
SER1= 0.5*exp(-0.5.*SNR1);
SER3= 0.5*exp(-0.5.*SNR3);

figure(5)
subplot (2,1,1);
plot(Dm1,SER1,'-*')
title('Symbol Error Rate in different distance for mode 1')
xlabel('Distance in m'); 
ylabel('Symbol Error Rate (SER)')

subplot (2,1,2);
plot(Dm3,SER3,'-*')
title('Symbol Error Rate in different distance for mode 3')
xlabel('Distance in m'); 
ylabel('Symbol Error Rate (SER)')

%% PACKET ERROR RATE(PER) CALCULATION & PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%% 

PER1=1-(1-SER1).^x;
figure(6)
subplot (2,1,1);
plot(Dm1,PER1,'-*');
xlabel('Distance in m'); 
ylabel('Packet Error Rate (PER)'); 
title('Packet Error Rate in different distance for mode 1');

PER3= 1-(1-SER3).^x;
subplot (2,1,2);
plot(Dm3,PER3,'-*');
xlabel('Distance in m'); 
ylabel('Packet Error Rate (PER)'); 
title('Packet Error Rate in different distance for mode 3');

%% ENERGY CONSUMPTION CALCULATION FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%
tb=588./10^6 ;% Time to transmit packet in seconds
PTx= 8.16./10.^3; %Transmitter power in w
PRx= 7.82./10.^3;% Receiver power in w
Dm3= [50 100 200 300 350 400 450 480 500];% Measurements distance for mode 3 (BLE 5)
Dm1=[50 100 120 150 170 200 210 220 230];% Measurements distance for mode 1  (BLE 4)

Number_of_information_bits_mode1 = 51; %Number of information bits per packet in uncoded mode/for mode 1  (BLE 4)
Number_of_information_bits_mode3 = 26; %Number of information bits per packet in coded mode/for mode 3 (BLE 5)

Number_of_received_packets_mode1 = [390  416  468  620  560 180 108 56 10]; %Number of received packets for mode 1(measurements data for BLE 4)
Number_of_received_packets_mode3 = [1006 1039  1213 990 850 240 55 32 12];%Number of received packets for mode 3(measurements data for BLE 5)

Ecd1=(PTx*tb*(1./(1-PER1)))+(PRx*tb*(1./(1-PER1)));% Theorytical Energy consumption analysis for mode 1  (BLE 4)
Ecd3=(PTx*tb*(1./(1-PER3)))+(PRx*tb*(1./(1-PER3)));% Theorytical Energy consumption analysis for mode 3 (BLE 5)
 
PER1_meas = [0   0.0003    0.0048    0.1040    0.3570    0.7446    0.8796    0.9439    0.98987];% PER practical measurements for mode 1 (BLE 4)

PER3_meas = [0.0009 0.0009 0.0041 0.2130 0.4640 0.8501 0.9531 0.9729 0.9945];% PER practical measurements for mode 3 (BLE 5)

Ecd1_measurements = (PTx*tb*(1./(1-PER1_meas)))+(PRx*tb*(1./(1-PER1_meas)));% Practical Energy consumption analysis for mode 1 (BLE 4)
Ecd3_measurements =(PTx*tb*(1./(1-PER3_meas)))+(PRx*tb*(1./(1-PER3_meas))) ; % Practical Energy consumption analysis for mode 3 (BLE 5)

Ecd1_per_info_bits = Ecd1_measurements / (Number_of_information_bits_mode1*Number_of_received_packets_mode1);% Energy consumption per information bit for mode 1 (BLE 4)
Ecd3_per_info_bits = Ecd3_measurements / (Number_of_information_bits_mode3*Number_of_received_packets_mode3); % Energy consumption per information bit for mode 3 (BLE 5)
% for mode 1

Su1= Number_of_information_bits_mode1.*x.*8% For  mode 1 (BLE 4)  
Su3= Number_of_information_bits_mode1.*x.*8% For  mode 3 (BLE 5) 
    

%% THEORYTICAL  ANALYSIS OF ENERGY CONSUMPTION MODEL PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%
figure(1)
subplot (4,1,1);
plot(Dm1,Ecd1,'-*')
title('Theorytical Energy consumption analysis for BLE4 in different distance ')
xlabel('Distance in m ') % x-axis label
ylabel('Energy consumption in mw') % y-axis label

subplot (4,1,2);
plot(Dm3,Ecd3,'-*')
title('Theorytical Energy consumption analysis for BLE5  in different distance ')
xlabel('Distance in m ') % x-axis label
ylabel('Energy consumption in mw') % y-axis label
%% PRACTICAL ANALYSIS OF ENERGY CONSUMPTION MODEl PLOTTING FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%

subplot (4,1,3);
plot(Dm1,Ecd1_measurements,'-*')
title('Practical Energy consumption analysis for BLE4 in different distance ')
xlabel('Distance in m ') % x-axis label
ylabel('Energy consumption in mw') % y-axis label

subplot (4,1,4);
plot(Dm3,Ecd3_measurements,'-*')
title('Practical Energy consumption analysis for BLE5  in different distance ')
xlabel('Distance in m ') % x-axis label
ylabel('Energy consumption in mw') % y-axis label
%% ENERGY CONSUMPTION PER INFORMATION BIT FOR BLE 4 (mode 1) AND BLE 5 (mode 3)%%
figure(8)
% plot(Dm1,Ecd1_per_info_bits,'-*')
subplot (2,1,1);
plot(Dm1,Su1,'-*')
title('Energy consumption per information bit for BLE4 in different distance ')
xlabel('Distance in m ') % x-axis label
ylabel('Energy consumption in mw') % y-axis label

subplot (2,1,2);
% plot(Dm3,Ecd3_per_info_bits,'-*')
plot(Dm3,Su3,'-*')
title('Energy consumption per information bit for BLE5 in different distance ')
xlabel('Distance in m ') % x-axis label
ylabel('Energy consumption in mw') % y-axis label

