clear; close all; clc;

p = ofdm1_params();

ofdmMod = comm.OFDMModulator( ...
    'FFTLength',            p.FFTLength, ...
    'CyclicPrefixLength',   p.CPLength, ...
    'NumGuardBandCarriers', p.NumGuardBandCarriers, ...
    'InsertDCNull',         true, ...
    'PilotInputPort',       true, ...
    'PilotCarrierIndices',  p.PilotCarriers, ...
    'NumSymbols',           p.NumSymbols);

modDim = info(ofdmMod);
Kdata  = modDim.DataInputSize(1);     % genelde 47


rng(2024);
trainBits = randi([0 1], Kdata*log2(p.ModOrder), 1);                                  %the first 2 OFDM symbols are the same for Moose to work
trainSyms = qammod(trainBits, p.ModOrder, 'InputType','bit', 'UnitAveragePower', true); 


Nbps = log2(p.ModOrder);
dataBits = randi([0 1], Kdata*(p.NumSymbols-2)*Nbps, 1);                              %generate random bits
dataSyms = qammod(dataBits, p.ModOrder, 'InputType','bit', 'UnitAveragePower', true); %modulate the random bits

% the first two symbols are the same then the rest come from dataSyms
txGrid = zeros(Kdata, p.NumSymbols);
txGrid(:,1) = trainSyms;
txGrid(:,2) = trainSyms;   
txGrid(:,3:end) = reshape(dataSyms, Kdata, p.NumSymbols-2);

%Pilots are 1
txPilots = ones(4, p.NumSymbols);

txWaveform = ofdmMod(txGrid, txPilots);
% txWaveform = txWaveform / max(abs(txWaveform)) * 0.7; %clipping 

% rxSig = rxSig_cfo

sdrTx = comm.SDRuTransmitter( ...
    'Platform',            p.Platform, ...
    'SerialNum',           p.SerialNumTx, ...
    'CenterFrequency',     p.CenterFreq, ...
    'Gain',                p.GainTx, ...
    'InterpolationFactor', p.USRPInterp);

disp('Transmitting');
while true
    sdrTx(txWaveform);
end


function p = ofdm1_params()
    p.Platform    = 'B200';
    p.SerialNumTx = '31FD9A3';     % <-- TX USRP
    p.CenterFreq  = 1.2e9;

    p.GainTx      = 50;
    p.USRPInterp  = 128;

    p.FFTLength   = 64;
    p.CPLength    = 16;
    p.NumSymbols  = 20;
    p.ModOrder    = 4;

    p.NumGuardBandCarriers = [6;6];
    p.PilotCarriers        = [12 26 40 54]';
end
