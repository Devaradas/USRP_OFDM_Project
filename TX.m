p = ofdm1_params(); 
disp('Initializing OFDM Transmitter parameters...');

%  Create OFDM Modulator System Object
ofdmMod = comm.OFDMModulator(...
    'FFTLength',            p.FFTLength, ...
    'CyclicPrefixLength',   p.CPLength, ...
    'NumGuardBandCarriers', p.NumGuardBandCarriers, ...
    'InsertDCNull',         true, ...
    'PilotInputPort',       true, ...
    'PilotCarrierIndices',  p.PilotCarriers, ...
    'NumSymbols',           p.NumSymbols, ...
    'OversamplingFactor',   p.Oversampling); 

% Show modulator info
info(ofdmMod)

sdrTx = comm.SDRuTransmitter(...
    'Platform',             p.Platform, ...
    'SerialNum',            p.SerialNum, ...
    'CenterFrequency',      p.CenterFreq, ...
    'Gain',                 p.GainTx, ...
    'InterpolationFactor',  p.USRPInterp);

% Data generation 
% Calculate data dimensions automatically from the Modulator object
modDim = info(ofdmMod);
numDataSyms = prod(modDim.DataInputSize);
numPilotSyms = prod(modDim.PilotInputSize);

% Generate payload
txBits = randi([0 1], numDataSyms * log2(p.ModOrder), 1);
txData = qammod(txBits, p.ModOrder, 'InputType', 'bit', 'UnitAveragePower', true);
txDataGrid = reshape(txData, modDim.DataInputSize);

% Generate pilots 
txPilots = ones(modDim.PilotInputSize);

disp('Starting transmission loop... Press Ctrl+C to exit loop');


% Generate a known Preamble Sequence
% We use a fixed RNG seed (9999) so the Receiver can generate 
% this exact same sequence for cross-correlation.
prev_rng = rng(9999); 
preambleBits = randi([0 1], modDim.DataInputSize(1) * log2(p.ModOrder), 1);
preambleSyms = qammod(preambleBits, p.ModOrder, 'InputType', 'bit', 'UnitAveragePower', true);

% Restore RNG to random for the rest of the data
rng(prev_rng);

% Inject the Preamble as the First Symbol
% We overwrite the first column of your random data grid with this known preamble.

txDataGrid(:, 1) = preambleSyms;

disp('Preamble added: Symbol 1 is now a fixed Reference Symbol.');

% Modulate  
txWaveform = ofdmMod(txDataGrid, txPilots);

% Scale to avoid clipping 
txWaveform = txWaveform / max(abs(txWaveform)) * 0.8;
figure('Name', 'Transmitter Check', 'NumberTitle', 'off');

% Time Domain 
subplot(2,1,1);
hold on;
symLen = (p.FFTLength + p.CPLength) * p.Oversampling;
preambleWave = real(txWaveform(1:symLen));
plot(1:symLen, preambleWave, 'r', 'LineWidth', 2);
dataWave = real(txWaveform(symLen+1 : 2*symLen));
plot(symLen+1 : 2*symLen, dataWave, 'b');
text(symLen/2, 0.5, 'PREAMBLE ', 'Color', 'red', 'FontWeight', 'bold');
text(symLen + symLen/2, 0.5, 'DATA ', 'Color', 'blue', 'FontWeight', 'bold');

% Formatting
xline(symLen, '--k');
legend('Preamble ', 'Data');
title('Time Domain');
xlabel('Sample Index'); ylabel('Amplitude');
axis tight; grid on;
hold off;
% Frequency Domain 
subplot(2,1,2);
pwelch(txWaveform, [], [], [], p.USRPInterp*1e6, 'centered'); 
title('Transmitted Spectrum');
while true
    % Push to hardware
    sdrTx(txWaveform);
end

function p = ofdm1_params()
    % The SDR Conf.
    p.Platform = 'B200';
    p.SerialNum = '31F5C24'; 
    p.CenterFreq = 2.45e9;
    p.GainTx = 45;
    p.USRPInterp = 512; % Master clock will be divided by this to get sample rate
    
    % OFDM Config.
    p.FFTLength = 64;
    p.CPLength = 16;
    p.NumSymbols = 100; 
    p.ModOrder = 4;    % QPSK
    
    % Subcarrier Mapping
    p.NullCarriers = [1:6, 33, 59:64]'; % Frequencies that will be off. Most left , most right and DC to offer a buffer and avoid LO leakage
    p.PilotCarriers = [12, 26, 40, 54]'; % Freq. that will carry pilots 
    p.NumGuardBandCarriers = [6; 6];  % There are 6 nulls to left and right
    
    % Synchronization 
    p.Oversampling = 4; % Oversampled to fulfill the requi. imposed by Meyr Oerder
    p.SyncBlockLen = 16;  %
end
