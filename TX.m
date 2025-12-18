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
modDim = info(ofdmMod);
numDataSyms = prod(modDim.DataInputSize);
numPilotSyms = prod(modDim.PilotInputSize);

% Create a "Frame Header" (Packet ID)
frameNum = 1; 
headerBits = int2bit(frameNum, 8); % 8 bits for ID

% Calculate remaining space for random data
totalBits = numDataSyms * log2(p.ModOrder);
numRandomBits = totalBits - length(headerBits);

% Generate Random Payload
randomBits = randi([0 1], numRandomBits, 1);

% Combine Header + Data
txBits = [headerBits; randomBits]; 

% Modulate Data
txData = qammod(txBits, p.ModOrder, 'InputType', 'bit', 'UnitAveragePower', true);
txDataGrid = reshape(txData, modDim.DataInputSize);

% Inject Sync Preamble (Overwrite Symbol 1)
% We use a fixed seed so Receiver knows this pattern MUST HAVE SAME AT
% REC. SIDE
prev_rng = rng(9999); 
preambleBits = randi([0 1], modDim.DataInputSize(1) * log2(p.ModOrder), 1);
preambleSyms = qammod(preambleBits, p.ModOrder, 'InputType', 'bit', 'UnitAveragePower', true);
rng(prev_rng); 

txDataGrid(:, 1) = preambleSyms; % Symbol 1 is Sync Preamble

disp('Payload generated: [Sync Preamble] + [Frame ID] + [Random Data]');

% Generate pilots 
txPilots = ones(modDim.PilotInputSize);

disp('Starting transmission loop... Press Ctrl+C to exit loop');

% Modulate  
txWaveform = ofdmMod(txDataGrid, txPilots);

% Scale to avoid clipping 
txWaveform = txWaveform / max(abs(txWaveform)) * 0.8;

% --- PLOTTING SECTION ---
figure('Name', 'Transmitter Check', 'NumberTitle', 'off');

% Time Domain 
subplot(2,1,1);
hold on;

symLen = (p.FFTLength + p.CPLength) * p.Oversampling;

% Plot Preamble (Red) - Start
preambleWave = real(txWaveform(1:symLen));
plot(1:symLen, preambleWave, 'r', 'LineWidth', 2);

% Plot Data (Blue) - Next
dataWave = real(txWaveform(symLen+1 : 2*symLen));
plot(symLen+1 : 2*symLen, dataWave, 'b');

text(symLen/2, 0.5, 'PREAMBLE', 'Color', 'red', 'FontWeight', 'bold');
text(symLen + symLen/2, 0.5, 'DATA', 'Color', 'blue', 'FontWeight', 'bold');

xline(symLen, '--k');
legend('Preamble', 'Data');
title('Time Domain Check');
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
    p.USRPInterp = 512; 
    
    % OFDM Config.
    p.FFTLength = 64;
    p.CPLength = 16;
    p.NumSymbols = 100; 
    p.ModOrder = 4;    % QPSK
    
    % Subcarrier Mapping
    p.NullCarriers = [1:6, 33, 59:64]'; 
    p.PilotCarriers = [12, 26, 40, 54]'; 
    p.NumGuardBandCarriers = [6; 6];  
    
    % Synchronization 
    p.Oversampling = 4; 
    p.SyncBlockLen = 16;  
end
