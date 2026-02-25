
p = ofdm_params();

sdrRx = comm.SDRuReceiver( ...
    'Platform',           p.Platform, ...
    'SerialNum',          p.SerialNumRx, ...
    'CenterFrequency',    p.CenterFreq, ...
    'Gain',               p.GainRx, ...
    'DecimationFactor',   p.USRPDecim, ...
    'SamplesPerFrame',    p.SamplesPerFrame, ...
    'OutputDataType',     'double');

figure('Name','RX Moose CFO + LS CE');
disp('RX listening...');

Nfft  = p.FFTLength;
Ng    = p.CPLength;
Nofdm = Nfft + Ng;
knownPilotsFull = ones(4, p.NumSymbols);


%demod full frame
ofdmDemodFull = comm.OFDMDemodulator( ...
    'FFTLength',            p.FFTLength, ...
    'CyclicPrefixLength',   p.CPLength, ...
    'NumGuardBandCarriers', p.NumGuardBandCarriers, ...
    'PilotOutputPort',      true, ...
    'PilotCarrierIndices',  p.PilotCarriers, ...
    'NumSymbols',           p.NumSymbols,...
    'RemoveDCCarrier', true);

%demod the pilots
ofdmDemod2 = comm.OFDMDemodulator( ...
    'FFTLength',            p.FFTLength, ...
    'CyclicPrefixLength',   p.CPLength, ...
    'NumGuardBandCarriers', p.NumGuardBandCarriers, ...
    'PilotOutputPort',      true, ...
    'PilotCarrierIndices',  p.PilotCarriers, ...
    'NumSymbols',           2);



while true
    rxSig = sdrRx();          % Nx1
    rxRow = rxSig(:).';       % STO metric için row

    % time sync (works)
    [bestOffset, metric] = sto_cp_correlation(rxRow, Nfft, Ng, Nofdm, p.AvgSymbolsForSTO);
    startIdx = bestOffset + 1;

    totalLen = Nofdm * p.NumSymbols;
    if startIdx <= 0 || (startIdx + totalLen - 1) > length(rxRow)
        continue;
    end

    rxFrame = rxSig(startIdx : startIdx + totalLen - 1);   % row
%% 

    % pilotları kullanarak faz farkı
    rxTrain = rxFrame(1 : 2*Nofdm).';   
    [Y1, P1] = ofdmDemod2(rxTrain.');
    % 
    % eps_hat = CFO_Moose_unwrap_fromPilots(P1, Nfft, Ng);
    % 
    % nn = (0:length(rxFrame)-1).';
    % rxFrame_corr = rxFrame(:) .* exp(-1j*2*pi*eps_hat*nn/Nfft);
    % 
    % rxFrame_corr = rxFrame(:).';
%% 

    C = sum(P1(:,2) .* conj(P1(:,1)));
    eps_hat = (angle(C)/(2*pi)) * (Nfft/(Nfft+Ng));  
    % 
    nn = (0:length(rxFrame)-1).';
    rxFrame_corr = rxFrame(:) .* exp(-1j*2*pi*eps_hat*nn/Nfft);
%% 

    %ilk 2 sembolü kullanarak faz farkı
    % CFO_hat = CFO_Moose(Y1);
    % nn = (0:length(rxFrame)-1);
    % rxFrame_corr = rxFrame .* exp(-1j*2*pi*CFO_hat*nn/Nfft);      
    % rxFrame_corr = rxFrame;

    %demodülsayon
    % rxFrame_corr = rxFrame_corr(:); % column
    [rxGrid, rxPilots] = ofdmDemodFull(rxFrame_corr);  

    %channel estimation
    Kdata = size(rxGrid,1);
    H_data = LS_CE_4pilots(rxPilots, knownPilotsFull, Kdata, 'linear');
    eqGrid = rxGrid ./ (H_data);
%% 
    %plots
    subplot(2,1,1);
    plot(0:Nofdm-1, metric, 'b-'); grid on;
    hold on; stem(bestOffset, metric(bestOffset+1), 'r', 'filled'); hold off;
    title(sprintf('CP STO metric | bestOffset=%d | Moose CFO=%.4f', bestOffset));
    xlabel('Offset (samples)'); ylabel('Metric');

    subplot(2,1,2);
    plot(eqGrid(:), 'r.'); grid on; axis equal;
    axis([-2 2 -2 2]);
    title('Constellation after Moose CFO + LS CE + CPE');
    drawnow;
end

%functions



function [bestOffset, M] = sto_cp_correlation(y, Nfft, Ng, Nofdm, K)
% CP correlation timing metric (kitap)
y = y(:).';
L = Nofdm;
M = zeros(1, L);

for d = 0:(L-1)
    acc = 0; cnt = 0;
    for m = 0:(K-1)
        idx = d + m*Nofdm + 1;
        if (idx + Nfft + Ng - 1) > length(y), break; end
        a = y(idx : idx+Ng-1);
        b = y(idx+Nfft : idx+Nfft+Ng-1);
        P = sum(a .* conj(b));
        R = sum(abs(b).^2) + 1e-12;
        acc = acc + (abs(P)^2) / (R^2);
        cnt = cnt + 1;
    end
    if cnt > 0, M(d+1) = acc / cnt; end
end

[~, ind] = max(M);
bestOffset = ind - 1;
end



function H_data = LS_CE_4pilots(rxPilots, knownPilots, Kdata, int_opt)
if nargin < 4, int_opt = 'linear'; end

S = size(rxPilots,2);

pilotPosOcc = [6 20 33 47];
allOcc = 1:51;
dataPosOcc = setdiff(allOcc, pilotPosOcc, 'stable');  % 47 positions

H_data = zeros(Kdata, S);

for s = 1:S
    H_p = rxPilots(:,s) ./ (knownPilots(:,s) + 1e-12);  % 4x1

    if lower(int_opt(1)) == 's'
        H_occ = interp1(pilotPosOcc, H_p, 1:51, 'spline', 'extrap');
    else
        H_occ = interp1(pilotPosOcc, H_p, 1:51, 'linear', 'extrap');
    end

    H_col = H_occ(dataPosOcc).';  % 47x1
    H_col(abs(H_col) < 1e-6) = 1e-6;

    if length(H_col) ~= Kdata
        H_col = mean(H_p) * ones(Kdata,1); % fallback
    end

    H_data(:,s) = H_col;
end
end

% ---------------- Params ----------------
function p = ofdm_params()
    p.Platform    = 'B200';
    p.SerialNumRx = '31F5C24';   % <-- RX USRP
    p.CenterFreq  = 1.2e9;

    p.GainRx      = 50;
    p.USRPDecim   = 128;
    p.SamplesPerFrame = 60000;

    p.FFTLength   = 64;
    p.CPLength    = 16;
    p.NumSymbols  = 20;
    p.ModOrder    = 4;

    p.NumGuardBandCarriers = [6;6];
    p.PilotCarriers        = [12 26 40 54]';

    p.AvgSymbolsForSTO = 6;
end
