clear; clc; close all;
rng(10);
%% ================================================================
% 1. SYSTEM PARAMETERS
% =================================================================
c = 299792458;
fc = 2e9;
lambda = c/fc;

zStageCentre = 0.20;       % Stage centre absolute height [m]
step = 0.01905;            % Movement region [m]

xGrid = (-10:10) * step;
yGrid = (-10:10) * step;
zGrid = zStageCentre + (-10:10) * step;

Nx = numel(xGrid);
Ny = numel(yGrid);
Nz = numel(zGrid);

% Track dimensions
trackX = max(xGrid) - min(xGrid);
trackY = max(yGrid) - min(yGrid);
trackZ = max(zGrid) - min(zGrid);

% Selected Z slice for XY heatmap
zSelected = zStageCentre - 0.1524;

% Fixed transmitter
trackY = max(yGrid) - min(yGrid);
dTx = 0.19;
txPos = [0, -(trackY/2+dTx), zStageCentre];

% Link parameters
sys.fc = fc;
sys.Pt = 0;            % dBm
sys.Gt = 3;            % dBi
sys.Gr = 3;            % dBi
sys.loss = 2;          % dB
sys.noise = -60.5;     % dBm

% Channel parameters
environment = 'MULTIPATH';             % LOS / MULTIPATH / NLOS
numPaths = 8;
K_dB = 6;
multipathScale_dB = 0;

channel = createChannel(environment,numPaths,K_dB,multipathScale_dB);

% Multipath phase reference = centre of movement region
channel.referencePosition = [0;0;zStageCentre];
fprintf('============================================\n');
fprintf(' MOVABLE ANTENNA SIMULATION\n');
fprintf('============================================\n');
fprintf('Frequency       : %.2f GHz\n',fc/1e9);
fprintf('Wavelength      : %.2f cm\n',lambda*100);
fprintf('Environment     : %s\n',environment);
fprintf('Noise floor     : %.1f dBm\n',sys.noise);
fprintf('Tx position     : (%.1f, %.1f, %.1f) cm\n', ...
    txPos(1)*100, ...
    txPos(2)*100, ...
    (txPos(3)-zStageCentre)*100);

%% ================================================================
% 2. SEARCH CONFIGURATION
% ================================================================
cfg.step = step;
cfg.minStepCm = 100/150 * step * 100;   % MIN_STEP: 100 API units = 1.27 cm
cfg.maxMoves = 10000;
cfg.secondsPerStep = [4.5 4.5 4.5];     % seconds per 1.905 cm step along X, Y, Z
cfg.measureSeconds = 0.3;               % seconds to wait/measure at each new position
cfg.nInitial        = 8;                % N_INITIAL
cfg.maxMeasurements = 60;               % MAX_MEASUREMENTS
cfg.travelPenaltyS  = 30;               % TRAVEL_PENALTY_S
cfg.gpNoise         = 0.02;             % NOISE

methods = runSearch('list');
axisM = xGrid;

[ix,iy,iz] = ndgrid(1:Nx,1:Ny,1:Nz);
allIdx = [ix(:),iy(:),iz(:)];

% ---------- Full XYZ map (needed first, so the search knows the true best) ----------
for ix = 1:Nx
    for iy = 1:Ny
        for iz = 1:Nz

            rx = [xGrid(ix), yGrid(iy), zGrid(iz)];

            [Prxyz(ix,iy,iz),SNRxyz(ix,iy,iz)] = link(txPos,rx,sys,channel);

        end
    end
end

[bestSNR3D,index3D] = max(SNRxyz(:));  % True maximum SNR

% ---------- Stop rule: end the search at the best position ----------
cfg.stopMode = 'none';
cfg.maxMoves = 10000;          

startIdx = [1,21,1];                                                        % Start positon of RX

results = cell(numel(methods),1);

for m = 1:numel(methods)

    fprintf('\nRunning %s...\n',methods{m});

    results{m} = runSearch( ...
        methods{m},allIdx,axisM,startIdx, ...
        zStageCentre,txPos,sys,channel,cfg);

end

%% ================================================================
% 3. XY SNR MAP
% ================================================================

Nx = length(xGrid);
Ny = length(yGrid);

SNRxy = zeros(Ny,Nx);

fprintf('\nGenerating XY map...\n');

for ix = 1:Nx
    for iy = 1:Ny

        rx = [xGrid(ix),yGrid(iy),zSelected];

        [~,SNRxy(iy,ix)] = link(txPos,rx,sys,channel);

    end
end

% Best position in this XY slice
[bestSNR2D,index] = max(SNRxy(:));
[iyBest,ixBest] = ind2sub(size(SNRxy),index);

bestRx2D = [xGrid(ixBest), yGrid(iyBest), zSelected];


fprintf('\nBEST RX POSITION IN XY SLICE\n');
fprintf('X     = %.1f cm\n',bestRx2D(1)*100);
fprintf('Y     = %.1f cm\n',bestRx2D(2)*100);
fprintf('Z     = %.2f cm\n',(bestRx2D(3)-zStageCentre)*100);
fprintf('SNR   = %.2f dB\n',bestSNR2D);

%% ================================================================
% 4. 3D RX TRAJECTORIES
% ================================================================
for m = 1:numel(methods)

    figure;
    hold on;

    P = results{m}.xyzCm;

    hPath = plot3(P(:,1),P(:,2),P(:,3),'-','LineWidth',1,'Color',[0.2 0.4 0.8]);

    hStart = plot3(P(1,1),P(1,2),P(1,3),'o','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','g','MarkerEdgeColor','k');

    hEnd = plot3(P(end,1),P(end,2),P(end,3),'s','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','r','MarkerEdgeColor','k');

    % Fixed Tx (same position and style as the SNR map)
    txXcm = txPos(1)*100;
    txYcm = txPos(2)*100;
    txZcm = (txPos(3)-zStageCentre)*100;

    hTx = plot3(txXcm,txYcm,txZcm,'p','MarkerSize',16,'MarkerFaceColor','r','MarkerEdgeColor','k');

    text(txXcm+1,txYcm,txZcm+1,'Tx','FontWeight','bold');

    xlabel('X (cm)');
    ylabel('Y (cm)');
    zlabel('Z (cm)');

    title(methods{m});
    grid on;
    axis equal;
    xlim([-20 20]);
    ylim([-40 20]);
    zlim([-20 20]);
    view(35,25);

    legend([hPath hStart hEnd hTx],'Path','Start','End','Tx','Location','best');

end

%% ================================================================
% 5. XY SNR MAP
% ================================================================
figure;

imagesc(xGrid*100,yGrid*100,SNRxy);
axis xy equal;

% Extend Y axis so Tx at -38 cm is visible
xlim([-20 20]);
ylim([-40 20]);

colorbar;
grid on;

xlabel('X Position (cm)');
ylabel('Y Position (cm)');

title(sprintf( 'XY SNR Map at Z = %.2f cm', (zSelected-zStageCentre)*100));

hold on;

% Best Rx
hRx = plot(bestRx2D(1)*100, bestRx2D(2)*100, 'kx','MarkerSize',12,'LineWidth',2);

% Tx
hTx = plot(txPos(1)*100, txPos(2)*100, 'rp','MarkerSize',14,'MarkerFaceColor','r');

text( txPos(1)*100+1, txPos(2)*100, 'Tx','FontWeight','bold');

legend([hRx hTx], 'Best Rx','Tx', 'Location','best');

%% ================================================================
% 6. EXPERIMENTAL X-SWEEP
% ================================================================

measuredSNR = [ 34.00, 33.50, 32.25, 31.50, 32.85, 33.15, 32.80, 33.00, 34.30, 34.40, 34.00, 33.85, 33.35, 32.35, 33.05, 33.60, 33.65, 32.30, 30.95, 32.00, 32.65 ];

xMeasured = linspace(-trackX/2, trackX/2, length(measuredSNR));

simulatedSNR = zeros(size(xMeasured));
simulatedPower = zeros(size(xMeasured));

% Experiment performed at Y = 0, stage Z = 0
for k = 1:length(xMeasured)

    rx = [xMeasured(k),0,zStageCentre];

    [simulatedPower(k),simulatedSNR(k)] = link(txPos,rx,sys,channel);

end

% Received power inferred from measured SNR
experimentalPower = measuredSNR + sys.noise;

%% ================================================================
% 7. EXPERIMENTAL VS SIMULATED SNR
% ================================================================

figure;

plot(xMeasured*100,measuredSNR, 'o-','LineWidth',1.5);

hold on;

plot(xMeasured*100,simulatedSNR, 's--','LineWidth',1.5);

grid on;
xlim([-20 20]);

xlabel('Rx X Position (cm)');
ylabel('SNR (dB)');
title('Experimental vs Simulated SNR');

legend('Experimental','Simulation','Location','best');

%% ================================================================
% 8. VALIDATION METRICS
% ================================================================

errorSNR = simulatedSNR-measuredSNR;

MAE = mean(abs(errorSNR));
RMSE = sqrt(mean(errorSNR.^2));

R = corrcoef(measuredSNR,simulatedSNR);
correlation = R(1,2);

[maxMeasuredSNR,iMeasured] = max(measuredSNR);
[maxSimulatedSNR,iSimulated] = max(simulatedSNR);

measuredBestX = xMeasured(iMeasured);
simulatedBestX = xMeasured(iSimulated);

fprintf('\nVALIDATION\n');
fprintf('MAE                = %.3f dB\n',MAE);
fprintf('RMSE               = %.3f dB\n',RMSE);
fprintf('Correlation         = %.3f\n',correlation);
fprintf('Measured best X     = %.2f cm\n',measuredBestX*100);
fprintf('Simulated best X    = %.2f cm\n',simulatedBestX*100);
fprintf('Best-position error = %.2f cm\n', abs(measuredBestX-simulatedBestX)*100);

   
%% ================================================================
% 9. BEST XYZ POSITION
% ================================================================
[ixBest,iyBest,izBest] = ind2sub(size(SNRxyz),index3D);

bestRx3D = [ xGrid(ixBest), yGrid(iyBest), zGrid(izBest)];

bestPower3D = Prxyz(ixBest,iyBest,izBest);

fprintf('\nBEST 3-D RX POSITION\n');
fprintf('X     = %.1f cm\n',bestRx3D(1)*100);
fprintf('Y     = %.1f cm\n',bestRx3D(2)*100);
fprintf('Z     = %.1f cm\n',(bestRx3D(3)-zStageCentre)*100);
fprintf('Power = %.2f dBm\n',bestPower3D);
fprintf('SNR   = %.2f dB\n',bestSNR3D);

%% ================================================================
% 10. SEARCH ALGORITHM PERFORMANCE COMPARISON
% =================================================================

fprintf('\n====================================\n');
fprintf('SEARCH ALGORITHM RESULTS\n');
fprintf('====================================\n');

fprintf('True Maximum SNR: %.3f dB\n',bestSNR3D);

for m = 1:numel(methods)

    r = results{m};

    snrGap = bestSNR3D - r.bestSNR;

    fprintf('\nAlgorithm: %s\n',methods{m});
    fprintf('Best SNR: %.3f dB\n',r.bestSNR);
    fprintf('SNR Gap: %.3f dB\n',snrGap);
    fprintf('Measurements: %d\n',r.measurements);
    fprintf('Distance: %.2f cm\n',r.distanceCm(end));
    fprintf('Total time: %.1f s\n',r.totalTimeSec);
    fprintf('Time to reach best: %.1f s\n',r.timeToBestSec);

end

%% ================================================================
% 11. 3-D SNR AT EVERY XYZ COORDINATE
% ================================================================

[X3,Y3,Z3] = ndgrid(xGrid,yGrid,zGrid);

Xcm = X3(:)*100;
Ycm = Y3(:)*100;
Zcm = (Z3(:)-zStageCentre)*100;

figure;

hSNR = scatter3( Xcm,Ycm,Zcm, 12,SNRxyz(:),'filled');

% Transparency allows points inside the volume to remain visible
hSNR.MarkerFaceAlpha = 0.25;
hSNR.MarkerEdgeAlpha = 0.25;

colorbar;
grid on;
axis equal;

xlabel('X Position (cm)');
ylabel('Y Position (cm)');
zlabel('Z Position (cm)');

title('SNR at Every XYZ Rx Position');

% Include both receiver volume and Tx
xlim([-20 20]);
ylim([-40 20]);
zlim([-20 20]);

view(35,25);
hold on;

% Best 3-D Rx
bestZcm = (bestRx3D(3)-zStageCentre)*100;

hRx = plot3( bestRx3D(1)*100, bestRx3D(2)*100, bestZcm, 'kx','MarkerSize',15,'LineWidth',3);

% Fixed Tx
txZcm = (txPos(3)-zStageCentre)*100;

hTx = plot3( txPos(1)*100, txPos(2)*100, txZcm, 'rp','MarkerSize',16,'MarkerFaceColor','r');

text(txPos(1)*100+1, txPos(2)*100, txZcm+1, 'Tx','FontWeight','bold');

legend([hRx hTx], 'Best Rx','Tx', 'Location','best');

%% ================================================================
% 12. EXPORT RESULTS
% ================================================================
% Experimental / simulation comparison
comparisonTable = table( xMeasured(:)*100, measuredSNR(:), simulatedSNR(:), experimentalPower(:), simulatedPower(:), 'VariableNames', {'X_cm','Measured_SNR_dB','Simulated_SNR_dB', 'Experimental_Power_dBm','Simulated_Power_dBm'});
writetable( comparisonTable, 'experimental_simulation_comparison.csv');

% Complete XYZ map
XYZtable = table( Xcm,Ycm,Zcm, Prxyz(:),SNRxyz(:), 'VariableNames', {'X_cm','Y_cm','Z_cm','Power_dBm','SNR_dB'});
writetable(XYZtable,'XYZ_SNR_map.csv');

fprintf('\nSaved:\n');
fprintf('  experimental_simulation_comparison.csv\n');
fprintf('  XYZ_SNR_map.csv\n');
fprintf('\nNumber of XYZ positions = %d\n',numel(SNRxyz));
fprintf('SIMULATION COMPLETE\n');

%% ================================================================
% LOCAL FUNCTION 2
% Generate one static propagation environment.
% ================================================================
function ch = createChannel(environment,L,K_dB,scale_dB)
ch.environment = upper(environment);
ch.K = 10^(K_dB/10);
ch.multipathScale = 10^(scale_dB/20);
switch ch.environment

    case 'LOS'

        ch.numPaths = 0;
        ch.alpha = [];
        ch.az = [];
        ch.el = [];
        ch.extraDistance = [];

    case {'MULTIPATH','NLOS'}

        ch.numPaths = L;

        % Random complex path coefficients
        ch.alpha = (randn(L,1)+1j*randn(L,1))/sqrt(2*L);

        % Random arrival directions
        ch.az = -pi + 2*pi*rand(L,1);
        ch.el = deg2rad(-60 + 120*rand(L,1));

        % Additional reflected path length
        if strcmp(ch.environment,'MULTIPATH')
            ch.extraDistance = 0.3 + 4.7*rand(L,1);
        else
            ch.extraDistance = 0.5 + 6.0*rand(L,1);
        end

    otherwise

        error('Environment must be LOS, MULTIPATH or NLOS.');

end

end




