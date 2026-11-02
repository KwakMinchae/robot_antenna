function interactiveSim
% Interactive 3-D Tx / Rx movable-antenna demonstration.
% The selected search algorithm takes one measurement per animation tick.

rng(10);

c = 299792458;
fc = 2e9;
lambda = c/fc;

sys = struct('fc',fc,'Pt',0,'Gt',3,'Gr',3, ...
             'loss',2,'noise',-60.5);

maxMeasurements = 100;

noImprovementCount = 0;
maxNoImprovement = 20;

snrTolerance = 0.05;

searchFinished = false;
returningToBest = false;

% Multipath channel
ch.environment = 'MULTIPATH';
ch.K = 10^(6/10);
ch.multipathScale = 1;
ch.numPaths = 8;

L = ch.numPaths;

ch.alpha = (randn(L,1)+1j*randn(L,1))/sqrt(2*L);
ch.az = -pi+2*pi*rand(L,1);
ch.el = deg2rad(-60+120*rand(L,1));
ch.extraDistance = .3+4.7*rand(L,1);
ch.referencePosition = [0;0;.20];

% Tx departure directions
ch.txAz = -pi+2*pi*rand(L,1);
ch.txEl = deg2rad(-60+120*rand(L,1));

% Initial positions (metres)
tx = [0,-.38,.20];
txReference = tx;

rx = [0,0,.20];
rxStart = rx;

% Movement boundaries
rxLo = [-.1905,-.1905,.0095];
rxHi = [.1905,.1905,.3905];

txLo = [-.30,-.65,0];
txHi = [.30,-.22,.45];

step = .01905;

methods = {'Random','Greedy','Bayesian', ...
           'CoarseToFine','AdaptivePattern'};

method = 'AdaptivePattern';

running = false;
tick = 0;
elapsedTime = 0;       % Simulated time in seconds
timeToBest = 0;        % Time when best SNR was found

secondsPerStep = 4.5;  % Seconds per 1.905 cm
measureSeconds = 0.3;  % Measurement settling time

totalDistance = 0;     % Total Rx travel distance (m)
samples = zeros(0,4);
visited = zeros(0,3);

bestSNR = -Inf;
bestRx = rx;

patternStep = step*4;
patternDir = 1;
searchIndex = 1;
pending = zeros(0,3);

currentTarget = rx;
hasTarget = false;

% Original runSearch.m integration
searchResult = [];
pathCm = zeros(0,3);
pathIndex = 1;

zStageCentre = 0.20;

% 21 x 21 x 21 Rx search grid
gridAxis = (-10:10)*step;

[ix,iy,iz] = ndgrid(1:21,1:21,1:21);
allIdx = [ix(:),iy(:),iz(:)];

% =========================================================
% GUI
% =========================================================

fig = uifigure( ...
    'Name','Interactive Movable Antenna - 3D Tx and Rx', ...
    'Position',[80 80 1120 710]);

ax = uiaxes(fig,'Position',[30 195 710 485]);

grid(ax,'on');
hold(ax,'on');

xlabel(ax,'X (cm)');
ylabel(ax,'Y (cm)');
zlabel(ax,'Z (cm)');

view(ax,35,25);

xlim(ax,100*[-.35 .35]);
ylim(ax,100*[-.7 .25]);
zlim(ax,[0 48]);

title(ax,'Tx (red), Rx (green), Rx trajectory (blue)');

% Rx movement boundary
plot3(ax, ...
    100*[rxLo(1) rxHi(1) rxHi(1) rxLo(1) rxLo(1)], ...
    100*[rxLo(2) rxLo(2) rxHi(2) rxHi(2) rxLo(2)], ...
    100*rxLo(3)*ones(1,5),'k:');

% Tx marker
hTx = plot3(ax,100*tx(1),100*tx(2),100*tx(3), ...
    'rp','MarkerSize',16,'MarkerFaceColor','r');

% Rx marker
hRx = plot3(ax,100*rx(1),100*rx(2),100*rx(3), ...
    'go','MarkerSize',11,'MarkerFaceColor','g');

% Rx path
hPath = plot3(ax,100*rx(1),100*rx(2),100*rx(3), ...
    'b-','LineWidth',1.3);

% Best Rx position
hBest = plot3(ax,100*rx(1),100*rx(2),100*rx(3), ...
    'kx','MarkerSize',12,'LineWidth',2);

% SNR graph
a2 = uiaxes(fig,'Position',[30 25 710 150]);

grid(a2,'on');
hold(a2,'on');

xlabel(a2,'Measurements');
ylabel(a2,'SNR (dB)');

hS = plot(a2,nan,nan,'b-');
hB = plot(a2,nan,nan,'g--');

% =========================================================
% Tx XYZ sliders
% =========================================================

uilabel(fig, ...
    'Position',[770 655 310 25], ...
    'Text','Transmitter coordinates (cm)', ...
    'FontWeight','bold');

sl = gobjects(1,3);
vals = gobjects(1,3);

names = {'Tx X','Tx Y','Tx Z'};

for k = 1:3

    y = 625-(k-1)*80;

    uilabel(fig, ...
        'Position',[770 y 70 22], ...
        'Text',names{k});

    sl(k) = uislider(fig, ...
        'Position',[850 y+9 205 3], ...
        'Limits',[txLo(k) txHi(k)]*100, ...
        'Value',tx(k)*100);

    vals(k) = uilabel(fig, ...
        'Position',[1060 y 55 22], ...
        'Text',sprintf('%.1f',tx(k)*100));

    sl(k).ValueChangingFcn = ...
        @(src,event) moveTx(k,event.Value);

    sl(k).ValueChangedFcn = ...
        @(src,event) moveTx(k,event.Value);

end

% =========================================================
% Algorithm selection
% =========================================================

uilabel(fig, ...
    'Position',[770 380 110 22], ...
    'Text','Search algorithm');

dd = uidropdown(fig, ...
    'Position',[880 380 200 24], ...
    'Items',methods, ...
    'Value',method);

dd.ValueChangedFcn = ...
    @(src,event) changeMethod(src.Value);

% =========================================================
% Control buttons
% =========================================================

btn = uibutton(fig,'push', ...
    'Position',[770 325 95 35], ...
    'Text','Start', ...
    'ButtonPushedFcn',@(~,~) startPause());

uibutton(fig,'push', ...
    'Position',[875 325 95 35], ...
    'Text','Reset Rx', ...
    'ButtonPushedFcn',@(~,~) resetRx());

uibutton(fig,'push', ...
    'Position',[980 325 105 35], ...
    'Text','New search', ...
    'ButtonPushedFcn',@(~,~) restartSearch());

uilabel(fig, ...
    'Position',[770 270 115 22], ...
    'Text','Playback interval');

speed = uislider(fig, ...
    'Position',[880 281 190 3], ...
    'Limits',[.05 1], ...
    'Value',.20);

status = uitextarea(fig, ...
    'Position',[770 95 330 165], ...
    'Value',{'Ready'}, ...
    'Editable','off');

% Animation timer
tm = timer( ...
    'ExecutionMode','fixedSpacing', ...
    'Period',.2, ...
    'BusyMode','drop', ...
    'TimerFcn',@(~,~) updateTick());

fig.CloseRequestFcn = @(~,~) closeApp();

redraw();

% =========================================================
% Tx movement
% =========================================================

    function moveTx(k,v)

    tx(k) = v/100;
    vals(k).Text = sprintf('%.1f',v);

    % Invalidate previous trajectory
    searchResult = [];
    searchFinished = false;

    if strcmp(tm.Running,'on')
        stop(tm);
    end

    running = false;
    btn.Text = 'Start';

    redraw();

end

% =========================================================
% Change algorithm
% =========================================================

    function changeMethod(m)

        method = m;
        restartSearch();

    end

% =========================================================
% Start / Pause
% =========================================================

    function startPause()

    if isempty(searchResult) || searchFinished
        restartSearch();
    end

    if ~running

        running = true;
        btn.Text = 'Pause';

        tm.Period = max(0.05,speed.Value);

        if strcmp(tm.Running,'off')
            start(tm);
        end

    else

        running = false;
        btn.Text = 'Start';

        if strcmp(tm.Running,'on')
            stop(tm);
        end

    end

end

% =========================================================
% Restart search
% =========================================================

    
function restartSearch()

    % Stop previous animation
    if strcmp(tm.Running,'on')
        stop(tm);
    end

    running = false;
    searchFinished = false;
    btn.Text = 'Start';

    % Reset playback
    searchResult = [];
    pathIndex = 1;
    samples = zeros(0,4);

    % Convert current Rx position into a grid index
    rxRelative = rx - [0,0,zStageCentre];

    startIdx = round(rxRelative/step) + 11;
    startIdx = max(1,min(21,startIdx));

    % Snap Rx to the original search grid
    rx = [gridAxis(startIdx(1)), ...
          gridAxis(startIdx(2)), ...
          zStageCentre + gridAxis(startIdx(3))];

    % Original runSearch configuration
    cfg.step = step;
    cfg.maxMoves = 10000;

    cfg.secondsPerStep = [4.5 4.5 4.5];
    cfg.measureSeconds = 0.3;

    cfg.nInitial = 8;
    cfg.maxMeasurements = 60;
    cfg.travelPenaltyS = 30;
    cfg.gpNoise = 0.02;

    cfg.coarsePoints = 5;
    cfg.finePoints = 3;
    cfg.searchZ = true;
    cfg.minStepCm = 100/150*step*100;

    cfg.patternMaxMeasurements = 60;
    cfg.patternInitialStepCm = 7.62;
    cfg.patternMinStepCm = 1.27;
    cfg.patternMaxStepCm = 15.24;
    cfg.patternExpansion = 1.5;
    cfg.patternContraction = 0.5;
    cfg.patternTolerance = 0.01;

    cfg.stopMode = 'none';

    % Execute the ORIGINAL algorithm
    searchResult = runSearch( ...
        method, ...
        allIdx, ...
        gridAxis, ...
        startIdx, ...
        zStageCentre, ...
        tx, ...
        sys, ...
        ch, ...
        cfg);

    % Save the actual trajectory
    pathCm = searchResult.xyzCm;

    % Initialise playback statistics
    elapsedTime = 0;
    totalDistance = 0;

    bestSNR = searchResult.bestSNR;
    bestRx = [ ...
        searchResult.bestXYZcm(1:2)/100, ...
        zStageCentre + searchResult.bestXYZcm(3)/100];

    timeToBest = searchResult.timeToBestSec;

    redraw();

end


% =========================================================
% Reset Rx
% =========================================================

    function resetRx()

        rx = rxStart;
        restartSearch();

    end

% =========================================================
% Animation update
% =========================================================

    

function updateTick()

    if ~isvalid(fig) || isempty(searchResult)
        return;
    end

    if searchFinished
        return;
    end

    % Check if animation is complete
    if pathIndex > size(pathCm,1)

        searchFinished = true;
        running = false;

        if strcmp(tm.Running,'on')
            stop(tm);
        end

        btn.Text = 'Start';

        fprintf('\nSearch complete\n');
        fprintf('Algorithm: %s\n',method);
        fprintf('Best SNR: %.2f dB\n',searchResult.bestSNR);
        fprintf('Measurements: %d\n',searchResult.measurements);
        fprintf('Total time: %.2f s\n',searchResult.totalTimeSec);

        return;
    end

    % Original trajectory coordinates are in cm,
    % relative to the Rx stage centre.
    p = pathCm(pathIndex,:);

    rx = [ ...
        p(1)/100, ...
        p(2)/100, ...
        zStageCentre + p(3)/100];

    % Retrieve ORIGINAL search statistics
    elapsedTime = searchResult.timeSec(pathIndex);
    totalDistance = searchResult.distanceCm(pathIndex)/100;

    % Record trajectory
    samples(end+1,:) = [pathIndex,rx];

    % Advance playback
    pathIndex = pathIndex+1;

    redraw();

end

% =========================================================
% Dynamic multipath channel
% =========================================================

    function snr = dynamicLink(t,r)

    [~,snr] = link(t,r,sys,ch);

end

% =========================================================
% Update graphics
% =========================================================

    function redraw()

        hTx.XData = tx(1)*100;
        hTx.YData = tx(2)*100;
        hTx.ZData = tx(3)*100;

        hRx.XData = rx(1)*100;
        hRx.YData = rx(2)*100;
        hRx.ZData = rx(3)*100;

        hBest.XData = bestRx(1)*100;
        hBest.YData = bestRx(2)*100;
        hBest.ZData = bestRx(3)*100;

        
if ~isempty(searchResult) && ~isempty(samples)

    k = min(pathIndex-1,size(pathCm,1));

    % Animate original Rx trajectory
    hPath.XData = pathCm(1:k,1);
    hPath.YData = pathCm(1:k,2);
    hPath.ZData = pathCm(1:k,3) + 100*zStageCentre;

    % Original SNR measurements
    hS.XData = 1:k;
    hS.YData = searchResult.snr(1:k);

    % Best SNR found so far
    hB.XData = 1:k;
    hB.YData = searchResult.bestSoFar(1:k);

end

% Current SNR
currentSNR = dynamicLink(tx,rx);

% GUI status
if isempty(searchResult)

    measurements = 0;

else

    measurements = sum(~isnan( ...
        searchResult.snr(1:min( ...
        max(pathIndex-1,0),numel(searchResult.snr)))));

end

status.Value = splitlines(sprintf( ...
    ['Algorithm: %s\n' ...
     'Rx: (%.1f, %.1f, %.1f) cm\n' ...
     'Current SNR: %.2f dB\n' ...
     'Best SNR: %.2f dB\n' ...
     'Elapsed Time: %.1f s\n' ...
     'Time to Best: %.1f s\n' ...
     'Distance: %.1f cm\n' ...
     'Measurements logged: %d'], ...
    method, ...
    100*rx, ...
    currentSNR, ...
    bestSNR, ...
    elapsedTime, ...
    timeToBest, ...
    totalDistance*100, ...
    measurements));

    drawnow limitrate;
    
    end

% =========================================================
% Close application
% =========================================================

    function closeApp()

        if strcmp(tm.Running,'on')
            stop(tm);
        end

        delete(tm);
        delete(fig);

    end

end
