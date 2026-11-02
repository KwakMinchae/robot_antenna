function result = runSearch(method, allPositions, gridAxis, startPosition, zCentre, txPos, sys, channel, cfg)
% Central entry point for every search algorithm.
%
%   methods = runSearch('list');                          % names in the registry
%   result  = runSearch(name, allPositions, gridAxis, startPosition, ...
%                       zCentre, txPos, sys, channel, cfg);
%
% =====================================================================
% HOW IT WORKS (mirrors the Python scripts)
% =====================================================================
% Python:   measure_at(target)  -> move the stage, wait, measure SNR
% MATLAB:   stageMeasureAt(stage, targetCm) does the same in simulation:
%           it walks the stage (X, then Y, then Z), adds the travel and
%           settle time, calls link(), and logs everything.
%
% Every algorithm is a function  result = runXxx(ctx)  that only decides
% WHERE to measure next. Walking, timing, distance, best-so-far and the
% result struct are handled by the stage helpers further down.
%
% =====================================================================
% HOW TO ADD AN ALGORITHM (2 steps)
% =====================================================================
%   1. Write a function in the "ALGORITHMS" section:
%
%        function result = runMySearch(ctx)
%            stage = stageInit(ctx);
%            for n = 1:20
%                target = [0 0 0];                          % cm (X Y Z from stage centre)
%                [stage, snr] = stageMeasureAt(stage, target);
%                % useful: stage.pos, stage.bestPos, stage.bestSNR,
%                %         stage.lowCm, stage.highCm, stage.stepCm
%            end
%            stage  = stageConfirm(stage);                  % go to best and re-measure
%            result = stageResult(stage);
%        end
%
%   2. Add one line to the list in "ALGORITHM LIST":
%
%        'MySearch',  @runMySearch;
%
% HOW TO REMOVE AN ALGORITHM: delete its line in the "ALGORITHM LIST".
%
% Algorithms that move ONE grid step at a time (like Random and Greedy)
% can use the step-search wrapper instead:
%
%        function [nextPosition, memory] = chooseMyAlgorithm(state, memory)
%            nextPosition = state.neighbors(1, :);          % your logic here
%        end
%        'MyAlgorithm',  @(ctx) runStepSearch(ctx, @chooseMyAlgorithm);
%
%   state.currentPosition : [xIndex yIndex zIndex]
%   state.neighbors       : legal one-step moves (one per row)
%   state.move            : move number (0, 1, 2, ...)
%   state.alreadyMeasured : true/false for every row of state.allPositions
%   state.snrAtPosition   : measured SNR for every position (NaN if unmeasured)
%   state.allPositions    : every grid position, one per row
%   state.gridSize        : points per axis (21)
%   state.cfg             : settings from the main script
% =====================================================================
 
    registry = getAlgorithmList();
 
    % ---------- Return the list of names if asked ----------
    if nargin == 1 && strcmpi(method, 'list')
        result = registry(:, 1)';
        return;
    end
 
    % ---------- Find the algorithm ----------
    row = find(strcmpi(registry(:, 1), method), 1);
    if isempty(row)
        error('Unknown search method "%s". Available: %s', method, strjoin(registry(:, 1)', ', '));
    end
 
    % ---------- Everything an algorithm may need ----------
    ctx.allPositions  = allPositions;     % every grid index [ix iy iz], one per row
    ctx.gridAxis      = gridAxis;         % axis values in metres
    ctx.startPosition = startPosition;    % start as grid index [ix iy iz]
    ctx.zCentre       = zCentre;
    ctx.txPos         = txPos;
    ctx.sys           = sys;
    ctx.channel       = channel;
    ctx.cfg           = cfg;
 
    runner = registry{row, 2};
    result = runner(ctx);
end
 
%% ================================================================
% ALGORITHM LIST  (EDIT HERE to add or remove algorithms)
%
% Each line is:   'Name shown in plots',  @functionThatTakesCtx;
% The order here is the order used in the plots.
% ================================================================
function registry = getAlgorithmList()
 
    registry = {
        'Random',        @(ctx) runStepSearch(ctx, @chooseRandom);
        'Greedy',        @(ctx) runStepSearch(ctx, @chooseGreedy);
        'Bayesian',      @runBayesian;
        'CoarseToFine',  @runCoarseToFine;
        'AdaptivePattern', @runAdaptivePattern
    };
 
end
 
%% ================================================================
% ALGORITHMS  (EDIT HERE: one function per algorithm)
% ================================================================
 
% ----------------------------------------------------------------
% Random
% ----------------------------------------------------------------
function [nextPosition, memory] = chooseRandom(state, memory)
 
    choice = randi(size(state.neighbors, 1));
    nextPosition = state.neighbors(choice, :);
 
end
 
% ----------------------------------------------------------------
% Greedy
% ----------------------------------------------------------------
function [nextPosition, memory] = chooseGreedy(state, memory)
 
    gridSize = state.gridSize;
    neighbors = state.neighbors;
 
    neighborNumbers = sub2ind([gridSize gridSize gridSize], neighbors(:, 1), neighbors(:, 2), neighbors(:, 3));
    unmeasuredList = find(~state.alreadyMeasured(neighborNumbers));
 
    if ~isempty(unmeasuredList)
        choice = unmeasuredList(randi(numel(unmeasuredList)));
        nextPosition = neighbors(choice, :);
    else
        [~, bestNeighbor] = max(state.snrAtPosition(neighborNumbers));
        nextPosition = neighbors(bestNeighbor, :);
    end
 
end
 

 
% ================================================================
% BAYESIAN SEARCH 
% ================================================================
function result = runBayesian(ctx)
 
    cfg          = ctx.cfg;
    gridAxis     = ctx.gridAxis;
    allPositions = ctx.allPositions;
 
    nInitial = getSetting(cfg, 'nInitial',        8);
    maxMeas  = getSetting(cfg, 'maxMeasurements', 60);
    penaltyS = getSetting(cfg, 'travelPenaltyS',  30);
    gpNoise  = getSetting(cfg, 'gpNoise',         0.02);
 
    gridSize  = numel(gridAxis);
    N         = size(allPositions, 1);
    candCm    = 100 * gridAxis(allPositions);            % every grid point in cm (GP inputs)
    lsOptions = [1.5 2.5 4 6 9 14];
 
    stage    = stageInit(ctx);
    measured = false(N, 1);
    snrAt    = nan(N, 1);
    order    = zeros(maxMeas, 1);
 
    % Start position is the first initial point; the rest are random
    sp       = ctx.startPosition;
    startIdx = sub2ind([gridSize gridSize gridSize], sp(1), sp(2), sp(3));
    others   = setdiff(1:N, startIdx);
    initialPicks = [startIdx, others(randperm(numel(others), nInitial - 1))];
 
    for n = 1:maxMeas
 
        if n <= nInitial
            idx = initialPicks(n);
        else
            mIdx = find(measured);
            X = candCm(mIdx, :);
            y = snrAt(mIdx);
 
            % pick the length scale with the best fit score
            fit = zeros(size(lsOptions));
            for k = 1:numel(lsOptions)
                [~, ~, fit(k)] = bayesGP(X, y, X(1, :), lsOptions(k), gpNoise);
            end
            [~, kBest] = max(fit);
            ls = lsOptions(kBest);
 
            % guess the map and compute Expected Improvement
            [mu, sd, ~] = bayesGP(X, y, candCm, ls, gpNoise);
            ei = bayesEI(mu, sd, max(y), 0.01);
 
            % prefer points that are quick to reach (axes move one at a time)
            if penaltyS > 0
                travelS = (abs(candCm - stage.pos) / stage.stepCm) * stage.secPerStep';
                ei = ei ./ (1 + travelS / penaltyS);
            end
 
            ei(mIdx) = -1;                               % never re-measure
            [~, idx] = max(ei);
        end
 
        [stage, s] = stageMeasureAt(stage, candCm(idx, :));
        measured(idx) = true;
        snrAt(idx)    = s;
        order(n)      = idx;
    end
 
    stage  = stageConfirm(stage);                        % go to best and re-measure
    result = stageResult(stage);
    result.order = order;
end

function [predMean, predSd, fitScore] = bayesGP(X, y, C, ls, noise)
 
    mu0 = sum(y) / numel(y);
    sd0 = std(y) + 1e-9;
    yn  = (y - mu0) / sd0;
 
    K  = exp(-0.5 * squaredDistances(X, X) / ls^2) + noise * eye(size(X, 1));
    L  = chol(K, 'lower');
    a  = L' \ (L \ yn);
 
    Ks = exp(-0.5 * squaredDistances(C, X) / ls^2);
    v  = L \ Ks';
 
    m        = Ks * a;
    variance = max(1 - sum(v.^2, 1)', 1e-12);
    fitScore = -0.5 * (yn' * a) - sum(log(diag(L)));
 
    predMean = m * sd0 + mu0;
    predSd   = sqrt(variance) * sd0;
end
 

function ei = bayesEI(mu, sd, bestSoFar, xi)
 
    d   = mu - bestSoFar - xi;
    z   = d ./ sd;
    cdf = 0.5 * erfc(-z / sqrt(2));
    pdf = exp(-0.5 * z.^2) / sqrt(2 * pi);
    ei  = d .* cdf + sd .* pdf;
end
 
% ================================================================
% COARSE-TO-FINE SEARCH 
% ================================================================
function result = runCoarseToFine(ctx)
 
    cfg = ctx.cfg;
 
    coarseN = getSetting(cfg, 'coarsePoints', 5);
    fineN   = getSetting(cfg, 'finePoints',   3);
    searchZ = getSetting(cfg, 'searchZ',      true);
 
    stage = stageInit(ctx);
    minStepCm = getSetting(cfg, 'minStepCm', 100/150 * stage.stepCm);   % MIN_STEP: 100 API units
 
    low  = stage.lowCm;
    high = stage.highCm;
    visits = zeros(0, 5);                                % level, x, y, z, snr (like c2f_results.csv)
 
    % Level 1 covers the whole range
    center = (low + high) / 2;
    half   = (high - low) / 2;
    if ~searchZ
        center(3) = stage.pos(3);
        half(3)   = 0;
    end
    num   = coarseN;
    level = 0;
 
    while true
        level   = level + 1;
        spacing = 2 * half(1) / (num - 1);
 
        axesPts = cell(1, 3);
        for a = 1:3
            pts = linspace(center(a) - half(a), center(a) + half(a), num);
            axesPts{a} = uniquetol(min(max(pts, low(a)), high(a)), 1e-9);
        end
        points = snakeOrder(axesPts{1}, axesPts{2}, axesPts{3});
 
        for p = 1:size(points, 1)
            [stage, s] = stageMeasureAt(stage, points(p, :));
            visits(end+1, :) = [level, points(p, :), s];
        end
 
        if spacing <= minStepCm
            break;
        end
 
        % zoom in: next grid spans +/- half the current spacing around the best point
        center = stage.bestPos;
        half   = [spacing/2, spacing/2, searchZ * spacing/2];
        num    = fineN;
    end
 
    stage  = stageConfirm(stage);                        % go to best and re-measure
    result = stageResult(stage);
    result.levels = level;
    result.visits = visits;
end


% ================================================================
% ADAPTIVE PATTERN SEARCH
% ================================================================
function result = runAdaptivePattern(ctx)

    cfg = ctx.cfg;

    % Initialise simulated XYZ stage
    stage = stageInit(ctx);

    % Search parameters
    maxMeas = getSetting(cfg, 'patternMaxMeasurements', 60);

    stepCm = getSetting(cfg, 'patternInitialStepCm', 7.62);

        step0 = stepCm;

    minStepCm = getSetting(cfg, 'patternMinStepCm', 1.27);

    maxStepCm = getSetting(cfg, 'patternMaxStepCm', 15.24);

    expansion = getSetting(cfg, 'patternExpansion', 1.5);

    contraction = getSetting(cfg,'patternContraction', 0.5);

    tolerance = getSetting(cfg, 'patternTolerance', 0.01);

    % Track positions already measured
    stage.reuse = false;

    % Six search directions
    directions = [
         1  0  0;
        -1  0  0;
         0  1  0;
         0 -1  0;
         0  0  1;
         0  0 -1
    ];

    % Measure initial receiver position
    [stage, currentSNR] = ...
        stageMeasureAt(stage, stage.pos);

    currentPos = stage.pos;

    % Store search progress
    iteration = 0;

    while stage.nMeas < maxMeas

        iteration = iteration + 1;

        bestCandidateSNR = currentSNR;
        bestCandidatePos = currentPos;

        % Evaluate neighbouring candidate positions
        for d = 1:size(directions,1)

            if stage.nMeas >= maxMeas
                break;
            end

            candidate = currentPos + ...
                stepCm * directions(d,:);

            % Enforce track boundaries
            candidate = max(candidate,stage.lowCm);
            candidate = min(candidate,stage.highCm);
            

            % Skip current position
            if norm(candidate-currentPos) < 1e-9
                continue;
            end

            % Move and measure SNR
            [stage,snr] = ...
                stageMeasureAt(stage,candidate);

            % Update best candidate
            if snr > bestCandidateSNR + tolerance

                bestCandidateSNR = snr;
                bestCandidatePos = candidate;

            end

        end

        % If an improvement was found
        if bestCandidateSNR > currentSNR + tolerance

            currentPos = bestCandidatePos;
            currentSNR = bestCandidateSNR;

            % Increase search step
            stepCm = min( ...
                stepCm*expansion,maxStepCm);

        else

            % Reduce search step
            stepCm = stepCm*contraction;

        end

            % Resolution too small: restart from a random grid point
        if stepCm < minStepCm
            stepCm = step0;
            gridCm = 100*ctx.gridAxis;
            currentPos = gridCm(randi(numel(gridCm),1,3));
            [stage,currentSNR] = stageMeasureAt(stage,currentPos);
        end

    end

    % Return to best measured position
    stage = stageConfirm(stage);

    % Package results using existing stage helper
    result = stageResult(stage);

    result.iterations = iteration;
    result.finalStepCm = stepCm;

end

%==============================================================================
function pts = snakeOrder(xs, ys, zs)
 
    pts = zeros(0, 3);
    for iz = 1:numel(zs)
        if mod(iz - 1, 2) == 0, yOrder = ys; else, yOrder = fliplr(ys); end
        for iy = 1:numel(yOrder)
            if mod((iy - 1) + (iz - 1), 2) == 0, xOrder = xs; else, xOrder = fliplr(xs); end
            for x = xOrder
                pts(end+1, :) = [x, yOrder(iy), zs(iz)];
            end
        end
    end
end
 
%% ================================================================
% SIMULATED STAGE 
% ================================================================
function stage = stageInit(ctx)
 
    cfg = ctx.cfg;
    secPerStep = getSetting(cfg, 'secondsPerStep', [1 1 1]);
    if numel(secPerStep) == 1, secPerStep = [1 1 1] * secPerStep; end
 
    stage.zCentre    = ctx.zCentre;
    stage.txPos      = ctx.txPos;
    stage.sys        = ctx.sys;
    stage.channel    = ctx.channel;
    stage.stepCm     = cfg.step * 100;                   % one grid step in cm
    stage.secPerStep = secPerStep(:)';                   % seconds per grid step, X Y Z
    stage.settleS    = getSetting(cfg, 'measureSeconds', 0);
    stage.lowCm      = 100 * ctx.gridAxis(1)   * [1 1 1];
    stage.highCm     = 100 * ctx.gridAxis(end) * [1 1 1];
 
    stage.pos        = 100 * ctx.gridAxis(ctx.startPosition);   % current position, cm
    stage.elapsed    = 0;
    stage.distCm     = 0;
    stage.bestSNR    = -inf;
    stage.bestPos    = stage.pos;
    stage.timeToBest = 0;
    stage.nMeas      = 0;
 
    % reuse = true: a position measured before is not measured again
    stage.reuse = false;
    stage.cache = containers.Map('KeyType', 'char', 'ValueType', 'double');
 
    % Logs: one row per grid step walked (first row = start position)
    stage.pathCm  = stage.pos;
    stage.snrLog  = NaN;
    stage.distLog = 0;
    stage.timeLog = 0;
end
 
function [stage, snr] = stageMeasureAt(stage, targetCm)
 
    stage = stageWalk(stage, targetCm);
 
    key = sprintf('%.6f_%.6f_%.6f', targetCm(1), targetCm(2), targetCm(3));
 
    if stage.reuse && isKey(stage.cache, key)
        snr = stage.cache(key);                          % visited before: reuse, no time
    else
        stage.elapsed = stage.elapsed + stage.settleS;
        rxPos = [targetCm(1)/100, targetCm(2)/100, stage.zCentre + targetCm(3)/100];
        [~, snr] = link(stage.txPos, rxPos, stage.sys, stage.channel);
        stage.nMeas = stage.nMeas + 1;
        if stage.reuse
            stage.cache(key) = snr;
        end
    end
 
    if snr > stage.bestSNR
        stage.bestSNR    = snr;
        stage.bestPos    = targetCm;
        stage.timeToBest = stage.elapsed;
    end
 
    % the last log row is the destination: fill in what was measured
    stage.snrLog(end)  = snr;
    stage.timeLog(end) = stage.elapsed;
end
 
function stage = stageWalk(stage, targetCm)
 
    pos = stage.pos;
    for a = 1:3
        nSteps = ceil(abs(targetCm(a) - pos(a)) / stage.stepCm - 1e-9);
        for k = 1:nSteps
            moved = min(stage.stepCm, abs(targetCm(a) - pos(a)));
            if k == nSteps
                newVal = targetCm(a);                    % land exactly, no float drift
            else
                newVal = pos(a) + sign(targetCm(a) - pos(a)) * moved;
            end
            stage.elapsed = stage.elapsed + stage.secPerStep(a) * moved / stage.stepCm;
            stage.distCm  = stage.distCm + moved;
            pos(a) = newVal;
 
            stage.pathCm(end+1, :)  = pos;
            stage.snrLog(end+1, 1)  = NaN;
            stage.distLog(end+1, 1) = stage.distCm;
            stage.timeLog(end+1, 1) = stage.elapsed;
        end
    end
    stage.pos = targetCm;
end
 
function stage = stageConfirm(stage)
 
    nBefore = stage.nMeas;
    [stage, ~] = stageMeasureAt(stage, stage.bestPos);
    stage.nMeas = nBefore;
end
 
function result = stageResult(stage)
 
    % best SNR so far on every row (NaN until the first measurement)
    b = stage.snrLog;
    b(isnan(b)) = -inf;
    b = cummax(b);
    b(isinf(b)) = NaN;
 
    result.xyzCm         = stage.pathCm;
    result.snr           = stage.snrLog;
    result.bestSoFar     = b;
    result.distanceCm    = stage.distLog;
    result.bestSNR       = stage.bestSNR;
    result.bestXYZcm     = stage.bestPos;
    result.measurements  = stage.nMeas;
    result.movesUsed     = size(stage.pathCm, 1) - 1;
    result.timeSec       = stage.timeLog;
    result.totalTimeSec  = stage.elapsed;
    result.timeToBestSec = stage.timeToBest;
end
 
%% ================================================================
% SMALL HELPERS
% ================================================================
% Read an optional setting from cfg, with a default
function value = getSetting(cfg, name, default)
 
    if isfield(cfg, name)
        value = cfg.(name);
    else
        value = default;
    end
end
 
% All legal one-step neighbours of a grid position
function neighbors = getNeighbors(position, gridSize)
 
    % +X, -X, +Y, -Y, +Z, -Z
    directions = [ 1  0  0;
                  -1  0  0;
                   0  1  0;
                   0 -1  0;
                   0  0  1;
                   0  0 -1];
 
    neighbors = position + directions;
 
    % Keep only neighbours that are inside the grid
    insideGrid = all(neighbors >= 1 & neighbors <= gridSize, 2);
    neighbors = neighbors(insideGrid, :);
end
 
% Squared distance between every point in A and every point in B
% Uses: |a - b|^2 = |a|^2 + |b|^2 - 2*a.b
function distSq = squaredDistances(A, B)
 
    sumSqA = sum(A.^2, 2);        % column: |a|^2 for each row of A
    sumSqB = sum(B.^2, 2)';       % row:    |b|^2 for each row of B
    crossTerm = 2 * (A * B');
 
    distSq = sumSqA + sumSqB - crossTerm;
 
    % Tiny negative values can appear from rounding, so clip them to 0
    distSq = max(distSq, 0);
end

% ----------------------------------------------------------------
% Step search: runs any "choose" function one grid step at a time and
% measures at every position it lands on (revisits reuse the stored value).
% ----------------------------------------------------------------
function result = runStepSearch(ctx, chooseFcn)

cfg          = ctx.cfg;
gridAxis     = ctx.gridAxis;
allPositions = ctx.allPositions;
gridSize     = numel(gridAxis);
numPositions = size(allPositions, 1);

maxMoves = getSetting(cfg, 'maxMoves', 10000);
stopMode = getSetting(cfg, 'stopMode', 'none');      % 'none' or 'patience'
patience = getSetting(cfg, 'patience', 50);

stage = stageInit(ctx);
stage.reuse = true;                                  % revisits cost no time

alreadyMeasured = false(numPositions, 1);
snrAtPosition   = nan(numPositions, 1);
memory = struct();

state.allPositions = allPositions;
state.gridSize     = gridSize;
state.cfg          = cfg;

currentPosition     = ctx.startPosition;
lastImprovementMove = 0;

for move = 0:maxMoves

    % ----- measure at the current position -----
    bestBefore = stage.bestSNR;
    [stage, s] = stageMeasureAt(stage, 100 * gridAxis(currentPosition));

    positionNumber = sub2ind([gridSize gridSize gridSize], currentPosition(1), currentPosition(2), currentPosition(3));
    alreadyMeasured(positionNumber) = true;
    snrAtPosition(positionNumber)   = s;

    if stage.bestSNR > bestBefore
        lastImprovementMove = move;
    end

    % ----- stop rules -----
    if move == maxMoves
        break;
    elseif strcmp(stopMode, 'patience') && (move - lastImprovementMove >= patience)
        break;
    end

    % ----- ask the algorithm for the next position -----
    state.move            = move;
    state.currentPosition = currentPosition;
    state.neighbors       = getNeighbors(currentPosition, gridSize);
    state.alreadyMeasured = alreadyMeasured;
    state.snrAtPosition   = snrAtPosition;

    [nextPosition, memory] = chooseFcn(state, memory);

    if sum(abs(nextPosition - currentPosition)) ~= 1
        error('Illegal move: must move exactly one grid step.');
    end
    if any(nextPosition < 1) || any(nextPosition > gridSize)
        error('Receiver left the track.');
    end
    currentPosition = nextPosition;
end

result = stageResult(stage);
end
