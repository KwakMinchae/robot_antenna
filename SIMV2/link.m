function [Pr,SNR,h] = link(tx,rx,sys,ch)

[Pr,SNR,h] = calculateLink( ...
    tx,rx,sys.fc,sys.Pt,sys.Gt,sys.Gr, ...
    sys.loss,sys.noise,ch);

end

function [Pr,SNR,h] = calculateLink( ...
    tx,rx,fc,Pt,Gt,Gr,loss,noise,ch)

c = 299792458;
lambda = c/fc;

% Direct Tx-Rx path
d = max(norm(rx-tx),1e-6);

direct = ...
    lambda/(4*pi*d) * ...
    exp(-1j*2*pi*d/lambda);

%% LOS only
if strcmp(ch.environment,'LOS')

    h = direct;

else

    % Receiver displacement from phase-reference position
    displacement = ...
        rx(:)-ch.referencePosition(:);

    hNLOS = 0;

    % Sum all reflected paths
    for l = 1:ch.numPaths

        az = ch.az(l);
        el = ch.el(l);

        % 3-D arrival direction
        u = [ ...
            cos(el)*cos(az);
            cos(el)*sin(az);
            sin(el)];

        % Phase change caused by Rx movement
        positionPhase = exp( ...
            -1j*2*pi/lambda * ...
            dot(u,displacement));

        % Reflected propagation distance
        dp = d + ch.extraDistance(l);

        % Reflected path
        hNLOS = hNLOS + ...
            ch.alpha(l) * ...
            lambda/(4*pi*dp) * ...
            exp(-1j*2*pi*dp/lambda) * ...
            positionPhase;

    end

    %% Rician multipath
    if strcmp(ch.environment,'MULTIPATH')

        h = sqrt(ch.K/(ch.K+1))*direct + sqrt(1/(ch.K+1))*ch.multipathScale*hNLOS;

        %% NLOS only
    else

        h = ch.multipathScale*hNLOS;

    end

end

% Link budget
Pr = ...
    Pt + Gt + Gr + ...
    20*log10(max(abs(h),1e-15)) - loss;

SNR = Pr-noise;

end