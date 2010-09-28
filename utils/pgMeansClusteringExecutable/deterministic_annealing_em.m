function [priors, mus, covariances, expectations, loglikelihood, iterations] = ...
    deterministic_annealing_em(x, priors, mus, covariances, maxIterations)
    %temperatures = [1.5 1.25 1];
    %temperatures = [25 5 1];
    %temperatures = [2 1.5 1.1 1];
    temperatures = [1];
    for temperature = temperatures
        %temperature
        [priors, mus, covariances, expectations, loglikelihood, iterations] = ...
        deterministic_annealing_em_em(x, priors, mus, covariances, maxIterations, temperature);
       %plot_clustering(x, mus, covariances);
       %pause(0.1);
    end
%   loglikelihood
%   priors
%   mus
%   covariances
    return;

% Sat Nov 25 14:29:16 CST 2006
% this is a new Gaussian EM method; it should be fairly fast due to some
% optimizations in the estep function that cause the code to proceed
% per-cluster. Note that there are no loops that go over every datapoint (since
% we generally assume that k << n).
function [priors, mus, covariances, expectations, loglikelihood, iterations] = ...
    deterministic_annealing_em_em(x, priors, mus, covariances, maxIterations, temperature)
    if (nargin < 5)
        maxIterations = 100;
    end
    last_loglikelihood = -1000;
    loglikelihood = 0;
    k = length(priors);
    iterations = 0;
    while ((iterations < maxIterations) && ...
           (abs(last_loglikelihood - loglikelihood) > 0.0001))
        iterations = iterations + 1;
        last_loglikelihood = loglikelihood;
        [expectations, loglikelihood] = estep(x, priors, mus, covariances, temperature);
        [priors, mus, covariances] = mstep(x, expectations);
       %plot_clustering(x, mus, covariances);
       %pause(0.1);
    end
    [priors, mus, covariances] = removeEmptyClusters(x, priors, mus, covariances);
    [expectations, loglikelihood] = estep(x, priors, mus, covariances, temperature);
   %plot_clustering(x, mus, covariances);
   %pause(1.1);
    return;

function [pcx, loglikelihood] = estep(x, priors, mus, covariances, temperature)
    [n, d] = size(x);
    k = length(priors);
    pcx = zeros(k, n) - 1e300;
    dlog2pi = d * log(2 * pi);
    % calculate the log probability of each cluster given each datapoint (the
    % expectations). Do this in such a way that avoids underflow by using
    % appropriate log probabilities. This also calculates the log probabilities
    % for all datapoints for one cluster at once, which is fast in matlab-land.
    lastMu = zeros(1,d);
    for i = 1:k
        if (priors(i) > 0)
            for j = 1:d
                x(:,j) = x(:,j) - mus(i,j) + lastMu(j);
            end
            lastMu = mus(i,:);
            preamble = log(priors(i)) - (dlog2pi+log(det(covariances(:,:,i))))/2;
            % THIS NEXT LINE IS THE SECOND MOST COSTLY IN THIS METHOD
            pcx(i,:) = preamble - sum(x*inv(covariances(:,:,i)).*x,2)'/2;
        end
    end

    % use deterministic annealing
    pcx = pcx / temperature;

    % calculate pcx and loglikelihood in a way that avoids underflow, by
    % subtracting off the max log probability before doing sum(exp(logprob))
    maxLogP = max(pcx, [], 1); % the max cluster log-probability for each datapoint
    expLogPcxMinusMax = zeros(k, n);
    for i = 1:k
        % THIS NEXT LINE IS TAKING THE MOST TIME IN THIS FUNCTION
        expLogPcxMinusMax(i,:) = exp(pcx(i,:) - maxLogP); 
    end
    sumPcxMinusMax = sum(expLogPcxMinusMax, 1);
    for i = 1:k
        pcx(i,:) = expLogPcxMinusMax(i,:) ./ sumPcxMinusMax;
    end
    loglikelihood = sum(log(sumPcxMinusMax)) + sum(maxLogP);
    return;

function [priors, mus, covariances] = mstep(x, pcx)
    [n, d] = size(x);
    k = size(pcx, 1);
    sumPcx = sum(pcx, 2)';
    for j = 1:k
        if (sumPcx(j) > 0)
            pcx(j,:) = pcx(j,:) / sumPcx(j);
        end
    end
    priors = sumPcx / n; % calculate the priors
    lastMu = zeros(1,d); % used below in calculation of mu
    mus = zeros(k, d);   % initialization of mus and covariances
    covariances = zeros(d, d, k);
    eps_offset = eps ^ (1/d); % used to fix all-zero covariances

    % calculate the mean and covariance of each cluster
    for i = 1:k
        if (priors(i) > 0)
            % THIS LINE TAKES THE SECOND MOST AMOUNT OF TIME IN THIS FUNCTION
            %mus(i,:) = (pcx(i,:) * x / sumPcx(i)) + lastMu;
            mus(i,:) = (pcx(i,:) * x) + lastMu;
            for j = 1:d
                x(:,j) = x(:,j) - mus(i,j) + lastMu(j);
            end
            lastMu = mus(i,:);

            if (priors(i) >= 2/n)
                % THIS LINE TAKES THE MOST TIME IN THIS FUNCTION -- OVER 50%
               %covariances(:,:,i) = (x.*repmat(pcx(i,:)',1,d))'*x ./ sumPcx(i);
                % THIS VERSION IS A LITTLE FASTER BY AVOIDING THE REPMAT AND
                % SOME DUPLICATE WORK
                for j1 = 1:d
                    xj1 = x(:,j1)' .* pcx(i,:);
                    covariances(j1,j1,i) = xj1 * x(:,j1);
                    for j2 = (j1+1):d
                        covariances(j1,j2,i) = xj1 * x(:,j2);
                        covariances(j2,j1,i) = covariances(j1,j2,i);
                    end
                end

                % if the sample size is too small to use a full covariance, then use
                % spherical covariance
                if (priors(i) <= d/n)
                    % the new covariance volume will be the same as the old one,
                    % but spherical.
                    %   let T = tr(Sigma), then
                    %   old volume == sqrt(T)
                    %   new volume == sqrt(d * (T / d))
                    covariances(:,:,i) = eye(d) * trace(covariances(:,:,i)) / d;
                end
            end
        end

        % calculate the eigenvalues of the covariance for use below
        [eigvecs, eigvals] = eig(covariances(:,:,i));
        eigvals = diag(eigvals);
        min_eigval = min(eigvals);
        max_eigval = max(eigvals);

        if (min_eigval < 0) % sanity check
            'negative eigenvalue!'
            eigvecs
            eigvals
            this_prior = priors(i)
            this_cov = covariances(:,:,i)
           % plot_clustering(x, mus(i,:), covariances(:,:,i));
            error('negative eigenvalue');
        end

        % if the covariance is too eccentric, then smooth it
        cond_limit = 1e5;
        old_cov = covariances(:,:,i);
        if ((min_eigval <= 0) || (max_eigval / min_eigval > cond_limit))
            old_trace = trace(covariances(:,:,i));
            % offset is what we add to the eigenvalues to get the covariance
            % (just) within the limits of acceptable eccentricity, by making the
            % whole thing a little more spherical (along every primary axis)
            offset = (max_eigval + eps_offset - cond_limit * min_eigval) / (cond_limit - 1);
            %covariances(:,:,i) = eigvecs * diag(eigvals + offset) * inv(eigvecs);
            covariances(:,:,i) = covariances(:,:,i) + offset * eye(d);
            % normalize the new covariance so that it has the same "volume" as
            % the original covariance; now it's just a bit more spherical
            if (old_trace > 0)
                new_trace = trace(covariances(:,:,i));
                covariances(:,:,i) = covariances(:,:,i) * old_trace / new_trace;
            end

            if (det(covariances(:,:,i)) <= 0)
                old_cov
                old_det = det(old_cov)
                offset
                eigvecs
                eigvals
                new_cov = covariances(:,:,i)
                new_det = det(new_cov)
                new_eigenvalues = eig(new_cov)
                error('could not fix a broken covariance')
            end
        end

       %if (det(covariances(:,:,i)) < eps)
       %    det(covariances(:,:,i))
       %    'determinant is too small!'
       %end

    end
    return;

% remove any clusters whose priors are too small
function [priors, mus, covariances] = removeEmptyClusters(x, priors, mus, covariances)
    n = size(x, 1);
    properClusters = find(priors >= 0.01);
    if (length(properClusters) ~= length(priors))
        priors = priors(properClusters);
        priors = priors / sum(priors);
        mus = mus(properClusters, :);
        covariances = covariances(:,:,properClusters);
    end
    return;

