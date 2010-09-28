function [mus, covariances, priors, ...
          timeSpentClustering, timeSpentTesting, expectations] = ...
            pgmeans_simulation(data, alpha, numProjections)
    
    % initialize the random number generators so that we get reproducible
    % results
    rand('state', 0);
    randn('state', 0);

    numRunsOfEM = 10;
   
    
    [n, d] = size(data);
    %data = data(randperm(n), :); % randomize, for fun

    k = 1;
    mus(k,:) = mean(data);
    
    covariances(:,:,k) = cov(data);
    
    priors(k) = 1;
    acceptedModel = 0;

    timeSpentClustering = 0;
    timeSpentTesting = 0;

    lillie_cv = lilliefors_cv(n, alpha);

    while (~ acceptedModel)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % TESTING
        % check if all projections and tests accept
        tic;
        acceptedModel = 1;
        ksstat_cv_worstLocation = [];
        for projNum = 1:numProjections
            % choose the projection vector
            proj = randn(d, 1);
            proj = proj ./ sqrt(proj' * proj);
           
            projData = data * proj; % project the data
            
            % project the model
            projMus = (mus * proj)';
            projSigmas = zeros(1,k);
            for j = 1:k
                projSigmas(j) = proj' * covariances(:,:,j) * proj;
            end
            projSigmas = sqrt(projSigmas);

            % find the KS statistic
            [ksstat, worst_ksstat_location, worst_pdf_location] = ks_stat(projData, priors, projMus, projSigmas);
            % it might be better to use the worst PDF location for the
            % multivariate model -- but for now I'll try this.

            if (acceptedModel)
                if ((k == 1) || (ksstat > lillie_cv)) 
                    % this is correct for k = 1, and for k > 1, since the
                    % lillie_cv will generally be greater than the true cv, if
                    % ksstat > lillie_cv, then we assume ksstat > true_cv, and
                    % we can avoid running monte carlo simulations
                    if (k > 1)
                        'shortcut!'
                    end
                    cv = lillie_cv;
                else % find the critical value by simulation
                    cv = simulate_ks_cv(alpha, priors, projMus, projSigmas, n);
                end
            else
                cv = 1; % we want to get a bunch of ks statistics for use later;
                        % but we don't want to reject...
            end
            ksstat_cv_worstLocation(end + 1,:) = [ksstat cv worst_pdf_location];

            if (ksstat > cv)
                acceptedModel = 0;
                if (projNum >= numRunsOfEM)
                    break;
                end
            end
        end
        timeSpentTesting = timeSpentTesting + toc;

        if (acceptedModel == 1)
            break;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ADD A NEW CLUSTER AND TRAIN ALL CLUSTERS
        tic;
        best_loglik = -1e300;

        % find the PDF of the dataset; so we can later find the
        % lowest-probability point
        pdf = zeros(n, 1);
        for i = 1:k
            if (priors(i) > 0)
                pdf = pdf + priors(i) * mvnpdf(data, mus(i,:), covariances(:,:,i));
            end
        end

        % determine which points should serve as the initialization for the mean
        % of the new cluster
        %diff_location = [ksstat_cv_worstLocation(:,2) - ksstat_cv_worstLocation(:,1), ksstat_cv_worstLocation(:,3)];
        diff_location = [ksstat_cv_worstLocation(:,2) ksstat_cv_worstLocation(:,3)];

        diff_location = sortrows(diff_location, 1); % THIS IS IMPORTANT -- KEEP THE "1"
        % remove all but the best half of the trials -- replace the remaining
        % with random trials
        diff_location = diff_location(1:ceil(numRunsOfEM/2),:);

        % add some random locations on the rather good chance that we didn't get
        % through enough tests to populate the new set of EM clusterings
        diff_location = [diff_location;
                         ones(5 * numRunsOfEM, 1) ceil(rand(5 * numRunsOfEM, 1) * n)];
        diff_location = sortrows(diff_location, 1); % THIS "1" IS VERY IMPORTANT: DON'T REMOVE IT
        [min_pdf, pointsToTry] = min(pdf);
        %pointsToTry = diff_location(1,2);
        i = 1;
        while (length(pointsToTry) < numRunsOfEM)
            pointsToTry = unique([pointsToTry, diff_location(i, 2)]);
            i = i + 1;
        end

       %% show where the initial clusters will be placed
       %plot_clustering(data, mus, covariances, 1:2);
       %hold on
       %foo = plot(data(pointsToTry,1), data(pointsToTry,2), 'gx');
       %set(foo, 'MarkerSize', 10, 'LineWidth', 5);
       %pause(2);
       %hold off


        % determine the "average" covariance of all existing clusters, to be
        % used to initialize the covariance of the new cluster
        avgCov = 0;
        for i = 1:k
            %avgCov = avgCov + priors(i) * trace(covariances(:,:,i)) / d;
            avgCov = avgCov + trace(covariances(:,:,i)) / d;
            %avgCov = avgCov + covariances(:,:,i) * priors(i);
        end
        % divide by 2 to make it a bit smaller, so it can really stake a claim
        % among the examples -- this division by 2 seems to be extremely
        % important to avoid underfitting
        %avgCov = eye(d) * avgCov / 2; 
        avgCov = eye(d) * (avgCov / k) / 2; 
        %avgCov = eye(d) * (avgCov / k) * 2; 
        %avgCov = cov(data);

        k = k + 1
        
        for i = 1:numRunsOfEM
        %for i = 1:(k - 1)
            mus(k,:) = data(pointsToTry(i),:);
            covariances(:,:,k) = avgCov;
            %mus(k,:) = mus(i,:) + randn(1, d);
            %covariances(:,:,k) = covariances(:,:,i);

            %if (k > 6)
            %  disp(['initial ', num2str(i)]);
            %  plot_clustering(data, mus, covariances);
            %  pause(1);
            %end
            priors(k) = 1/k;
            tempPriors = priors ./ sum(priors);

USE_MY_ALG = 1;

if USE_MY_ALG
            [new_priors, new_mus, new_covariances, expectations, loglik] = ...
                deterministic_annealing_em(data, tempPriors, mus, covariances, 100);

else
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            initModel = [];
            initModel.W = tempPriors';
            initModel.M = mus;
            initModel.R = zeros(k, d * d);
            for cluster = 1:k
                initModel.R(cluster,:) = reshape(chol(covariances(:,:,cluster)), 1, d * d);
            end
            [new_priors, new_mus, new_cov_cholesky, Tlogl, loglik] = em_add(data, [], k, 0, 0, 0, initModel, 100);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
            
            if (loglik > best_loglik)
                best_loglik = loglik;
                best_mus = new_mus;

if USE_MY_ALG
                best_priors = new_priors;
                best_covariances = new_covariances;
else
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                best_priors = new_priors';
                best_covariances = [];
                for cluster = 1:k
                    cov_cholesky = reshape(new_cov_cholesky(cluster,:), d, d);
                    best_covariances(:,:,cluster) = cov_cholesky' * cov_cholesky;
                end
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
            end
        end;
        priors = best_priors;
        mus = best_mus;
        covariances = best_covariances;
        if (length(priors) ~= k)
            ['k was shortened from ', num2str(k), ' to ', num2str(length(priors))]
            k = length(priors);
            'accepting this model because no cluster could be added!'
            break;
        end
        timeSpentClustering = timeSpentClustering + toc;
        %if (k > 15)
        %   ['final for k = ', num2str(k)]
        %   plot_clustering(data, mus, covariances);
        %   pause(1.0)
        %end
    end
    timeSpentTesting
    timeSpentClustering

    %plot_clustering(data, mus, covariances);

    % in case we haven't calculated it yet
    expectations = estep(data, priors, mus, covariances);

    return;

% GREG'S ESTEP FUNCTION JUST TO CALCULATE EXPECTATIONS
function [pcx, loglikelihood] = estep(x, priors, mus, covariances)
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
    %pcx = pcx / temperature;

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

