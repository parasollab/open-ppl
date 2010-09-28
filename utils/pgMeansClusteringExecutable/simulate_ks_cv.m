function cvs = simulate_ks_cv(alphas, priors, mus, sigmas, n)
    k = length(priors);
    minAlpha = min(alphas);
    numTrials = max([3 * ceil(1/minAlpha), 2000]);
    simulation_n = k * 20;
    if (n < simulation_n)
        simulation_n = n;
    end
    %stats = sort(fast_ks_simulator(priors, mus, sigmas, simulation_n, numTrials));
    %stats = sort(fastest_ks_simulator(priors, mus, sigmas, simulation_n, numTrials));
    stats = sort(multithread_ks_simulator(priors, mus, sigmas, simulation_n, numTrials));
    cvs = stats(ceil(numTrials * (1 - alphas)));
    cvs = cvs .* sqrt(simulation_n / n);
   %for i = 1:length(cvs)
   %    cvs(i) = dallal_convert(cvs(i), simulation_n, n);
   %end
    return;

    

%function cv = simulate_ks_cv(alpha, priors, mus, sigmas, n, multiplier, useFullN)
%    if (nargin < 6)
%        multiplier = 3;
%    end
%    if (nargin < 7)
%        useFullN = 0;
%    end
%
%    last_cv = 0.001;
%    cv = 2;
%    ksstats = [];
%    priors = priors ./ sum(priors);
%    test_chunk_size = multiplier * ceil(1 / alpha);
%    if (useFullN)
%        simulation_n = n;
%    else
%        simulation_n = 100;
%    end
%    %simulation_n = ceil(length(priors) * 20 / min(priors));
%    while ((abs(last_cv - cv) ./ last_cv) > 0.001);
%        last_cv = cv;
%        for t = 1:test_chunk_size
%            [x, labels, estPriors, estMus, estSigmas] = gen_1d_gaussians(priors, mus, sigmas, simulation_n);
%            % should I use the generating mus/sigmas/priors, the estimated
%            % mus/sigmas/priors, or something completely different?
%            ksstats = [ksstats, ks_stat(x, estPriors, estMus, estSigmas)];
%            %ksstats = [ksstats, ks_stat(x, priors, mus, sigmas)];
%        end
%        %test_chunk_size = 100; % after the first time through, use a chunk size of 100
%        ksstats = sort(ksstats);
%        cv = ksstats(ceil((1 - alpha) * length(ksstats)));
%    end
%    numSimulations = length(ksstats)
%    cv = cv * sqrt(simulation_n) / sqrt(n);
%    return;
%
