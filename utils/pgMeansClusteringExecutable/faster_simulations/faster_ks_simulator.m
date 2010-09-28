% simulate the given univariate mixture to find the ks statistics from sample
% estimates for "trials" trials, and return all the statistics calculated
%
% THIS ONE USES MY SPECIAL SAUCE faster_erfc MEX FUNCTION
function ks_stats = fast_ks_simulator(priors, mus, sigmas, n, trials)
    k = length(priors);
    priorCdf = sum(tril(repmat(priors, k, 1))');
    % larger than 1, so we make sure we catch all possible [0,1] probabilities in our labeling net
    priorCdf(end) = 1.1;
    ks_stats = zeros(1, trials);
    ecdf_lower = linspace(0, 1 - 1/n, n)';
    ecdf_upper = linspace(1/n, 1, n)';

    mus = reshape(mus, length(mus), 1);
    sigmas = reshape(sigmas, length(sigmas), 1);

    sqrt2 = sqrt(2);

    for i = 1:trials
        labels = zeros(n, 1);
        cluster_chooser = rand(n, 1);
        for j = 1:k
            labels = labels + (cluster_chooser <= priorCdf(j));
        end
        data = randn(n, 1) .* sigmas(labels) + mus(labels);
        cdf = zeros(n, 1);
        for j = 1:k
            inCluster = find(labels == j);
            estPrior = length(inCluster) / n;
            if (estPrior > 0)
                estMu = mean(data(inCluster));
                estSigma = max([std(data(inCluster)), eps]);
                % 0.5*erfc((x-mu)/(sqrt(2) * sigma)) is exactly what the
                % function normcdf does; I am just inlining it here
                cdf = cdf + estPrior * faster_erfc(-(data - estMu) ./ (estSigma * sqrt2));
            end
        end
        % the 0.5 was lifted out of the erfc calculation above, so we put it back in here
        cdf = sort(cdf * 0.5); 
        ks_stats(i) = max(max(abs([ecdf_lower - cdf, ecdf_upper - cdf])));
    end

