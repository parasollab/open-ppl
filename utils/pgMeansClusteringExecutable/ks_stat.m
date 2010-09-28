% calculate the CDF of the data according to the priors, means, and standard
% deviations, and then calculate the KS statistic for the whole thing
%
% Parameters:
%   - data: the vector dataset of size 1xn
%   - priors: the k cluster priors
%   - mus: the k cluster means
%   - sigmas: the k cluster standard deviations
%
% Return values:
%   - ksstat: the Kolmogorov-Smirnov test statistic
%   - ksstat_location: the index into data where the largest difference was detected
%   - lowest_pdf_location: the index into data which has the smallest assigned
%                          probability density
%
function [ksstat, ksstat_location, lowest_pdf_location] = ks_stat(data, priors, mus, sigmas)
    k = length(priors);
    n = length(data);
    [data, order] = sort(reshape(data, n, 1)); % if this fails, then data has incorrect format

    % calculate the CDF
    cdf = zeros(size(data));
    pdf = zeros(size(data));
    for i = 1:k
        if (priors(i) > 0)
            cdf = cdf + priors(i) * normcdf(data, mus(i), sigmas(i));
            pdf = pdf + priors(i) * normpdf(data, mus(i), sigmas(i));
        end
    end

    [lower, lowerLocation] = max(abs(cdf - linspace(0, 1 - 1/n, n)'));
    [upper, upperLocation] = max(abs(cdf - linspace(1/n, 1, n)'));
    if (lower > upper)
        ksstat_location = order(lowerLocation);
        ksstat = lower;
    else
        ksstat_location = order(upperLocation);
        ksstat = upper;
    end
    [lowest_pdf, lowest_pdf_location] = min(pdf);
    lowest_pdf_location = order(lowest_pdf_location);

    %[h, p, ksstat, ks_cv] = kstest(data, [data, cdf]);
    return;

