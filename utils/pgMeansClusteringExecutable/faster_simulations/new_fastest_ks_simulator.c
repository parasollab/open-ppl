#include <math.h>
#include <mex.h>
#include "qsort.h"

#define islt(a,b) ((*a)<(*b))

/* I tried using my own random number generators to make things faster, but
 * calling matlab appears to be faster, and is more reliable
 */
/* adapted from http://www.taygeta.com/random/gaussian.html */
double gaussian() {
    /* this method generates two normal variates (y1,y2) each time it runs the
     * loop, but we can only return one. Therefore, we use the static x1 as a
     * flag for indicating whether we should loop or just return an
     * already-calculated value.
     */
    static double x1 = 0.0, x2, w, y1, y2;
    if (x1 == 0.0) {
        do {
            x1 = 2.0 * ((double)rand() / ((double)RAND_MAX + 1.0)) - 1.0;
            x2 = 2.0 * ((double)rand() / ((double)RAND_MAX + 1.0)) - 1.0;
            w = x1 * x1 + x2 * x2;
        } while (w >= 1.0);
        w = sqrt((-2.0 * log(w)) / w);
        y1 = x1 * w;
        y2 = x2 * w;
        x1 = 1.0;
    } else {
        y1 = y2;
        x1 = 0.0;
    }

    return y1;
}

/* the purpose of this function is to generate the distribution of the
 * Kolmogorov-Smirnov statistic of a univariate mixture of Gaussians, and
 * hopefully to do it very quickly.
 *
 * Further improvements may be possible, by generating our own random numbers,
 * rather than calling back to Matlab to achieve this. However, right now it's
 * working and it's fast, so I'm going to keep it as-is. It's unclear to me
 * how to reliably generate random numbers in here with random seeds. Oh! I
 * could just use Matlab's random number generator to get a seed.
 */

/* function ks_stats = fast_ks_simulator(priors, mus, sigmas, n, trials) */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    mxArray *mxPriors, *mxMus, *mxSigmas, *mxN, *mxTrials;
    double *priors, *mus, *sigmas, *ks_stats;
    double *priorCdf, *data, *cdf;
    double r, cdfi_times_n_minus_i, inv_n;
    int k, n, trials;
    int i, j, trial;

    mxArray *arguments[2];
    mxArray *dataResult, *clusterChooserResult;
    double *argument_data, *clusterChooser;
    double *estPriors, *estMus, *estSigmas, *estSecondMoment;
    double maxDiff, diff1, diff2;
    int *labels, *priorCounter;

    if (nrhs != 5) { return; }

    /* extract all the input... */
    mxPriors = (mxArray *)prhs[0];
    mxMus    = (mxArray *)prhs[1];
    mxSigmas = (mxArray *)prhs[2];
    mxN      = (mxArray *)prhs[3];
    mxTrials = (mxArray *)prhs[4];

    priors = mxGetPr(mxPriors);    
    mus    = mxGetPr(mxMus);       
    sigmas = mxGetPr(mxSigmas);    
    n      = mxGetPr(mxN)[0];      
    trials = mxGetPr(mxTrials)[0]; 
    /* printf("n = %d\n", n);
    printf("trials = %d\n", trials); */

    inv_n = 1.0 / (double)n;

    k = mxGetM(mxPriors) * mxGetN(mxPriors); /* we better make sure it's a vector! */
    /*printf("k = %d\n", k); */

    /* calculate the cdf of the priors, for the purpose of generating labels */
    priorCdf = (double *)malloc(sizeof(double) * k);
    priorCdf[0] = priors[0];
    for (i = 1; i < k - 1; i++) {
        priorCdf[i] = priors[i] + priorCdf[i - 1];
    }
    priorCdf[k - 1] = 1.1; /* too large on purpose to catch everything when
                              using random numbers in [0,1] to generate labels */

    /* allocate and initialize a few things... */
    estPriors       = (double *)malloc(sizeof(double) * k);
    estMus          = (double *)malloc(sizeof(double) * k);
    estSigmas       = (double *)malloc(sizeof(double) * k);
    estSecondMoment = (double *)malloc(sizeof(double) * k);
    cdf             = (double *)malloc(sizeof(double) * n);
    priorCounter    = (int *)malloc(sizeof(int) * k);
    /* labels          = (int *)malloc(sizeof(int) * n); */
    /* allocate data if we are not getting it from matlab */
    /*data         = (double *)malloc(sizeof(double) * n); */

    plhs[0] = mxCreateDoubleMatrix(trials, 1, mxREAL);
    ks_stats = mxGetPr(plhs[0]);

    /* everything we call in matlab we pass the same arguments to */
    arguments[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
    argument_data = mxGetPr(arguments[0]);
    argument_data[0] = n;
    argument_data[1] = 1;

    /* run a whole bunch of trials */
    for (trial = 0; trial < trials; trial++) {
        printf("trial = %d\n", trial);
        for (j = 0; j < k; j++) {
            estSigmas[j] = estMus[j] = estPriors[j] = estSecondMoment[j] = 0.0;
            priorCounter[j] = 0;
        }

        /* use matlab to generate random gaussian data */
        mexCallMATLAB(1, &dataResult, 1, arguments, "randn");
        data = mxGetPr(dataResult);

        /*printf("trial = %d\n", trial); */
        /* use matlab to generate random numbers for the label generation */
        mexCallMATLAB(1, &clusterChooserResult, 1, arguments, "rand");
        clusterChooser = mxGetPr(clusterChooserResult);
        for (i = 0; i < n; i++) {
            /* labels[i] = -1; */
            /*r = (double)rand() / ((double)RAND_MAX + 1.0);*/
            for (j = 0; j < k; j++) {
                if (clusterChooser[i] < priorCdf[j]) {
                /*if (r < priorCdf[j]) {*/
                    /* labels[i] = j; */
                    data[i] = data[i] * sigmas[j] + mus[j];
                    estMus[j] += data[i];
                    estSecondMoment[j] += data[i] * data[i];
                    priorCounter[j]++;
                    break;
                }
            }
        }

        /* estimate the parameters of the model: priors, mus, sigmas */
        for (j = 0; j < k; j++) {
            if (priorCounter[j] > 0) {
                estMus[j] /= (double)priorCounter[j];
                estPriors[j] = ((double)priorCounter[j]) * inv_n;
                estSigmas[j] = (estSecondMoment[j] * inv_n) - estMus[j] * estMus[j];
                estSigmas[j] = sqrt(((double)(priorCounter[j] - 1)) / (2.0 * estSigmas[j]));
                /* otherwise they will have their default values of 0 */
            }
        }

        /*
        for (i = 0; i < n; i++) {
            diff1 = data[i] - estMus[labels[i]];
            estSigmas[labels[i]] += diff1 * diff1;
        }
        */

#if 0
        for (j = 0; j < k; j++) {
            /* ok, so this is not actually sigma; it's 1 / (sigma * sqrt(2)),
             * which will make our erfc calculations faster
             * also, we add 1e-8 to avoid divide by zero bugs
             */
            /* old way */
            /* estSigmas[j] = 1.0 / sqrt(2.0 * estSigmas[j] / ((double)priorCounter[j] - 1.0 + 1e-8)); */

            /* new way -- if priorCounter[j] == 0, then estSigmas[j] should also
             * remain zero */
            if (priorCounter[j] > 0) {
                estSigmas[j] = sqrt(((double)(priorCounter[j] - 1)) / (2.0 * estSigmas[j]));
            }
        }
#endif

        for (i = 0; i < n; i++) {
            cdf[i] = 0.0;
            for (j = 0; j < k; j++) {
                /* this is the most costly section of code, I think, so it needs
                 * to be minimalized as much as possible
                 */
                cdf[i] += estPriors[j] * erfc((estMus[j] - data[i]) * estSigmas[j]);
            }
            cdf[i] = cdf[i] * 0.5; /* lifted out of the inner loop */
        }

        /* sort the cdf so that we can compare it to the empirical cdf */
        QSORT(double, cdf, n, islt);

        /* find the ks statistic */
        maxDiff = -1.0;
        for (i = 0; i < n; i++) {
            /*
            if (isnan(cdf[i])) { printf("cdf[%d] is NaN\n", i); }
            if (isinf(cdf[i])) { printf("cdf[%d] is Inf\n", i); }
            */

            /* old way -- correct way! */
            /*
            diff1 = fabs(cdf[i] - (double)i / (double)n);
            diff2 = fabs(cdf[i] - (double)(i + 1) / (double)n);
            */

            /* new way -- avoids division, faster (?) but also correct */
            cdfi_times_n_minus_i = cdf[i] * n - i;
            diff1 = fabs(cdfi_times_n_minus_i);
            diff2 = fabs(cdfi_times_n_minus_i - 1);

            if (diff1 < diff2) {
                diff1 = diff2;
            }
            if (diff1 > maxDiff) {
                maxDiff = diff1;
            }
        }

        /* old way -- correct */
        /* ks_stats[trial] = maxDiff; */

        /* new way -- faster (?) but also correct */
        ks_stats[trial] = maxDiff * inv_n;
    }

    /* free(data); */
    /* free(labels); */
    free(cdf);
    free(estMus);
    free(priorCdf);
    free(estPriors);
    free(estSigmas);
    free(priorCounter);
    free(estSecondMoment);
}

/*
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

    */
