#include <math.h>
#include <mex.h>
#include <pthread.h>
#include "qsort.h" /* use a specialized sort routine; faster than the C
                      library's qsort() */
/* #define _REENTRANT */

/* comparator for the specialized sort */
#define islt(a,b) ((*a)<(*b))

/* Sun Dec 10 23:07:30 CST 2006
 * This is a multithreaded version. Since alcor/mizar are dual-threaded
 * dual-processor machines, it appears that 2 threads yield the greatest
 * benefit. Also, even using only one thread this function appears to be much
 * faster than earlier incarnations on which it's based. The two key differences
 * between this version in single-thread mode and earlier versions are:
 *  1. I don't use matlab to supply the uniform random numbers; instead I call
 *     rand().
 *  2. Instead of calling matlab every trial to generate random gaussian
 *     numbers, I generate all that will be needed at once to distribute to the
 *     threads.
 */

/* This code is to speed up the generation of the distribution of KS statistics
 * for arbitrary mixtures of univariate Gaussians. It does about 2 million
 * evaluations per second (n * trials), which is reasonably fast. The fastest I
 * could get matlab to go on its own was about 630,000 evaluations per second,
 * so this is about 3.2 times faster.
 *
 * I have fiddled a lot with it and tried to get rid of the "labels" array, by
 * using the covariance estimator based on E((x-u)^2) = E(x^2) - E(x)^2, but
 * estimating the second moment (E(x^2)) results in really large numbers that
 * aren't nice, and doesn't really speed things up.
 *
 * Other things I've tried include generating my own random numbers inside here
 * instead of calling matlab; matlab calls actually seem to be faster. Also, I
 * tried statically allocating all the short arrays (indexed by k) rather than
 * dynamically allocating them. This didn't seem to make a difference, so I'll
 * stick with the more flexible dynamic allocation.
 *
 * Finally, at this point this code is not the bottleneck for PG-means anymore;
 * the EM algorithm is. So time to work on that!
 */


struct threadData {
    int id, k, n, numThreadTrials;
    double *gaussianData, *priorChooserData;
    double *mus, *sigmas, *priorCdf;

    double *ks_stat_result;
};

void *threadedExperiments(void *arguments) {
    struct threadData *thread_args = (struct threadData *)arguments;
    double *cdf, *estPriors, *estMus, *estSigmas, r, diff1, diff2,
           inv_n, cdfi_times_n_minus_i, maxDiff;
    int trial, i, j, *labels, *priorCounter, trialOffset;

    inv_n = 1.0 / (double)thread_args->n;

    estPriors    = (double *)malloc(sizeof(double) * thread_args->k);
    estMus       = (double *)malloc(sizeof(double) * thread_args->k);
    estSigmas    = (double *)malloc(sizeof(double) * thread_args->k);
    cdf          = (double *)malloc(sizeof(double) * thread_args->n);
    labels       = (int *)malloc(sizeof(int) * thread_args->n);
    priorCounter = (int *)malloc(sizeof(int) * thread_args->k);

    /* run a whole bunch of trials */
    for (trial = 0; trial < thread_args->numThreadTrials; trial++) {
        trialOffset = trial * thread_args->n;

        for (j = 0; j < thread_args->k; j++) {
            estSigmas[j] = estMus[j] = estPriors[j] = 0.0;
            priorCounter[j] = 0;
        }

        /* assign points to clusters and reshape the data and start estimating
         * the means and priors
         */
        for (i = 0; i < thread_args->n; i++) {
            /* FIXME: we may have a non-threadsafe call to rand() here, the best
               way to fix it may be to serially pre-allocate a bunch of random
               numbers, and then divide them up for all the threads.
               */
            /* r = (double)rand() / ((double)RAND_MAX + 1.0); */

            /* this should be thread-safe */
            r = thread_args->priorChooserData[i];
            for (j = 0; j < thread_args->k; j++) {
                if (r < thread_args->priorCdf[j]) {
                    labels[i] = j;
                    thread_args->gaussianData[i + trialOffset] =
                        thread_args->gaussianData[i + trialOffset] * thread_args->sigmas[j]
                        + thread_args->mus[j];
                    estMus[j] += thread_args->gaussianData[i + trialOffset];
                    priorCounter[j]++;
                    break;
                }
            }
        }

        /* estimate the parameters of the model: priors, mus, sigmas */
        for (j = 0; j < thread_args->k; j++) {
            if (priorCounter[j] > 0) {
                estMus[j] /= (double)priorCounter[j];
                estPriors[j] = ((double)priorCounter[j]) * inv_n;
            }
            /* otherwise they will have their default values of 0 */
        }

        /* start estimating sigmas */
        for (i = 0; i < thread_args->n; i++) {
            diff1 = thread_args->gaussianData[i + trialOffset] - estMus[labels[i]];
            estSigmas[labels[i]] += diff1 * diff1;
        }

        for (j = 0; j < thread_args->k; j++) {
            if (priorCounter[j] > 0) {
                /* ok, so this is not actually sigma; it's 1 / (sigma * sqrt(2)),
                 * which will make our erfc calculations faster
                 */
                estSigmas[j] = sqrt(((double)(priorCounter[j] - 1)) / (2.0 * estSigmas[j]));
            }
        }

        for (i = 0; i < thread_args->n; i++) {
            cdf[i] = 0.0;
            for (j = 0; j < thread_args->k; j++) {
                /* this is the most costly section of code, I think, so it needs
                 * to be minimalized as much as possible
                 */
                cdf[i] += estPriors[j] * erfc((estMus[j] - thread_args->gaussianData[i + trialOffset]) * estSigmas[j]);
            }
            cdf[i] = cdf[i] * 0.5; /* lifted out of the inner loop */
        }

        /* sort the cdf so that we can compare it to the empirical cdf */
        QSORT(double, cdf, thread_args->n, islt);

        /* find the ks statistic */
        maxDiff = -1.0;
        for (i = 0; i < thread_args->n; i++) {
            cdfi_times_n_minus_i = cdf[i] * thread_args->n - i;
            diff1 = fabs(cdfi_times_n_minus_i);
            diff2 = fabs(cdfi_times_n_minus_i - 1);

            if (diff1 < diff2) { diff1 = diff2; }
            if (diff1 > maxDiff) { maxDiff = diff1; }
        }

        thread_args->ks_stat_result[trial] = maxDiff * inv_n; /* new way */
    }

    free(cdf);
    free(estMus);
    free(estPriors);
    free(estSigmas);
    free(labels);
    free(priorCounter);
}

/* function ks_stats = fast_ks_simulator(priors, mus, sigmas, n, trials) */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    /* declare lots of variables */
    mxArray *arguments[2], *gaussianDataResult, *priorChooserResult;
    double *priors, *mus, *sigmas, *ks_stats, *gaussianData, *priorChooserData, *argument_data, *priorCdf;
    int k, n, trials, i, j, trialsPerThread;
    const int NUM_THREADS = 2;
    pthread_t threads[NUM_THREADS];
    struct threadData thread_data_args[NUM_THREADS];

    if (nrhs != 5) { return; }

    /* extract all the input... */
    priors = mxGetPr(prhs[0]);
    mus    = mxGetPr(prhs[1]);
    sigmas = mxGetPr(prhs[2]);
    n      = mxGetScalar(prhs[3]);
    trials = mxGetScalar(prhs[4]);

    k = mxGetM(prhs[0]) * mxGetN(prhs[0]); /* we better make sure it's a vector! */

    /* allocate and initialize a few things... */
    priorCdf     = (double *)malloc(sizeof(double) * k);

    plhs[0] = mxCreateDoubleMatrix(trials, 1, mxREAL);
    ks_stats = mxGetPr(plhs[0]);

    /* calculate the cdf of the priors, for the purpose of generating labels */
    priorCdf[0] = priors[0];
    for (i = 1; i < k - 1; i++) {
        priorCdf[i] = priors[i] + priorCdf[i - 1];
    }
    priorCdf[k - 1] = 1.1; /* too large on purpose to catch everything when
                              using random numbers in [0,1] to generate labels */

    trialsPerThread = trials / NUM_THREADS;

   /* use matlab to generate Gaussian random numbers for the dataset */
    arguments[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
    argument_data = mxGetPr(arguments[0]);
    argument_data[0] = trials * n;
    argument_data[1] = 1;

    mexCallMATLAB(1, &gaussianDataResult, 1, arguments, "randn");
    gaussianData = mxGetPr(gaussianDataResult);

    /* use matlab to generate random values for the cluster component choices */
    mexCallMATLAB(1, &priorChooserResult, 1, arguments, "rand");
    priorChooserData = mxGetPr(priorChooserResult);

    for (i = 0; i < NUM_THREADS; i++) {
        thread_data_args[i].id = i;
        thread_data_args[i].k = k;
        thread_data_args[i].n = n;
        thread_data_args[i].n += n % NUM_THREADS;
        thread_data_args[i].numThreadTrials = trialsPerThread;
        if (i + 1 == NUM_THREADS) {
            /* tack on a few extra trials that were not evenly divisible onto
             * the last thread's workload
             */
            thread_data_args[i].numThreadTrials += trials % NUM_THREADS;
        }
        thread_data_args[i].gaussianData = gaussianData + i * n * trialsPerThread;
        thread_data_args[i].priorChooserData = priorChooserData + i * n * trialsPerThread;
        thread_data_args[i].mus = mus;
        thread_data_args[i].sigmas = sigmas;
        thread_data_args[i].priorCdf = priorCdf;
        thread_data_args[i].ks_stat_result = ks_stats + i * trialsPerThread;

        pthread_create(&threads[i], NULL, threadedExperiments, &thread_data_args[i]);
    }

    for (i = 0; i < NUM_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    free(priorCdf);
}

