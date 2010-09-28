% adapted from Dallal and Wilkinson: "An analytic approximation to the
% distribution of Lilliefor's test statistic for normality" (American
% Statistician, 1986)
function dmax = lilliefors_cv(n, alpha)
    if (n <= 100)
        dmax = 1.0;
        deltaDmax = 0.5;
        for i = 1:40
            a = exp(-7.01256 * dmax^2 * (n + 2.78019)  ...
                    + 2.99587 * dmax * (n + 2.78019) ^ (1/2) ...
                    - .122119 + 0.974598/(sqrt(n)) + 1.67997/n);
            diff = a - alpha;
            dmax;
            if (a > alpha)
                dmax = dmax + deltaDmax;
            else
                dmax = dmax - deltaDmax;
            end
            deltaDmax = deltaDmax / 2;
        end
    else
        dmax = 1.0;
        deltaDmax = 0.5;
        for i = 1:40
            a = exp(-7.01256 * (dmax*(n/100)^(0.49))^2 * (100 + 2.78019)  ...
                    + 2.99587 * (dmax*(n/100)^(0.49)) * (100 + 2.78019) ^ (1/2) ...
                    - .122119 + 0.974598/(sqrt(100)) + 1.67997/100);
            diff = a - alpha;
            dmax;
            if (a > alpha)
                dmax = dmax + deltaDmax;
            else
                dmax = dmax - deltaDmax;
            end
            deltaDmax = deltaDmax / 2;
        end
    end

