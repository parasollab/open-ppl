#include <vector>
#include <parallel/algorithm>
#include <iostream>
#include "timer.h"

 
 
bool find_funct(double value)
{return (value>0.5);}

int main(int argc, char *argv[])
{
    int size=atoi( argv[1] );
    int num_threads= atoi( argv[2] );
     omp_set_num_threads(num_threads);

    std::vector<double> nums(size);
    std::vector<double> numsOut(size); 

    //Init
    std::for_each(nums.begin(), nums.end(), [](double &n){ n=4; });
    std::for_each(numsOut.begin(), numsOut.end(), [](double &n){ n=2; });
    for(int i=0; i<size; ++i){
			srand48(i);
			nums[i]=drand48();
    } 

#ifdef DEBUG
    std::cout << "before:";
    for (auto const &n : nums)
    {
        std::cout << ' ' << n;
    }
    std::cout << '\n';
#endif

    start_timer();

    //Apply sort 
    __gnu_parallel::sort(nums.begin(), nums.end());

    //Apply unique_copy
    __gnu_parallel::unique_copy(nums.begin(), nums.end(), numsOut.begin());

    //Apply for-each 
    __gnu_parallel::for_each(numsOut.begin(), numsOut.end(), [](double &n){  int k=0;
									 for(k=0; k<1000; k++){
									  n =n+ (k*k/100) ;
 									  }
 								     });

    //Finally, we will apply a simple "find_if" routine 
    std::vector<double>::iterator it=__gnu_parallel::find_if(numsOut.begin(), numsOut.end(), find_funct );

    double time = stop_timer();

#ifdef DEBUG
    std::cout << "after: ";
    for (double &n : numsOut)
    {
        std::cout << ' ' << n;
    }
    std::cout << '\n';
#endif



    // Result
    std::cout << "The first value is " << *it << '\n';
    std::cout << "Execution time: " << time << '\n';
}
