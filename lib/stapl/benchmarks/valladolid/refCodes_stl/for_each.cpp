#include <vector>
#include <parallel/algorithm>
#include <iostream>
#include "timer.h"
 
 
int main(int argc, char *argv[])
{
    int begin=atoi( argv[1] );
    int end=atoi( argv[2] );
    int size=atoi( argv[3] );
    int num_threads= atoi( argv[4] );
     omp_set_num_threads(num_threads);

    std::vector<double> nums(size);
    
    //Init
    std::for_each(nums.begin(), nums.end(), [](double &n){ n=4; });
    

#ifdef DEBUG
    std::cout << "before:";
    for (auto const &n : nums)
    {
        std::cout << ' ' << n;
    }
    std::cout << '\n';
#endif

    start_timer();
  
    //Apply for-each 
    __gnu_parallel::for_each(nums.begin()+begin, nums.begin()+end, [](double &n){  int k=0;
									 for(k=0; k<1000; k++){
									  n =n+ (k*k/100) ;
 									  }
 								     });

 
    double time = stop_timer();
    std::cout << "Execution time: " << time << '\n';

#ifdef DEBUG
    std::cout << "after: ";
    for (double &n : nums)
    {
        std::cout << ' ' << n;
    }
    std::cout << '\n';
#endif

}
