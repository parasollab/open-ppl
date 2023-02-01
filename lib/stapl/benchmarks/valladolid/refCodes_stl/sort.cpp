#include <vector>
#include <parallel/algorithm>
#include <iostream>
#include "timer.h"
 

int main(int argc, char *argv[])
{
    int size=atoi( argv[1] );
    int num_threads= atoi( argv[2] );
     omp_set_num_threads(num_threads);

    std::vector<double> nums(size);    
 
    //Init
    std::for_each(nums.begin(), nums.end(), [](double &n){ n=0; });
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
