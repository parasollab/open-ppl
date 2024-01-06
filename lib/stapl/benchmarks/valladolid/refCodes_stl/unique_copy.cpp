#include <vector>
#include <parallel/algorithm>
#include <iostream>
#include "timer.h"

 
int main(int argc, char *argv[])
{
    int size=atoi( argv[1] );
    int num_threads= atoi( argv[2] );
    int num_duplicates = argc == 4 ? atoi(argv[3]) : 27;
     omp_set_num_threads(num_threads);

    std::vector<double> nums(size); 
    std::vector<double> numsOut(size); 
    
    //Init
    std::for_each(nums.begin(), nums.end(), [](double &n){ n=24; });
    for(int i=0; i<size; ++i){
        if( i> num_duplicates-1){
          nums[i]=i;
        }
	numsOut[i]=2;
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

    //Apply unique_copy
    __gnu_parallel::unique_copy(nums.begin()+2, nums.end()-4, numsOut.begin());

    double time = stop_timer();
    std::cout << "Execution time: " << time << '\n';

#ifdef DEBUG
    std::cout << "after: ";
    for (double &n : numsOut)
    {
        std::cout << ' ' << n;
    }
    std::cout << '\n';
#endif

}
