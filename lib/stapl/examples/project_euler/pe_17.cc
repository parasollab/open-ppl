#include <iostream>
#include <string>
#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/runtime.hpp>

typedef long t_type;
typedef stapl::array<t_type> ary_t_tp;
typedef stapl::array_view<ary_t_tp> ary_t_vw_tp;

//Converts symbolic number to its word letter count value
//Ex. 1 which is "one" would return 3 (The number of letters in the word)
struct num_to_wd
{
 // Function takes map as argument
 num_to_wd(std::map<t_type,t_type> const& map)
   :m_map(map)
 {}

 // Class functions
 // Handle three digit numbers
 t_type tn_hn(t_type i);
 t_type hn_thn(t_type i);

 template<typename T>
 t_type operator()(T i)
 {
   //initial letter count
   t_type letter_count=0;

   // 1st 3 digits (ex. xxxxxxXXX)
   t_type huns=i%1000;

   // 2nd 3 digits (ex. xxxXXXxxx)
   t_type thsns=(i%1000000)/1000;

   // 3rd 3 digits (ex. XXXxxxxxx)
   t_type mills=(i%1000000000)/1000000;

   // 4th 3 digits (ex. XXXxxxxxxxxx)
   t_type bills=i/1000000000;

   // Cases supported by range
   // 0 to 1 hundred
   if(0<=i && i<100){
     letter_count += tn_hn(i);
     return letter_count;
   }

   // 1 hundred to 1 thousand
   else if(100<=i && i<1000){
     letter_count += hn_thn(huns);
     return letter_count;
   }

   // 1 thousand to 1 million
   else if(1000<=i && i<1000000)
   {
     letter_count += hn_thn(thsns);
     letter_count += m_map[1000];
     letter_count += hn_thn(huns);
     return letter_count;
   }

   // 1 million to 1 billion
   else if(1000000<=i && i<1000000000)
   {
     letter_count += hn_thn(mills);
     letter_count += m_map[1000000];
     letter_count += hn_thn(thsns);
     if(thsns!=0)
     {
         letter_count += hn_thn(thsns);
         letter_count += m_map[1000];
     }
     letter_count += hn_thn(huns);
     return letter_count;
   }

   // 1 billion to 1 trillion
   else if(1000000000<=i && i<1000000000000)
   {
     letter_count += hn_thn(bills);
     letter_count += m_map[1000000000];
     letter_count += hn_thn(mills);
     if (mills!=0)
     {
         letter_count += hn_thn(mills);
         letter_count += m_map[1000000];
     }
     if (thsns!=0)
     {
         letter_count += hn_thn(thsns);
         letter_count += m_map[1000];
     }
     letter_count += hn_thn(huns);
     return letter_count;
   }
   else
     return 0;
 }

 void define_type(stapl::typer& t)
 {
   t.member(m_map);
 }

private:
 std::map<t_type,t_type> m_map;
};

//Takes in 1 to 2 digit number
//Converts to letter count values
//Handles conversions in the range of 1 - 99
t_type num_to_wd::tn_hn(t_type i)
{

 t_type ones=i%10; // stores 1st digit (ex. xX)
 t_type tens=(i/10)*10; //stores 2nd digit (ex. Xx)

 if (i < 20) // Special cases (1-20) (ex. 11="eleven")
  return  m_map[i];
 else // all other cases
  return  m_map[tens] + m_map[ones];
}

//Takes in 1 to 3 digit number
//Converts to letter count values
//Handles numbers in the range of 1 - 999 with help of previous function
t_type num_to_wd::hn_thn(t_type i)
{
 // Stores number in tens and units place
 t_type tens=i%100;

 //Stores number in hundreds place
 t_type huns=(i/100);

 t_type temp;

 // Checks for cases:
 //  1. If number is in the range of 1 - 99
 //  2. If the number is an even hundred number (ex. 100, 200, 300)
 //  3. All other cases
 if (huns==0)
 {
   temp = tn_hn(tens);
   return temp;
 }
 else if (tens==0)
   return m_map[huns] + m_map[100];
 else
 {
   temp = tn_hn(tens);
   return m_map[huns] + m_map[100] + temp;
 }
}

stapl::exit_code stapl_main(int argc, char **argv)
{

 t_type n = std::stoll(argv[1]);

 //Exit if argument is greater than 1 trillion.
 if (n >= 1000000000000)
 {
   std::cout << "Input size must be less than 1 trillion." << std::endl;
   return EXIT_FAILURE;
 }


 stapl::counter<stapl::default_timer> exec_timer;
 exec_timer.start();

 // Map stores numbers as keys and their letter counts as values
 std::map<t_type, t_type> nums;
 nums[0]=0; // empty
 nums[1]=3; // one
 nums[2]=3; // two
 nums[3]=5; // three
 nums[4]=4; // four
 nums[5]=4; // five
 nums[6]=3; // six
 nums[7]=5; // seven
 nums[8]=5; // eight
 nums[9]=4; // nine
 nums[10]=3; // ten
 nums[11]=6; // eleven
 nums[12]=6; // twelve
 nums[13]=8; // thirteen
 nums[14]=8; // fourteen
 nums[15]=7; // fifteen
 nums[16]=7; // sixteen
 nums[17]=9; // seventeen
 nums[18]=8; // eighteen
 nums[19]=8; // nineteen
 nums[20]=6; // twenty
 nums[30]=6; // thirty
 nums[40]=5; // forty
 nums[50]=5; // fifty
 nums[60]=5; // sixty
 nums[70]=7; // seventy
 nums[80]=6; // eighty
 nums[90]=6; // ninety
 nums[100]=7; // hundred
 nums[1000]=8; // thousand
 nums[1000000]=7; // million
 nums[1000000000]=7; // billion


 num_to_wd ntw(nums);

 // Create array size n (n is argument provided by user)
 ary_t_tp in(n);

 // Array view declaration
 ary_t_vw_tp in_vw(in);

 // Creates sequence from 1 to the size of the array
 stapl::iota(in_vw, 1);

 // Applies work function that returns the letter count value of a number
 // Then performs a cumulative sum
 // Returns the sum of the letter counts from 1 to n
 size_t ans = stapl::map_reduce(ntw, stapl::plus<t_type>(), in_vw);


 double t =  exec_timer.stop();

 stapl::do_once([&]
 {
   std::cout << "LETTER COUNT: " << ans << std::endl
             << "Computation finished (time taken: " << t << " s)"
             << std::endl;
 });

 return EXIT_SUCCESS;
}


