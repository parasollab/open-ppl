/* createEnv.c

   GUang
  */

#include<iostream.h>
#include<fstream.h>
#include<math.h>
#include<stdlib.h>



int main(int argc, char ** argv) {
   ofstream os("parking.env");

   int nodes = 20;
   os << nodes+1 << "\n\n";
   os  << "MultiBody  Active\n";
   os << "1\n";
   os << "FreeBody  0  carReal.g  -30 -5 0 0 0 0\n";
   os << "Connection\n0\n\n";

   double a, b, f;
   for(int i=0; i<nodes; ++i) {
     a = drand48()*72 - 36;
     b = drand48()*32 - 16;
     f = drand48();
     os << "\n";
     os << "MultiBody  Passive\n";
     os << "1\n";
     os << "#VIZMO_COLOR 0 0 0\n";
     os << "#VIZMO_SOLID 1\n";
     if(i == 0) 
        os << "FixedBody  0  bbox.g 0 0 0 0 0 0 "  << "\n";
     else
        os << "FixedBody  0  triangle.g " << a << "  " << b << " 0 0 0 " << f*360 << "\n"; 
     os << "Connection\n0\n";
   }
}

       

