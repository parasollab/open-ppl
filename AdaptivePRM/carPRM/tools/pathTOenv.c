/* pathTOenv.c

   GUang
  */

#include<iostream.h>
#include<fstream.h>
#include<stdio.h>


int main(int argc, char ** argv) {
   ifstream is("mapnodes.path");
   ifstream is2("parking.env");
   ofstream os("wholePath.env");
   char line[200];
   is.getline(line, 100);
   is.getline(line, 100);
   int nodes;
   is >> nodes;
   int fac = 40;
   sscanf(argv[1], "%d", &fac);
   int nreal = nodes/fac;
   int objects;
   is2 >> objects;
   os << nreal + objects;
   while(is2.getline(line, 100)) {
      os << line << "\n";
   }

   double a, b, c, d, e,f;
   for(int i=0; i<nodes; ++i) {
   is >> a >> b >> c >> d >> e >> f;
     if((i+1)%fac == 0 || i==nodes-1) {
     os << "\n";
     os << "MultiBody  Passive\n";
     os << "1\n";
     os << "#VIZMO_COLOR 0 0 0\n";
     os << "#VIZMO_SOLID 0\n";
     os << "FixedBody  0  carReal.g " << a << "  " << b << " 0 0 0 " << f*360 << "\n";
     os << "Connection\n0\n";
     }
   }
}

       

