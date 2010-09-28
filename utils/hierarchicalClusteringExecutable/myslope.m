function [outvec] = myslope(invec)
N=length(invec)-1;
j=0;
outvec=1:N;
for i=1:N
   j=i+1; 
   outvec(i)=invec(i)-invec(j);  
end
