#ifndef BaseSampler_h
#define BaseSampler_h

#include <iostream>

template <typename CFG>
class BaseSampler
{
 private:
 public:
  virtual ~BaseSampler() {}

  virtual bool operator()(const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts) = 0;

  //for debugging
  friend ostream& operator<<(ostream& os, const BaseSampler& b)
    {
      b.print(os);
      return os;
    }
  virtual void print(ostream& os) const = 0;
  virtual const char* name() const = 0;
};


template <typename CFG, typename InputIterator, typename OutputIterator>
OutputIterator
sample(BaseSampler<CFG>& sampler,
       InputIterator first, InputIterator last,
       OutputIterator result, 
       int max_attempts = 1) 
{
  while(first != last) {
    vector<CFG> result_cfg;
    if(sampler(*first, result_cfg, max_attempts)) {
      result = copy(result_cfg.begin(), result_cfg.end(), result);
    }
    first++;
  }
  return result;
}

#endif
