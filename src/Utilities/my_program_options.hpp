#ifndef _MY_PROGRAM_OPTIONS_HPP_
#define _MY_PROGRAM_OPTIONS_HPP_

#include <sstream>
#include <vector>
#include <string>
#include <iterator>

#include "program_options.hpp"
#include "boost/utility/enable_if.hpp"

namespace po = boost::program_options;

namespace internal {
  using boost::false_type;
  using boost::true_type;

  template <typename T> struct is_container : public false_type {};
  template <typename T> struct is_container<std::vector<T> > : public true_type {};
  //template <typename T> struct is_container<std::list<T> > : public true_type {};

  template <typename T> struct value_type {
    typedef T type;
  };
  template <typename T> struct value_type<std::vector<T> > {
    typedef T type;
  };
}

namespace boost { namespace program_options {

using namespace internal;
using boost::enable_if;
using boost::disable_if;

template <typename T>
class ranged_value : public typed_value<T> {
public:
  typedef typename internal::value_type<T>::type v_type;

  ranged_value(const v_type& min, const v_type& max, T* store_to) :
    typed_value<T>(store_to), m_min(min), m_max(max) 
  {
    m_inclusive = true;
  }

  ~ranged_value() {}

  ranged_value* default_value(const T& v) 
  {
    return _default_value<T>(v);
  }

  template <typename Q>
  ranged_value* _default_value(const Q& v,
			       typename disable_if<is_container<Q> >::type* dummy = 0) 
  {
    typed_value<T>::default_value(v);
    return this;
  }

  template <typename Q>
  ranged_value* _default_value(const Q& v,
			       typename enable_if<is_container<Q> >::type* dummy = 0) 
  {
    using namespace std;
    string s;
    ostringstream os(s);
    copy(v.begin(), v.end(), ostream_iterator<v_type>(os, " "));
    return default_value(v, s);
  }

  ranged_value* default_value(const T& v, const std::string& textual) 
  {
    typed_value<T>::default_value(v, textual);
    return this;
  }

  ranged_value* notifier(boost::function1<void, const T&> f) 
  {
    typed_value<T>::notifier(f);
    return this;
  }

  ranged_value* composing() 
  {
    typed_value<T>::composing();
    return this;
  }

  ranged_value* implicit() 
  {
    typed_value<T>::implicit();
    return this;
  }

  ranged_value* multitoken() 
  {
    typed_value<T>::multitoken();
    return this;
  }

  ranged_value* zero_tokens() 
  {
    typed_value<T>::zero_tokens();
    return this;
  }

  ranged_value* inclusive() 
  {
    m_inclusive = true;
  }

  ranged_value* exclusive() 
  {
    m_inclusive = false;
  }

  bool is_inclusive() const 
  {
    return m_inclusive;
  }

  bool is_exclusive() const 
  {
    return !m_inclusive;
  }

  void xparse(boost::any& value_store,
	      const std::vector<std::string>& new_tokens) const 
  {
    typed_value<T>::xparse(value_store, new_tokens);

    T t;
    try {
      t = boost::any_cast<T>(value_store);
    }
    catch (const boost::bad_any_cast&) {
      throw validation_error("bad_any_cast");
    }
    validate_range<T>(t);
  }

  template <typename Q>
  void validate_range(const Q& t,
		      typename disable_if<is_container<Q> >::type* dummy = 0) const 
  {
    if(m_inclusive)
      if((t < m_min) || (t > m_max)) {
	std::ostringstream os;
	os << "value (" << t << ") is outside of range [" << m_min << "," << m_max << "]";
	throw validation_error(os.str());
      }
    else
      if((t <= m_min) || (t >= m_max)) {
	std::ostringstream os;
	os << "value (" << t << ") is outside of range (" << m_min << "," << m_max << ")";
	throw validation_error(os.str());
      }
  }

  template <typename Q>
  void validate_range(const Q& t,
		      typename enable_if<is_container<Q> >::type* dummy = 0) const 
  {
    if(m_inclusive) 
      for(typename Q::const_iterator tt = t.begin(); tt != t.end(); ++tt) 
	if((*tt < m_min) || (*tt > m_max)) {
	  std::ostringstream os;
	  os << "value (" << *tt 
	     << ") is outside of range [" << m_min << "," << m_max << "]";
	  throw validation_error(os.str());
	}      
    else 
      for(typename T::const_iterator tt = t.begin(); tt != t.end(); ++tt)
	if((*tt <= m_min) || (*tt >= m_max)) {
	  std::ostringstream os;
	  os << "value (" << *tt 
	     << ") is outside of range (" << m_min << "," << m_max << ")";
	  throw validation_error(os.str());
	}
  }
  
protected:
  v_type m_min, m_max;
  bool m_inclusive;
};


template <typename T>
ranged_value<T>*
value_range(const typename internal::value_type<T>::type& min,
	    const typename internal::value_type<T>::type& max,
	    T* v)
{
  ranged_value<T>* r = new ranged_value<T>(min, max, v);
  return r;
}

template <typename T>
ranged_value<T>*
value_range(const typename internal::value_type<T>::type& min,
	    const typename internal::value_type<T>::type& max)
{
  return value_range<T>(min, max, (T*)0);
}

}}

#endif
