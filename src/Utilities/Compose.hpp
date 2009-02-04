#ifndef _COMPOSE_HPP_
#define _COMPOSE_HPP_

template <typename InputIterator, typename BinaryOperator, typename UnaryOperator>  
struct Compose 
{
  bool 
  operator()(InputIterator first, InputIterator last, 
	     BinaryOperator binary_op, UnaryOperator op) {
    if (first == last)
      return false;
    else {
      bool result = op(*first++);
      while (first != last)
	result = binary_op(result, op(*first++)); 
      return result;  
    }
  }  
};


template <typename InputIterator, typename UnaryOperator>  
struct Compose<InputIterator, logical_and<bool>, UnaryOperator>  
{
  bool 
  operator()(InputIterator first, InputIterator last, 
	     logical_and<bool> binary_op, UnaryOperator op) {
    if (first == last)
      return false;
    else {
      bool result = op(*first++);
      if (result == false)
	return result;	
      while (first != last) {
	result = binary_op(result, op(*first++)); 
	if (result == false)
	  return result;	
      }
      return result;  
    }
  }  
};


template <typename InputIterator, typename UnaryOperator>  
struct Compose<InputIterator, logical_or<bool>, UnaryOperator>  
{
  bool 
  operator()(InputIterator first, InputIterator last, 
	     logical_or<bool> binary_op, UnaryOperator op) {
    if (first == last)
      return false;
    else {
      bool result = op(*first++);
      if (result == true)
	return result;	
      while (first != last) {
	result = binary_op(result, op(*first++)); 
	if (result == true)
	  return result;	
      }
      return result;  
    }
  }  
};


template <typename InputIterator, typename UnaryOperator>  
struct Compose_Negate  
{
  bool 
  operator()(InputIterator It, UnaryOperator op) {
    return !op(*It); 
  }  
};


#endif // _COMPOSE_HPP_

