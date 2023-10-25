#ifndef NONSTD_FACTORY_H_
#define NONSTD_FACTORY_H_

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// Produces new dynamic objects of a given type.
  //////////////////////////////////////////////////////////////////////////////
  template <typename product_type>
  struct factory
  {

    /// Generate a new product and return a pointer to it.
    /// @param[in] _a The product arguments.
    template <typename... args>
    product_type* operator()(args... _a) const
    {
      return new product_type(_a...);
    }

  };

}

#endif
