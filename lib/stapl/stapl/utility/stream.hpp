/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_STREAM_HPP
#define STAPL_UTILITY_STREAM_HPP

#include <stapl/runtime.hpp>
#include <boost/shared_ptr.hpp>

namespace stapl {

namespace stream_impl {
//////////////////////////////////////////////////////////////////////
/// @brief A wrapper around a stream that is a @ref p_object. A pointer
/// to an instance is stored in stapl::stream instances provided to work
/// functions to allow access to the stream on the location.
///
/// Currently only location 0 opens the stream and all other locations will
/// forward read/writes to location 0.
///
/// @ingroup utility
///
/// @todo Expand the class to be more than a wrapper around a stream by
/// implementing the full set of stream interfaces and expanding
/// functionality to include MPI-IO capabilities.
//////////////////////////////////////////////////////////////////////
template <typename Stream>
struct stream_wrapper_impl
  : public p_object
{
private:
  Stream m_stream;

public:
  stream_wrapper_impl() = default;

  stream_wrapper_impl(char const* const name)
  {
    open(name);
  }

  stream_wrapper_impl(char const* const name,
                      std::ios_base::openmode mode)
  {
    open(name, mode);
  }

  void open(char const* const name)
  {
    if (this->get_location_id() == 0)
    {
      stapl_assert(!m_stream.is_open(),"stapl::stream -- file is open.");
      m_stream.open(name);
      stapl_assert(m_stream.is_open(),"stapl::stream -- file was not opened.");
    }
  }

  void open(char const* const name, std::ios_base::openmode mode)
  {
    if (this->get_location_id() == 0)
    {
      stapl_assert(!m_stream.is_open(),"stapl::stream -- file is open.");
      m_stream.open(name, mode);
      stapl_assert(m_stream.is_open(),"stapl::stream -- file was not opened.");
    }
  }

  void close(void)
  {
    // Ensure all messages have been received.
    rmi_fence();
    if (this->get_location_id() == 0)
    {
      if (m_stream.is_open())
        m_stream.close();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if the stream has an associated file.
  //////////////////////////////////////////////////////////////////////
  bool is_open(void) const
  {
    if (this->get_location_id() == 0)
      return m_stream.good();
    else
      return sync_rmi(size_t(0), this->get_rmi_handle(),
               &stream_wrapper_impl::is_open);
  }

  ~stream_wrapper_impl()
  {
    // Ensure all messages have been received.
    rmi_fence();
  }

protected:
  template <typename T>
  T get_value()
  {
    T val;
    m_stream >> val;
    return val;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Call read on the local stream and return shared_ptr to the
  /// populated buffer.
  ///
  /// This method is invoked by
  ///   @ref read(typename Stream::char_type*, std::streamsize).
  ///
  /// @param count Number of characters to be read from the stream
  /// @return string containing the characters from the stream
  //////////////////////////////////////////////////////////////////////
  std::basic_string<typename Stream::char_type>
  fetch(std::streamsize count)
  {
    std::basic_string<typename Stream::char_type> buf;
    buf.resize(count, ' ');

    m_stream.read(&*buf.begin(), count);
    return buf;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Call write on the local stream to write the unformatted data
  /// provided to the stream.
  ///
  /// This method is invoked by
  ///   @ref write(typename Stream::char_type* s, std::streamsize count)
  ///
  /// @param data string that contains an unformatted byte sequence to be
  /// written to the stream
  /// @param count Number of characters to be written to the stream
  //////////////////////////////////////////////////////////////////////
  void write_impl(std::basic_string<typename Stream::char_type> const& data,
                  std::streamsize count)
  {
    m_stream.write(&*data.begin(), count);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Call getline on the local stream and return shared_ptr to the
  /// populated buffer.
  ///
  /// This method is invoked by getline.
  ///
  /// @param count Maximum number of charaters to extract
  /// @param delim Deliminter used to separate lines.
  /// @return shared_ptr to a populated char array.
  /// @todo Convert boost::shared_ptr to std::shared_ptr.
  /// @todo Use std::make_shared when serialization of arrays is available.
  //////////////////////////////////////////////////////////////////////
  boost::shared_ptr<typename Stream::char_type>
  fetchline(std::streamsize count, typename Stream::char_type delim)
  {
    boost::shared_ptr<typename Stream::char_type>
      buf(new typename Stream::char_type[count]);

    m_stream.getline(buf.get(), count, delim);
    return buf;
  }
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Read a value from the stream.
  /// @param val variable where the value read from stream is assigned.
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void read(T& val)
  {
    if (this->get_location_id() == 0)
      m_stream >> val;
    else
      val = sync_rmi(size_t(0), this->get_rmi_handle(),
        &stream_wrapper_impl::get_value<T>);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Read a value from the stream.
  /// @param val A proxy over the variable that will have the value read
  /// from the stream assigned to it.
  ///
  /// @note This specialization is provided in order to pass the correct
  /// type of the value to be read from the stream to get_value.  This is
  /// needed because a proxy cannot be default constructed.
  //////////////////////////////////////////////////////////////////////
  template <typename T, typename Accessor>
  void read(proxy<T, Accessor>& val)
  {
    if (this->get_location_id() == 0)
      m_stream >> val;
    else
      val = sync_rmi(size_t(0), this->get_rmi_handle(),
        &stream_wrapper_impl::get_value<T>);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Read blocks of characters from the stream.
  /// @param s Buffer to be populated with characters extracted from stream
  /// @param count Number of characters to be extracted from the stream
  //////////////////////////////////////////////////////////////////////
  void read(typename Stream::char_type* s, std::streamsize count)
  {
    if (this->get_location_id() == 0)
      m_stream.read(s, count);
    else
    {
      std::basic_string<typename Stream::char_type> result =
        sync_rmi(size_t(0), this->get_rmi_handle(),
                 &stream_wrapper_impl::fetch, count);

      std::copy_n(result.begin(), count, s);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Read values from the stream without formatting.
  /// @param s Buffer to be populated with characters extracted from stream
  /// @param count Maximum number of elements to extract from stream
  /// @param delim Character delimiting a new line in the file.
  /// @todo Convert boost::shared_ptr to std::shared_ptr.
  //////////////////////////////////////////////////////////////////////
  void getline(typename Stream::char_type* s, std::streamsize count,
               typename Stream::char_type delim)
  {
    if (this->get_location_id() == 0)
      m_stream.getline(s, count, delim);
    else
    {
      boost::shared_ptr<typename Stream::char_type> result =
        sync_rmi(size_t(0), this->get_rmi_handle(),
                 &stream_wrapper_impl::fetchline, count, delim);

      typename Stream::char_type const* i = result.get();
      typename Stream::char_type*       o = s;
      for (std::streamsize c = 0; *i != '\0' && c != count; ++i, ++o, ++c)
        *o = *i;
      if (*i == '\0')
        *o = *i;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Write the specified value @p val to the stream.
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void write(T const& val)
  {
    if (this->get_location_id() == 0)
      m_stream << val;
    else
      async_rmi(size_t(0), this->get_rmi_handle(),
        &stream_wrapper_impl::write<T>, val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Write the specified @p count of characters from buffer @p s
  /// to the stream.
  //////////////////////////////////////////////////////////////////////
  void write(typename Stream::char_type* s, std::streamsize count)
  {
    if (this->get_location_id() == 0)
      m_stream.write(s, count);
    else
    {
       std::basic_string<typename Stream::char_type> buf;
       buf.resize(count, ' ');
       std::copy_n(s, count, buf.begin());

       async_rmi(size_t(0), this->get_rmi_handle(),
                 &stream_wrapper_impl::write_impl, buf, count);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method invoked when std::endl is passed to the stream.  The
  /// std::endl is inserted in the stream on location 0.
  //////////////////////////////////////////////////////////////////////
  void endl(void)
  {
    if (this->get_location_id() == 0)
      m_stream << std::endl;
    else
      async_rmi(size_t(0), this->get_rmi_handle(),
        &stream_wrapper_impl::endl);
  }
};

} // namespace stream_impl


//////////////////////////////////////////////////////////////////////
/// @brief A wrapper around a stream whose instances can be kept as
/// data members of work functions. The wrapper allows work functions
/// to access to the stream any location to which they may be sent via
/// RMI invocation.
///
/// Instances of this struct may be copied.  All copies refer to the same
/// underlying stream_wrapper_impl.  Methods of this struct forward requests
/// to the stream_wrapper_impl instance where they are processed.
///
/// @ingroup utility
///
/// @todo Expand the class to be more than a wrapper around a stream by
/// implementing the full set of stream interfaces and expanding
/// functionality to include MPI-IO capabilities.
//////////////////////////////////////////////////////////////////////
template <typename Stream>
struct stream
{
private:
  typedef stream_impl::stream_wrapper_impl<Stream> impl_type;

  bool                                m_owner;
  p_object_pointer_wrapper<impl_type> m_impl;

public:
  typedef typename Stream::char_type char_type;

  stream(void)
    : m_owner(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the stream and attempt to associate it with the file
  /// specified.
  /// @param name file with which the stream should be associated.
  //////////////////////////////////////////////////////////////////////
  stream(char const* const name)
    : m_owner(true), m_impl(new impl_type(name))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the stream and attempt to associate it with the file
  /// specified using the specified mode.
  /// @param name file with which the stream should be associated.
  /// @param mode flags specifying the mode in which the file should be opened
  //////////////////////////////////////////////////////////////////////
  stream(char const* const name, std::ios_base::openmode mode)
    : m_owner(true), m_impl(new impl_type(name, mode))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a copy of the stream that does not take ownership of
  /// the underlying stream_wrapper_impl instance.
  /// @param other original stream instance to copy
  //////////////////////////////////////////////////////////////////////
  stream(stream const& other)
    : m_owner(false), m_impl(other.m_impl)
  { }

  ~stream(void)
  {
    if (m_owner && m_impl != nullptr)
    {
      m_impl->close();
      delete m_impl;
      m_impl = nullptr;
    }
    else if (!m_owner)
    {
      m_impl = nullptr;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Associate the stream with the named file.
  /// @param name file to be opened
  //////////////////////////////////////////////////////////////////////
  void open(char const* const name)
  {
    if (m_impl != nullptr)
      m_impl->open(name);
    else
    {
      m_owner = true;
      m_impl = new stream_impl::stream_wrapper_impl<Stream>(name);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Associate the stream with the named file.
  /// @param name file to be opened
  /// @param mode flags specifying the mode in which the file should be opened
  //////////////////////////////////////////////////////////////////////
  void open(char const* const name, std::ios_base::openmode mode)
  {
    if (m_impl != nullptr)
      m_impl->open(name, mode);
    else
    {
      m_owner = true;
      m_impl = new stream_impl::stream_wrapper_impl<Stream>(name, mode);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Close the file associated with the stream and disassociate the
  /// stream from it.
  //////////////////////////////////////////////////////////////////////
  void close(void)
  {
    if (m_impl != nullptr)
      m_impl->close();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void read(T& val)
  {
    stapl_assert(m_impl != nullptr,"stream is not open. read failed.");
    m_impl->read(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform an unformatted read of @p count characters from the
  /// stream and store the characters in the buffer @p s.
  //////////////////////////////////////////////////////////////////////
  void read(char_type* s, std::streamsize count)
  {
    stapl_assert(m_impl != nullptr,"stream is not open. read failed.");
    m_impl->read(s, count);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reads characters from the stream until the delimiter @p delim
  /// is encountered or @p count characters are read and store the results
  /// in the buffer @p s.
  //////////////////////////////////////////////////////////////////////
  void getline(char_type* s, std::streamsize count,
               char_type delim = char_type('\n'))
  {
    stapl_assert(m_impl != nullptr,"stream is not open. getline failed.");
    m_impl->getline(s, count, delim);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void write(T& val)
  {
    stapl_assert(m_impl != nullptr,"stream is not open. write failed.");
    m_impl->write(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform an unformatted write of @p count characters from the
  /// buffer @p s to the stream.
  //////////////////////////////////////////////////////////////////////
  void write(char_type* s, std::streamsize count)
  {
    stapl_assert(m_impl != nullptr,"stream is not open. write failed.");
    m_impl->write(s, count);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method invoked when std::endl is written to the stream.
  //////////////////////////////////////////////////////////////////////
  void endl(void)
  {
    m_impl->endl();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if the stream has an associated file.
  //////////////////////////////////////////////////////////////////////
  bool is_open(void) const
  {
    return m_impl->is_open();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.transient(m_owner, false);
    t.member(m_impl);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Output operator for stream wrapper.
/// @param os stream to which the value will be written
/// @param val variable to be written
/// @return reference to the stream to allow chaining of the operator
//////////////////////////////////////////////////////////////////////
template <typename Stream, typename T>
stream<Stream>& operator<<(stream<Stream>& os, T const& val)
{
  os.write(val);
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of the output operator to intercept std::endl
/// @param os stream to which the value will be written
/// @param endlPar function pointer to std::endl
/// @return reference to the stream to allow chaining of the operator
///
/// @note Required because file streams don't allow customization of the
/// buffer (i.e., by invoking set_rdbuf), which would allow the sync method
/// of the buffer to be overwritten and the call intercepted using the
/// non-specialized output operator.
//////////////////////////////////////////////////////////////////////
template <typename Stream>
stream<Stream>& operator<<(stream<Stream>& os,
                           std::ostream& (*endlPar) (std::ostream& os))
{
  os.endl();
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Input operator for the stream wrapper
/// @param os Stream from which to read
/// @param val Variable to which the value read will be assigned
//////////////////////////////////////////////////////////////////////
template <typename Stream, typename T>
stream<Stream>& operator>>(stream<Stream>& os, T& val)
{
  os.read(val);
  return os;
}

}
#endif
