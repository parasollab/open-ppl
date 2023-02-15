/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COMMUNICATOR_MPI_COMMUNICATOR_HPP
#define STAPL_RUNTIME_COMMUNICATOR_MPI_COMMUNICATOR_HPP

#include "../config.hpp"
#include "communicator.hpp"
#include "../exception.hpp"
#include "../instrumentation.hpp"
#include "../message.hpp"
#include "../runqueue.hpp"
#include "../concurrency/concurrency.hpp"
#include "../concurrency/task_queue.hpp"
#include "../utility/option.hpp"
#include "../utility/string.hpp"
#include <algorithm>
#include <functional>
#include <iterator>
#include <mutex>
#include <numeric>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/range/size.hpp>
#ifndef MPICH_IGNORE_CXX_SEEK
// Force MPICH not to define SEEK_SET, SEEK_CUR and SEEK_END
# define MPICH_IGNORE_CXX_SEEK 1
#endif
#include <mpi.h>

#ifndef STAPL_RUNTIME_MPI_RECV_MIN
/// Default minimum number of buffers that wait on an @c MPI_Irecv().
# define STAPL_RUNTIME_MPI_RECV_MIN 8
#endif

#ifndef STAPL_RUNTIME_MPI_RECV_MAX
/// Default maximum number of buffers that wait on an @c MPI_Irecv().
# define STAPL_RUNTIME_MPI_RECV_MAX 128
#endif

#ifndef STAPL_RUNTIME_MPI_SEND_MAX
/// Default maximum number of buffers that wait on an @c MPI_Isend().
# define STAPL_RUNTIME_MPI_SEND_MAX 2048
#endif

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of processes per node, based on how many processes
///        are co-scheduled with process @c 0.
///
/// This function acts also as an implicit barrier.
///
/// @warning This function will estimate incorrectly if the machine is not used
///          in the same way on all nodes.
///
/// @ingroup processCommunication
//////////////////////////////////////////////////////////////////////
inline unsigned int get_processes_per_node(MPI_Comm comm)
{
  int rank = MPI_PROC_NULL;
  MPI_Comm_rank(comm, &rank);

  int len = MPI_MAX_PROCESSOR_NAME;
  char name[MPI_MAX_PROCESSOR_NAME];
  MPI_Get_processor_name(name, &len);

  char root_name[MPI_MAX_PROCESSOR_NAME];
  int  root_len = 0;
  char* str     = root_name;

  if (rank==0) {
    str      = name;
    root_len = len;
  }
  MPI_Bcast(&root_len, 1, MPI_INT, 0, comm);   // send string length
  MPI_Bcast(str, root_len, MPI_CHAR, 0, comm); // send string
  if (rank!=0) {
    root_name[root_len] = '\0';
  }

  // find number of ranks/node based on the number of ranks coscheduled with
  // rank 0
  int num_procs_per_node = (std::strcmp(name, str)==0);
  MPI_Allreduce(MPI_IN_PLACE, &num_procs_per_node, 1, MPI_INT, MPI_SUM, comm);

  return num_procs_per_node;
}


//////////////////////////////////////////////////////////////////////
/// @brief Implements a communicator based on MPI-2.
///
/// The number of processes to use is determined by the underlying MPI
/// implementation. Usually, this is set by the <tt>mpiexec -n</tt> command,
/// although some implementations use command line flags.
///
/// Assumptions:
/// -# MPI errors are fatal (i.e., abort), as specified by the standard.
/// -# homogeneous runtime environment (i.e., all MPI processes are running on
///    the same platform).
/// -# MPI is not thread-safe.
///
/// @ingroup processCommunication
//////////////////////////////////////////////////////////////////////
class mpi_communicator
{
public:
  using size_type = std::size_t;

private:
  /// Minimum number of buffers that wait an @c MPI_Irecv() to be completed.
  static size_type sm_recv_min;
  /// Maximum number of buffers that wait an @c MPI_Irecv() to be completed.
  static size_type sm_recv_max;
  /// Maximum number of buffers that wait an @c MPI_Isend() to be completed.
  static size_type sm_send_max;
  /// @c true if the MPI layer was initialized by the runtime.
  static bool      sm_mpi_initialized;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the environment is initialized.
  //////////////////////////////////////////////////////////////////////
  static bool environment_is_initialized(void) noexcept
  {
    int flag = 0;
    MPI_Initialized(&flag);
    return (flag!=0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes the environment.
  ///
  /// @c MPI_Init_thread() with the required thread support will be called if
  /// the MPI library has not been initialized yet (i.e @c MPI_Initialized()
  /// returns false).
  ///
  /// Through the @p opts object the user can set
  /// @c STAPL_RUNTIME_MPI_RECV_MIN and @c STAPL_RUNTIME_MPI_RECV_MAX for
  /// the minimum and maximum number of pending @c MPI_Irecv() as well as
  /// @c STAPL_RUNTIME_MPI_SEND_MAX for the maximum number of @c MPI_Isend()
  /// before the communicator is considered overloaded.
  ///
  /// @param opts @ref option object passed from the user.
  ///
  /// @warning If the MPI library does not support the required thread support,
  ///          then a failure will occur. Adjust your MPI library accordingly.
  //////////////////////////////////////////////////////////////////////
  static void environment_initialize(option const& opts)
  {
    // initialize MPI and find thread support
    const int required = (concurrency::hardware_concurrency()>1
                            ? MPI_THREAD_SERIALIZED
                            : MPI_THREAD_SINGLE);
    int provided       = MPI_THREAD_SINGLE;
    sm_mpi_initialized = !environment_is_initialized();
    if (sm_mpi_initialized) {
      if (opts.has_argv())
        MPI_Init_thread(&opts.get_argc(), &opts.get_argv(),
                        required, &provided);
      else
        MPI_Init_thread(nullptr, nullptr, required, &provided);
    }
    else {
      MPI_Query_thread(&provided);
    }
    if (provided<required)
      STAPL_RUNTIME_ERROR("MPI does not provide the requested thread support.");

    // load environment variables
    sm_recv_min = opts.get<size_type>("STAPL_RUNTIME_MPI_RECV_MIN",
                                      STAPL_RUNTIME_MPI_RECV_MIN);
    if (sm_recv_min==0)
      STAPL_RUNTIME_ERROR("STAPL_RUNTIME_MPI_RECV_MIN cannot be 0.");

    sm_recv_max = opts.get<size_type>("STAPL_RUNTIME_MPI_RECV_MAX",
                                      STAPL_RUNTIME_MPI_RECV_MAX);
    if (sm_recv_max<sm_recv_min)
      STAPL_RUNTIME_ERROR("STAPL_RUNTIME_MPI_RECV_MAX cannot be less than "
                          "STAPL_RUNTIME_MPI_RECV_MIN.");

    sm_send_max = opts.get<size_type>("STAPL_RUNTIME_MPI_SEND_MAX",
                                      STAPL_RUNTIME_MPI_SEND_MAX);
    if (sm_send_max==0)
      STAPL_RUNTIME_ERROR("STAPL_RUNTIME_MPI_SEND_MAX cannot be 0.");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finalizes the environment.
  ///
  /// @c MPI_Finalize() will be called if the runtime was responsible for
  /// initializing the MPI library in the first place.
  ///
  /// @see mpi_communicator::initialize_environment()
  //////////////////////////////////////////////////////////////////////
  static void environment_finalize(void) noexcept
  {
    if (sm_mpi_initialized)
      MPI_Finalize();
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Duplicates the given communicator.
  //////////////////////////////////////////////////////////////////////
  static MPI_Comm create_duplicate(MPI_Comm comm) noexcept
  {
    MPI_Comm newcomm = MPI_COMM_NULL;
    MPI_Comm_dup(comm, &newcomm);
    if (newcomm==MPI_COMM_NULL)
      STAPL_RUNTIME_ERROR("Could not duplicate communicator.");
    return newcomm;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a Cartesian communicator from @p comm and the given
  ///        dimensions.
  //////////////////////////////////////////////////////////////////////
  static MPI_Comm create_cartesian(MPI_Comm comm, std::vector<int>& dims)
  {
    int comm_size = MPI_PROC_NULL;
    MPI_Comm_size(comm, &comm_size);
    if (comm_size<=0)
      STAPL_RUNTIME_ERROR("Parent communicator is invalid.");
    if (comm_size!=
          std::accumulate(dims.begin(), dims.end(), 1, std::multiplies<int>()))
      STAPL_RUNTIME_ERROR("Cartesian grid is not the same size as the parent "
                          "communicator.");
    std::vector<int> periods(dims.size(), 1);
    MPI_Comm newcomm = MPI_COMM_NULL;
    MPI_Cart_create(comm, dims.size(), &(dims[0]), &(periods[0]), 1, &newcomm);
    return newcomm;
  }

private:
  enum Tag
  {
    /// Tag for default message channel.
    DEFAULT_TAG = 0,
    /// Tag for long message channel.
    LONG_TAG = 1
  };

  using send_container_type    =
    boost::unordered_map<int, message_raw_ptr_slist>;
  using request_container_type = std::vector<MPI_Request>;
  using task_queue_type        = task_queue<void(mpi_communicator&)>;

  MPI_Comm              m_comm;
  int                   m_size;
  int                   m_rank;
  /// Number of ranks per node.
  unsigned int          m_nppn;

  /// Pending sent messages.
  send_container_type   m_send_msg;
  /// Number of pending sent messages.
  size_type             m_pending_send;
  /// Queue of pending forwarded messages.
  message_raw_ptr_slist m_fwd_msg;
  /// Queue of pending to be received messages.
  message_raw_ptr_slist m_recv_msg;
  /// @c true if there are too many messages pending.
  bool                  m_overloaded;
  /// Queue of functions that were not called because @c *this was locked.
  task_queue_type       m_q;
  mutable std::mutex    m_mtx;

  void check_overloaded(void) noexcept
  {
    if (m_overloaded) {
      if (m_pending_send<=sm_send_max) {
        m_overloaded = false;
        runqueue::set_overloaded(false);
      }
    }
    else {
      if (m_pending_send>sm_send_max) {
        m_overloaded = true;
        runqueue::set_overloaded(true);
      }
    }
  }

public:
  mpi_communicator(void)
  : m_comm(MPI_COMM_NULL),
    m_size(MPI_PROC_NULL),
    m_rank(MPI_PROC_NULL),
    m_nppn(0),
    m_pending_send(0),
    m_overloaded(false)
  { }

  mpi_communicator(mpi_communicator const&) = delete;
  mpi_communicator& operator=(mpi_communicator const&) = delete;

  ////////////////////////////////////////////////////////////////////
  /// @brief Initializes this @ref mpi_communicator object.
  ///
  /// This function can receive options through the @p opts object. The user can
  /// define the MPI commmunicator to use by setting it accordingly with the
  /// @c MPI_Comm tag in @p opts, as well a Cartesian topology through the tag
  /// @c MPI_Cart.
  ///
  /// @param opts @ref option object to initialize this @ref mpi_communicator.
  ////////////////////////////////////////////////////////////////////
  void initialize(option const& opts)
  {
    std::lock_guard<std::mutex> lock{m_mtx};

    // get the user-specified communicator if any
    MPI_Comm comm = MPI_COMM_WORLD;
    opts.try_get_noenv("MPI_Comm", comm);

    // create the communicator
    std::string cart_topo;
    opts.try_get("MPI_Cart", cart_topo);
    if (!cart_topo.empty()) {
      // cartesian topology requested
      std::vector<int> dims = split_string_to_vector<int>(cart_topo, " ,");
      if (dims.empty())
        STAPL_RUNTIME_ERROR("MPI_Cart is only allowed to have integers "
                            "separated by spaces or commas.");
      m_comm = create_cartesian(comm, dims);
    }
    else {
      // no topology requested
      m_comm = create_duplicate(comm);
    }

    // set-up communicator
    char name[] = "STAPL_RTS_COMM";
    MPI_Comm_set_name(m_comm, name);
    MPI_Comm_size(m_comm, &m_size);
    MPI_Comm_rank(m_comm, &m_rank);

    // find number of proceses/node and also implicit barrier for all processes
    // to complete initialization
    m_nppn = get_processes_per_node(m_comm);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Finalizes this @ref mpi_communicator object.
  ///
  /// All pending messages will be canceled. An implicit barrier is part of the
  /// cleanup to ensure correctness.
  ////////////////////////////////////////////////////////////////////
  void finalize(void)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    MPI_Comm_free(&m_comm);
    m_size = MPI_PROC_NULL;
    m_rank = MPI_PROC_NULL;
    m_nppn = 0;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Starts this @ref mpi_communicator object.
  ///
  /// New messages will be allocated to waiting for incoming communication. An
  /// implicit barrier is part of the setup to ensure that no process will start
  /// communication before other processes had the opportunity to finish their
  /// setup.
  ////////////////////////////////////////////////////////////////////
  void start(void)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    create_receive_message(sm_recv_min);
    MPI_Barrier(m_comm); // all processes should complete setup
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Stops this @ref mpi_communicator object.
  ///
  /// All pending messages will be canceled. An implicit barrier is part of the
  /// cleanup to ensure correctness.
  ////////////////////////////////////////////////////////////////////
  void stop(void)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    cancel_receive_messages();
    wait_sent_messages();
    MPI_Barrier(m_comm); // all processes should complete cleanup
  }

  bool initialized(void) const noexcept
  { return (m_comm!=MPI_COMM_NULL); }

  int get_id(void) const noexcept
  { return m_rank; }

  int size(void) const noexcept
  { return m_size; }

  int get_num_procs_per_node(void) const noexcept
  { return m_nppn; }

  void exit(int exit_code) const
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    MPI_Abort((initialized() ? m_comm : MPI_COMM_WORLD), exit_code);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sets the default communication message size.
  ///
  /// @warning This is a collective operation and is supposed to be called by
  ///          one thread on each process only.
  ///
  /// @param size Size of the @ref message objects.
  ///
  /// @see message
  ////////////////////////////////////////////////////////////////////
  void set_default_message_size(const std::size_t size)
  {
    std::lock_guard<std::mutex> lock{m_mtx};

    // clear out messages from queues
    cancel_receive_messages();
    wait_sent_messages();

    // set new default size and post the receives
    message::set_default_body_capacity(size);
    create_receive_message(sm_recv_min);

    // check if value is the same across all processes; implicit barrier
    unsigned long sz = size;
    MPI_Bcast(&sz, 1, MPI_UNSIGNED_LONG, 0, m_comm);
    if (sz!=size)
      STAPL_RUNTIME_ERROR("Default message size has to be the same in all "
                          "processes.");
    if (sz==0)
      STAPL_RUNTIME_ERROR("Default message size cannot be 0.");
  }

  void lock(void)
  { m_mtx.lock(); }

  void unlock(void)
  { m_mtx.unlock(); }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns an @c MPI_Request object associated with @p m.
  ////////////////////////////////////////////////////////////////////
  MPI_Request& request_get(message& m)
  {
    return m.handle().mpi_request();
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @p n @c MPI_Request objects associated with @p m.
  ////////////////////////////////////////////////////////////////////
  request_container_type& request_get(message& m, const size_type n)
  {
    return m.handle().mpi_requests(n);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns all the n @c MPI_Request objects associated with @p m.
  ////////////////////////////////////////////////////////////////////
  request_container_type& request_get_all(message& m)
  {
    return m.handle().mpi_requests();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Posts an @c MPI_Irecv() with the given message.
  ///
  /// @param m Message to post for reception.
  //////////////////////////////////////////////////////////////////////
  void post_receive_message(message* m)
  {
    STAPL_RUNTIME_ASSERT(m->capacity()==message::default_capacity());
    MPI_Request& req = request_get(*m);
    STAPL_RUNTIME_ASSERT(req==MPI_REQUEST_NULL);
    MPI_Irecv(m->data(), m->capacity(), MPI_BYTE,
              MPI_ANY_SOURCE, DEFAULT_TAG, m_comm, &req);
    m_recv_msg.push_back(*m);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a number of @c MPI_Irecv(), waiting for messages to be
  ///        received.
  ///
  /// @param n Number of messages to pre-fire for receiving.
  ///
  /// @todo Use a @ref message::construct() that returns multiple messages.
  //////////////////////////////////////////////////////////////////////
  void create_receive_message(const size_type n = 1)
  {
    for (size_type i=0; i<n; ++i) {
      message* const m = message::construct();
      post_receive_message(m);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Cancels all pending @c MPI_Irecv().
  //////////////////////////////////////////////////////////////////////
  void cancel_receive_messages(void)
  {
    while (!m_recv_msg.empty()) {
      message& m = m_recv_msg.front();
      m_recv_msg.pop_front();
      MPI_Request& req = m.handle().mpi_request();
      STAPL_RUNTIME_ASSERT(req!=MPI_REQUEST_NULL);
      MPI_Cancel(&req);
      MPI_Wait(&req, MPI_STATUS_IGNORE);
      message::destroy(&m);
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Tests @p m if it was received.
  ///
  /// @return A @c std::pair with the sender process id and the size of the
  ///         message if it was succesfully received, otherwise a @c std::pair
  ///         of @c MPI_PROC_NULL and 0.
  ////////////////////////////////////////////////////////////////////
  static std::pair<int, std::size_t> test_receive(message& m) noexcept
  {
    MPI_Request& req = m.handle().mpi_request();
    STAPL_RUNTIME_ASSERT(req!=MPI_REQUEST_NULL);
    int flag = 0;
    MPI_Status status;
    MPI_Test(&req, &flag, &status);
    if (flag==0) {
      return std::make_pair(MPI_PROC_NULL, 0);
    }

    const int proc = status.MPI_SOURCE;
    int size = MPI_UNDEFINED;
    MPI_Get_count(&status, MPI_BYTE, &size);
    STAPL_RUNTIME_CHECK( (size!=MPI_UNDEFINED) && (proc!=MPI_PROC_NULL),
                         "Problem receiving message" );

    return std::make_pair(proc, std::size_t(size));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for @p m to be received.
  ////////////////////////////////////////////////////////////////////
  static std::pair<int, std::size_t> wait_receive(message& m) noexcept
  {
    MPI_Request& req = m.handle().mpi_request();
    STAPL_RUNTIME_ASSERT(req!=MPI_REQUEST_NULL);
    MPI_Status status;
    MPI_Wait(&req, &status);

    const int proc = status.MPI_SOURCE;
    int size = MPI_UNDEFINED;
    MPI_Get_count(&status, MPI_BYTE, &size);
    STAPL_RUNTIME_CHECK( (size!=MPI_UNDEFINED) && (proc!=MPI_PROC_NULL),
                         "Problem receiving message" );

    return std::make_pair(proc, std::size_t(size));
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Tries to receive a long message from process @p source of size
  ///        @c s.
  ///
  /// @return A pointer to the message if it was received, otherwise @c nullptr.
  ////////////////////////////////////////////////////////////////////
  message* try_receive_long(int source, const std::size_t s)
  {
    message* const m = message::construct(s, true);

    MPI_Request& req = request_get(*m);
    STAPL_RUNTIME_ASSERT(req==MPI_REQUEST_NULL);
    MPI_Irecv(m->data(), m->capacity(), MPI_BYTE,
              source, LONG_TAG, m_comm, &req);
    int flag = 0;
    MPI_Status status;
    MPI_Test(&req, &flag, &status);
    if (flag==0) {
      // not received yet; push to the front to prevent breaking ordering
      m_recv_msg.push_front(*m);
      return 0;
    }

    const int proc = status.MPI_SOURCE;
    int size = MPI_UNDEFINED;
    MPI_Get_count(&status, MPI_BYTE, &size);
    STAPL_RUNTIME_CHECK( (size!=MPI_UNDEFINED) && (proc==source),
                         "Problem receiving message" );
    STAPL_RUNTIME_ASSERT(size==int(m->size()));

    return m;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits to receive a long message from process @p source of size
  ///        @c s.
  ////////////////////////////////////////////////////////////////////
  message* wait_receive_long(int source, const std::size_t s)
  {
    message* const m = message::construct(s, true);

    MPI_Status status;
    MPI_Recv(m->data(), m->capacity(), MPI_BYTE,
             source, LONG_TAG, m_comm, &status);

    const int proc = status.MPI_SOURCE;
    int size = MPI_UNDEFINED;
    MPI_Get_count(&status, MPI_BYTE, &size);
    STAPL_RUNTIME_CHECK( (size!=MPI_UNDEFINED) && (proc==source),
                         "Problem receiving message" );
    STAPL_RUNTIME_ASSERT(size==int(m->size()));

    return m;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if @p m was successfully sent.
  ////////////////////////////////////////////////////////////////////
  static bool test_sent_message(message& m) noexcept
  {
    request_container_type& reqs = m.handle().mpi_requests();
    STAPL_RUNTIME_ASSERT(reqs.size()>0);
    int flag = 0;
    MPI_Testall(reqs.size(), &(reqs[0]), &flag, MPI_STATUSES_IGNORE);
    return (flag!=0);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for @p m to be sent.
  ////////////////////////////////////////////////////////////////////
  static void wait_sent_message(message& m) noexcept
  {
    request_container_type& reqs = m.handle().mpi_requests();
    STAPL_RUNTIME_ASSERT(reqs.size()>0);
    MPI_Waitall(reqs.size(), &(reqs[0]), MPI_STATUSES_IGNORE);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Notifies that @p m has been sent to @p dest.
  ///
  /// If @p m was not successfully sent yet, it will be added to a queue to be
  /// checked later.
  ////////////////////////////////////////////////////////////////////
  void notify_sent_message(int dest, message& m)
  {
    if (!test_sent_message(m)) {
      m_send_msg[dest].push_back(m);
      ++m_pending_send;
    }
    else {
      message::destroy(&m);
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Notifies that @p m has been sent to multiple destinations.
  ///
  /// If @p m was not successfully sent yet, it will be added to a queue to be
  /// checked later.
  ////////////////////////////////////////////////////////////////////
  void notify_sent_message(message& m)
  {
    if (!test_sent_message(m)) {
      m_send_msg[size()].push_back(m);
      ++m_pending_send;
    }
    else {
      message::destroy(&m);
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Removes all the successfully sent messages from the pending queues.
  ////////////////////////////////////////////////////////////////////
  void test_sent_messages(void)
  {
    // test all sent messages
    for (auto it = m_send_msg.begin(); it != m_send_msg.end(); ) {
      auto& q = it->second;
      STAPL_RUNTIME_ASSERT(!q.empty());
      do {
        message& m = q.front();
        if (!test_sent_message(m))
          break;
        q.pop_front();
        --m_pending_send;
        message::destroy(&m);
      } while (!q.empty());
      if (q.empty()) {
        // remove empty queues
        it = m_send_msg.erase(it);
      }
      else {
        ++it;
      }
    }
    STAPL_RUNTIME_STATISTICS("mpi_communicator send queue size",
                             m_pending_send);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Waits for all the messages in the pending and forwarded queues to
  ///        be sent.
  ////////////////////////////////////////////////////////////////////
  void wait_sent_messages(void)
  {
    // wait for all sent messages
    for (auto it = m_send_msg.begin(); it != m_send_msg.end(); ++it) {
      auto& q = it->second;
      STAPL_RUNTIME_ASSERT(!q.empty());
      const size_type queue_size = q.size();
      do {
        message& m = q.front();
        wait_sent_message(m);
        q.pop_front();
        message::destroy(&m);
      } while (!q.empty());
      m_pending_send -= queue_size;
    }
    m_send_msg.clear();

    // wait for forwarded messages
    while (!m_fwd_msg.empty()) {
      message& m = m_fwd_msg.front();
      if (m.handle().mpi_requests().size()>0)
        wait_sent_message(m);
      m_fwd_msg.pop_front();
      message::destroy(&m);
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Notifies that @p m has been forwarded.
  ////////////////////////////////////////////////////////////////////
  void notify_fwd_message(message& m)
  {
    m_fwd_msg.push_back(m);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Removes all the successfully forwarded messages from the pending
  ///        queue and pushes them back in @p c.
  ////////////////////////////////////////////////////////////////////
  template<typename Container>
  void test_fwd_messages(Container& c)
  {
    while (!m_fwd_msg.empty()) {
      message& m = m_fwd_msg.front();
      if (m.handle().mpi_requests().size()>0 && !test_sent_message(m)) {
        break;
      }
      m_fwd_msg.pop_front();
      c.push_back(&m);
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sends @p m to @p dest.
  ////////////////////////////////////////////////////////////////////
  void send_impl(const int dest, message& m)
  {
    MPI_Request& req = request_get(m);
    STAPL_RUNTIME_ASSERT(req==MPI_REQUEST_NULL);
    MPI_Isend(m.data(), m.size(), MPI_BYTE, dest, DEFAULT_TAG, m_comm, &req);
    notify_sent_message(dest, m);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sends @p m, which is a long message, to @p dest.
  ///
  /// This function sends the size of the @p m to the regular message channel
  /// and the actual message to the long message channel.
  ////////////////////////////////////////////////////////////////////
  void send_long_impl(const int dest, message& m)
  {
    auto& reqs = request_get_all(m);
    STAPL_RUNTIME_ASSERT(reqs.size()>1             &&
                         reqs[0]==MPI_REQUEST_NULL &&
                         reqs[1]==MPI_REQUEST_NULL);
    MPI_Isend(m.data(), sizeof(std::size_t), MPI_BYTE,
              dest, DEFAULT_TAG, m_comm, &(reqs[0]));
    MPI_Isend(m.data(), m.size(), MPI_BYTE,
              dest, LONG_TAG, m_comm, &(reqs[1]));
    notify_sent_message(dest, m);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sends @p m to processes in @p r.
  ///
  /// @param r    Processes to send @p m to.
  /// @param m    Message to send.
  /// @param keep @c true if the current process should keep a copy of @p m.
  ////////////////////////////////////////////////////////////////////
  template<typename ForwardRange>
  void send_impl(ForwardRange const& r, message& m, const bool keep)
  {
    auto& reqs = request_get_all(m);
    message_handle::size_type i = 0;
    for (const int dest : r) {
      STAPL_RUNTIME_ASSERT((dest!=get_id()) &&
                           (i<reqs.size())  &&
                           (reqs[i]==MPI_REQUEST_NULL));
      MPI_Isend(m.data(), m.size(), MPI_BYTE,
                dest, DEFAULT_TAG, m_comm, &(reqs[i]));
      ++i;
    }

    // if the message is required in this process, then declare it forwarded,
    // otherwise declare it as sent
    if (keep)
      notify_fwd_message(m);
    else
      notify_sent_message(m);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sends @p m, which is a long message, to processes in @p r.
  ///
  /// @param r    Processes to send @p m to.
  /// @param m    Message to send.
  /// @param keep @c true if the current process should keep a copy of @p m.
  ////////////////////////////////////////////////////////////////////
  template<typename ForwardRange>
  void send_long_impl(ForwardRange&& r, message& m, const bool keep)
  {
    auto& reqs = request_get_all(m);
    message_handle::size_type i = 0;
    for (const int dest : r) {
      STAPL_RUNTIME_ASSERT((dest!=get_id())            &&
                           ((i+1)<reqs.size())         &&
                           (reqs[i]==MPI_REQUEST_NULL) &&
                           (reqs[i+1]==MPI_REQUEST_NULL));
      MPI_Isend(m.data(), sizeof(std::size_t), MPI_BYTE,
                dest, DEFAULT_TAG, m_comm, &(reqs[i]));
      MPI_Isend(m.data(), m.size(), MPI_BYTE,
                dest, LONG_TAG, m_comm, &(reqs[i+1]));
      i += 2;
    }

    // if the message is required in this process, then declare it forwarded,
    // otherwise declare it as sent
    if (keep)
      notify_fwd_message(m);
    else
      notify_sent_message(m);
  }

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Sends the message to the given destination.
  ///
  /// @param dest Destination of the message
  /// @param p    Message to be sent.
  ////////////////////////////////////////////////////////////////////
  void send(const int dest, message_ptr p)
  {
    message& m = *(p.release());
    STAPL_RUNTIME_ASSERT(!m.body_empty() && dest!=get_id());
    const bool long_msg = m.is_long();
    if (long_msg)
      request_get(m, 2);

    std::unique_lock<std::mutex> lock{m_mtx, std::try_to_lock};
    if (lock.owns_lock()) {
      // drain the queue and sent the message
      test_sent_messages();
      m_q.drain(*this);
      if (!long_msg)
        send_impl(dest, m);
      else
        send_long_impl(dest, m);
      check_overloaded();
    }
    else {
      // enqueue the operation
      if (!long_msg)
        m_q.add([dest, &m](mpi_communicator& c) { c.send_impl(dest, m); });
      else
        m_q.add([dest, &m](mpi_communicator& c) { c.send_long_impl(dest, m); });
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sends the message to multiple destinations.
  ///
  /// @param r    Range of destinations.
  /// @param p    Message to be sent.
  /// @param keep @c true if the current process should keep a copy of @p m.
  ////////////////////////////////////////////////////////////////////
  template<typename ForwardRange>
  void send(ForwardRange&& r, message_ptr p, const bool keep)
  {
    message& m   = *(p.release());
    const auto n = boost::size(r);
    STAPL_RUNTIME_ASSERT(!m.body_empty() && ((n!=0) || keep));
    const bool long_msg = m.is_long();
    if (long_msg)
      request_get(m, (2*n));
    else
      request_get(m, n);

    std::unique_lock<std::mutex> lock{m_mtx, std::try_to_lock};
    if (lock.owns_lock()) {
      // drain the queue and sent the message
      test_sent_messages();
      m_q.drain(*this);
      if (!long_msg)
        send_impl(r, m, keep);
      else
        send_long_impl(r, m, keep);
      check_overloaded();
    }
    else {
      // enqueue the operation
      using range_type = typename std::decay<ForwardRange>::type;
      if (!long_msg) {
        m_q.add(std::bind([&m, keep](mpi_communicator& c, range_type const& r)
                          {
                            c.send_impl(r, m, keep);
                          },
                          std::placeholders::_1,
                          std::forward<ForwardRange>(r)));
      }
      else {
        m_q.add(std::bind([&m, keep](mpi_communicator& c, range_type const& r)
                          {
                            c.send_long_impl(r, m, keep);
                          },
                          std::placeholders::_1,
                          std::forward<ForwardRange>(r)));
      }
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Receives all messages that can be received and puts them in the
  ///        given list.
  ///
  /// Tests the buffers initialized for reception, then removes, sorts and
  /// replaces the ones that have already completed. The call can be specified
  /// either blocking or non-blocking and only tests one more buffer than the
  /// number of received messages. It can receive more messages than the length
  /// of the receiving queue.
  ///
  /// @param wait @c true if the call should be blocking, @c false if not.
  ///
  /// @return The number of received messages.
  ///
  /// @bug The receiving queue can grow but it never shrinks.
  /// @todo The condition for waiting should include forwarded messages as well.
  ///       Right now, it just ignores waiting if there are forwarded messages.
  ////////////////////////////////////////////////////////////////////
  message_slist receive(const bool wait)
  {
    message_slist msg_list;

    std::lock_guard<std::mutex> lock{m_mtx};
    m_q.drain(*this);

    // We count the number of messages a single call to try_receive() allows to
    // receive. In the case where this number exceeds the number we anticipated,
    // we increase this number and the size of the queue.
    // Semantically, the intensity of the communication on this node is
    // increasing.
    // TODO Reversely if after a certain number of checks we did not use a
    // significant amount of the pre-fired receiving messages, we shrink the
    // size.
    const size_type queue_size = m_recv_msg.size();
    const size_type max_recv   = std::max(2 * queue_size, sm_recv_max);
    size_type recv_counter     = 0;

    // check if everything has been sent
    test_sent_messages();

    // wait for a message
    if (wait && m_fwd_msg.empty()) {
      // no forwarded messages, so we can afford to wait

      // receive a message
      STAPL_RUNTIME_ASSERT(!m_recv_msg.empty());
      message* m = &(m_recv_msg.front());
      const std::pair<int, std::size_t> p = wait_receive(*m);
      m_recv_msg.pop_front();

      if (p.second==sizeof(std::size_t)) {
        // long message incoming; prepare to receive it
        const std::size_t size = m->size();
        m->reset();
        post_receive_message(m);
        m = wait_receive_long(p.first, size);
      }
      else if (p.second<=message::default_capacity()) {
        // regular message received
        create_receive_message();
      }

      msg_list.push_back(m);
      ++recv_counter;
    }
    else {
      // add to the queue all completed forwarded messages
      test_fwd_messages(msg_list);
    }

    check_overloaded();

    // message receiving loop
    for (; recv_counter<max_recv; ++recv_counter) {
      // receive a message
      STAPL_RUNTIME_ASSERT(!m_recv_msg.empty());
      message* m = &(m_recv_msg.front());
      const std::pair<int, std::size_t> p = test_receive(*m);
      if (p.first==MPI_PROC_NULL)
        break;
      m_recv_msg.pop_front();

      if (p.second==sizeof(std::size_t)) {
        // long message incoming; prepare to receive it
        const std::size_t size = m->size();
        m->reset();
        post_receive_message(m);
        m = try_receive_long(p.first, size);
        if (m==0)
          break;
      }
      else if (p.second<=message::default_capacity()) {
        // regular message received
        create_receive_message();
      }

      msg_list.push_back(m);
    }
    STAPL_RUNTIME_ASSERT(sm_recv_min<=m_recv_msg.size());

    // increase the receiving queue if more than the posted were received and
    // the queue did not exit maximum size
    if (recv_counter>queue_size && queue_size<m_recv_msg.size()) {
      const size_type nsize = std::min(queue_size, (sm_recv_max - queue_size));
      create_receive_message(nsize);
    }

    STAPL_RUNTIME_STATISTICS("mpi_communicator receives succeeded",
                             recv_counter);
    STAPL_RUNTIME_STATISTICS("mpi_communicator receive queue size",
                             m_recv_msg.size());

    return msg_list;
  }
};
mpi_communicator::size_type mpi_communicator::sm_recv_min        = 0;
mpi_communicator::size_type mpi_communicator::sm_recv_max        = 0;
mpi_communicator::size_type mpi_communicator::sm_send_max        = 0;
bool                        mpi_communicator::sm_mpi_initialized = false;

} // namespace runtime

} // namespace stapl

#endif
