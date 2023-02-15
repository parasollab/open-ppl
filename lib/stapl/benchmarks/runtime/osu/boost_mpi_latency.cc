/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark for @ref boost::mpi::communicator::send() and
/// @ref boost::mpi::communicator::recv() latency for two transports of data
/// (ping-pong).
///
/// The user can set the threshold that chooses between a regular send and
/// receive vs one that uses the Boost.MPI skeleton objects.
///
/// It can be compared against
/// -# @c mpi/one-sided/osu_put_latency.c
/// -# @c mpi/pt2pt/osu_latency.c
/// in OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# Boost MPI%s Latency Test"

#include <boost/mpi/environment.hpp>
#include <boost/mpi/communicator.hpp>
#include <boost/mpi/collectives.hpp>
#include <boost/mpi/skeleton_and_content.hpp>
#include <boost/serialization/array.hpp>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <utility>
#include <boost/lexical_cast.hpp>

#ifdef _ENABLE_OPENACC_
#include <openacc.h>
#endif

#ifdef _ENABLE_CUDA_
#include <cuda.h>
#include <cuda_runtime.h>
#endif

#ifndef FIELD_WIDTH
# define FIELD_WIDTH 20
#endif

#ifndef FLOAT_PRECISION
# define FLOAT_PRECISION 2
#endif

#define MESSAGE_ALIGNMENT  64
#define MAX_ALIGNMENT      65536
#define MAX_MSG_SIZE       (1<<22)
#define MYBUFSIZE          (MAX_MSG_SIZE + MAX_ALIGNMENT)
#define LOOP_LARGE         100
#define SKIP_LARGE         10
#define LARGE_MESSAGE_SIZE 8192

#ifdef _ENABLE_OPENACC_
#   define OPENACC_ENABLED 1
#else
#   define OPENACC_ENABLED 0
#endif

#ifdef _ENABLE_CUDA_
#   define CUDA_ENABLED 1
#else
#   define CUDA_ENABLED 0
#endif

char s_buf_original[MYBUFSIZE];
char r_buf_original[MYBUFSIZE];

int skip = 1000;
int loop = 10000;


#ifdef _ENABLE_CUDA_
CUcontext cuContext;
#endif


enum po_ret_type
{
  po_cuda_not_avail,
  po_openacc_not_avail,
  po_bad_usage,
  po_help_message,
  po_okay,
};


enum accel_type
{
  none,
  cuda,
  openacc
};


struct opts
{
  char src;
  char dst;
  enum accel_type accel;
  int threshold;
} options;


void usage(void);
int init_cuda_context(void);
int destroy_cuda_context(void);
int process_options(int argc, char *argv[]);
int allocate_memory(char **sbuf, char **rbuf, int rank);
void print_header(int rank);
void touch_data(void *sbuf, void *rbuf, int rank, size_t size);
void free_memory(void *sbuf, void *rbuf, int rank);


class window
{
private:
  std::size_t m_size;
  char*       m_buf;

public:
  constexpr window(char* buf, std::size_t size) noexcept
  : m_size(size),
    m_buf(buf)
  { }

  template<class Archive>
  void serialize(Archive &ar, const unsigned int)
  {
    ar & m_size;
    ar & boost::serialization::make_array(m_buf, m_size);
  }
};


struct mpi_transmitter
{
  template<typename T>
  static void send(T&& t,
                   const int dest,
                   const int tag,
                   boost::mpi::communicator const& comm)
  {
    comm.send(dest, tag, std::forward<T>(t));
  }

  template<typename T>
  static void receive(T& t,
                      const int src,
                      const int tag,
                      boost::mpi::communicator const& comm)
  {
    comm.recv(src, tag, t);
  }

  template<typename T>
  static void send_skeleton(T&& t,
                            const int dest,
                            const int tag,
                            boost::mpi::communicator const& comm)
  {
    using namespace boost::mpi;

    typedef typename std::decay<T>::type value_type;

    skeleton_proxy<const value_type> p = skeleton(t);
    comm.send(dest, tag, p);
    content c = get_content(t);
    comm.send(dest, tag, c);
  }

  template<typename T>
  static void receive_skeleton(T& t,
                               const int src,
                               const int tag,
                               boost::mpi::communicator const& comm)
  {
    using namespace boost::mpi;

    typedef T value_type;

    skeleton_proxy<T> p = skeleton(t);
    comm.recv(src, tag, p);
    content c = get_content(t);
    comm.recv(src, tag, c);
  }
};


int main(int argc, char *argv[])
{
  double t_start = 0.0, t_end = 0.0;
  int po_ret = process_options(argc, argv);

  if (po_okay == po_ret && cuda == options.accel) {
    if (init_cuda_context()) {
      std::fprintf(stderr, "Error initializing cuda context\n");
      std::exit(EXIT_FAILURE);
    }
  }

  namespace mpi = boost::mpi;

  mpi::environment env(argc, argv);
  mpi::communicator world;

  int numprocs = world.size();
  int myid = world.rank();

  if (0 == myid) {
    switch (po_ret) {
      case po_cuda_not_avail:
        std::fprintf(stderr, "CUDA support not enabled.  Please recompile "
                             "benchmark with CUDA support.\n");
      break;
      case po_openacc_not_avail:
        std::fprintf(stderr, "OPENACC support not enabled.  Please recompile "
                             "benchmark with OPENACC support.\n");
      break;
      case po_bad_usage:
      case po_help_message:
        usage();
      break;
    }
  }

  switch (po_ret) {
    case po_cuda_not_avail:
    case po_openacc_not_avail:
    case po_bad_usage:
      std::exit(EXIT_FAILURE);
    case po_help_message:
      std::exit(EXIT_SUCCESS);
    case po_okay:
    break;
  }

  if (numprocs != 2) {
    std::fprintf(stderr, "This test requires exactly two processes\n");
    std::exit(EXIT_FAILURE);
  }

  char *s_buf = 0, *r_buf = 0;
  if (allocate_memory(&s_buf, &r_buf, myid)) {
    /* Error allocating memory */
    std::exit(EXIT_FAILURE);
  }

  print_header(myid);

  /* Latency test */
  for (int size = 0; size <= MAX_MSG_SIZE; size = (size ? size * 2 : 1)) {
    touch_data(s_buf, r_buf, myid, size);

    if (size > LARGE_MESSAGE_SIZE) {
      loop = LOOP_LARGE;
      skip = SKIP_LARGE;
    }

    const bool past_threshold = (size>=options.threshold);
    const window sw(s_buf, size);
    window rw(r_buf, size);

    world.barrier();

    if (myid == 0) {
      if (!past_threshold) {
        for (int i = 0; i < loop + skip; i++) {
          if (i == skip)
            t_start = MPI_Wtime();
          mpi_transmitter::send(sw, 1, 1, world);
          mpi_transmitter::receive(rw, 1, 1, world);
        }
      }
      else {
        for (int i = 0; i < loop + skip; i++) {
          if (i == skip)
            t_start = MPI_Wtime();
          mpi_transmitter::send_skeleton(sw, 1, 1, world);
          mpi_transmitter::receive_skeleton(rw, 1, 1, world);
        }
      }
      t_end = MPI_Wtime();
    }
    else if (myid == 1) {
      if (!past_threshold) {
        for (int i = 0; i < loop + skip; i++) {
          mpi_transmitter::receive(rw, 0, 1, world);
          mpi_transmitter::send(sw, 0, 1, world);
        }
      }
      else {
        for (int i = 0; i < loop + skip; i++) {
          mpi_transmitter::receive_skeleton(rw, 0, 1, world);
          mpi_transmitter::send_skeleton(sw, 0, 1, world);
        }
      }
    }

    if (myid == 0) {
      const double latency = (t_end - t_start) * 1e6 / (2.0 * loop);
      std::fprintf(stdout, "%-*d%*.*f\n", 10, size, FIELD_WIDTH,
      FLOAT_PRECISION,
              latency);
      std::fflush(stdout);
    }
  }

  free_memory(s_buf, r_buf, myid);

  if (cuda == options.accel) {
    if (destroy_cuda_context()) {
      std::fprintf(stderr, "Error destroying cuda context\n");
      std::exit(EXIT_FAILURE);
    }
  }

  return EXIT_SUCCESS;
}


void usage(void)
{
  if (CUDA_ENABLED || OPENACC_ENABLED) {
    std::printf("Usage: boost_mpi_latency [options] [RANK0 RANK1]\n\n");
    std::printf("RANK0 and RANK1 may be `D' or `H' which specifies whether\n"
                "the buffer is allocated on the accelerator device or host\n"
                "memory for each mpi rank\n\n");
  }

  else {
    std::printf("Usage: boost_mpi_latency -t THRESHOLD [options]\n\n");
  }

  std::printf("options:\n");
  std::printf("  -t THRESHOLD  data size threshold in bytes after which to "
                "switch to Boost.MPI skeleton-based send/recv\n");

  if (CUDA_ENABLED || OPENACC_ENABLED) {
    std::printf("  -d TYPE       accelerator device buffers can be of TYPE "
                "`cuda' or `openacc'\n");
  }

  std::printf("  -h            print this help message\n");
  fflush(stdout);
}


int process_options(int argc, char *argv[])
{
  extern char * optarg;
  extern int optind;

  char const * optstring = (CUDA_ENABLED || OPENACC_ENABLED) ? "+d:t:h"
                                                             : "+t:h";
  int c;

  /*
   * set default options
   */
  options.src = 'H';
  options.dst = 'H';

  bool threshold_set = false;

  if (CUDA_ENABLED) {
    options.accel = cuda;
  }

  else if (OPENACC_ENABLED) {
    options.accel = openacc;
  }

  else {
    options.accel = none;
  }

  while ((c = getopt(argc, argv, optstring)) != -1) {
    switch (c) {
      case 'd':
        /* optarg should contain cuda or openacc */
        if (0 == strncasecmp(optarg, "cuda", 10)) {
          if (!CUDA_ENABLED) {
            return po_cuda_not_avail;
          }
          options.accel = cuda;
        }

        else if (0 == strncasecmp(optarg, "openacc", 10)) {
          if (!OPENACC_ENABLED) {
            return po_openacc_not_avail;
          }
          options.accel = openacc;
        }

        else {
          return po_bad_usage;
        }
      break;
      case 'h':
        return po_help_message;
      case 't': {
        try {
          options.threshold = boost::lexical_cast<int>(optarg);
          threshold_set = true;
        }
        catch (boost::bad_lexical_cast const&) {
          return po_bad_usage;
        }
      } break;
      default:
        return po_bad_usage;
    }
  }

  if (!threshold_set)
    return po_bad_usage;

  if (CUDA_ENABLED || OPENACC_ENABLED) {
    if ((optind + 2) == argc) {
      options.src = argv[optind][0];
      options.dst = argv[optind + 1][0];

      switch (options.src) {
        case 'D':
        case 'H':
        break;
        default:
          return po_bad_usage;
      }

      switch (options.dst) {
        case 'D':
        case 'H':
        break;
        default:
          return po_bad_usage;
      }
    }

    else if (optind != argc) {
      return po_bad_usage;
    }
  }

  return po_okay;
}


int init_cuda_context(void)
{
#ifdef _ENABLE_CUDA_
  CUresult curesult = CUDA_SUCCESS;
  CUdevice cuDevice;
  int local_rank, dev_count;
  int dev_id = 0;
  char * str;

  if ((str = getenv("LOCAL_RANK")) != NULL) {
    cudaGetDeviceCount(&dev_count);
    local_rank = atoi(str);
    dev_id = local_rank % dev_count;
  }

  curesult = cuInit(0);
  if (curesult != CUDA_SUCCESS) {
    return 1;
  }

  curesult = cuDeviceGet(&cuDevice, dev_id);
  if (curesult != CUDA_SUCCESS) {
    return 1;
  }

  curesult = cuCtxCreate(&cuContext, 0, cuDevice);
  if (curesult != CUDA_SUCCESS) {
    return 1;
  }
#endif
  return 0;
}


int allocate_device_buffer(char ** buffer)
{
#ifdef _ENABLE_CUDA_
  cudaError_t cuerr = cudaSuccess;
#endif

  switch (options.accel) {
#ifdef _ENABLE_CUDA_
    case cuda:
    cuerr = cudaMalloc((void **)buffer, MYBUFSIZE);

    if (cudaSuccess != cuerr) {
      fprintf(stderr, "Could not allocate device memory\n");
      return 1;
    }
    break;
#endif
#ifdef _ENABLE_OPENACC_
    case openacc:
    *buffer = acc_malloc(MYBUFSIZE);
    if (NULL == *buffer) {
      fprintf(stderr, "Could not allocate device memory\n");
      return 1;
    }
    break;
#endif
    default:
      fprintf(stderr, "Could not allocate device memory\n");
      return 1;
  }

  return 0;
}


char *
align_buffer(void * ptr, unsigned long align_size)
{
  return (char *) (((unsigned long) ptr + (align_size - 1)) / align_size
      * align_size);
}


int allocate_memory(char ** sbuf, char ** rbuf, int rank)
{
  unsigned long align_size = getpagesize();

  assert(align_size <= MAX_ALIGNMENT);

  switch (rank) {
    case 0:
      if ('D' == options.src) {
        if (allocate_device_buffer(sbuf)) {
          std::fprintf(stderr, "Error allocating cuda memory\n");
          return 1;
        }

        if (allocate_device_buffer(rbuf)) {
          std::fprintf(stderr, "Error allocating cuda memory\n");
          return 1;
        }
      }

      else {
        *sbuf = align_buffer(s_buf_original, align_size);
        *rbuf = align_buffer(r_buf_original, align_size);
      }
    break;
    case 1:
      if ('D' == options.dst) {
        if (allocate_device_buffer(sbuf)) {
          std::fprintf(stderr, "Error allocating cuda memory\n");
          return 1;
        }

        if (allocate_device_buffer(rbuf)) {
          std::fprintf(stderr, "Error allocating cuda memory\n");
          return 1;
        }
      }

      else {
        *sbuf = align_buffer(s_buf_original, align_size);
        *rbuf = align_buffer(r_buf_original, align_size);
      }
    break;
  }

  return 0;
}


void print_header(int rank)
{
  if (0 == rank) {
    switch (options.accel) {
      case cuda:
        std::printf(BENCHMARK, "-CUDA");
      break;
      case openacc:
        std::printf(BENCHMARK, "-OPENACC");
      break;
      default:
        std::printf(BENCHMARK, "");
      break;
    }

    switch (options.accel) {
      case cuda:
      case openacc:
        std::printf("# Send Buffer on %s and Receive Buffer on %s\n",
                    'D' == options.src ? "DEVICE (D)" : "HOST (H)",
                    'D' == options.dst ? "DEVICE (D)" : "HOST (H)");
      default:
        std::printf("%-*s%*s\n", 10, "# Size", FIELD_WIDTH, "Latency (us)");
        std::fflush(stdout);
    }
  }
}


void set_device_memory(void * ptr, int data, size_t size)
{
#ifdef _ENABLE_OPENACC_
  size_t i;
  char * p = (char *)ptr;
#endif

  switch (options.accel) {
#ifdef _ENABLE_CUDA_
    case cuda:
    cudaMemset(ptr, data, size);
    break;
#endif
#ifdef _ENABLE_OPENACC_
    case openacc:
#pragma acc parallel loop deviceptr(p)
    for (i = 0; i < size; i++) {
      p[i] = data;
    }
    break;
#endif
    default:
    break;
  }
}


void touch_data(void * sbuf, void * rbuf, int rank, size_t size)
{
  if ((0 == rank && 'H' == options.src) || (1 == rank && 'H' == options.dst)) {
    std::memset(sbuf, 'a', size);
    std::memset(rbuf, 'b', size);
  }
  else {
    set_device_memory(sbuf, 'a', size);
    set_device_memory(rbuf, 'b', size);
  }
}


int free_device_buffer(void * buf)
{
  switch (options.accel) {
#ifdef _ENABLE_CUDA_
    case cuda:
    cudaFree(buf);
    break;
#endif
#ifdef _ENABLE_OPENACC_
    case openacc:
    acc_free(buf);
    break;
#endif
    default:
      /* unknown device */
      return 1;
  }

  return 0;
}


int destroy_cuda_context(void)
{
#ifdef _ENABLE_CUDA_
  CUresult curesult = CUDA_SUCCESS;
  curesult = cuCtxDestroy(cuContext);

  if (curesult != CUDA_SUCCESS) {
    return 1;
  }
#endif
  return 0;
}


void free_memory(void * sbuf, void * rbuf, int rank)
{
  switch (rank) {
    case 0:
      if ('D' == options.src) {
        free_device_buffer(sbuf);
        free_device_buffer(rbuf);
      }
    break;
    case 1:
      if ('D' == options.dst) {
        free_device_buffer(sbuf);
        free_device_buffer(rbuf);
      }
    break;
  }
}
