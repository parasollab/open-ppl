/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/config.hpp>
#include <stapl/runtime/system.hpp>

// demangling support for gcc
#ifdef __GNUG__
# include <cxxabi.h>
# include <cstdlib>
# include <new>
#endif

// pid, memory, processor count
#if defined(STAPL_RUNTIME_BGP_TARGET)
# include <unistd.h>
# include <sys/resource.h>
# include <malloc.h>
# include <spi/kernel_interface.h>
# include <common/bgp_personality.h>
# include <common/bgp_personality_inlines.h>
#elif defined(STAPL_RUNTIME_BGQ_TARGET)
# include <unistd.h>
# include <sys/resource.h>
# include <firmware/include/personality.h>
# include <spi/include/kernel/process.h>
#elif defined(STAPL_RUNTIME_LINUX_TARGET)
# include <unistd.h>
# include <sys/types.h>
# include <sys/resource.h>
# include <sys/sysinfo.h>
#elif defined(STAPL_RUNTIME_WINDOWS_TARGET)
# include <process.h> // for _getpid()
# include <windows.h> // for MEMORYSTATUSEX, SYSTEM_INFO
#elif defined(__APPLE__)
# include <unistd.h>
# include <sys/types.h>
# include <sys/sysctl.h>
# include <sys/resource.h>
#else
# include <unistd.h>
# include <sys/types.h>
# include <sys/resource.h>
#endif


namespace stapl {

namespace runtime {


// --------------------------------------------------------------------------
// Name mangling
// --------------------------------------------------------------------------

// Returns a demangled version of the given mangled name
c_string demangle(const char* name)
{
#ifdef __GNUG__
  int status = 0;
# if !defined(__PATHSCALE__) && (__GNUC__==4 && __GNUC_MINOR__<5)
  // gcc 4.4.x has issues with passing null_ptr and length to
  // __cxa_demangle(). See http://gcc.gnu.org/bugzilla/show_bug.cgi?id=42230
  // PathScale ekoPATH 4 has issues with passing null_ptr for
  // length.
  char* demangled = abi::__cxa_demangle(name, 0, 0, &status);
# else
  size_t length = 0;
  char* demangled = abi::__cxa_demangle(name, 0, &length, &status);
# endif
  switch (status) {
    case -1: {
      // memory allocation failure
      throw std::bad_alloc();
    } break;
    case -2: {
      // invalid name under the C++ ABI mangling rules
      std::free(demangled);
      return c_string(name);
    } break;
    case -3: {
      // invalid argument
      std::free(demangled);
      return c_string{"invalid argument to __cxa_demangle()"};
    } break;
    default:
      break;
  }
  c_string s{demangled};
  free(demangled);
  return s;
#else // __GNUG__
  return c_string{name};
#endif
}

// Returns a demangled version of the given mangled name
c_string demangle(c_string const& name)
{
  return demangle(name.c_str());
}


// --------------------------------------------------------------------------
// Process information
// --------------------------------------------------------------------------

// Returns the process ID of the calling process
int getpid(void) noexcept
{
#ifdef STAPL_RUNTIME_WINDOWS_TARGET
  return _getpid();
#else
  return int(::getpid());
#endif
}


// --------------------------------------------------------------------------
// Memory information
// --------------------------------------------------------------------------

// Returns the total amount of physical memory in bytes
std::size_t get_total_physical_memory(void) noexcept
{
#if defined(STAPL_RUNTIME_BGP_TARGET)
  // from https://www.alcf.anl.gov/resource-guides/determining-memory-use
  _BGP_Personality_t p;
  Kernel_GetPersonality(&p, sizeof(p));
  long long node_mem = BGP_Personality_DDRSizeMB(&p);
  const int node_config = BGP_Personality_processConfig(&p);
  if (node_config == _BGP_PERS_PROCESSCONFIG_VNM)
    node_mem /= 4;
  else if (node_config == _BGP_PERS_PROCESSCONFIG_2x2)
    node_mem /= 2;
  return (node_mem/vnodes * 1024 * 1024); // virtual node RAM size
#elif defined(STAPL_RUNTIME_BGQ_TARGET)
  Personality_t p;
  Kernel_GetPersonality(&p, sizeof(p));
  const std::size_t node_mem = std::size_t(p.DDR_Config.DDRSizeMB);
  const std::size_t vnodes   = std::size_t(Kernel_ProcessCount());
  return (node_mem/vnodes * 1024 * 1024); // virtual node RAM size
#elif defined(STAPL_RUNTIME_LINUX_TARGET) && \
      !defined(STAPL_RUNTIME_CRAY_TARGET)
  struct sysinfo info;
  sysinfo(&info);
  return (std::size_t(info.totalram) * info.mem_unit);
#elif defined(_AIX)
  return std::size_t(sysconf(_SC_AIX_REALMEM) * 1024);
#elif defined(STAPL_RUNTIME_WINDOWS_TARGET)
  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof(statex);
  GlobalMemoryStatusEx(&statex);
  return std::size_t(statex.ullTotalPhys);
#elif defined(__APPLE__)
  int mib[2] = { CTL_HW, HW_MEMSIZE };
  uint64_t mem = 0;
  size_t length = sizeof(mem);
  sysctl(mib, 2, &mem, &length, NULL, 0);
  return std::size_t(mem);
#else
  return std::size_t(sysconf(_SC_PHYS_PAGES) * sysconf(_SC_PAGE_SIZE));
#endif
}

// Returns the amount of physical memory that is being used in bytes
std::size_t get_used_physical_memory(void) noexcept
{
#if defined(STAPL_RUNTIME_BGP_TARGET)
  const struct mallinfo m = mallinfo();
  const long long alloc = (m.hblkhd + m.uordblks);
  return std::size_t(alloc);
#elif defined(STAPL_RUNTIME_LINUX_TARGET) && \
      !defined(STAPL_RUNTIME_CRAY_TARGET)
  return (get_total_physical_memory() - get_available_physical_memory());
#elif defined(STAPL_RUNTIME_WINDOWS_TARGET)
  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof(statex);
  GlobalMemoryStatusEx(&statex);
  return std::size_t(statex.ullTotalPhys - statex.ullAvailPhys);
#elif defined(STAPL_RUNTIME_SOLARIS_TARGET)
  return std::size_t( (sysconf(_SC_PHYS_PAGES) - sysconf(_SC_AVPHYS_PAGES)) *
                      sysconf(_SC_PAGE_SIZE) );
#else
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return (std::size_t(usage.ru_maxrss)*1024);
#endif
}

// Returns the amount of physical memory available in bytes
std::size_t get_available_physical_memory(void) noexcept
{
#if defined(STAPL_RUNTIME_LINUX_TARGET) && \
    !defined(STAPL_RUNTIME_CRAY_TARGET)
  struct sysinfo info;
  sysinfo(&info);
  return (std::size_t(info.freeram) * info.mem_unit);
#elif defined(STAPL_RUNTIME_WINDOWS_TARGET)
  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof(statex);
  GlobalMemoryStatusEx(&statex);
  return std::size_t(statex.ullAvailPhys);
#elif defined(STAPL_RUNTIME_SOLARIS_TARGET)
  return std::size_t(sysconf(_SC_AVPHYS_PAGES) * sysconf(_SC_PAGE_SIZE));
#else
  return (get_total_physical_memory() - get_used_physical_memory());
#endif
}

// Returns the percentage of physical memory that is being used (0-100)
std::size_t get_physical_memory_load(void) noexcept
{
#if defined(STAPL_RUNTIME_WINDOWS_TARGET)
  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof(statex);
  GlobalMemoryStatusEx(&statex);
  return std::size_t(statex.ullAvailPhys);
#else
  return ((100*get_used_physical_memory())/get_total_physical_memory());
#endif
}


// --------------------------------------------------------------------------
// Processing Resources information
// --------------------------------------------------------------------------

// Returns the number of hardware threads on the system
std::size_t get_num_hardware_threads(void) noexcept
{
#if defined(STAPL_RUNTIME_BGP_TARGET)
  const std::size_t node_procs = 4; // has 4 cores per physical node
  const std::size_t vnodes     = std::size_t(Kernel_ProcessCount());
  return (node_procs/vnodes);
#elif defined(STAPL_RUNTIME_BGQ_TARGET)
  const std::size_t node_procs = 16; // has 16 cores per physical node
  const std::size_t vnodes     = std::size_t(Kernel_ProcessCount());
  return (node_procs/vnodes);
#elif defined(STAPL_RUNTIME_WINDOWS_TARGET)
  SYSTEM_INFO sysinfo;
  GetSystemInfo(&sysinfo);
  return std::size_t(sysinfo.dwNumberOfProcessors);
#else
  return std::size_t(sysconf(_SC_NPROCESSORS_ONLN));
#endif
}

} // namespace runtime

} // namespace stapl
