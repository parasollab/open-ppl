/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_LOGGER_H
#define STAPL_BENCHMARKS_FMM_LOGGER_H

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <pthread.h>
#include <queue>
#include <stdint.h>
#include <string>
#include <sstream>
#include <sys/time.h>
#include "thread.h"
#include <vector>

#if PAPI
#include <cstring>
#include <papi.h>
#endif

/// Structure for pthread based tracer
struct Tracer
{
  /// pthread id
  pthread_t thread;
  /// Begin timer of tracer
  double    begin;
  /// End timer of tracer
  double    end;
  /// Constructor
  Tracer() {}
};

/// Timer and Tracer logger
namespace logger {
  /// Map of timer event name to timed value
  typedef std::map<std::string,double> Timer;
  /// Iterator of timer event name map
  typedef Timer::iterator              T_iter;
  /// Queue of tracers
  typedef std::queue<Tracer>           Tracers;
  /// Map of pthread id to thread id
  typedef std::map<pthread_t,int>      ThreadMap;

  /// Timer base value
  Timer           beginTimer;
  /// Timings of all events
  Timer           timer;
  /// Tracers for all events
  Tracers         tracers;
  /// Pthread communicator
  pthread_mutex_t mutex;
#if PAPI
  /// PAPI event set
  int                    PAPIEventSet = PAPI_NULL;
  /// Vector of PAPI event names
  std::vector<char*>     PAPIEventNames;
  /// Vector of PAPI event codes
  std::vector<int>       PAPIEventCodes;
  /// Vector of PAPI event values
  std::vector<uint64_t>  PAPIEventValues;
#endif

  /// Max length of event name
  int stringLength = 20;
  /// Decimal precision
  int decimal = 7;
  /// Print to screen
  bool verbose = false;

  /// Timer function
  double get_time()
  {
    // Time value
    struct timeval tv;
    // Get time of day in seconds and microseconds
    gettimeofday(&tv, NULL);
    // Combine seconds and microseconds and return
    return double(tv.tv_sec+tv.tv_usec*1e-6);
  }

  /// Cycle counter
  inline uint64_t get_cycle()
  {
    // Define low and high 32 bits of cycle counter
    uint32_t low = 0, high = 0;
    // Call rdtsc
    asm volatile ("rdtsc" : "=a" (low), "=d" (high));
    // Return 64 bit cycle counter
    return (uint64_t(high) << 32) | uint64_t(low);
  }

  /// Cycle counter with thread ID
  inline uint64_t get_cycle(uint32_t * id)
  {
    // Define low and high 32 bits of cycle counter
    uint32_t low = 0, high = 0;
    // Count only for valid thread ID
    if (!id) return 0;
    // Call rdtscp
    asm volatile ("rdtscp" : "=a" (low), "=d" (high), "=c" (*id));
    // Return 64 bit cycle counter
    return (uint64_t(high) << 32) | uint64_t(low);
  }

  /// Print message to standard output
  inline void printTitle(std::string title)
  {
    // If verbose flag is true
    if (verbose) {
      //  Append space to end of title
      title += " ";
      //  Align string length
      std::cout << "--- " << std::setw(stringLength)
                //  Left shift
                << std::left
                //  Set to fill with '-'
                << std::setfill('-')
                //  Fill until end of line
                << title << std::setw(10) << "-"
                //  Set back to fill with ' '
                << std::setfill(' ') << std::endl;
    // End if for verbose flag
    }
  }

  /// Start timer for given event
  inline void startTimer(std::string event)
  {
    // Get time of day and store in beginTimer
    beginTimer[event] = get_time();
  }

  /// Print timings of a specific event
  inline void printTime(std::string event)
  {
    // If verbose flag is true
    if (verbose) {
      //  Set format
      std::cout << std::setw(stringLength) << std::left
    << event << " : " << std::setprecision(decimal) << std::fixed
    //  Print event and timer
    << timer[event] << " s" << std::endl;
    // End if for verbose flag
    }
  }

  /// Stop timer for given event
  double stopTimer(std::string event, int print=1)
  {
    // Get time of day and store in endTimer
    double endTimer = get_time();
    // Accumulate event time to timer
    timer[event] += endTimer - beginTimer[event];
    // Print event and timer to screen
    if (verbose && print) printTime(event);
    // Return the event time
    return endTimer - beginTimer[event];
  }

  /// Write timings of all events
  inline void writeTime(int mpirank=0) {
    // File name
    std::stringstream name;
    // Set format
    name << "time" << std::setfill('0') << std::setw(6)
         // Create file name for timer
         << mpirank << ".dat";
    // Open timer log file
    std::ofstream timerFile(name.str().c_str(), std::ios::app);
    // Loop over all events
    for (T_iter E=timer.begin(); E!=timer.end(); E++) {
      //  Set format
      timerFile << std::setw(stringLength) << std::left
    //  Print event and timer
    << E->first << " " << E->second << std::endl;
    // End loop over all events
    }
    // Close timer log file
    timerFile.close();
    // Clear timer
    timer.clear();
  }

  /// Erase single event in timer
  inline void resetTimer(std::string event)
  {
    // Erase event from timer
    timer.erase(event);
  }

  /// Erase all events in timer
  inline void resetTimer()
  {
    // Clear timer
    timer.clear();
  }

  /// Start PAPI event
  inline void startPAPI()
  {
#if PAPI
    // Initialize PAPI library
    PAPI_library_init(PAPI_VER_CURRENT);
    // Get all PAPI event strings
    char * allEvents = getenv("EXAFMM_PAPI_EVENTS");
    // PAPI event name
    char eventName[256];
    // While event string is not empty
    while (allEvents) {
      //  Get single event string
      char * event = strchr(allEvents, ',');
      // Count string length
      int n = (event == NULL ? (int)strlen(allEvents) : event - allEvents);
      //  PAPI event code
      int eventCode;
      //  Get PAPI event name
      snprintf(eventName, n+1, "%s", allEvents);
      // Event name to event code
      if (PAPI_event_name_to_code(eventName, &eventCode) == PAPI_OK) {
        //   Push event name to vector
        PAPIEventNames.push_back(strdup(eventName));
        //   Push event code to vector
        PAPIEventCodes.push_back(eventCode);
      //  End if for event name to event code
      }
      //  Stop if event string is empty
      if (event == NULL) break;
      //  Else move to next event string
      else allEvents = event + 1;
    // End while loop for event string
    };
    // If PAPI events are set
    if (!PAPIEventCodes.empty()) {
      // Create PAPI event set
      PAPI_create_eventset(&PAPIEventSet);
      // Loop over PAPI events
      for (int i=0; i<int(PAPIEventCodes.size()); i++) {
  //  Add PAPI event
  PAPI_add_event(PAPIEventSet, PAPIEventCodes[i]);
      // End loop over PAPI events
      }
      // Start PAPI counter
      PAPI_start(PAPIEventSet);
    }
#endif
  }

  /// Stop PAPI event
  inline void stopPAPI()
  {
#if PAPI
    // If PAPI events are set
    if (!PAPIEventCodes.empty()) {
      //  Resize PAPI event value vector
      PAPIEventValues.resize(PAPIEventCodes.size());
      //  Stop PAPI counter
      PAPI_stop(PAPIEventSet, &PAPIEventValues[0]);
    // End if for PAPI events
    }
#endif
  }

  /// Print PAPI event
  inline void printPAPI()
  {
#if PAPI
    // If PAPI events are set and verbose is true
    if (!PAPIEventCodes.empty() && verbose) {
      printTitle("PAPI stats ");
      //  Loop over PAPI events
      for (int i=0; i<int(PAPIEventCodes.size()); i++) {
        //   Set format
        std::cout << std::setw(stringLength) << std::left
      << PAPIEventNames[i] << " : " << std::setprecision(decimal) << std::fixed
      //   Print event and timer
      << PAPIEventValues[i] << std::endl;
      //  End loop over PAPI events
      }
    // End if for PAPI events
    }
#endif
  }

#if TRACE
  /// Initialize tracer
  inline void initTracer()
  {
    // Initialize pthread communicator
    pthread_mutex_init(&mutex,NULL);
  }

  /// Start tracer for given event
  inline void startTracer(Tracer &tracer)
  {
    // Lock shared variable access
    pthread_mutex_lock(&mutex);
    // Store pthread id
    tracer.thread = pthread_self();
    // Start timer
    tracer.begin  = get_time();
    // Unlock shared variable access
    pthread_mutex_unlock(&mutex);
  }

  /// Stop tracer for given event
  inline void stopTracer(Tracer & tracer)
  {
    // Lock shared variable access
    pthread_mutex_lock(&mutex);
    // Stop timer
    tracer.end = get_time();
    // Push tracer to queue of tracers
    tracers.push(tracer);
    // Unlock shared variable access
    pthread_mutex_unlock(&mutex);
  }

  /// Write tracers of all events
  inline void writeTracer(int mpirank=0)
  {
    // Start timer
    startTimer("Write tracer");
    // File name
    std::stringstream name;
    // Set format
    name << "trace" << std::setfill('0') << std::setw(6)
         // Create file name for tracer
         << mpirank << ".svg";
    // Open tracer log file
    std::ofstream traceFile(name.str().c_str());
    // Header statements for tracer log file
    traceFile << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        << "<!DOCTYPE svg PUBLIC \"-_W3C_DTD SVG 1.0_EN\" \""
        << "http://www.w3.org/TR/SVG/DTD/svg10.dtd\">\n"
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n"
        << "  width=\"200mm\" height=\"40mm\" viewBox=\"0 0 20000 4000\">\n"
        << "  <g>\n";
    // Counter for number of threads to trace
    int num_thread = 0;
    // Map pthread ID to thread ID
    ThreadMap threadMap;
    // Base time
    double base = tracers.front().begin;
    // Scale the length of bar plots
    double scale = 30000.0;
    // While queue of traces is not empty
    while (!tracers.empty()) {
      //  Get tracer at front of the queue
      Tracer tracer = tracers.front();
      //  Pop tracer at front
      tracers.pop();
      //  Set pthread ID of tracer
      pthread_t thread = tracer.thread;
      //  Set begin time of tracer
      double begin  = tracer.begin;
      //  Set end time of tracer
      double end    = tracer.end;
      //  Set color of tracer
      int    color  = 0x0000ff;
      //  If it's a new pthread ID
      if (threadMap[thread] == 0) {
        //   Map it to an incremented thread ID
        threadMap[thread] = num_thread++;
      //  End if for new pthread ID
      }
      //  Subtract base time from begin time
      begin -= base;
      //  Subtract base time from end time
      end   -= base;
      //  x position of bar plot
      traceFile << "    <rect x=\"" << begin * scale
                //  y position of bar plot
                << "\" y=\"" << threadMap[thread] * 100.0
                //  width of bar
                << "\" width=\"" << (end - begin) * scale
                << "\" height=\"90.0\" fill=\"#"<< std::setfill('0')
                // height of bar
                << std::setw(6) << std::hex << color
                //  stroke color and width
                << "\" stroke=\"#000000\" stroke-width=\"1\"/>\n";
    // End while loop for queue of tracers
    }
    // Footer for tracer log file
    traceFile << "  </g>\n" "</svg>\n";
    // Close tracer log file
    traceFile.close();
    // Stop timer
    stopTimer("Write tracer",verbose);
  }
#else
  inline void initTracer() {}
  inline void startTracer(Tracer) {}
  inline void stopTracer(Tracer) {}
  inline void writeTracer() {}
  inline void writeTracer(int) {}
#endif

#if DAG_RECORDER == 2
  /// Start DAG recorder
  inline void startDAG()
  {
    // Start DAG recorder
    dr_start(0);
  }

  /// Stop DAG recorder
  inline void stopDAG()
  {
    // Stop DAG recorder
    dr_stop();
  }

  /// Write DAG to file
  inline void writeDAG()
  {
    // Write DAG to file
    dr_dump();
  }
#else
  inline void startDAG() {}
  inline void stopDAG() {}
  inline void writeDAG() {}
#endif
};

#endif // STAPL_BENCHMARKS_FMM_LOGGER_H
