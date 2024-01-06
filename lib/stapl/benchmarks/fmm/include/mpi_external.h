#ifndef STAPL_BENCHMARKS_FMM_MPI_EXTERNAL_H
#define STAPL_BENCHMARKS_FMM_MPI_EXTERNAL_H


#include <stapl/runtime.hpp>
#include <iostream>

#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif

void reduce_d(double* val)
{
#ifndef STAPL_DONT_USE_MPI
  MPI_Allreduce(MPI_IN_PLACE, val, 1, MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD);
#endif
}

void shiftBodies(std::vector<Body> & bodies,int mpirank, int mpisize) {
#ifndef STAPL_DONT_USE_MPI
  // New number of bodies
  int newSize;
  // Current number of bodies
  int oldSize = bodies.size();
  // Size of body structure
  const int body_size = sizeof(bodies[0]);
  // Send to next rank (wrap around)
  const int isend = (mpirank + 1          ) % mpisize;
  // Receive from previous rank (wrap around)
  const int irecv = (mpirank - 1 + mpisize) % mpisize;
  // Send, receive request handles
  MPI_Request sreq,rreq;

  // Send current number of bodies
  MPI_Isend(&oldSize, 1, MPI_INT, irecv, 0, MPI_COMM_WORLD, &sreq);
  // Receive new number of bodies
  MPI_Irecv(&newSize, 1, MPI_INT, isend, 0, MPI_COMM_WORLD, &rreq);
  // Wait for send to complete
  MPI_Wait(&sreq, MPI_STATUS_IGNORE);
  // Wait for receive to complete
  MPI_Wait(&rreq, MPI_STATUS_IGNORE);

  std::vector<Body> recvBodies(newSize);
  // Send bodies to next rank
  MPI_Isend(&bodies[0], oldSize*body_size, MPI_BYTE, irecv,
              1, MPI_COMM_WORLD, &sreq);
  // Receive bodies from previous rank
  MPI_Irecv(&recvBodies[0], newSize*body_size, MPI_BYTE, isend,
              1, MPI_COMM_WORLD, &rreq);
  // Wait for send to complete
  MPI_Wait(&sreq, MPI_STATUS_IGNORE);
  // Wait for receive to complete
  MPI_Wait(&rreq, MPI_STATUS_IGNORE);
  // Copy bodies from buffer
  bodies = recvBodies;
#endif
}
#endif

