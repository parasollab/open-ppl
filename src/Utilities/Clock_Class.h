// $Id$
/**@file Clock_Class.h
  *This class provides methods to handle clocks to time events.
  *@date 1/27/99
  *@author Chris Jones
*/

/////////////////////////////////////////////////////////////////////////////////////////
#ifndef _CLOCK_CLASS_H_
#define _CLOCK_CLASS_H_

/////////////////////////////////////////////////////////////////////////////////////////

/**Provide timing information.
  *This class is used to measure the running time between StartClock and
  *StopClock. Client side could provide clock name, when StopClock is called
  *the name will be print out, and running time as well.
  */
class Clock_Class {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Constructors and Destructor.*/
    //-----------------------------------------------------------
    //@{
      ///Default constructor. Set every thing to zero
      Clock_Class();

      ///Destrcutor. Do nothing
      ~Clock_Class();
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Clock methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Clock Methods.*/
    //-----------------------------------------------------------
    //@{

      ///Set every thing to zero
      int ClearClock();

      ///Start the clock and the name is identity of this clock.
      int StartClock( char *Name );

      ///Stop the clock and calculate the total running time.
      int StopClock();

      ///Call StopClock and PrintClock.
      int StopPrintClock();

    //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Input and Ouput Methods.*/
    //-----------------------------------------------------------
    //@{

      /**Output the clock name given in StartClock and runinng time accosited with this name
        *to the standard output.
        */
      void PrintClock();

      ///Output the clock name given in StartClock to the standard output
      void PrintName();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Access Methods.*/
    //-----------------------------------------------------------
    //@{

      ///Get how many seconds of running time are. (integer, without fraction part!!)
      int GetClock();

      ///Get how many seconds of running time are.
      double GetClock_SEC();

      ///Get how many 1e-6 seconds of running time are.
      int GetClock_USEC();
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data Members and MEmber Methods Methods (Undocumented)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

private:

  int s_time, u_time;
  int s_utime, u_utime;
  char ClockName[50];
};

#endif // _CLOCK_CLASS_H_
