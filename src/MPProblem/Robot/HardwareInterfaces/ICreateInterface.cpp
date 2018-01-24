#include "ICreateInterface.h"

#include "libplayerc++/playerc++.h"
#include "nonstd/numerics.h"
#include "nonstd/timer.h"

#include "MPProblem/Robot/Actuator.h"


/// The polling period to use for all iCreate interfaces, named here so that
/// it is not a magic number. We set this equal to 300ms because we have
/// observed that m_position2d->SetSpeed takes about 200ms to return.
static constexpr double iCreateCommunicationTime = .3,
                        iCreatePollingPeriod = .3;

/*------------------------------- Construction -------------------------------*/

ICreateInterface::
ICreateInterface(const std::string& _ip, const unsigned short _port)
    : ServerQueueInterface(iCreatePollingPeriod, "iCreate", _ip, _port,
        iCreateCommunicationTime) {
  // Create connections to the iCreate's onboard controller (i.e., the netbook
  // sitting on top of it running player).
  m_client = new PlayerCc::PlayerClient(_ip, _port);
  m_position2d = new PlayerCc::Position2dProxy(m_client, 0); // TODO magic 0

  /// @TODO Check player interface for how to gracefully recover from a failed
  ///       conection attempt.

  m_client->StartThread();

  StartQueue();
}


ICreateInterface::
~ICreateInterface() {
  // Stop the command queue.
  StopQueue();

  // Halt the robot and connection.
  ClearCommandQueue();
  FullStop();
  m_client->Stop();

  delete m_position2d;
  delete m_client;
}

/*------------------------------ Command Queue -------------------------------*/

bool
ICreateInterface::
FullStop() {
  // Halt the robot.
  m_position2d->SetSpeed(0, 0);
  m_lastControls.clear();

  /// @TODO We may need a short wait here for the SetSpeed control to reach the
  ///       robot - as is we may see non-zero velocities.

  // Query the current speeds.
  const double xSpeed = m_position2d->GetXSpeed(),
               ySpeed = m_position2d->GetYSpeed(),
               yawSpeed = m_position2d->GetYawSpeed();

  return nonstd::approx(xSpeed, 0.)
     and nonstd::approx(ySpeed, 0.)
     and nonstd::approx(yawSpeed, 0.);
}

/*-------------------------- Hardware Communication --------------------------*/

void
ICreateInterface::
SendToRobot(const Command& _command) {
  const auto& controlSet = _command.controls;

  // If we are requesting the same action, do not resend.
  if(controlSet == m_lastControls)
    return;
  // If we received an empty control set, this is a wait command.
  else if(controlSet.empty()) {
    FullStop();
    return;
  }
  m_lastControls = controlSet;

  // Determine the translation and rotation signals that we need to send to the
  // iCreate. The control should provide us with a velocity for the X, Y, and
  // theta directions (it is currently labeled 'GetForce' because it was
  // designed for second-order dynamics, but the creates only use first-order
  // and will thus receive a velocity).
  const std::vector<double> vel = AggregatedControlVector(controlSet);

  // Assert that the vector makes sense. The creates have a three-dimensional
  // c-space, so this should be a 3-d vector with 0 as the second element.
  if(vel.size() != 3)
    throw RunTimeException(WHERE, "Control vector has wrong size " +
        std::to_string(vel.size()) + " (expected 3).");
  if(vel[1] != 0.) {
    throw RunTimeException(WHERE, "iCreates should not receive y-controls, but "
        "got request for vel_y = " + std::to_string(vel[1]) + ".");
  }

  // We have determined experimentally that we must divide the rotation speed by
  // two to get 'accurate' results. This seems to be a bug in player.
  const double translation = vel[0],
               rotation    = vel[2] / 2.;

  m_position2d->SetSpeed(translation, rotation);
}

/*----------------------------------------------------------------------------*/
