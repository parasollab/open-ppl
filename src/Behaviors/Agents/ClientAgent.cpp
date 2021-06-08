#include "ClientAgent.h"

#include "Communication/Messages/Message.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/XMLNode.h"

/*-------------------------------------- Construction -------------------------------------*/

ClientAgent::
ClientAgent(Robot* const _r) : Agent(_r) {}

ClientAgent::
ClientAgent(Robot* const _r, XMLNode& _node) : Agent(_r,_node) {
  m_controlChannel = _node.Read("controlChannel",true,"","Channel to listen for controls on.");
}

ClientAgent::
ClientAgent(Robot* const _r, const ClientAgent& _a) : Agent(_r,_a)
{ }

ClientAgent::
~ClientAgent() {}

std::unique_ptr<Agent>
ClientAgent::
Clone(Robot* const _r) const {
  return std::unique_ptr<ClientAgent>(new ClientAgent(_r, *this));
}

/*----------------------------------- Simulator Interface ---------------------------------*/

void
ClientAgent::
Initialize() {

  if(!m_communicator->GetSubscriber(m_controlChannel))
    throw RunTimeException(WHERE) << "Client agent not subscribed to control channel: "
                                  << m_controlChannel
                                  << "."
                                  << std::endl;

  m_running = true;
  auto listen = [this](){this->ListenForControls();};
  m_thread = std::thread(listen);
}

void
ClientAgent::
Uninitialize() {
  /*std::lock_gaurd<std::mutex> guard(m_lock);

  m_running = false;
  if(m_thread.joinable())
    m_thread.join();*/
}

void
ClientAgent::
Step(const double _dt) {
  /*if(m_controlQueue.empty())
    return;

  auto c = m_controlQueue.pop_front();
  */
  auto c = ListenForControls();

  ExecuteControls(c.second,c.first);
}

/*------------------------------------ Helper Functions ------------------------------------*/

std::pair<size_t,ControlSet>
ClientAgent::
ListenForControls() {
  //while(m_running) {
    std::string query = "control_request/" + m_robot->GetLabel();
    auto response = m_communicator->Query(m_controlChannel,query);
    //if(response == "quit")
    //	break;
    auto control = MessageToControlSet(response,m_robot->GetMPProblem());
    //m_controlQueue.push_back(control);
  //}
  return control;
}

/*------------------------------------------------------------------------------------------*/
