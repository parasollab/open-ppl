#include "Chain.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"

#include "Utilities/PMPLExceptions.h"

#include <algorithm>
#include <queue>


/*-------------------------------- Generation --------------------------------*/

std::vector<Chain>
Chain::
Decompose(const MultiBody* _mb) {
  // Composite multibodies are not supported.
  /// @todo We can probably support composite bodies here, but it is not
  ///       immediately required. Need to test this separately.
  if(_mb->IsComposite())
    throw RunTimeException(WHERE) << "Composite bodies are not yet supported.";

  std::vector<Chain> chains;

  // Start from the base and scan outward. Repeat until the entire multibody is
  // decomposed.
  std::queue<const Body*> roots;
  roots.push(_mb->GetBase());

  while(!roots.empty()) {
    // Get the next root.
    const Body* const current = roots.front();
    roots.pop();

    // Find the longest linear chain from the current root.
    chains.emplace_back(Chain::FindLongestLinearChain(_mb, current, true));

    // Get the last body in the chain.
    const Body* const last = chains.back().GetEnd();

    // If last has any forward connections, they are now roots for new
    // subchains.
    for(size_t i = 0; i < last->ForwardConnectionCount(); ++i)
      roots.push(last->GetForwardConnection(i).GetNextBody());
  }

  return chains;
}


std::pair<Chain, Chain>
Chain::
Bisect() const noexcept {
  const size_t halfSize = m_bodies.size() / 2;

  BodyList bodies1(halfSize, nullptr),
           bodies2(halfSize, nullptr);

  auto midpoint = m_bodies.begin() + halfSize;

  std::copy(m_bodies.begin(), midpoint, bodies1.begin());
  std::copy(midpoint, m_bodies.end(), bodies2.begin());

  return {Chain(m_multibody, std::move(bodies1), m_forward),
          Chain(m_multibody, std::move(bodies2), m_forward)};
}

/*-------------------------------- Modifiers ---------------------------------*/

Chain&
Chain::
Reverse() noexcept {
  std::reverse(m_bodies.begin(), m_bodies.end());
  m_forward = !m_forward;

  return *this;
}


Chain&
Chain::
Append(const Chain& _other) noexcept {
  // Assert that the two chains have the same traversal direction.
  if(IsForward() != _other.IsForward())
    throw RunTimeException(WHERE) << "Cannot append two chains with opposite "
                                  << "traversal orders."
                                  << "\n\tChain 1: " << IsForward()
                                  << "\n\tChain 2: " << _other.IsForward();

  // Assert that the end of this chain is connected to the root of _other.
  if(!IsValidEnd(_other.GetRoot()))
    throw RunTimeException(WHERE) << "Cannot append disconnected chains."
                                  << "\n\tChain 1: " << *this
                                  << "\n\tChain 2: " << _other;

  // We're still here, so the chains are compatible for joining.
  // Copy the bodies in _other into this.
  std::copy(_other.m_bodies.begin(), _other.m_bodies.end(),
      std::back_inserter(m_bodies));

  return *this;
}


Chain&
Chain::
PopRoot() noexcept {
  m_bodies.pop_front();
  return *this;
}


Chain&
Chain::
PopEnd() noexcept {
  m_bodies.pop_back();
  return *this;
}


Chain&
Chain::
PushRoot(const Body* const _newRoot) noexcept {
  if(!IsValidRoot(_newRoot))
    throw RunTimeException(WHERE) << "Tried to push an invalid root into the "
                                  << "chain."
                                  << "\n\tBody: " << _newRoot->GetIndex()
                                  << "\n\tChain: " << *this;

  m_bodies.push_front(_newRoot);
  return *this;
}


Chain&
Chain::
PushEnd(const Body* const _newEnd) noexcept {
  if(!IsValidEnd(_newEnd))
    throw RunTimeException(WHERE) << "Tried to push an invalid end into the "
                                  << "chain."
                                  << "\n\tBody: " << _newEnd->GetIndex()
                                  << "\n\tChain: " << *this;

  m_bodies.push_back(_newEnd);
  return *this;
}

/*-------------------------------- Iteration ---------------------------------*/

Chain::iterator
Chain::
begin() const noexcept {
  return m_bodies.begin();
}


Chain::iterator
Chain::end() const noexcept {
  return m_bodies.end();
}

/*--------------------------------- Queries ----------------------------------*/

const Body*
Chain::
GetRoot() const noexcept {
  return m_bodies.front();
}


const Body*
Chain::
GetEnd() const noexcept {
  return m_bodies.back();
}


bool
Chain::
IsForward() const noexcept {
  return m_forward;
}


size_t
Chain::
Size() const noexcept {
  return m_bodies.size();
}


bool
Chain::
IsValidRoot(const Body* const _newRoot) const noexcept {
  const Body* const root = GetRoot();

  if(IsForward()) {
    // Check that _newRoot is the previous body of one of root's backward
    // connections.
    for(size_t i = 0; i < root->BackwardConnectionCount(); ++i)
      if(root->GetBackwardConnection(i).GetPreviousBody() == _newRoot)
        return true;
  }
  else {
    // Check that _newRoot is the next body of one of root's forward connections.
    for(size_t i = 0; i < root->ForwardConnectionCount(); ++i)
      if(root->GetForwardConnection(i).GetNextBody() == _newRoot)
        return true;
  }

  return false;
}


bool
Chain::
IsValidEnd(const Body* const _newEnd) const noexcept {
  const Body* const end = GetEnd();

  if(IsForward()) {
    // Check that _newEnd is the next body of one of end's forward connections.
    for(size_t i = 0; i < end->ForwardConnectionCount(); ++i)
      if(end->GetForwardConnection(i).GetNextBody() == _newEnd)
        return true;
  }
  else {
    // Check that _newEnd is the previous body of one of end's backward
    // connections.
    for(size_t i = 0; i < end->BackwardConnectionCount(); ++i)
      if(end->GetBackwardConnection(i).GetPreviousBody() == _newEnd)
        return true;
  }

  return false;
}

/*---------------------------------- Helpers ---------------------------------*/

Chain
Chain::
FindLongestLinearChain(const MultiBody* const _mb, const Body* const _root,
    const bool _forward) noexcept {
  BodyList bodies;

  // Start from the root body and traverse until we find a link with
  // more than one forward connection.
  const Body* current = _root;
  while(true) {
    // Push the current link into the chain.
    bodies.push_back(current);

    // If there is not exactly one connection from current to the next link(s)
    // in the traversal direction, we are finished.
    const size_t numConnections = _forward ? current->ForwardConnectionCount()
                                           : current->BackwardConnectionCount();
    if(numConnections != 1)
      break;

    // Set current to the next body in this traversal direction.
    current = _forward ? current->GetForwardConnection(0).GetNextBody()
                       : current->GetBackwardConnection(0).GetPreviousBody();
  }

  return Chain(_mb, std::move(bodies), _forward);
}

/*------------------------------- Construction -------------------------------*/

Chain::
Chain(const MultiBody* const _mb, BodyList&& _bodies, const bool _forward)
  : m_multibody(_mb), m_bodies(std::move(_bodies)), m_forward(_forward) {
  // There is no purpose to an empty chain; please do not make one.
  if(m_bodies.empty())
    throw RunTimeException(WHERE) << "Cowardly refusing to instantiate an "
                                  << "empty chain.";
}

/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Chain& _c) {
  _os << "{";
  for(auto body : _c)
    _os << " " << body->GetIndex();
  return _os << " }";
}

/*----------------------------------------------------------------------------*/
