#ifndef PMPL_CHAIN_H_
#define PMPL_CHAIN_H_

#include <deque>
#include <iostream>
#include <utility>
#include <vector>

class Connection;
class Body;
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// A chain represents some subset of a linked multibody. It defines a root,
/// end-effector, and ordering (relative to the multibody base).
///
/// Each chain represents a single linear subset of the represented multibody.
/// For multibodies with branchings and/or closures, Decompose will return a
/// Chain for each linear segment. No body will be repeated in the set (the
/// chains are disjoint).
///
/// This object assumes that the represented multibody will not change
/// structure. If it does, any chains built from the previous version will
/// likely be invalid.
////////////////////////////////////////////////////////////////////////////////
class Chain final {

  public:

    ///@name Local Types
    ///@{

    typedef std::deque<Connection*>  JointList;
    typedef JointList::const_iterator iterator;

    ///@}
    ///@name Generation
    ///@{
    /// Methods for generating chains of a multibody.

    /// Decompose a multibody into a set of linear chains.
    /// @param _mb The multibody to decompose.
    /// @return A set of Chains representing linear subsets of _mb.
    static std::vector<Chain> Decompose(MultiBody* _mb);

    /// Bisect this to produce two subchains. Their order
    ///         will be the same as this.
    std::pair<Chain, Chain> Bisect() const noexcept;

    ///@}
    ///@name Modifiers
    ///@{
    /// All modifiers return a self-reference for easy function chaining (bad
    /// pun intended).

    /// Reverse the traversal order of this chain.
    Chain& Reverse() noexcept;

    /// Merge another chain into this one. The two chains must have the same
    /// traversal order, and the other chain's root must be connected to this
    /// chain's end.
    /// @param _other The chain to append to this one.
    Chain& Append(const Chain& _other) noexcept;

    /// Pop the root from the chain.
    Chain& PopRoot() noexcept;

    /// Pop the end from the chain.
    Chain& PopEnd() noexcept;

    /// Push a new root into the chain. It must be a valid root for this chain.
    /// @param _newRoot The new root.
    Chain& PushRoot(const Body* const _newRoot) noexcept;

    /// Push a new end into the chain. It must be a valid end for this chain.
    /// @param _newEnd The new end.
    Chain& PushEnd(const Body* const _newEnd) noexcept;

    /// Gets the last body of the chain to use for Reachable Volume calculation when an end joint doesn't exist
    const Body* GetLastBody() const noexcept;
    //Body* GetLastBody() noexcept;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the bodies in the chain in traversal order.

    iterator begin() const noexcept;
    iterator end() const noexcept;

    ///@}
    ///@name Queries
    ///@{

    /// Get the first body
    const Body* GetBase() const noexcept;
    //Body* GetBase() noexcept;

    /// Get the root joint.
    const Connection* GetRoot() const noexcept;

    /// Get the end joint.
    Connection* GetEnd() const noexcept;

    /// Is this chain oriented in the same way as the multibody?
    bool IsForward() const noexcept;

    /// Get the number of links in the chain.
    size_t Size() const noexcept;

    /// Check if another body is a valid root for this chain (i.e., if it is
    /// a parent of the current root in the current traversal order).
    /// @param _newRoot The body to check.
    /// @return True if _newRoot is a valid root for this.
    bool IsValidRoot(const Body* const _newRoot) const noexcept;

    /// Check if another body is a valid end for this chain (i.e., if it is
    /// a child of the current end in the current traversal order).
    /// @param _newEnd The body to check.
    /// @return True if _newEnd is a valid end for this.
    bool IsValidEnd(const Body* const _newEnd) const noexcept;

    /// Check if the current chain has only one link
    bool IsSingleLink() const noexcept;

    ///@}

    ///@}
    ///@name Construction
    ///@{
    /// Construct a chain.
    /// @param _mb The multibody.
    /// @param _bodies The bodies in this chain, from root to end.
    /// @param _forward True if _end is further away from the base than _root.
    Chain(MultiBody* _mb, JointList&& _joints,
	    const Body* _lastBody, const bool _forward);

    Chain(MultiBody* _mb, JointList&& _joints,
          const bool _forward);

    Chain(MultiBody* _mb, JointList&& _joints, const Body* _base, 
	    const Body* _lastBody, const bool _forward);

  private:

    ///@name Helpers
    ///@{

    /// Find the longest linear chain in a multibody starting from a designated
    /// root link.
    /// @param _mb The multibody.
    /// @param _root The root link to search from (will be included in the chain).
    /// @param _forward Search forward (out from base)?
    /// @return The longest linear segment in _mb starting from _root and
    ///         traversing outward if _forward.
    static Chain FindLongestLinearChain(MultiBody* _mb,
        Body* _root, const bool _forward) noexcept;



    ///@}
    ///@name Internal State
    ///@{

      MultiBody* m_multibody{nullptr};  ///< The multibody.
      JointList m_joints;                   ///< The joints in this chain.
      const Body* m_base{nullptr};                        ///< the first body of the chain
      const Body* m_lastBody{nullptr};                   ///< the last body of a chain, null if a last joint exists (e.g, when bisecting a chain)
      bool m_forward{true};                ///< Is the chain in forward order?

    ///@}

};

// Debug.
std::ostream& operator<<(std::ostream&, const Chain&);

#endif
