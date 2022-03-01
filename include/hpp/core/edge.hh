//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_CORE_EDGE_HH
# define HPP_CORE_EDGE_HH

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>
# include <hpp/util/serialization-fwd.hh>

namespace hpp {
  namespace core {
    /// \addtogroup roadmap
    /// \{

    /// Edge of a roadmap
    ///
    /// Links two nodes and stores a path linking the configurations stored in
    /// the nodes the edge links.
    class HPP_CORE_DLLAPI Edge
    {
    public:
      Edge (NodePtr_t n1, NodePtr_t n2, const PathPtr_t& path) :
	n1_ (n1), n2_ (n2), path_ (path)
      {
      }
      NodePtr_t from () const
      {
	return n1_;
      }
      NodePtr_t to () const
      {
	return n2_;
      }
      PathPtr_t path () const
      {
	return path_;
      }

    protected:
      Edge() {}
    private:
      NodePtr_t n1_;
      NodePtr_t n2_;
      PathPtr_t path_;

      HPP_SERIALIZABLE();
    }; // class Edge
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_EDGE_HH
