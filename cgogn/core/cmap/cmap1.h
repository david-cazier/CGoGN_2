/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef CGOGN_CORE_CMAP_CMAP1_H_
#define CGOGN_CORE_CMAP_CMAP1_H_

#include <cgogn/core/cmap/cmap0.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap1_T : public CMap0_T<MAP_TRAITS, MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 1;

	static const uint8 PRIM_SIZE = 1;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE ;
	using Inherit = CMap0_T<MAP_TRAITS, MAP_TYPE>;
	using Self = CMap1_T<MAP_TRAITS, MAP_TYPE>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using Vertex	= typename Inherit::Vertex;
	using Face		= Cell<Orbit::PHI1>;

	using Boundary = Vertex;
	using ConnectedComponent = Face;

	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using Attribute = typename Inherit::template Attribute<T, ORBIT>;
	template <typename T>
	using VertexAttribute = Attribute<T, Vertex::ORBIT>;
	template <typename T>
	using FaceAttribute = Attribute<T, Face::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;

protected:

	ChunkArray<Dart>* phi1_;
	ChunkArray<Dart>* phi_1_;

	void init()
	{
		phi1_ = this->topology_.template add_attribute<Dart>("phi1");
		phi_1_ = this->topology_.template add_attribute<Dart>("phi_1");
	}

public:

	CMap1_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap1_T);

	~CMap1_T() override
	{}

	/*!
	 * \brief Check the integrity of embedding information
	 */
	inline bool check_embedding_integrity()
	{
		bool result = true;

		if (this->template is_embedded<Vertex>())
			result = result && this->template is_well_embedded<Vertex>();

		if (this->template is_embedded<Face>())
			result = result && this->template is_well_embedded<Face>();

		return result;
	}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

protected:

	/*!
	* \brief Init an newly added dart.
	* The dart is defined as a fixed point for PHI1.
	*/
	inline void init_dart(Dart d)
	{
		Inherit::init_dart(d);
		(*phi1_)[d.index] = d;
		(*phi_1_)[d.index] = d;
	}

	/**
	 * @brief Check the integrity of a dart
	 * @param d the dart to check
	 * @return true if the integrity constraints are locally statisfied
	 * PHI1 and PHI_1 are inverse relations.
	 */
	inline bool check_integrity(Dart d) const
	{
		return (phi1(phi_1(d)) == d &&
				phi_1(phi1(d)) == d);
	}

	/**
	 * @brief Check the integrity of a boundary dart
	 * @param d the dart to check
	 * @return true if the bondary constraints are locally statisfied
	 * No boundary dart is accepted.
	 */
	inline bool check_boundary_integrity(Dart d) const
	{
		return !this->is_boundary(d);
	}

	/*!
	 * \brief Link two darts with the phi1 permutation what either merge or split their orbit(s).
	 * @param d: the first dart
	 * @param e: the second dart
	 * - Before: d->f and e->g
	 * - After:  d->g and e->f
	 * Join the orbits of dart d and e if they are distinct
	 * - Starting from two cycles : d->f->...->d and e->g->...->e
	 * - It makes one cycle d->g->...->e->f->...->d
	 * If e = g then insert e in the cycle of d : d->e->f->...->d
	 * If d and e are in the same orbit of phi1, this orbit is split in two cycles.
	 * - Starting with d->g->...e->f->...->d
	 * - It makes two cycles : d->f->...->d and e->g->...->e
	 */
	void phi1_sew(Dart d, Dart e)
	{
		Dart f = phi1(d);
		Dart g = phi1(e);
		(*phi1_)[d.index] = g;
		(*phi1_)[e.index] = f;
		(*phi_1_)[g.index] = d;
		(*phi_1_)[f.index] = e;
	}

	/*!
	 * \brief Remove the successor of a given dart from its permutation
	 * @param d a dart
	 * - Before: d->e->f
	 * - After:  d->f and e->e
	 */
	void phi1_unsew(Dart d)
	{
		Dart e = phi1(d);
		Dart f = phi1(e);
		(*phi1_)[d.index] = f;
		(*phi1_)[e.index] = e;
		(*phi_1_)[f.index] = d;
		(*phi_1_)[e.index] = e;
	}

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

public:

	/*!
	 * \brief phi1
	 * @param d
	 * @return phi1(d)
	 */
	inline Dart phi1(Dart d) const
	{
		return (*phi1_)[d.index];
	}

	/*!
	 * \brief phi_1
	 * @param d
	 * @return phi_1(d)
	 */
	Dart phi_1(Dart d) const
	{
		return (*phi_1_)[d.index];
	}

	/**
	 * \brief Composition of PHI calls
	 * @param d
	 * @return The result of successive applications of PHI1 on d.
	 * The template parameter contains a sequence (Base10 encoded) of PHI indices.
	 * If N=0 the identity is used.
	 */
	template <uint64 N>
	inline Dart phi(Dart d) const
	{
		static_assert((N%10)<=1,"Composition of PHI: invalid index");
		if (N >=10)
			return this->phi1(phi<N/10>(d));

		if (N == 1)
			return this->phi1(d);

		return d;
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

protected:

	/*!
	 * \brief Add a face in the map.
	 * \param size : the number of darts in the built face
	 * \return A dart of the built face
	 */
	inline Dart add_face_topo(uint32 size)
	{
		cgogn_message_assert(size > 0u, "Cannot create an empty face");

		if (size == 0)
			cgogn_log_warning("add_face_topo") << "Attempt to create an empty face results in a single dart.";

		Dart d = this->add_dart();
		for (uint32 i = 1u; i < size; ++i)
			split_vertex_topo(d);
		return d;
	}

public:

	/*!
	 * \brief Add a face in the map.
	 * \param size : the number of vertices in the built face
	 * \return The built face. If the map has Vertex or Face attributes,
	 * the new inserted cells are automatically embedded on new attribute elements.
	 */
	Face add_face(uint32 size)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Face f(add_face_topo(size));

		if (this->template is_embedded<Vertex>())
		{
			foreach_dart_of_orbit(f, [this] (Dart d)
			{
				this->new_orbit_embedding(Vertex(d));
			});
		}

		if (this->template is_embedded<Face>())
			this->new_orbit_embedding(f);

		return f;
	}

protected:

	inline void remove_face_topo(Dart d)
	{
		Dart it = phi1(d);
		while(it != d)
		{
			Dart next = phi1(it);
			this->remove_dart(it);
			it = next;
		}

		this->remove_dart(d);
	}

public:

	/*!
	 * \brief Remove a face from the map.
	 * \param d : a dart of the face to remove
	 */
	inline void remove_face(Face f)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		remove_face_topo(f.dart);
	}

protected:

	/**
	 * \brief Split a vertex.
	 * \param d : a dart of the vertex
	 * \return A dart of inserted vertex
	 * A new vertex is inserted after v in the PHI1 orbit.
	 */
	inline Dart split_vertex_topo(Dart d)
	{
		Dart e = this->add_dart();	// Create a new dart e
		phi1_sew(d, e);				// Insert e between d and phi1(d)
		return e;
	}

public:

	/**
	 * \brief Split a vertex.
	 * \param d : a vertex
	 * \return The inserted vertex
	 * A new vertex is inserted after v in the PHI1 orbit.
	 * If the map has Vertex or Face attributes, the inserted cells
	 * are automatically embedded on new attribute elements.
	 */
	inline Vertex split_vertex(Vertex v)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Vertex nv(split_vertex_topo(v.dart));

		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(nv);

		if (this->template is_embedded<Face>())
			this->template copy_embedding<Face>(nv.dart, v.dart);

		return nv;
	}

protected:

	/**
	 * \brief Remove a vertex from its face and delete it.
	 * @param d : a dart of the vertex
	 * The vertex that preceeds the vertex of d in the face is linked
	 * to the successor of the vertex of d.
	 */
	inline void remove_vertex_topo(Dart d)
	{
		Dart e = phi_1(d);
		if (e != d) phi1_unsew(e);
		this->remove_dart(d);
	}

public:

	/**
	 * \brief Remove a vertex from its face and delete it.
	 * @param v : a vertex
	 */
	inline void remove_vertex(Vertex v)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		remove_vertex_topo(v.dart);
	}

protected:

	inline void reverse_face_topo(Dart d)
	{
		Dart e = phi1(d);			// Dart e is the first edge of the new face

		if (e == d) return;			// Only one edge: nothing to do
		if (phi1(e) == d) return;	// Only two edges: nothing to do

		phi1_unsew(d);				// Detach e from the face of d

		Dart dNext = phi1(d);
		while (dNext != d)			// While the face of d contains more than two edges
		{
			phi1_unsew(d);			// Unsew the edge after d
			phi1_sew(e, dNext);		// Sew it after e (thus in reverse order)
			dNext = phi1(d);
		}
		phi1_sew(e, d);				// Sew the last edge
	}

	/*******************************************************************************
	 * Connectivity information
	 *******************************************************************************/

public:

	inline uint32 degree(Vertex) const
	{
		return 1;
	}

	inline uint32 codegree(Face f) const
	{
		return this->nb_darts_of_orbit(f);
	}


	inline bool has_codegree(Face f, uint32 codegree) const
	{
		if (codegree < 1u) return false;
		Dart it = f.dart ;
		for (uint32 i = 1u; i < codegree; ++i)
		{
			it = phi1(it) ;
			if (it == f.dart)
				return false;
		}
		it = phi1(it) ;
		return (it == f.dart);
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

protected:

	template <typename FUNC>
	inline void foreach_dart_of_PHI1(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi1(it);
		} while (it != d);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1, "Orbit not supported in a CMap1");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1(c.dart, f); break;
			case Orbit::PHI2:
			case Orbit::PHI1_PHI2:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI21:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap1"); break;
		}
	}

	template <typename FUNC>
	inline void foreach_dart_of_PHI1_until(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if (!f(it))
				break;
			it = phi1(it);
		} while (it != d);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
		static_assert(check_func_return_type(FUNC, bool), "Wrong function return type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1, "Orbit not supported in a CMap1");

		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_PHI1_until(c, f); break;
			case Orbit::PHI2:
			case Orbit::PHI1_PHI2:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI21:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in a CMap1"); break;
		}
	}

public:

	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit(f, [&func](Dart v) {func(Vertex(v));});
	}
};

template <typename MAP_TRAITS>
struct CMap1Type
{
	using TYPE = CMap1_T<MAP_TRAITS, CMap1Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using CMap1 = CMap1_T<MAP_TRAITS, CMap1Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP1_CPP_))
extern template class CGOGN_CORE_API CMap1_T<DefaultMapTraits, CMap1Type<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarker<CMap1<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap1<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap1<DefaultMapTraits>>;
extern template class CGOGN_CORE_API CellMarker<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Face::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap1<DefaultMapTraits>, CMap1<DefaultMapTraits>::Face::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP1_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP1_H_
