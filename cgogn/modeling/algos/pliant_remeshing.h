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

#ifndef MODELING_ALGOS_PLIANT_REMESHING_H_
#define MODELING_ALGOS_PLIANT_REMESHING_H_

#include <geometry/functions/basics.h>
#include <core/cmap/cmap2.h>
#include <core/utils/masks.h>

namespace cgogn
{

namespace modeling
{

template <typename VEC3, typename MAP_TRAITS>
void pliant_remeshing(
	CMap2<MAP_TRAITS>& map,
	typename CMap2<MAP_TRAITS>::template VertexAttribute<VEC3>& position
)
{
	using Scalar = typename VEC3::Scalar;
	using Map = CMap2<MAP_TRAITS>;
	using Vertex = typename Map::Vertex;
	using Edge = typename Map::Edge;

	Scalar mean_edge_length = 0;

	CellCache<Map> cache(map);
	cache.template update<Edge>();

	// compute mean edge length
	map.foreach_cell([&] (Edge e)
	{
		std::pair<Vertex,Vertex> v = map.vertices(e);
		VEC3 edge = position[v.first] - position[v.second];
		mean_edge_length += edge.norm();
	},
	cache);

	mean_edge_length /= cache.template size<Edge>();
	Scalar min_edge_length= Scalar(0.75) * mean_edge_length;
	Scalar max_edge_length = Scalar(1.25) * mean_edge_length;

	// cut long edges (and adjacent faces)
	map.foreach_cell([&] (Edge e)
	{
		std::pair<Vertex,Vertex> v = map.vertices(e);
		VEC3 edge = position[v.first] - position[v.second];
		if(edge.norm() > max_edge_length)
		{
			Dart e2 = map.phi2(e.dart);
			Vertex nv = map.cut_edge(e);
			position[nv] = Scalar(0.5) * (position[v.first] + position[v.second]);
			map.cut_face(nv, Vertex(map.phi_1(e.dart)));
			if(!map.is_boundary(e2))
				map.cut_face(Vertex(map.phi1(e2)), Vertex(map.phi_1(e2)));
		}
	},
	cache);

	// collapse short edges
	map.foreach_cell([&] (Edge e)
	{
		std::pair<Vertex,Vertex> v = map.vertices(e);
		VEC3 edge = position[v.first] - position[v.second];
		Scalar length = edge.norm();
		if(length < min_edge_length)
		{
			bool collapse = true;
			VEC3 p = position[v.second];
			map.foreach_adjacent_vertex_through_edge(v.second, [&] (Vertex vv)
			{
				VEC3 vec = p - position[vv];
				if (vec.norm() > max_edge_length)
					collapse = false;
			});
			if(collapse)
			{
//				Vertex cv = map.collapse_edge(e);
//				position[cv] = p;
			}
		}
	});

	// equalize valences with edge flips
	typename Map::DartMarker dm(map);
	map.foreach_cell(
		[&] (Edge e)
		{
			map.flip_edge(e); // flip edge
			Dart d = e.dart;
			Dart d2 = map.phi2(d);
			dm.mark_orbit(Edge(map.phi1(d)));
			dm.mark_orbit(Edge(map.phi_1(d))); // mark adjacent
			dm.mark_orbit(Edge(map.phi1(d2))); // edges
			dm.mark_orbit(Edge(map.phi_1(d2)));
		},
		// this filter only keeps edges that are not marked
		// and whose incident vertices' degree meet some requirements
		[&] (Edge e) -> bool
		{
			if (dm.is_marked(e.dart)) return false;
			std::pair<Vertex,Vertex> v = map.vertices(e);
			unsigned int w = map.degree(v.first);
			unsigned int x = map.degree(v.second);
			unsigned int y = map.degree(Vertex(map.phi1(map.phi1(v.first.dart))));
			unsigned int z = map.degree(Vertex(map.phi1(map.phi1(v.second.dart))));
			int32 flip = 0;
			flip += w > 6 ? 1 : (w < 6 ? -1 : 0);
			flip += x > 6 ? 1 : (x < 6 ? -1 : 0);
			flip += y < 6 ? 1 : (y > 6 ? -1 : 0);
			flip += z < 6 ? 1 : (z > 6 ? -1 : 0);
			return flip > 1;
		}
	);
}

} // namespace modeling

} // namespace cgogn

#endif // MODELING_ALGOS_PLIANT_REMESHING_H_