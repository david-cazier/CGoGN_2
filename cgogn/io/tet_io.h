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

#ifndef IO_TET_IO_H_
#define IO_TET_IO_H_

#include <map>

#include <core/utils/logger.h>
#include <io/dll.h>
#include <io/volume_import.h>

namespace cgogn
{

namespace io
{

template<typename MAP_TRAITS, typename VEC3>
class TetVolumeImport : public VolumeImport<MAP_TRAITS>
{
	using Inherit = VolumeImport<MAP_TRAITS>;
	using Self = TetVolumeImport<MAP_TRAITS,VEC3>;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");
		std::ifstream fp(filename, std::ios::in);

		std::string line;
		line.reserve(512);

		// reading number of vertices
		{
		std::getline(fp, line);
		std::istringstream iss(line);
		iss >> this->nb_vertices_;
		}

		// reading number of tetrahedra
		{
		std::getline(fp, line);
		std::istringstream iss(line);
		iss >> this->nb_volumes_;
		this->volumes_types.reserve(this->nb_volumes_);
		this->volumes_vertex_indices_.reserve(4u*this->nb_volumes_);
		}

		//reading vertices
		for(uint32 i = 0u; i < this->nb_vertices_; ++i)
		{
			do
			{
				std::getline(fp, line);
			} while (line.empty());

			const uint32 new_id = this->vertex_attributes_.template insert_lines<1>();
			auto& v = position->operator [](new_id);
			std::istringstream iss(line);
			iss >> v[0];
			iss >> v[1];
			iss >> v[2];
			// TODO : if required read other vertices attributes here
		}


		// reading volumes
		for (uint32 i = 0u; i < this->nb_volumes_ ; ++i)
		{
			do
			{
				std::getline(fp, line);
			} while (line.empty());

			std::istringstream iss(line);
			uint32 nbv;
			iss >> nbv; // type of volumes

			if (!iss.good())
			{

				iss.clear();
				char connector;
				iss >> connector >> connector;
				if (connector == 'C')
				{
					--this->nb_volumes_;
					std::array<uint32,4> ids;
					iss >> ids[0] >> ids[1] >> ids[2] >> ids[3];
					this->add_connector(ids[0], ids[1], ids[2], ids[3]);
				}
					continue;
			}

			std::vector<uint32> ids;
			ids.resize(nbv);
			for (auto& id : ids)
				iss >> id;

			switch (nbv) {
				case 4: this->add_tetra(*position, ids[1], ids[2], ids[3], ids[0], false); break;
				case 5: this->add_pyramid(*position, ids[0], ids[1], ids[2], ids[3],ids[4], true); break;
				case 6: this->add_triangular_prism(*position, ids[0], ids[1], ids[2], ids[3], ids[4], ids[5], true); break;
				case 8: this->add_hexa(*position, ids[4], ids[5], ids[7], ids[6], ids[0], ids[1], ids[3], ids[2], true); break;
				default:
					cgogn_log_warning("TetVolumeImport") << "Elements with " << nbv << " vertices are not handled. Ignoring.";
					break;
			}
		}

		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_TET_IO_CPP_))
extern template class CGOGN_IO_API TetVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API TetVolumeImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API TetVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API TetVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_TET_IO_CPP_))

} // namespace io
} // namespace cgogn

#endif // IO_TET_IO_H_