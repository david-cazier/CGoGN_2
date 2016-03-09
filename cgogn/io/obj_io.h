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

#ifndef IO_OBJ_IO_H_
#define IO_OBJ_IO_H_

#include <io/surface_import.h>

namespace cgogn
{

namespace io
{

template<typename MAP_TRAITS, typename VEC3>
class ObjSurfaceImport : public SurfaceImport<MAP_TRAITS> {
public:
	using Self = ObjSurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit = SurfaceImport<MAP_TRAITS>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	virtual ~ObjSurfaceImport() override {}
protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);
		ChunkArray<VEC3>* position =
				this->vertex_attributes_.template add_attribute<VEC3>("position");

		std::string line, tag;

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(102400);

		unsigned int i = 0;
		do
		{
			if (tag == std::string("v"))
			{
				std::stringstream oss(line);

				double x, y, z;
				oss >> x;
				oss >> y;
				oss >> z;

				VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

				unsigned int vertex_id = this->vertex_attributes_.template insert_lines<1>();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		this->nb_vertices_ = static_cast<unsigned int>(vertices_id.size());

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("f"));

		this->faces_nb_edges_.reserve(vertices_id.size() * 2);
		this->faces_vertex_indices_.reserve(vertices_id.size() * 8);

		std::vector<unsigned int> table;
		table.reserve(64);
		do
		{
			if (tag == std::string("f")) // lecture d'une face
			{
				std::stringstream oss(line);

				table.clear();
				while (!oss.eof())  // lecture de tous les indices
				{
					std::string str;
					oss >> str;

					unsigned int ind = 0;

					while ((ind < str.length()) && (str[ind] != '/'))
						ind++;

					if (ind > 0)
					{
						unsigned int index;
						std::stringstream iss(str.substr(0, ind));
						iss >> index;
						table.push_back(index);
					}
				}

				unsigned int n = static_cast<unsigned int>(table.size());
				this->faces_nb_edges_.push_back(static_cast<unsigned short>(n));
				for (unsigned int j = 0; j < n; ++j)
				{
					unsigned int index = table[j] - 1; // indices start at 1
					this->faces_vertex_indices_.push_back(vertices_id[index]);
				}
				this->nb_faces_++;
			}
			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		return true;
	}
};

} // namespace io
} // namespace cgogn

#endif // IO_OBJ_IO_H_