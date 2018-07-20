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

#include <QApplication>
#include <QMatrix4x4>
#include <QKeyEvent>

#include <QOGLViewer/qoglviewer.h>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#include <cgogn/rendering/map_render.h>
#include <cgogn/rendering/drawer.h>

#include <cgogn/rendering/shaders/shader_scalar_per_vertex.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>

#include <cgogn/geometry/algos/ear_triangulation.h>
#include <cgogn/geometry/algos/bounding_box.h>
#include <cgogn/geometry/algos/normal.h>
#include <cgogn/geometry/algos/filtering.h>
#include <cgogn/geometry/types/plane_3d.h>


class Corefinement : public QOGLViewer
{
	using Vec3 = Eigen::Vector3d;
	using Scalar = Eigen::Vector3d::Scalar;

	using Map2 = cgogn::CMap2;
	using Vertex = Map2::Vertex;
	using Edge = Map2::Edge;
	using Face = Map2::Face;
	using Volume = Map2::Volume;
	template<typename T>
	using VertexAttribute = Map2::VertexAttribute<T>;
	template<typename T>
	using EdgeAttribute = Map2::EdgeAttribute<T>;

public:

	Corefinement() :
		map_(),
		vertex_position_(),
		cell_cache_(map_),
		bb_(),
		vbo_pos_(nullptr),
		vbo_scalar_(nullptr),
		map_render_(nullptr),
		features_drawer_(nullptr),
		features_renderer_(nullptr),
		features_proximity(0.3),
		nb_(0u),
		map_rendering_(true),
		vertices_rendering_(false),
		edge_rendering_(false),
		feature_points_rendering_(true)
	{}

	Corefinement(const Corefinement&) = delete;
	Corefinement& operator=(const Corefinement&) = delete;

	virtual ~Corefinement()
	{
		vbo_pos_.reset();
		vbo_scalar_.reset();
		map_render_.reset();
		features_drawer_.reset();
		features_renderer_.reset();
	}

	virtual void init()
	{
		glClearColor(0.1f,0.1f,0.3f,0.0f);

		vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
		cgogn::rendering::update_vbo(vertex_position_, vbo_pos_.get());

		float size = float(bb_.max_size()) / 500.0f;

		param_point_sprite_ = cgogn::rendering::ShaderPointSprite::generate_param();
		param_point_sprite_->color_ = QColor(180,180,180);
		param_point_sprite_->size_ = size;
		param_point_sprite_->set_position_vbo(vbo_pos_.get());

		param_edge_ = cgogn::rendering::ShaderBoldLine::generate_param();
		param_edge_->color_ = QColor(10,10,80);
		param_edge_->width_= 1.5f;
		param_edge_->set_position_vbo(vbo_pos_.get());

		vbo_scalar_ = cgogn::make_unique<cgogn::rendering::VBO>(1);
		cgogn::rendering::update_vbo(scalar_field_, vbo_scalar_.get());

		param_scalar_ = cgogn::rendering::ShaderScalarPerVertex::generate_param();
		param_scalar_->color_map_ = cgogn::rendering::ShaderScalarPerVertex::ColorMap::BGR;
		param_scalar_->show_iso_lines_ = true;
		param_scalar_->min_value_ = 0.0f;
		param_scalar_->max_value_ = 0.0f;
		param_scalar_->set_all_vbos(vbo_pos_.get(), vbo_scalar_.get());

		map_.foreach_cell([&] (Vertex v)
		{
			scalar_field_[v] = vertex_position_[v][0];
		});
		update_color();

		map_render_ = cgogn::make_unique<cgogn::rendering::MapRender>();
		update_topology();

		features_drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
		features_renderer_ = features_drawer_->generate_renderer();

		lines_drawer_ = cgogn::make_unique<cgogn::rendering::DisplayListDrawer>();
		lines_renderer_ = lines_drawer_->generate_renderer();
	}

	virtual void draw()
	{
		QMatrix4x4 proj;
		QMatrix4x4 view;
		camera()->getProjectionMatrix(proj);
		camera()->getModelViewMatrix(view);

		if(feature_points_rendering_)
		{
			features_renderer_->draw(proj, view);
			lines_renderer_->draw(proj, view);
		}

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 2.0f);
		if (map_rendering_)
		{
			param_scalar_->bind(proj,view);
			map_render_->draw(cgogn::rendering::TRIANGLES);
			param_scalar_->release();
		}
		glDisable(GL_POLYGON_OFFSET_FILL);

		if (vertices_rendering_)
		{
			param_point_sprite_->bind(proj,view);
			map_render_->draw(cgogn::rendering::POINTS);
			param_point_sprite_->release();
		}

		if (edge_rendering_)
		{
			param_edge_->bind(proj,view);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			map_render_->draw(cgogn::rendering::LINES);
			glDisable(GL_BLEND);
			param_edge_->release();
		}

	}

	void update_topology()
	{
		map_render_->init_primitives(map_, cgogn::rendering::POINTS);
		map_render_->init_primitives(map_, cgogn::rendering::LINES);
		map_render_->init_primitives(map_, cgogn::rendering::TRIANGLES);
	}

	/**
	 * @brief transform the scalar field so that its values lie between 0 and 1
	 * and inverse the min and max.
	 */
	void update_color()
	{
		// Search the maximal and minimal value of the scalar field
		Scalar min = std::numeric_limits<Scalar>::max();
		Scalar max = std::numeric_limits<Scalar>::lowest();
		for(auto& v : scalar_field_)
		{
			min = std::min(min, v);
			max = std::max(max, v);
		}
		// Update the shader parameters
		param_scalar_->min_value_ = float(min);
		param_scalar_->max_value_ = float(max);

		// Update de VBO
		cgogn::rendering::update_vbo(scalar_field_, vbo_scalar_.get());
	}

	void draw_segments(const std::vector<Edge>& edges, float r, float g, float b)
	{
		lines_drawer_->line_width(2.0f);
		lines_drawer_->begin(GL_LINES);
		lines_drawer_->color3f(r, g, b);

		for (auto& e: edges) {
			lines_drawer_->vertex3fv(vertex_position_[Vertex(e.dart)]);
			lines_drawer_->vertex3fv(vertex_position_[Vertex(map_.phi1(e.dart))]);
		}
		lines_drawer_->end();
	}

	void draw_vertices(const std::vector<Vertex>& vertices,
					   float r, float g, float b, float ratio)
	{
		float radius = ratio * float(bb_.max_size()) / 50.0f;
		features_drawer_->ball_size(radius);
		features_drawer_->begin(GL_POINTS);
		features_drawer_->color3f(r, g, b);

		for (auto& v: vertices) {
			features_drawer_->vertex3fv(vertex_position_[v]);
		}
		features_drawer_->end();
	}

	virtual void closeEvent(QCloseEvent*)
	{
		cgogn::rendering::ShaderProgram::clean_all();
	}

	void import(const std::string& filename)
	{
		cgogn::io::import_surface<Vec3>(map_, filename);
		cgogn_log_info("import") << "2D mesh imported";

		vertex_position_ = map_.template get_attribute<Vec3, Vertex>("position");
		if (!vertex_position_.is_valid())
		{
			cgogn_log_error("import") << "Missing attribute position. Aborting.";
			std::exit(EXIT_FAILURE);
		}

		scalar_field_ = map_.template add_attribute<Scalar, Vertex>("scalar_field_");

		cell_cache_.build<Vertex>();
		cell_cache_.build<Edge>();

		cgogn::geometry::compute_AABB(vertex_position_, bb_);

		setSceneRadius(bb_.diag_size()/2.0);
		Vec3 center = bb_.center();
		setSceneCenter(qoglviewer::Vec(center[0], center[1], center[2]));
		showEntireScene();
	}

	void select_edges()
	{
		std::vector<Edge> cut_edges;

		Vec3 center = bb_.center();
		cgogn::geometry::Plane3D plane(Vec3(1.0,0.0,0.0),center);

		map_.foreach_cell([&](Vertex v)
		{
			Vec3 p = vertex_position_[v];
			Scalar a = plane.distance(p);
			scalar_field_[v] = a;
		},
		cell_cache_);

		map_.foreach_cell([&](Edge e)
		{
			Scalar a = scalar_field_[Vertex(e.dart)];
			Scalar b = scalar_field_[Vertex(map_.phi1(e.dart))];

			if ( a*b < -Scalar(0)
				 || cgogn::almost_equal_relative(a, Scalar(0))
				 || cgogn::almost_equal_relative(b, Scalar(0))
				 )
				cut_edges.push_back(e);
		},
		cell_cache_);

		map_.foreach_cell([&](Vertex v)
		{
			scalar_field_[v] = std::abs(scalar_field_[v]);
		},
		cell_cache_);

		update_color();
		lines_drawer_->new_list();
		draw_segments(cut_edges,1.0,1.0,1.0);
		lines_drawer_->end_list();
	}

	virtual void keyPressEvent(QKeyEvent *e)
	{
		switch (e->key()) {
			case Qt::Key_M:
				map_rendering_ = !map_rendering_;
				break;
			case Qt::Key_V:
				vertices_rendering_ = !vertices_rendering_;
				break;
			case Qt::Key_E:
				edge_rendering_ = !edge_rendering_;
				break;
			case Qt::Key_A:
				feature_points_rendering_ = !feature_points_rendering_;
				break;
			case Qt::Key_0:
			{
				select_edges();
			}
			case Qt::Key_1:
			{
				break;
			}
			case Qt::Key_2:
			{
				break;
			}
		}
		QOGLViewer::keyPressEvent(e);
		update();
	}

private:
	Map2 map_;

	VertexAttribute<Vec3> vertex_position_;
	VertexAttribute<Scalar> scalar_field_;

	Map2::CellCache cell_cache_;

	cgogn::geometry::AABB<Vec3> bb_;

	std::unique_ptr<cgogn::rendering::VBO> vbo_pos_;
	std::unique_ptr<cgogn::rendering::VBO> vbo_scalar_;

	std::unique_ptr<cgogn::rendering::MapRender> map_render_;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> features_drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> features_renderer_;

	std::unique_ptr<cgogn::rendering::DisplayListDrawer> lines_drawer_;
	std::unique_ptr<cgogn::rendering::DisplayListDrawer::Renderer> lines_renderer_;

	std::unique_ptr<cgogn::rendering::ShaderBoldLine::Param> param_edge_;
	std::unique_ptr<cgogn::rendering::ShaderPointSprite::Param> param_point_sprite_;
	std::unique_ptr<cgogn::rendering::ShaderScalarPerVertex::Param> param_scalar_;

	Scalar features_proximity;

	unsigned int nb_;
	bool map_rendering_;
	bool vertices_rendering_;
	bool edge_rendering_;
	bool feature_points_rendering_;
};
