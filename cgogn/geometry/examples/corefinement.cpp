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

#include "corefinement.h"

int main(int argc, char** argv)
{
	std::string filename;
	if (argc < 2)
	{
		cgogn_log_info("corefinement") << "USAGE: " << argv[0] << " [filename]";
		filename = std::string(CGOGN_STR(CGOGN_TEST_MESHES_PATH)) + std::string("obj/hand_remeshed.obj");
		cgogn_log_info("corefinement") << "Using default mesh \"" << filename << "\".";
	}
	else
		filename = std::string(argv[1]);

	QApplication application(argc, argv);
	qoglviewer::init_ogl_context();

	// Instantiate the viewer.
	Corefinement viewer;
	viewer.setWindowTitle("Corefinement");
	viewer.import(filename);
	viewer.show();

	// Run main loop.
	return application.exec();
}
