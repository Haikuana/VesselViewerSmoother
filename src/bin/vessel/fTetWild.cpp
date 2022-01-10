// This file is part of fTetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2019 Yixin Hu <yixin.hu@nyu.edu>
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "fTetWild.h"

#ifdef USE_FTETWILD_REMESHING

using namespace floatTetWild;
using namespace Eigen;

//extern "C" void exactinit();
int FtetWild(SimpleMesh &mesh_,double ideal_edge_rel, double grid_unit) {
#ifdef STORE_SAMPLE_POINTS
    cout<<"STORE_SAMPLE_POINTS defined"<<endl;
#endif

#ifndef WIN32
    setenv("GEO_NO_SIGNAL_HANDLER", "1", 1);
#endif

	std::vector<std::vector<double>> vertices;
	std::vector<std::vector<int>> faces;

    GEO::initialize();

    // Import standard command line arguments, and custom ones
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("pre");
    GEO::CmdLine::import_arg_group("algo");

    bool run_tet_gen = false;
    bool skip_simplify = false;

    Mesh mesh;
    Parameters &params = mesh.params;

    CLI::App command_line{"float-tetwild"};
    command_line.add_option("-i,--input", params.input_path,
                            "Input surface mesh INPUT in .off/.obj/.stl/.ply format. (string, required)")->check(
            CLI::ExistingFile);
    command_line.add_option("-o,--output", params.output_path,
                            "Output tetmesh OUTPUT in .msh format. (string, optional, default: input_file+postfix+'.msh')");

    command_line.add_option("--tag", params.tag_path, "");
//    const int UNION = 0;
//    const int INTERSECTION = 1;
//    const int DIFFERENCE = 2;
    int boolean_op = -1;
    std::string csg_file="";
    command_line.add_option("--op", boolean_op, "");
	
    command_line.add_option("-l,--lr", params.ideal_edge_length_rel,
                            "ideal_edge_length = diag_of_bbox * L. (double, optional, default: 0.05)");
    command_line.add_option("-e,--epsr", params.eps_rel,
                            "epsilon = diag_of_bbox * EPS. (double, optional, default: 1e-3)");

    command_line.add_option("--max-its", params.max_its, "");
    command_line.add_option("--stop-energy", params.stop_energy, "");
    command_line.add_option("--stage", params.stage, "");
    command_line.add_option("--stop-p", params.stop_p, "");

    command_line.add_option("--postfix", params.postfix, "");
    command_line.add_option("--log", params.log_path, "Log info to given file.");
    command_line.add_option("--level", params.log_level, "Log level (0 = most verbose, 6 = off).");

    command_line.add_flag("-q,--is-quiet", params.is_quiet, "Mute console output. (optional)");
    command_line.add_flag("--skip-simplify", skip_simplify, "");
    command_line.add_flag("--not-sort-input", params.not_sort_input, "");
    command_line.add_flag("--correct-surface-orientation", params.correct_surface_orientation, "");

    command_line.add_option("--envelope-log", params.envelope_log, "");
    command_line.add_flag("--smooth-open-boundary", params.smooth_open_boundary, "");
    command_line.add_flag("--manifold-surface", params.manifold_surface, "");
    command_line.add_option("--csg", csg_file, "json file containg a csg tree")->check(CLI::ExistingFile);

    command_line.add_flag("--use-old-energy", floatTetWild::use_old_energy, "");//tmp

    bool disable_wn = false;
    command_line.add_flag("--disable-wn", disable_wn, "Disable winding number.");
    bool use_floodfill = false;
    command_line.add_flag("--use-floodfill", use_floodfill, "Use flood-fill to extract interior volume.");
    command_line.add_flag("--use-general-wn", params.use_general_wn, "Use general winding number.");

	//params.min_edge_len_rel = min_edge_rel;
    params.eps_rel = grid_unit;
	params.ideal_edge_length_rel = ideal_edge_rel;
	//params.manifold_surface = true;
	//params.max_its = 5;
	//params.eps_rel = 5e-4;
	//skip_simplify = true;
	//params.stop_energy = 50;
	//skip_simplify = false;

#ifdef LIBIGL_WITH_TETGEN
    command_line.add_flag("--tetgen", run_tet_gen, "run tetgen too. (optional)");
#endif
    unsigned int max_threads = std::numeric_limits<unsigned int>::max();
#ifdef FLOAT_TETWILD_USE_TBB
    command_line.add_option("--max-threads", max_threads, "maximum number of threads used");
#endif

#ifdef FLOAT_TETWILD_USE_TBB
    const size_t MB = 1024 * 1024;
    const size_t stack_size = 64 * MB;
    unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
    num_threads = std::min(max_threads, num_threads);
    params.num_threads = num_threads;
#if USE_COUT
    std::cout << "TBB threads " << num_threads << std::endl;
#endif
    tbb::task_scheduler_init scheduler(num_threads, stack_size);
#endif

	params.is_quiet = false;
    Logger::init(!params.is_quiet, params.log_path);
    params.log_level = std::max(0, std::min(6, params.log_level));
    spdlog::set_level(static_cast<spdlog::level::level_enum>(params.log_level));
    spdlog::flush_every(std::chrono::seconds(3));

    GEO::Logger *geo_logger = GEO::Logger::instance();
    geo_logger->unregister_all_clients();
    geo_logger->register_client(new GeoLoggerForward(logger().clone("geogram")));
    geo_logger->set_pretty(false);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<Vector3> input_vertices;
    std::vector<FTETVector3i> input_faces;
    std::vector<int> input_tags;

    if (!params.tag_path.empty()) {
        input_tags.reserve(input_faces.size());
        std::string line;
        std::ifstream fin(params.tag_path);
        if (fin.is_open()) {
            while (getline(fin, line)) {
                input_tags.push_back(std::stoi(line));
            }
            fin.close();
        }
    }

    igl::Timer timer;
    GEO::Mesh sf_mesh;
    json tree_with_ids;
	{
		input_vertices.resize(mesh_.vertices.size());
		for (size_t i = 0; i < mesh_.vertices.size(); i++)
			input_vertices[i] << mesh_.vertices[i][0], mesh_.vertices[i][1], mesh_.vertices[i][2];

		input_faces.resize(mesh_.faces.size());
		for (size_t i = 0; i < mesh_.faces.size(); i++)
			input_faces[i] << mesh_.faces[i][0], mesh_.faces[i][2], mesh_.faces[i][1];

		MeshIO::load_mesh(input_vertices, input_faces, sf_mesh, input_tags);

        if (input_tags.size() != input_faces.size()) {
            input_tags.resize(input_faces.size());
            std::fill(input_tags.begin(), input_tags.end(), 0);
        }
    }
    AABBWrapper tree(sf_mesh);

    if (!params.init(tree.get_sf_diag())) {
        return EXIT_FAILURE;
    }

#ifdef LIBIGL_WITH_TETGEN
    if(run_tet_gen)
    {
        Eigen::MatrixXd tetgen_pts(input_vertices.size(), 3);
        Eigen::MatrixXi tetgen_faces(input_faces.size(), 3);

        for(size_t i = 0; i < input_vertices.size(); ++i)
        {
            tetgen_pts.row(i) = input_vertices[i].cast<double>();
        }

        for(size_t i = 0; i < input_faces.size(); ++i)
        {
            tetgen_faces.row(i) = input_faces[i];
        }

        std::stringstream buf;
        buf.precision(100);
        buf.setf(std::ios::fixed, std::ios::floatfield);
        buf<<"Qpq2.0a"<<params.ideal_edge_length*params.ideal_edge_length*params.ideal_edge_length*sqrt(2.)/12.;

        Eigen::MatrixXi tetgen_generated_tets;
        Eigen::MatrixXd tetgen_generated_points;
        Eigen::MatrixXi tetgen_generated_faces;

        timer.start();
        igl::copyleft::tetgen::tetrahedralize(tetgen_pts, tetgen_faces, buf.str(), tetgen_generated_points, tetgen_generated_tets, tetgen_generated_faces);
        timer.stop();
        logger().info("Tetgen time {}s", timer.getElapsedTimeInSec());
        stats().record(StateInfo::tetgen_id, timer.getElapsedTimeInSec(), tetgen_generated_points.rows(), tetgen_generated_tets.rows(), 0, 0);
    }
#endif

    stats().record(StateInfo::init_id, 0, input_vertices.size(), input_faces.size(), -1, -1);

    timer.start();
    simplify(input_vertices, input_faces, input_tags, tree, params, skip_simplify);
    tree.init_b_mesh_and_tree(input_vertices, input_faces, mesh);
    logger().info("preprocessing {}s", timer.getElapsedTimeInSec());
    logger().info("");
    stats().record(StateInfo::preprocessing_id, timer.getElapsedTimeInSec(), input_vertices.size(),
                   input_faces.size(), -1, -1);
    if (params.log_level <= 1)
        output_component(input_vertices, input_faces, input_tags);

    timer.start();
    std::vector<bool> is_face_inserted(input_faces.size(), false);
    FloatTetDelaunay::tetrahedralize(input_vertices, input_faces, tree, mesh, is_face_inserted);
    logger().info("#v = {}", mesh.get_v_num());
    logger().info("#t = {}", mesh.get_t_num());
    logger().info("tetrahedralizing {}s", timer.getElapsedTimeInSec());
    logger().info("");
    stats().record(StateInfo::tetrahedralization_id, timer.getElapsedTimeInSec(), mesh.get_v_num(), mesh.get_t_num(),
                   -1, -1);

    timer.start();
    insert_triangles(input_vertices, input_faces, input_tags, mesh, is_face_inserted, tree, false);
    logger().info("cutting {}s", timer.getElapsedTimeInSec());
    logger().info("");
    stats().record(StateInfo::cutting_id, timer.getElapsedTimeInSec(), mesh.get_v_num(), mesh.get_t_num(),
                   mesh.get_max_energy(), mesh.get_avg_energy(),
                   std::count(is_face_inserted.begin(), is_face_inserted.end(), false));

#if 1
	timer.start();
	optimization(input_vertices, input_faces, input_tags, is_face_inserted, mesh, tree, { {1, 1, 1, 1} });
	logger().info("mesh optimization {}s", timer.getElapsedTimeInSec());
	logger().info("");
	stats().record(StateInfo::optimization_id, timer.getElapsedTimeInSec(), mesh.get_v_num(), mesh.get_t_num(),
		mesh.get_max_energy(), mesh.get_avg_energy());

	timer.start();
    correct_tracked_surface_orientation(mesh, tree);
    logger().info("correct_tracked_surface_orientation done");
    if(!csg_file.empty())
        boolean_operation(mesh, tree_with_ids);
    else if(boolean_op >= 0)
        boolean_operation(mesh, boolean_op);
    else {
        if (params.smooth_open_boundary) {
            smooth_open_boundary(mesh, tree);
            for (auto &t: mesh.tets) {
                if (t.is_outside)
                    t.is_removed = true;
            }
        } else {
            if(!disable_wn) {
                if(use_floodfill) {
                    filter_outside_floodfill(mesh);
                } else
                    filter_outside(mesh);
            }
        }
    }
    if(params.manifold_surface){
        manifold_surface(mesh);
    }
    stats().record(StateInfo::wn_id, timer.getElapsedTimeInSec(), mesh.get_v_num(), mesh.get_t_num(),
                   mesh.get_max_energy(), mesh.get_avg_energy());
    logger().info("after winding number");
    logger().info("#v = {}", mesh.get_v_num());
    logger().info("#t = {}", mesh.get_t_num());
    logger().info("winding number {}s", timer.getElapsedTimeInSec());
    logger().info("");
#endif

	MeshIO::write_surface_mesh(params.output_path + "_" + params.postfix + "_sf.obj", mesh, false);
	MeshIO::write_tetmesh("volume.tet", mesh,false);
	MeshIO::savedata_surface_mesh(mesh, mesh_.vertices, mesh_.faces, false);

    return EXIT_SUCCESS;
}

void connect_2_meshes(std::string m1, std::string m2, std::string m) {
    Eigen::MatrixXd v1, v2, _;
    Eigen::MatrixXi f1, f2;

    igl::readSTL(m1, v1, f1, _);
    igl::readSTL(m2, v2, f2, _);

    MatrixXd V(v1.rows() + v2.rows(), v1.cols());
    V << v1, v2;

    int v1_rows = v1.rows();
    for (int i = 0; i < f2.rows(); i++) {
        for (int j = 0; j < 3; j++)
            f2(i, j) += v1_rows;
    }
    MatrixXi F(f1.rows() + f2.rows(), f1.cols());
    F << f1, f2;

//    igl::writeOFF(m+".off", V, F);
    igl::writeSTL(m+".stl", V, F);
    std::ofstream fout(m+"_tags.txt");
    for (int i = 0; i < f1.rows(); i++)
        fout << 1 << endl;
    for (int i = 0; i < f2.rows(); i++)
        fout << 2 << endl;
    fout.close();

    //pausee();
}

#include <igl/readMESH.h>
void test_manifold(std::string& file_name){
    Eigen::MatrixXd V;
    Eigen::MatrixXi T, F;
    igl::readMESH(file_name, V, T, F);

    Mesh mesh;

    manifold_surface(mesh);
}

#endif
