#pragma once
#ifndef _LOAD_VESSEL_
#define _LOAD_VESSEL_

#include "datatype.h"
#include "MC33.h"

class Vessel
{
public:
	Vessel(int Nslice_, GPara* paras_in, std::vector<PixelVessel> &voxels_, std::vector<float> &smooth_faces) {
		Nslice = Nslice_;
		voxels = voxels_;
		paras = paras_in;
#if 1
		compute_pos_corners();
		//std::cout << "load done" << std::endl;
		//remove_lonely_pixels(mapvein_voxels);
		cluster_pixels(map_vessel_voxels, 5);
		renew_pixels(map_vessel_voxels, voxels);
		//std::cout << "remove done" << std::endl;
		build_grids(map_vessel_voxels);
		generate_mesh();
		//std::cout << "mesh done" << std::endl;
		smooth_mesh(surface_);
		//std::cout << "smooth done" << std::endl;
		compute_normal(surface_, normals, smooth_faces);
		//std::cout << "normal done" << std::endl;  
#endif // 0

#if 0
		compute_pos_corners();
		compute_surface(map_vessel_voxels, faces_);
		convert_save(corners_pts, faces_, "vessel/vein.obj");
		compute_normal(corners_pts, faces_, normals, smooth_faces);
#endif // 0
		voxels_ = voxels;
	}

	~Vessel() {
		if (kdTree_localspace != NULL)
		{
			delete kdTree_localspace;
			kdTree_localspace = NULL;
		}
	}

private:

	void read_Meshu(std::map<int, Point_3> vertices, std::vector<std::vector<int>> faces,
		Meshu& mesh_)
	{
		std::vector<Meshu::Vertex_index> vis1(vertices.size());
		int i = 0;
		for (auto it = vertices.begin(); it != vertices.end(); it++, i++)
		{
			vis1[i] = mesh_.add_vertex(it->second);
		}
		for (int i = 0; i < faces.size(); i++)
		{
			face_descriptor f = mesh_.add_face(vis1[faces[i][0]],
				vis1[faces[i][1]],
				vis1[faces[i][2]]);
			if (f == Meshu::null_face())
			{
				//std::cerr << "The face could not be added because of an orientation error." << std::endl;
				f = mesh_.add_face(vis1[faces[i][0]],
					vis1[faces[i][2]],
					vis1[faces[i][1]]);
				assert(f != Meshu::null_face());
			}
		}
		//std::cout << "load mesh done!!!" << std::endl;

	}

	void compute_normal(std::map<int, Point_3> corners_pts,
		std::vector<std::vector<int>> faces_, std::vector<Vector_3>& vnors,
		std::vector<float>& smooth_faces) {

		Meshu mesh;
		read_Meshu(corners_pts, faces_, mesh);

		auto fnormals = mesh.add_property_map<face_descriptor, Vector_3>
			("f:normals", CGAL::NULL_VECTOR).first;
		auto vnormals = mesh.add_property_map<vertex_descriptor, Vector_3>
			("v:normals", CGAL::NULL_VECTOR).first;

		CGAL::Polygon_mesh_processing::compute_normals(mesh,
			vnormals,
			fnormals,
			CGAL::Polygon_mesh_processing::parameters::vertex_point_map(mesh.points()).
			geom_traits(K()));

		for (vertex_descriptor vd : vertices(mesh)) {
			vnors.push_back(vnormals[vd]);
		}
		//std::cout << "Vertex normals done:" << std::endl;

		if (corners_pts.size() != vnors.size())
		{
			std::cout << "wrong: unequal size of corner and normals " << std::endl;
		}

		std::vector<Point_3> vs;
		for (auto it = corners_pts.begin(); it != corners_pts.end(); it++)
		{
			vs.push_back(it->second);
		}

		for (int i = 0; i < faces_.size(); i++)
		{
			for (int j = 0; j < faces_[i].size(); j++)
			{
				smooth_faces.push_back(float(vnors[faces_[i][j]].x()));
				smooth_faces.push_back(float(vnors[faces_[i][j]].y()));
				smooth_faces.push_back(float(vnors[faces_[i][j]].z()));
				smooth_faces.push_back(float(vs[faces_[i][j]].x()));
				smooth_faces.push_back(float(vs[faces_[i][j]].y()));
				smooth_faces.push_back(float(vs[faces_[i][j]].z()));
			}
		}
}

	void convert_save(std::map<int, Point_3> corners_pts,
		std::vector<std::vector<int>>& faces, std::string file) {
#if SAVE_FILES
		std::ofstream fout_obj;
		fout_obj.open(file.c_str());
		if (!fout_obj.is_open())
			return;
#endif	
		int i = 0;
		std::map<int, int> inde_t;
		for (auto it = corners_pts.begin(); it != corners_pts.end(); it++, i++)
		{
#if SAVE_FILES
			fout_obj << "v" << " "
				<< it->second[0] << " "
				<< it->second[1] << " "
				<< it->second[2] << "\n";
#endif
			inde_t[it->first] = i;
		}
		for (int fit = 0; fit < faces.size(); fit++)
		{
			for (int k = 0; k < faces[fit].size(); k++)
			{
				if (inde_t.find(faces[fit][k]) == inde_t.end())
				{
					std::cout << "wrong index" << std::endl;
				}
				faces[fit][k] = inde_t.at(faces[fit][k]);
			}
#if SAVE_FILES
			fout_obj << "f " << faces[fit][0] + 1 << " " << faces[fit][1] + 1 << " " << faces[fit][2] + 1 << "\n";
#endif
		}
#if SAVE_FILES
		fout_obj.close();
#endif

		//std::cout << "convert and save done!!!" << std::endl;
	}

	void visit_neighbor(int vox, std::map<int, PixelVessel>& voxels)
	{
		if (voxels.at(vox).bvisited)
		{
			return;
		}
		voxels.at(vox).bvisited = true;

		PixelVessel v = voxels.at(vox);
		//x
		//if (v.x < width-1)
		{
			int xplus = vox + paras->height;
			if (voxels.find(xplus) != voxels.end())
			{
				voxels.at(vox).xfacet = 0;
				//visit_neighbor(xplus, voxels);
			}
		}
		//-x
		//if (v.x > 0)
		{
			int xminus = vox - paras->height;
			if (voxels.find(xminus) != voxels.end())
			{
				voxels.at(vox).x_facet = 0;
				//visit_neighbor(xminus, voxels);
			}
		}
		//y
		//if (v.y < height-1)
		{
			int yplus = vox + 1;
			if (voxels.find(yplus) != voxels.end())
			{
				voxels.at(vox).yfacet = 0;
				//visit_neighbor(yplus, voxels);
			}
		}
		//-y
		//if (v.y >0)
		{
			int yminus = vox - 1;
			if (voxels.find(yminus) != voxels.end())
			{
				voxels.at(vox).y_facet = 0;
				//visit_neighbor(yminus, voxels);
			}
		}
		//z
		//if (v.z <Nslice -1)
		{
			int zplus = vox + paras->height * paras->width;
			if (voxels.find(zplus) != voxels.end())
			{
				voxels.at(vox).zfacet = 0;
				//visit_neighbor(zplus,voxels);
			}
		}
		//-z
		//if (v.z > 0)
		{
			int zminus = vox - paras->height * paras->width;
			if (voxels.find(zminus) != voxels.end())
			{
				voxels.at(vox).z_facet = 0;
				//visit_neighbor(zminus, voxels);
			}
		}
	}

	void compute_surface(std::map<int, PixelVessel> map_voxels, std::vector<std::vector<int>>& faces) {
		for (auto it = map_voxels.begin(); it != map_voxels.end(); it++)
		{
			if (it->second.bvisited)
			{
				continue;
			}
			visit_neighbor(it->first, map_voxels);
		}
		for (auto it = map_voxels.begin(); it != map_voxels.end(); it++)
		{
			if (it->second.xfacet)
			{
				std::vector<int> facet1 = { it->second.corners[1],it->second.corners[2],
					it->second.corners[6] };
				std::vector<int> facet2 = { it->second.corners[1],
					it->second.corners[6], it->second.corners[5] };
				faces.push_back(facet1);
				faces.push_back(facet2);
			}
			if (it->second.x_facet)
			{
				std::vector<int> facet1 = { it->second.corners[0],it->second.corners[4],
					it->second.corners[7] };
				std::vector<int> facet2 = { it->second.corners[0],
					it->second.corners[7], it->second.corners[3] };
				faces.push_back(facet1);
				faces.push_back(facet2);
			}
			if (it->second.yfacet)
			{
				std::vector<int> facet1 = { it->second.corners[2],it->second.corners[3],
					it->second.corners[7] };
				std::vector<int> facet2 = { it->second.corners[2],
					it->second.corners[7], it->second.corners[6] };
				faces.push_back(facet1);
				faces.push_back(facet2);
			}
			if (it->second.y_facet)
			{
				std::vector<int> facet1 = { it->second.corners[0],it->second.corners[1],
					it->second.corners[5] };
				std::vector<int> facet2 = { it->second.corners[0],
					it->second.corners[5], it->second.corners[4] };
				faces.push_back(facet1);
				faces.push_back(facet2);
			}
			if (it->second.zfacet)
			{
				std::vector<int> facet1 = { it->second.corners[4],it->second.corners[5],
					it->second.corners[6] };
				std::vector<int> facet2 = { it->second.corners[4],
					it->second.corners[6], it->second.corners[7] };
				faces.push_back(facet1);
				faces.push_back(facet2);
			}
			if (it->second.z_facet)
			{
				std::vector<int> facet1 = { it->second.corners[0],it->second.corners[3],
					it->second.corners[2] };
				std::vector<int> facet2 = { it->second.corners[0],
					it->second.corners[2], it->second.corners[1] };
				faces.push_back(facet1);
				faces.push_back(facet2);
			}
		}
	}

	void smooth_mesh(SimpleMesh* meshin)
	{
		std::vector<std::set<int>> vvneigh(meshin->vertices.size());
		for (int i = 0; i < meshin->faces.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				int v1 = meshin->faces[i][j];
				int v2 = meshin->faces[i][(j + 1) % 3];
				vvneigh[v1].insert(v2);
				vvneigh[v2].insert(v1);
			}
		}
		double alpha = 0.85;
		for (int k = 0; k < 5; k++)
		{
			std::vector<std::vector<double>> new_vs(meshin->vertices.size());
#pragma omp parallel for
			for (int i = 0; i < meshin->vertices.size(); i++)
			{
				std::set<int> cur_neighbors;
				collect_v_neighbors(i, cur_neighbors, 2, vvneigh);
				Vector_3 vb(0, 0, 0);
				for (auto it = cur_neighbors.begin(); it != cur_neighbors.end(); it++)
				{
					vb += Vector_3(meshin->vertices[*it][0], meshin->vertices[*it][1], meshin->vertices[*it][2]);
				}
				if (cur_neighbors.size() >= 1)
				{
					vb *= 1.0 / cur_neighbors.size();
					std::vector<double> v(3);
					v[0] = alpha * meshin->vertices[i][0] + (1.0 - alpha) * vb.x();
					v[1] = alpha * meshin->vertices[i][1] + (1.0 - alpha) * vb.y();
					v[2] = alpha * meshin->vertices[i][2] + (1.0 - alpha) * vb.z();
					new_vs[i] = v;
				}
			}
			meshin->vertices = new_vs;
		}
	}


	void collect_v_neighbors(int v, std::set<int>& neighbors, int n_ring_, const std::vector<std::set<int>>& vvneigh) {
		if (n_ring_ < 1)
		{
			return;
		}
		if (n_ring_ >= 1)
		{
			std::set<int> vvadj = vvneigh[v];
			for (auto it = vvadj.begin(); it != vvadj.end(); it++)
			{
				neighbors.insert(*it);
			}
			n_ring_--;
		}
		if (n_ring_ >= 1)
		{
			std::set<int> vvadj = vvneigh[v];
			for (auto it = vvadj.begin(); it != vvadj.end(); it++)
			{
				collect_v_neighbors(*it, neighbors, n_ring_, vvneigh);
			}
		}
	}

	void renew_pixels(std::map<int, PixelVessel> voxels, std::vector<PixelVessel>& voxel_data)
	{
		voxel_data.clear();
		for (auto it = voxels.begin(); it != voxels.end(); it++)
		{
			voxel_data.push_back(it->second);
		}
	}

	void build_grids(std::map<int, PixelVessel>& voxels)
	{
		dataPts.pts.resize(voxels.size());
		int i = 0;
		for (auto it = voxels.begin(); it != voxels.end(); it++, i++) {
			dataPts.pts[i].x = it->second.center[0];
			dataPts.pts[i].y = it->second.center[1];
			dataPts.pts[i].z = it->second.center[2];
		}
		kdTree_localspace = new NanoKdtree(3, //dim
			dataPts,
			nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
		kdTree_localspace->buildIndex();

	}

	double cal_radius_for_v(Point_3 pt)
	{
		int					nb_neigh = 1;
		const double query_pt[3] = { pt.x(),pt.y(),pt.z() };
		std::vector<size_t>		ret_index(nb_neigh);
		std::vector<double>		out_dist_sqr(nb_neigh);
		nb_neigh = kdTree_localspace->knnSearch(&query_pt[0], nb_neigh, &ret_index[0], &out_dist_sqr[0]);

		double dis_min = sqrt(out_dist_sqr[0]);
		return dis_min;
	}

	void generate_mesh()
	{
		double image_wid = IMAGEWIDTHSIZE;
		double image_hei = IMAGEWIDTHSIZE / paras->width * paras->height;
		double image_sli = IMAGEWIDTHSIZE / paras->width * paras->slice * SCALEVOXEL;

		grid_dimension[0] = 1.5 * paras->width;
		grid_dimension[1] = 1.5 * paras->height;
		grid_dimension[2] = 1.5 * paras->slice;

		double pixel_width = 2.0 * image_wid / paras->width;
		double voxel_size_x = (2.0 * image_wid + 40 * pixel_width) / grid_dimension[0];
		double voxel_size_y = voxel_size_x;
		double voxel_size_z = voxel_size_x * SCALEVOXEL;

		//fill in grids' values
		GRD_data_type* grid_data_ = new GRD_data_type[grid_dimension[0] * grid_dimension[1] * grid_dimension[2]];
		std::vector<Point_3> pdata_(grid_dimension[0] * grid_dimension[1] * grid_dimension[2]);
#pragma omp parallel for
		for (int iz = 0; iz < grid_dimension[2]; iz++)
		{
			for (int iy = 0; iy < grid_dimension[1]; iy++)
			{
				for (int ix = 0; ix < grid_dimension[0]; ix++)
				{
					double cen_x = float(ix) / float(grid_dimension[0]) * (2.0 * image_wid + 40 * pixel_width)
						- image_wid - 20 * pixel_width;
					double cen_y = float(iy) / float(grid_dimension[1]) * (2.0 * image_hei + 40 * pixel_width)
						- image_hei - 20 * pixel_width;
					double cen_z = float(iz) / float(grid_dimension[2]) * (2.0 * image_sli + 40 * pixel_width)
						- image_sli - 20 * pixel_width;
					Point_3 p(cen_x, cen_y, cen_z);
					int id_ = iz * grid_dimension[0] * grid_dimension[1] + iy * grid_dimension[0] + ix;
					pdata_[id_] = p;
				}
			}
		}
#pragma omp parallel for
		for (int i = 0; i < pdata_.size(); i++)
		{
			grid_data_[i] = cal_radius_for_v(pdata_[i]);
		}

		//create grids	
		grids.set_r0(-image_wid, -image_hei, -image_sli);
		grids.set_data_pointer(grid_dimension[0], grid_dimension[1], grid_dimension[2], grid_data_);
		grids.set_ratio_aspect(voxel_size_x, voxel_size_y, voxel_size_z);

		CMarchingCube.set_grid3d(&grids);

		surface* S = CMarchingCube.calculate_isosurface(pixel_width * 1.8);//1.8
#if 0
		std::string filen_ = "marching_cube.obj";
		S->save_obj(filen_.data());
#endif

		int Nv = S->get_num_vertices();
		int Nt = S->get_num_triangles();
		surface_ = new SimpleMesh;
		surface_->vertices.resize(Nv);
		surface_->faces.resize(Nt);
#pragma omp parallel for
		for (int i = 0; i < Nv; i++)
		{
			std::vector<double> vt = { S->getVertex(i)[0]- image_wid,
				S->getVertex(i)[1]- image_hei,
				S->getVertex(i)[2]- image_sli };
			surface_->vertices[i] = vt;
		}
#pragma omp parallel for
		for (int i = 0; i < Nt; i++)
		{
			std::vector<int> f;
			f.push_back(S->getTriangle(i)[0]);
			f.push_back(S->getTriangle(i)[1]);
			f.push_back(S->getTriangle(i)[2]);
			surface_->faces[i] = f;
		}

	}

	void compute_pos_corners() {
		double image_wid = IMAGEWIDTHSIZE;
		double image_hei = IMAGEWIDTHSIZE / paras->width * paras->height;
		double image_sli = IMAGEWIDTHSIZE / paras->width * paras->slice * SCALEVOXEL;

		double voxel_size_x = 2.0 * IMAGEWIDTHSIZE / paras->width;
		double voxel_size_y = voxel_size_x;
		double voxel_size_z = voxel_size_x * SCALEVOXEL;

		for (int i = 0; i < voxels.size(); i++)
		{
			PixelVessel p = voxels[i];
			double cen_x = float(p.x) / float(paras->width - 1) * (2.0 * image_wid - voxel_size_x)
				- image_wid + voxel_size_x / 2.0;
			double cen_y = float(p.y) / float(paras->height - 1) * (2.0 * image_hei - voxel_size_y)
				- image_hei + voxel_size_y / 2.0;
			double cen_z = float(p.z) / float(paras->slice - 1) * (2.0 * image_sli - voxel_size_z)
				- image_sli + voxel_size_z / 2.0;
			voxels[i].center[0] = cen_x;
			voxels[i].center[1] = cen_y;
			voxels[i].center[2] = cen_z;
			map_vessel_voxels[voxels[i].index_] = voxels[i];
		}
	}


	void cluster_pixels(std::map<int, PixelVessel>& voxels, int thres_number)
	{
		int count = 0;
		int label_id = 0;
		voxels.begin()->second.label = 0;
		std::set<int> pre_visited, cur_visited;
		std::set<int> candidates;
		for (auto it = voxels.begin(); it != voxels.end(); it++)
		{
			candidates.insert(it->first);
		}
		pre_visited.insert(voxels.begin()->first);
		do {
			cur_visited.clear();
			for (auto it = pre_visited.begin(); it != pre_visited.end(); it++)
			{
				candidates.erase(*it);
				visit_neighbor(*it, voxels, cur_visited);
			}
			if (cur_visited.empty())
			{
				if (candidates.empty())
					break;
				else
				{
					label_id++;
					int cur_c = *candidates.begin();
					voxels.at(cur_c).label = label_id;
					cur_visited.insert(cur_c);
				}
			}
			pre_visited = cur_visited;
			count++;
		} while (count < voxels.size());

		//check
		bool all_visited = true;
		for (auto it = voxels.begin(); it != voxels.end(); it++)
		{
			if (!it->second.bvisited) {
				all_visited = false;
				break;
			}
		}

		std::vector<std::set<int>> clusters(label_id + 1);
		for (auto it = voxels.begin(); it != voxels.end(); it++)
		{
			clusters[it->second.label].insert(it->first);
		}

		for (int t = 0; t < clusters.size(); t++)
		{
			if (clusters[t].size() < thres_number)
			{
				for (auto it = clusters[t].begin(); it != clusters[t].end(); it++)
				{
					voxels.erase(*it);
				}
			}
		}
	}

	void visit_neighbor(int vox, std::map<int, PixelVessel>& voxels, std::set<int>& visited)
	{
		if (voxels.at(vox).bvisited)
		{
			return;
		}
		voxels.at(vox).bvisited = true;

		PixelVessel v = voxels.at(vox);
		//x
		if (v.x < paras->width - 1)
		{
			int xplus = vox + paras->height;
			if (voxels.find(xplus) != voxels.end() && !voxels.at(xplus).bvisited)
			{
				voxels.at(xplus).label = voxels.at(vox).label;
				visited.insert(xplus);
			}
		}
		//-x
		if (v.x > 0)
		{
			int xminus = vox - paras->height;
			if (voxels.find(xminus) != voxels.end() && !voxels.at(xminus).bvisited)
			{
				voxels.at(xminus).label = voxels.at(vox).label;
				visited.insert(xminus);
			}
		}
		//y
		if (v.y < paras->height - 1)
		{
			int yplus = vox + 1;
			if (voxels.find(yplus) != voxels.end() && !voxels.at(yplus).bvisited)
			{
				voxels.at(yplus).label = voxels.at(vox).label;
				visited.insert(yplus);
			}
		}
		//-y
		if (v.y > 0)
		{
			int yminus = vox - 1;
			if (voxels.find(yminus) != voxels.end() && !voxels.at(yminus).bvisited)
			{
				voxels.at(yminus).label = voxels.at(vox).label;
				visited.insert(yminus);
			}
		}
		//z
		if (v.z < paras->slice - 1)
		{
			int zplus = vox + paras->height * paras->width;
			if (voxels.find(zplus) != voxels.end() && !voxels.at(zplus).bvisited)
			{
				voxels.at(zplus).label = voxels.at(vox).label;
				visited.insert(zplus);
			}
		}
		//-z
		if (v.z > 0)
		{
			int zminus = vox - paras->height * paras->width;
			if (voxels.find(zminus) != voxels.end() && !voxels.at(zminus).bvisited)
			{
				voxels.at(zminus).label = voxels.at(vox).label;
				visited.insert(zminus);
			}
		}
	}

	void read_Meshu(SimpleMesh* meshin, Meshu& mesh_)
	{
		std::vector<Meshu::Vertex_index> vis1(meshin->vertices.size());
		for (int i = 0; i < meshin->vertices.size(); i++)
		{
			vis1[i] = mesh_.add_vertex(Point_3(meshin->vertices[i][0], meshin->vertices[i][1], meshin->vertices[i][2]));
		}
		for (int i = 0; i < meshin->faces.size(); i++)
		{
			face_descriptor f = mesh_.add_face(vis1[meshin->faces[i][0]],
				vis1[meshin->faces[i][1]],
				vis1[meshin->faces[i][2]]);
			if (f == Meshu::null_face())
			{
				//std::cerr << "The face could not be added because of an orientation error." << std::endl;
				f = mesh_.add_face(vis1[meshin->faces[i][0]],
					vis1[meshin->faces[i][2]],
					vis1[meshin->faces[i][1]]);
				assert(f != Meshu::null_face());
			}
		}
		//std::cout << "load mesh done!!!" << std::endl;

	}

	void compute_normal(SimpleMesh* meshin, std::vector<Vector_3>& vnors, std::vector<float>& smooth_faces) {

		Meshu mesh;
		read_Meshu(meshin, mesh);

		auto fnormals = mesh.add_property_map<face_descriptor, Vector_3>
			("f:normals", CGAL::NULL_VECTOR).first;
		auto vnormals = mesh.add_property_map<vertex_descriptor, Vector_3>
			("v:normals", CGAL::NULL_VECTOR).first;

		CGAL::Polygon_mesh_processing::compute_normals(mesh,
			vnormals,
			fnormals,
			CGAL::Polygon_mesh_processing::parameters::vertex_point_map(mesh.points()).
			geom_traits(K()));

		for (vertex_descriptor vd : vertices(mesh)) {
			vnors.push_back(vnormals[vd]);
		}
		//std::cout << "Vertex normals done:" << std::endl;

		if (meshin->vertices.size() != vnors.size())
		{
			std::cout << "wrong: unequal size of corner and normals " << std::endl;
		}

		std::vector<Point_3> vs;
		for (int i = 0; i < meshin->vertices.size(); i++)
		{
			vs.push_back(Point_3(meshin->vertices[i][0], meshin->vertices[i][1], meshin->vertices[i][2]));
		}

		for (int i = 0; i < meshin->faces.size(); i++)
		{
			for (int j = 0; j < meshin->faces[i].size(); j++)
			{
				smooth_faces.push_back(float(vnors[meshin->faces[i][j]].x()));
				smooth_faces.push_back(float(vnors[meshin->faces[i][j]].y()));
				smooth_faces.push_back(float(vnors[meshin->faces[i][j]].z()));
				smooth_faces.push_back(float(vs[meshin->faces[i][j]].x()));
				smooth_faces.push_back(float(vs[meshin->faces[i][j]].y()));
				smooth_faces.push_back(float(vs[meshin->faces[i][j]].z()));
			}
		}
	}


private:
	GPara							*paras;
	int								Nslice;
	std::vector<PixelVessel>		voxels;
	std::map<int, PixelVessel>		map_vessel_voxels;

	//surface
	std::map<int, Point_3>			corners_pts;
	std::vector<std::vector<int>>	faces_;

	SimpleMesh						mesh_;

	//normals
	std::vector<Vector_3>			normals;

	PointCloud						dataPts;
	NanoKdtree*						kdTree_localspace;

	int								grid_dimension[3];
	grid3d							grids;
	MC33							CMarchingCube;
	SimpleMesh* surface_;

};






#endif



