#pragma once
#ifndef _LOAD_VESSEL_ORI
#define _LOAD_VESSEL_ORI

#include "datatype.h"
//#include "MC33.h"

class Vessel_ori
{
public:
	Vessel_ori(int Nslice_, std::vector<PixelVessel> &voxels_, std::vector<float> &smooth_faces) {
		Nslice = Nslice_;
		voxels = voxels_;
		compute_pos_corners();
		//cluster_pixels(map_vessel_voxels, voxels,20);

		compute_surface(map_vessel_voxels, faces_);
		//std::cout << "surface done" << std::endl;
		convert_save(corners_pts, faces_,mesh_, "vein.obj");
		//std::cout << "save done" << std::endl;
		smooth_mesh(mesh_);
		//std::cout << "smooth done" << std::endl;
		compute_normal(mesh_, normals, smooth_faces);
		//std::cout << "normal done" << std::endl;
		voxels_ = voxels;
	}

	~Vessel_ori() {
	}

private:

	void smooth_mesh(SimpleMesh &mesh)
	{
		std::vector<std::set<int>> vvneigh(mesh.vertices.size());
		for (int i = 0; i < mesh.faces.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				int v1 = mesh.faces[i][j];
				int v2 = mesh.faces[i][(j + 1) % 3];
				vvneigh[v1].insert(v2);
				vvneigh[v2].insert(v1);
			}
		}
		double alpha = 0.8;
		for (int k = 0; k < 3; k++)
		{
			std::vector<Point_3> new_vs(mesh.vertices.size());
//#pragma omp parallel for
			for (int i = 0; i < mesh.vertices.size(); i++)
			{
				std::set<int> cur_neighbors;
				collect_v_neighbors(i, cur_neighbors, 2, vvneigh);
				Vector_3 vb(0, 0, 0);
				for (auto it = cur_neighbors.begin(); it != cur_neighbors.end(); it++)
				{
					vb += mesh.vertices[*it]-Point_3(0,0,0);
				}
				if (cur_neighbors.size() >= 1)
				{
					vb *= 1.0 / cur_neighbors.size();
					Vector_3 v = alpha * (mesh.vertices[i] - Point_3(0, 0, 0)) + (1.0 - alpha) * vb;
					new_vs[i] = Point_3(v.x(),v.y(),v.z());
				}
			}
			mesh.vertices = new_vs;
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


	void compute_pos_corners() {
		double image_wid = IMAGEWIDTHSIZE;
		double image_hei = IMAGEWIDTHSIZE / width*height;
		double image_sli = IMAGEWIDTHSIZE / width*Nslice*SCALEVOXEL;

		double voxel_size_x = 2.0*IMAGEWIDTHSIZE / width;
		double voxel_size_y = voxel_size_x;
		double voxel_size_z = voxel_size_x*SCALEVOXEL;

		for (int i = 0; i < voxels.size(); i++)
		{
			PixelVessel p = voxels[i];
			double cen_x = float(p.x) / float(width - 1)*(2.0*image_wid - voxel_size_x)
				- image_wid + voxel_size_x / 2.0;
			double cen_y = float(p.y) / float(height - 1)*(2.0*image_hei - voxel_size_y)
				- image_hei + voxel_size_y / 2.0;
			double cen_z = float(p.z) / float(Nslice - 1)*(2.0*image_sli - voxel_size_z)
				- image_sli + voxel_size_z / 2.0;
			voxels[i].center[0] = cen_x;
			voxels[i].center[1] = cen_y;
			voxels[i].center[2] = cen_z;

			std::vector<std::vector<double>> ps = { 
				{ cen_x - voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z + voxel_size_z / 2 } };

			voxels[i].corners[0] = voxels[i].x*(height + 1) + voxels[i].y +
				voxels[i].z*(width + 1)*(height + 1);
			voxels[i].corners[1] = (voxels[i].x + 1)*(height + 1) + voxels[i].y +
				voxels[i].z*(width + 1)*(height + 1);
			voxels[i].corners[2] = (voxels[i].x + 1)*(height + 1) + (voxels[i].y + 1) +
				voxels[i].z*(width + 1)*(height + 1);
			voxels[i].corners[3] = (voxels[i].x + 0)*(height + 1) + (voxels[i].y + 1) +
				voxels[i].z*(width + 1)*(height + 1);
			voxels[i].corners[4] = voxels[i].x*(height + 1) + voxels[i].y +
				(voxels[i].z + 1)*(width + 1)*(height + 1);
			voxels[i].corners[5] = (voxels[i].x + 1)*(height + 1) + voxels[i].y +
				(voxels[i].z + 1)*(width + 1)*(height + 1);
			voxels[i].corners[6] = (voxels[i].x + 1)*(height + 1) + (voxels[i].y + 1) +
				(voxels[i].z + 1)*(width + 1)*(height + 1);
			voxels[i].corners[7] = (voxels[i].x + 0)*(height + 1) + (voxels[i].y + 1) +
				(voxels[i].z + 1)*(width + 1)*(height + 1);

			for (int k = 0;k<8;k++)
			{
				corners_pts[voxels[i].corners[k]] = Point_3(ps[k][0], ps[k][1], ps[k][2]);
			}
			voxels[i].corners_pts[0] = ps[0][0]; voxels[i].corners_pts[1] = ps[0][1]; voxels[i].corners_pts[2] = ps[0][2];
			voxels[i].corners_pts[3] = ps[3][0]; voxels[i].corners_pts[4] = ps[3][1]; voxels[i].corners_pts[5] = ps[3][2];
			voxels[i].corners_pts[6] = ps[1][0]; voxels[i].corners_pts[7] = ps[1][1]; voxels[i].corners_pts[8] = ps[1][2];
			voxels[i].corners_pts[9] = ps[2][0]; voxels[i].corners_pts[10] = ps[2][1]; voxels[i].corners_pts[11] = ps[2][2];
			voxels[i].corners_pts[12] = ps[4][0]; voxels[i].corners_pts[13] = ps[4][1]; voxels[i].corners_pts[14] = ps[4][2];
			voxels[i].corners_pts[15] = ps[7][0]; voxels[i].corners_pts[16] = ps[7][1]; voxels[i].corners_pts[17] = ps[7][2];
			voxels[i].corners_pts[18] = ps[5][0]; voxels[i].corners_pts[19] = ps[5][1]; voxels[i].corners_pts[20] = ps[5][2];
			voxels[i].corners_pts[21] = ps[6][0]; voxels[i].corners_pts[22] = ps[6][1]; voxels[i].corners_pts[23] = ps[6][2];
		
			map_vessel_voxels[voxels[i].index_] = voxels[i];
		}
	}

	void compute_surface(std::map<int, PixelVessel> map_voxels, std::vector<std::vector<int>> &faces) {
		for (auto it = map_voxels.begin();it!= map_voxels.end();it++)
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
				std::vector<int> facet1 = {it->second.corners[1],it->second.corners[2], 
					it->second.corners[6]};
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
					it->second.corners[5]};
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


	void visit_neighbor(int vox,std::map<int, PixelVessel> &voxels)
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
			int xplus = vox + height;
			if (voxels.find(xplus) != voxels.end())
			{
				voxels.at(vox).xfacet = 0;
				//visit_neighbor(xplus, voxels);
			}
		}
		//-x
		//if (v.x > 0)
		{
			int xminus = vox - height;
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
			int zplus = vox + height*width;
			if (voxels.find(zplus) != voxels.end())
			{
				voxels.at(vox).zfacet = 0;
				//visit_neighbor(zplus,voxels);
			}
		}
		//-z
		//if (v.z > 0)
		{
			int zminus = vox - height*width;
			if (voxels.find(zminus) != voxels.end())
			{
				voxels.at(vox).z_facet = 0;
				//visit_neighbor(zminus, voxels);
			}
		}
	}

	void convert_save(std::map<int, Point_3> corners_pts,
		std::vector<std::vector<int>> &faces,SimpleMesh &mesh,std::string file) {
#if SAVE_FILES
		std::ofstream fout_obj;
		fout_obj.open(file.c_str());
		if (!fout_obj.is_open())
			return;
#endif	
		int i = 0;
		std::map<int, int> inde_t;
		for (auto it = corners_pts.begin(); it != corners_pts.end(); it++,i++)
		{
#if SAVE_FILES
			fout_obj << "v" << " "
				<< it->second[0] << " "
				<< it->second[1] << " "
				<< it->second[2] << "\n";
#endif
			inde_t[it->first] = i;
		}
		for (auto it = corners_pts.begin(); it != corners_pts.end(); it++)
		{
			mesh.vertices.push_back(it->second);
		}
		for (int fit = 0; fit < faces.size(); fit++)
		{
			for (int k = 0;k<faces[fit].size();k++)
			{
				if (inde_t.find(faces[fit][k]) == inde_t.end())
				{
					std::cout << "wrong index" << std::endl;
				}
				faces[fit][k] = inde_t.at(faces[fit][k]);				
			}
			mesh.faces.push_back(faces[fit]);
#if SAVE_FILES
			fout_obj << "f " << faces[fit][0] + 1 << " " << faces[fit][1] + 1 << " " << faces[fit][2] + 1 << "\n";
#endif
		}
#if SAVE_FILES
		fout_obj.close();
#endif

		//std::cout << "convert and save done!!!" << std::endl;
	}

	void save(std::map<int, Point_3> corners_pts,
		std::vector<std::vector<int>> faces, std::string file) {
		std::ofstream fout_obj;
		fout_obj.open(file.c_str());
		if (!fout_obj.is_open())
			return;

		for (auto it = corners_pts.begin(); it != corners_pts.end(); it++)
		{
			fout_obj << "v" << " "
				<< it->second[0] << " "
				<< it->second[1] << " "
				<< it->second[2] << "\n";
		}
		for (int fit = 0; fit < faces.size(); fit++)
		{
			fout_obj << "f " << faces[fit][0] + 1 << " " << faces[fit][1] + 1 << " " << faces[fit][2] + 1 << "\n";
		}

		fout_obj.close();
		//std::cout << "save done!!!" << std::endl;
	}

	void read_Meshu(SimpleMesh& meshin, Meshu& mesh_)
	{
		std::vector<Meshu::Vertex_index> vis1(meshin.vertices.size());
		for (int i = 0; i < meshin.vertices.size(); i++)
		{
			vis1[i] = mesh_.add_vertex(meshin.vertices[i]);
		}
		for (int i = 0; i < meshin.faces.size(); i++)
		{
			face_descriptor f = mesh_.add_face(vis1[meshin.faces[i][0]],
				vis1[meshin.faces[i][1]],
				vis1[meshin.faces[i][2]]);
			if (f == Meshu::null_face())
			{
				//std::cerr << "The face could not be added because of an orientation error." << std::endl;
				f = mesh_.add_face(vis1[meshin.faces[i][0]],
					vis1[meshin.faces[i][2]],
					vis1[meshin.faces[i][1]]);
				assert(f != Meshu::null_face());
			}
		}
		//std::cout << "load mesh done!!!" << std::endl;

	}

	void compute_normal(SimpleMesh &meshin, std::vector<Vector_3> &vnors,
		std::vector<float> &smooth_faces) {

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

		if (meshin.vertices.size() != vnors.size())
		{
			std::cout << "wrong: unequal size of corner and normals " << std::endl;
		}

		for (int i = 0;i<faces_.size();i++)
		{
			for (int j = 0;j<faces_[i].size();j++)
			{
				smooth_faces.push_back(float(vnors[faces_[i][j]].x()));
				smooth_faces.push_back(float(vnors[faces_[i][j]].y()));
				smooth_faces.push_back(float(vnors[faces_[i][j]].z()));
				smooth_faces.push_back(float(meshin.vertices[faces_[i][j]].x()));
				smooth_faces.push_back(float(meshin.vertices[faces_[i][j]].y()));
				smooth_faces.push_back(float(meshin.vertices[faces_[i][j]].z()));
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
		if (v.x < width - 1)
		{
			int xplus = vox + height;
			if (voxels.find(xplus) != voxels.end() && !voxels.at(xplus).bvisited)
			{
				voxels.at(xplus).label = voxels.at(vox).label;
				visited.insert(xplus);
			}
		}
		//-x
		if (v.x > 0)
		{
			int xminus = vox - height;
			if (voxels.find(xminus) != voxels.end() && !voxels.at(xminus).bvisited)
			{
				voxels.at(xminus).label = voxels.at(vox).label;
				visited.insert(xminus);
			}
		}
		//y
		if (v.y < height - 1)
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
		if (v.z < slice - 1)
		{
			int zplus = vox + height * width;
			if (voxels.find(zplus) != voxels.end() && !voxels.at(zplus).bvisited)
			{
				voxels.at(zplus).label = voxels.at(vox).label;
				visited.insert(zplus);
			}
		}
		//-z
		if (v.z > 0)
		{
			int zminus = vox - height * width;
			if (voxels.find(zminus) != voxels.end() && !voxels.at(zminus).bvisited)
			{
				voxels.at(zminus).label = voxels.at(vox).label;
				visited.insert(zminus);
			}
		}
	}

	void cluster_pixels(std::map<int, PixelVessel>& voxels, std::vector<PixelVessel> &voxelsin,int thres_number)
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

private:

	int Nslice;
	std::vector<PixelVessel> voxels;
	std::map<int, PixelVessel> map_vessel_voxels;

	//surface
	std::map<int, Point_3> corners_pts;
	std::vector<std::vector<int>> faces_;

	SimpleMesh				mesh_;

	//normals
	std::vector<Vector_3> normals;

};






#endif



