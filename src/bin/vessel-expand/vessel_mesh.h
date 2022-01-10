#pragma once
#ifndef _LOAD_VESSEL_
#define _LOAD_VESSEL_

#include "datatype.h"

class Vessel
{
public:
	Vessel(int Nslice_, std::vector<PixelVessel> &voxels_, std::vector<float> &smooth_faces) {
		Nslice = Nslice_;
		voxels = voxels_;
		compute_pos_corners();
		compute_surface(map_vessel_voxels, faces_);
		convert_save(corners_pts, faces_, "vessel/vein.obj");
		compute_normal(corners_pts, faces_, normals, smooth_faces);
		voxels_ = voxels;
	}

	~Vessel() {
	}

private:

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
		std::vector<std::vector<int>> &faces,std::string file) {
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

	void subdivision(std::map<int, Point_3> &corners_pts,
		std::vector<std::vector<int>> &faces)
	{
		//subdivision
		std::map<int, Point_3> vs;
		std::vector<std::vector<int>> fs;
		Mesh_sub temp;
		read_Meshsub(corners_pts, faces, temp);
		CGAL::Subdivision_method_3::Loop_subdivision(temp, 1);
		std::map<Mesh_sub::Vertex_iterator, int> vs_id;
		int id = 0;
		for (auto vi = temp.vertices_begin(); vi != temp.vertices_end(); vi++)
		{
			vs[id]=(Point_3(vi->point().x(), vi->point().y(), vi->point().z()));
			vs_id[vi] = id;
			id++;
		}

		for (auto fit = temp.facets_begin(); fit != temp.facets_end(); fit++)
		{
			std::vector<int> facet;
			Mesh_sub::Halfedge_around_facet_circulator hc = fit->facet_begin(), hs = hc;
			do
			{
				facet.push_back(vs_id.at(hc->vertex()));
			} while (++hc != hs);
			fs.push_back(facet);
		}
		corners_pts = vs;
		faces = fs;
		std::cout << "subdivision done!!!" << std::endl;
	}

	void remesh(std::map<int, Point_3> &corners_pts,
		std::vector<std::vector<int>> &faces_) {

		std::map<int, Point_3> vs;
		std::vector<std::vector<int>> fs;

		//remesh
		Meshu mesh;
		read_Meshu(corners_pts, faces_, mesh);

		double target_edge_length = IMAGEWIDTHSIZE/width;
		unsigned int nb_iter = 2;
		std::vector<edge_descriptor> border;
		PMP::border_halfedges(faces(mesh), mesh, boost::make_function_output_iterator(halfedge2edge(mesh, border)));
		PMP::split_long_edges(border, target_edge_length, mesh);
		//std::cout << "Start remeshing of "<< " (" << num_faces(mesh) << " faces)..." << std::endl;
		PMP::isotropic_remeshing(
			faces(mesh),
			target_edge_length,
			mesh,
			PMP::parameters::number_of_iterations(nb_iter)
			.protect_constraints(true)//i.e. protect border, here
		);
		std::ofstream output("vessel/remeshing.off");
		output << mesh;

		std::vector<int> reindex;
		reindex.resize(mesh.num_vertices());
		int n = 0;
		BOOST_FOREACH(Meshu::Vertex_index v, mesh.vertices()) {
			vs[n]=(mesh.point(v));
			reindex[v] = n++;
		}
		BOOST_FOREACH(Meshu::Face_index f, mesh.faces()) {
			std::vector<int> facet;
			BOOST_FOREACH(Meshu::Vertex_index v, CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
				facet.push_back(reindex[v]);
			}
			fs.push_back(facet);
		}
		corners_pts = vs;
		faces_ = fs;
		std::cout << "remesh done!!!" << std::endl;
	}

	void read_Meshsub(std::map<int, Point_3> vertices, std::vector<std::vector<int>> faces,
		Mesh_sub &mesh_)
	{
		Point_3 *vertex = new Point_3[vertices.size()];
		int **face = new int*[faces.size()];

		for (int i = 0; i < faces.size(); i++)
		{
			face[i] = new int[3];
			for (int j = 0; j < 3; j++)
			{
				face[i][j] = faces[i][j];
			}
		}
		int i = 0;
		for (auto it = vertices.begin(); it != vertices.end(); it++,i++)
		{
			vertex[i] = it->second;
		}

		Builder_obj<HD> builder(vertex, face, vertices.size(), faces.size(), 3);
		mesh_.delegate(builder);

		delete[]vertex;
		for (int i = 0; i < faces.size(); i++)
		{
			delete[]face[i];
		}
		delete[]face;

		std::cout << "load mesh done!!!" << std::endl;

	}

	void read_Meshu(std::map<int, Point_3> vertices, std::vector<std::vector<int>> faces,
		Meshu &mesh_)
	{
		std::vector<Meshu::Vertex_index> vis1(vertices.size());
		int i = 0;
		for (auto it = vertices.begin(); it != vertices.end(); it++,i++)
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
		std::vector<std::vector<int>> faces_, std::vector<Vector_3> &vnors,
		std::vector<float> &smooth_faces) {

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

		for (int i = 0;i<faces_.size();i++)
		{
			for (int j = 0;j<faces_[i].size();j++)
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

private:

	int Nslice;
	std::vector<PixelVessel> voxels;
	std::map<int, PixelVessel> map_vessel_voxels;

	//surface
	std::map<int, Point_3> corners_pts;
	std::vector<std::vector<int>> faces_;

	//normals
	std::vector<Vector_3> normals;

};






#endif



