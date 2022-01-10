#include "vessel_ball.h"
//#include "fTetWild.h"

void VesselBall::load_allimages()
{
	int file_size_ = 0;
	//VEIN
	std::vector<std::string> veinmaskfile;
	std::vector<std::pair<QString, int>>	veinmask_files;
	getFiles("vessel/vein/*.png", veinmaskfile);
	for (int i = 0; i < veinmaskfile.size(); i++)
	{
		QString filename_ = QString::fromStdString(veinmaskfile[i]);
		int index = filename_.split("/").last().split(".").at(0).split("_").last().toInt();
		QString fullname("vessel/vein/");
		fullname.append(filename_);
		veinmask_files.push_back(std::pair<QString, int>(fullname, index));
	}
	sort(veinmask_files.begin(), veinmask_files.end(), sort_qstringpair_secondgreater);

	//artery
	std::vector<std::string> arterymaskfile;
	std::vector<std::pair<QString, int>>	arterymask_files;
	getFiles("vessel/artery/*.png", arterymaskfile);
	for (int i = 0; i < arterymaskfile.size(); i++)
	{
		QString filename_ = QString::fromStdString(arterymaskfile[i]);
		int index = filename_.split("/").last().split(".").at(0).split("_").last().toInt();
		QString fullname("vessel/artery/");
		fullname.append(filename_);
		arterymask_files.push_back(std::pair<QString, int>(fullname, index));
	}
	sort(arterymask_files.begin(), arterymask_files.end(), sort_qstringpair_secondgreater);

	//micro
	std::vector<std::string> micromaskfile;
	std::vector<std::pair<QString, int>>	micromask_files;
	getFiles("vessel/micro/*.png", micromaskfile);
	for (int i = 0; i < micromaskfile.size(); i++)
	{
		QString filename_ = QString::fromStdString(micromaskfile[i]);
		int index = filename_.split("/").last().split(".").at(0).split("_").last().toInt();
		QString fullname("vessel/micro/");
		fullname.append(filename_);
		micromask_files.push_back(std::pair<QString, int>(fullname, index));
	}
	sort(micromask_files.begin(), micromask_files.end(), sort_qstringpair_secondgreater);

	//size
	if (veinmask_files.size() > 0)
	{
		QImage im_;
		im_.load(veinmask_files[0].first);
		width = im_.width();
		height = im_.height();
		slice = veinmask_files.size();
	}

	file_size_ = arterymask_files.size() > 0 ? arterymask_files.size() : file_size_;
	file_size_ = micromask_files.size() > 0 ? micromask_files.size() : file_size_;
	file_size_ = veinmaskfile.size() > 0 ? veinmaskfile.size() : file_size_;

	int min_x = 10000, max_x = 0, min_y = 10000, max_y = 0;
	for (int i = 0; i < file_size_; i++)
	{
		if (veinmaskfile.size() > 0) {
			//vein
			QImage im_;
			im_.load(veinmask_files[i].first);
			int check_value;
			if (qGray(im_.pixel(0, 0)) == 0)
			{
				check_value = 0;
			}
			else
			{
				check_value = 255;
			}
			for (int wid = 0; wid < im_.width(); wid++)
			{
				for (int hei = 0; hei < im_.height(); hei++)
				{
					int gray_ = qGray(im_.pixel(wid, im_.height() - 1 - hei));
					if (gray_ != check_value)
					{
						min_x = wid < min_x ? wid : min_x;
						min_y = hei < min_y ? hei : min_y;
						max_x = wid > max_x ? wid : max_x;
						max_y = hei > max_y ? hei : max_y;

						PixelVessel v; v.x = wid; v.y = hei; v.z = i;
						vein_voxels.push_back(v);
					}
				}
			}
		}
		if (arterymask_files.size() > 0) {
			//artery
			QImage im_a;
			im_a.load(arterymask_files[i].first);
			int check_valuea;
			if (qGray(im_a.pixel(0, 0)) == 0)
			{
				check_valuea = 0;
			}
			else
			{
				check_valuea = 255;
			}
			for (int wid = 0; wid < im_a.width(); wid++)
			{
				for (int hei = 0; hei < im_a.height(); hei++)
				{
					int gray_ = qGray(im_a.pixel(wid, im_a.height() - 1 - hei));
					if (gray_ != check_valuea)
					{
						min_x = wid < min_x ? wid : min_x;
						min_y = hei < min_y ? hei : min_y;
						max_x = wid > max_x ? wid : max_x;
						max_y = hei > max_y ? hei : max_y;

						PixelVessel v; v.x = wid; v.y = hei; v.z = i;
						int index = wid * im_a.height() + hei + i * im_a.width() * im_a.height();
						v.index_ = index;
						artery_voxels.push_back(v);
					}
				}
			}
		}
		if (micromask_files.size() > 0) {
			//micro
			QImage im_m;
			im_m.load(micromask_files[i].first);
			int check_valuem;
			if (qGray(im_m.pixel(0, 0)) == 0)
			{
				check_valuem = 0;
			}
			else
			{
				check_valuem = 255;
			}
			for (int wid = 0; wid < im_m.width(); wid++)
			{
				for (int hei = 0; hei < im_m.height(); hei++)
				{
					int gray_ = qGray(im_m.pixel(wid, im_m.height() - 1 - hei));
					if (gray_ != check_valuem)
					{
						min_x = wid < min_x ? wid : min_x;
						min_y = hei < min_y ? hei : min_y;
						max_x = wid > max_x ? wid : max_x;
						max_y = hei > max_y ? hei : max_y;

						PixelVessel v; v.x = wid; v.y = hei; v.z = i;
						int index = wid * im_m.height() + hei + i * im_m.width() * im_m.height();
						v.index_ = index;
						micro_voxels.push_back(v);
					}
				}
			}
		}
	}
	width = max_x - min_x + 1;
	height = max_y - min_y + 1;

	for (int i = 0; i < vein_voxels.size(); i++)
	{
		vein_voxels[i].x -= min_x;
		vein_voxels[i].y -= min_y;
		vein_voxels[i].index_ = vein_voxels[i].x * height +
			vein_voxels[i].y + vein_voxels[i].z * width * height;
	}
	for (int i = 0; i < artery_voxels.size(); i++)
	{
		artery_voxels[i].x -= min_x;
		artery_voxels[i].y -= min_y;
		artery_voxels[i].index_ = artery_voxels[i].x * height +
			artery_voxels[i].y + artery_voxels[i].z * width * height;
	}
	for (int i = 0; i < micro_voxels.size(); i++)
	{
		micro_voxels[i].x -= min_x;
		micro_voxels[i].y -= min_y;
		micro_voxels[i].index_ = micro_voxels[i].x * height +
			micro_voxels[i].y + micro_voxels[i].z * width * height;
	}
	std::cout << "load images done!!!" << std::endl;
}

void VesselBall::renew_pixels(std::map<int, PixelVessel> voxels, std::vector<PixelVessel>& voxel_data)
{
	voxel_data.clear();
	for (auto it = voxels.begin(); it != voxels.end(); it++)
	{
		voxel_data.push_back(it->second);
	}
}

void VesselBall::build_grids(std::map<int, PixelVessel>& voxels)
{
	dataPts.pts.resize(voxels.size());
	int i = 0;
	for (auto it = voxels.begin(); it != voxels.end(); it++, i++) {
		dataPts.pts[i].x = it->second.center[0];
		dataPts.pts[i].y = it->second.center[1];
		dataPts.pts[i].z = it->second.center[2];
	}
	if (kdTree_localspace != NULL)
	{
		delete kdTree_localspace;
		kdTree_localspace = NULL;
	}
	kdTree_localspace = new NanoKdtree(3, //dim
		dataPts,
		nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	kdTree_localspace->buildIndex();

}

double VesselBall::cal_radius_for_v(Point_3 pt)
{
	int					nb_neigh = 1;
	const double query_pt[3] = { pt.x(),pt.y(),pt.z() };
	std::vector<size_t>		ret_index(nb_neigh);
	std::vector<double>		out_dist_sqr(nb_neigh);
	nb_neigh = kdTree_localspace->knnSearch(&query_pt[0], nb_neigh, &ret_index[0], &out_dist_sqr[0]);

	double dis_min = sqrt(out_dist_sqr[0]);
	return dis_min;
}

void VesselBall::generate_mesh()
{
	double image_wid = IMAGEWIDTHSIZE;
	double image_hei = IMAGEWIDTHSIZE / width * height;
	double image_sli = IMAGEWIDTHSIZE / width * slice * SCALEVOXEL;

	grid_dimension[0] = 1.2 * width;
	grid_dimension[1] = 1.2 * height;
	grid_dimension[2] = 1.2 * slice;

	double pixel_width = 2.0 * image_wid / width;
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

	surface* S = CMarchingCube.calculate_isosurface(pixel_width*1.5 );
#if 1
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
		std::vector<double> vt = { S->getVertex(i)[0],S->getVertex(i)[1],S->getVertex(i)[2] };
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

void VesselBall::remove_lonely_pixels(std::map<int, PixelVessel>& voxels)
{
	int count_ = 0;
	int Nmax = voxels.size();
	do {
		std::map<int, PixelVessel> new_voxels;
		bool is_clear = true;
		for (auto it = voxels.begin(); it != voxels.end(); it++)
		{
			bool is_removed = is_lonely(it->first, voxels);
			if (is_removed)
			{
				is_clear = false;
			}
			else
				new_voxels.insert(*it);
		}
		voxels = new_voxels;
		if (is_clear)
			break;

		count_++;
	} while (count_ < Nmax);
	int f = 1;
}

void VesselBall::cluster_pixels(std::map<int, PixelVessel>& voxels, int thres_number)
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

void VesselBall::visit_neighbor(int vox, std::map<int, PixelVessel>& voxels, std::set<int>& visited)
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

bool VesselBall::is_lonely(int vox, std::map<int, PixelVessel>& voxels)
{
	PixelVessel v = voxels.at(vox);
	int count_ = 0;
	//x
	if (v.x < width - 1)
	{
		int xplus = vox + height;
		if (voxels.find(xplus) != voxels.end())
		{
			count_++;
		}
	}
	//-x
	if (v.x > 0)
	{
		int xminus = vox - height;
		if (voxels.find(xminus) != voxels.end())
		{
			count_++;
		}
	}
	//y
	if (v.y < height - 1)
	{
		int yplus = vox + 1;
		if (voxels.find(yplus) != voxels.end())
		{
			count_++;
		}
	}
	//-y
	if (v.y > 0)
	{
		int yminus = vox - 1;
		if (voxels.find(yminus) != voxels.end())
		{
			count_++;
		}
	}
	//z
	if (v.z < slice - 1)
	{
		int zplus = vox + height * width;
		if (voxels.find(zplus) != voxels.end())
		{
			count_++;
		}
	}
	//-z
	if (v.z > 0)
	{
		int zminus = vox - height * width;
		if (voxels.find(zminus) != voxels.end())
		{
			count_++;
		}
	}

	return count_ < 2;
}

void VesselBall::compute_pos_corners() {
	double image_wid = IMAGEWIDTHSIZE;
	double image_hei = IMAGEWIDTHSIZE / width * height;
	double image_sli = IMAGEWIDTHSIZE / width * slice * SCALEVOXEL;

	double voxel_size_x = 2.0 * IMAGEWIDTHSIZE / width;
	double voxel_size_y = voxel_size_x;
	double voxel_size_z = voxel_size_x * SCALEVOXEL;

	for (int i = 0; i < vein_voxels.size(); i++)
	{
		PixelVessel p = vein_voxels[i];
		double cen_x = float(p.x) / float(width - 1) * (2.0 * image_wid - voxel_size_x)
			- image_wid + voxel_size_x / 2.0;
		double cen_y = float(p.y) / float(height - 1) * (2.0 * image_hei - voxel_size_y)
			- image_hei + voxel_size_y / 2.0;
		double cen_z = float(p.z) / float(slice - 1) * (2.0 * image_sli - voxel_size_z)
			- image_sli + voxel_size_z / 2.0;
		vein_voxels[i].center[0] = cen_x;
		vein_voxels[i].center[1] = cen_y;
		vein_voxels[i].center[2] = cen_z;
		mapvein_voxels[vein_voxels[i].index_] = vein_voxels[i];
	}

	for (int i = 0; i < artery_voxels.size(); i++)
	{
		PixelVessel p = artery_voxels[i];
		double cen_x = float(p.x) / float(width - 1) * (2.0 * image_wid - voxel_size_x)
			- image_wid + voxel_size_x / 2.0;
		double cen_y = float(p.y) / float(height - 1) * (2.0 * image_hei - voxel_size_y)
			- image_hei + voxel_size_y / 2.0;
		double cen_z = float(p.z) / float(slice - 1) * (2.0 * image_sli - voxel_size_z)
			- image_sli + voxel_size_z / 2.0;
		artery_voxels[i].center[0] = cen_x;
		artery_voxels[i].center[1] = cen_y;
		artery_voxels[i].center[2] = cen_z;
		mapartery_voxels[artery_voxels[i].index_] = artery_voxels[i];
	}

	//micro
	for (int i = 0; i < micro_voxels.size(); i++)
	{
		PixelVessel p = micro_voxels[i];
		double cen_x = float(p.x) / float(width - 1) * (2.0 * image_wid - voxel_size_x)
			- image_wid + voxel_size_x / 2.0;
		double cen_y = float(p.y) / float(height - 1) * (2.0 * image_hei - voxel_size_y)
			- image_hei + voxel_size_y / 2.0;
		double cen_z = float(p.z) / float(slice - 1) * (2.0 * image_sli - voxel_size_z)
			- image_sli + voxel_size_z / 2.0;
		micro_voxels[i].center[0] = cen_x;
		micro_voxels[i].center[1] = cen_y;
		micro_voxels[i].center[2] = cen_z;
		mapmicro_voxels[micro_voxels[i].index_] = micro_voxels[i];
	}
}

void VesselBall::remesh_ftetwild()
{
	// FtetWild(*surface_, 0.007, 2e-4);
}

void VesselBall::smooth_mesh(SimpleMesh* meshin)
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
	double alpha = 0.5;
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

void VesselBall::collect_v_neighbors(int v, std::set<int>& neighbors, int n_ring_, const std::vector<std::set<int>> &vvneigh) {
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
			collect_v_neighbors(*it, neighbors, n_ring_,vvneigh);
		}
	}
}

void VesselBall::read_Meshu(SimpleMesh* meshin, Meshu& mesh_)
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
	std::cout << "load mesh done!!!" << std::endl;

}

void VesselBall::compute_normal(SimpleMesh* meshin, std::vector<Vector_3>& vnors, std::vector<float>& smooth_faces) {

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
	std::cout << "Vertex normals done:" << std::endl;

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
