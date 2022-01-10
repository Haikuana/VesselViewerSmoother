#pragma once
#ifndef _VESSEL_BALL_
#define _VESSEL_BALL_

#include "load_vessel.h"
#include "MC33.h"

class VesselBall
{
public:
	VesselBall() {}
	~VesselBall() {}

	void exec()
	{
		clock_t t1 = clock();
		load_allimages();
		compute_pos_corners();
		std::cout << "load done" << std::endl;
		//remove_lonely_pixels(mapvein_voxels);
		cluster_pixels(mapvein_voxels,20);	
		renew_pixels(mapvein_voxels, vein_voxels);
		std::cout << "remove done" << std::endl;
		build_grids(mapvein_voxels);
		generate_mesh();
		std::cout << "mesh done" << std::endl;
		//remesh_ftetwild();
		//std::cout << "remesh done" << std::endl;

		smooth_mesh(surface_);
		std::cout << "smooth done" << std::endl;
		compute_normal(surface_, vein_normals, vein_smooth_faces);
		std::cout << "normal done" << std::endl;

		clock_t t2 = clock();
		std::cout << "total rumtime: " << (t2 - t1) / 1000 << std::endl;
	}
	std::vector<int>		get_size()
	{
		std::vector<int> s = {width,height,slice};
		return s;
	}

	std::vector<PixelVessel> get_vein()
	{
		return vein_voxels;
	}

	std::vector<PixelVessel> get_artery()
	{
		return artery_voxels;
	}

	std::vector<PixelVessel> get_micro()
	{
		return micro_voxels;
	}

	std::vector<float> get_vein_smooth_faces()
	{
		return vein_smooth_faces;
	}

	std::vector<float> get_artery_smooth_faces()
	{
		return artery_smooth_faces;
	}

	std::vector<float> get_micro_smooth_faces()
	{
		return micro_smooth_faces;
	}

private:

	void read_Meshu(SimpleMesh *meshin,Meshu& mesh_);

	void compute_normal(SimpleMesh* meshin, std::vector<Vector_3>& vnors, std::vector<float>& smooth_faces);

	void load_allimages();

	void renew_pixels(std::map<int, PixelVessel> voxels, std::vector<PixelVessel>& voxel_data);

	void build_grids(std::map<int, PixelVessel>& voxels);

	double cal_radius_for_v(Point_3 pt);

	void generate_mesh();

	void remove_lonely_pixels(std::map<int, PixelVessel>& voxels);

	void cluster_pixels(std::map<int, PixelVessel>& voxels, int thres_number);

	void visit_neighbor(int vox, std::map<int, PixelVessel>& voxels, std::set<int>& visited);
	
	bool is_lonely(int vox, std::map<int, PixelVessel>& voxels);

	void compute_pos_corners();

	void remesh_ftetwild();

	void smooth_mesh(SimpleMesh* meshin);

	void collect_v_neighbors(int v, std::set<int>& neighbors, int n_ring_, const std::vector<std::set<int>>& vvneigh);
private:

	int width = 0;
	int height = 0;
	int slice = 0;

	std::vector<PixelVessel> vein_voxels;
	std::vector<PixelVessel> artery_voxels;
	std::vector<PixelVessel> micro_voxels;

	std::map<int, PixelVessel> mapvein_voxels;
	std::map<int, PixelVessel> mapartery_voxels;
	std::map<int, PixelVessel> mapmicro_voxels;

	PointCloud						dataPts;
	NanoKdtree*						kdTree_localspace;

	int								grid_dimension[3];
	grid3d							grids;
	MC33							CMarchingCube;
	SimpleMesh*						surface_;

	//surface
	std::vector<float> vein_smooth_faces;
	std::vector<float> artery_smooth_faces;
	std::vector<float> micro_smooth_faces;
	//normals
	std::vector<Vector_3> vein_normals;
	std::vector<Vector_3> artery_normals;
	std::vector<Vector_3> micro_normals;
};

#endif



