#pragma once
#ifndef _VESSEL_CUBE_
#define _VESSEL_CUBE_

#include "load_vessel.h"

int width = 0;
int height = 0;
int slice = 0;

class VesselCube
{
public:
	VesselCube() {}
	~VesselCube() {}

	void exec()
	{
		load_allimages();
		compute_pos_corners();
		compute_surface(mapvein_voxels, vein_faces);
		compute_surface(mapartery_voxels, artery_faces);
		compute_surface(mapmicro_voxels, micro_faces);

		std::cout << "pre-compute done!!!" << std::endl;
		convert_save(veincorners_pts, vein_faces, "vein.obj");
		convert_save(arterycorners_pts, artery_faces, "artery.obj");
		convert_save(microcorners_pts, micro_faces, "micro.obj");

		compute_normal(veincorners_pts, vein_faces, vein_normals, vein_smooth_faces);
		compute_normal(arterycorners_pts, artery_faces, artery_normals, artery_smooth_faces);
		compute_normal(microcorners_pts, micro_faces, micro_normals, micro_smooth_faces);
		//subdivision(veincorners_pts, vein_faces);
		//remesh(veincorners_pts, vein_faces);
		//save(veincorners_pts, vein_faces, "vessel/smooth_vein.obj");
		//subdivision(arterycorners_pts, artery_faces);
		//remesh(arterycorners_pts, artery_faces);
		//save(arterycorners_pts, artery_faces, "vessel/smooth_artery.obj");
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

	void load_allimages()
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

	void compute_pos_corners() {
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

			std::vector<std::vector<double>> ps = {
				{ cen_x - voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z + voxel_size_z / 2 } };

			vein_voxels[i].corners[0] = vein_voxels[i].x * (height + 1) + vein_voxels[i].y +
				vein_voxels[i].z * (width + 1) * (height + 1);
			vein_voxels[i].corners[1] = (vein_voxels[i].x + 1) * (height + 1) + vein_voxels[i].y +
				vein_voxels[i].z * (width + 1) * (height + 1);
			vein_voxels[i].corners[2] = (vein_voxels[i].x + 1) * (height + 1) + (vein_voxels[i].y + 1) +
				vein_voxels[i].z * (width + 1) * (height + 1);
			vein_voxels[i].corners[3] = (vein_voxels[i].x + 0) * (height + 1) + (vein_voxels[i].y + 1) +
				vein_voxels[i].z * (width + 1) * (height + 1);
			vein_voxels[i].corners[4] = vein_voxels[i].x * (height + 1) + vein_voxels[i].y +
				(vein_voxels[i].z + 1) * (width + 1) * (height + 1);
			vein_voxels[i].corners[5] = (vein_voxels[i].x + 1) * (height + 1) + vein_voxels[i].y +
				(vein_voxels[i].z + 1) * (width + 1) * (height + 1);
			vein_voxels[i].corners[6] = (vein_voxels[i].x + 1) * (height + 1) + (vein_voxels[i].y + 1) +
				(vein_voxels[i].z + 1) * (width + 1) * (height + 1);
			vein_voxels[i].corners[7] = (vein_voxels[i].x + 0) * (height + 1) + (vein_voxels[i].y + 1) +
				(vein_voxels[i].z + 1) * (width + 1) * (height + 1);

			for (int k = 0; k < 8; k++)
			{
				veincorners_pts[vein_voxels[i].corners[k]] = Point_3(ps[k][0], ps[k][1], ps[k][2]);
			}
			vein_voxels[i].corners_pts[0] = ps[0][0]; vein_voxels[i].corners_pts[1] = ps[0][1]; vein_voxels[i].corners_pts[2] = ps[0][2];
			vein_voxels[i].corners_pts[3] = ps[3][0]; vein_voxels[i].corners_pts[4] = ps[3][1]; vein_voxels[i].corners_pts[5] = ps[3][2];
			vein_voxels[i].corners_pts[6] = ps[1][0]; vein_voxels[i].corners_pts[7] = ps[1][1]; vein_voxels[i].corners_pts[8] = ps[1][2];
			vein_voxels[i].corners_pts[9] = ps[2][0]; vein_voxels[i].corners_pts[10] = ps[2][1]; vein_voxels[i].corners_pts[11] = ps[2][2];
			vein_voxels[i].corners_pts[12] = ps[4][0]; vein_voxels[i].corners_pts[13] = ps[4][1]; vein_voxels[i].corners_pts[14] = ps[4][2];
			vein_voxels[i].corners_pts[15] = ps[7][0]; vein_voxels[i].corners_pts[16] = ps[7][1]; vein_voxels[i].corners_pts[17] = ps[7][2];
			vein_voxels[i].corners_pts[18] = ps[5][0]; vein_voxels[i].corners_pts[19] = ps[5][1]; vein_voxels[i].corners_pts[20] = ps[5][2];
			vein_voxels[i].corners_pts[21] = ps[6][0]; vein_voxels[i].corners_pts[22] = ps[6][1]; vein_voxels[i].corners_pts[23] = ps[6][2];

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

			std::vector<std::vector<double>> ps = {
				{ cen_x - voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y - voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z + voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,cen_y + voxel_size_y / 2,cen_z + voxel_size_z / 2 } };

			artery_voxels[i].corners[0] = artery_voxels[i].x * (height + 1) + artery_voxels[i].y +
				artery_voxels[i].z * (width + 1) * (height + 1);
			artery_voxels[i].corners[1] = (artery_voxels[i].x + 1) * (height + 1) + artery_voxels[i].y +
				artery_voxels[i].z * (width + 1) * (height + 1);
			artery_voxels[i].corners[2] = (artery_voxels[i].x + 1) * (height + 1) + (artery_voxels[i].y + 1) +
				artery_voxels[i].z * (width + 1) * (height + 1);
			artery_voxels[i].corners[3] = (artery_voxels[i].x + 0) * (height + 1) + (artery_voxels[i].y + 1) +
				artery_voxels[i].z * (width + 1) * (height + 1);
			artery_voxels[i].corners[4] = artery_voxels[i].x * (height + 1) + artery_voxels[i].y +
				(artery_voxels[i].z + 1) * (width + 1) * (height + 1);
			artery_voxels[i].corners[5] = (artery_voxels[i].x + 1) * (height + 1) + artery_voxels[i].y +
				(artery_voxels[i].z + 1) * (width + 1) * (height + 1);
			artery_voxels[i].corners[6] = (artery_voxels[i].x + 1) * (height + 1) + (artery_voxels[i].y + 1) +
				(artery_voxels[i].z + 1) * (width + 1) * (height + 1);
			artery_voxels[i].corners[7] = (artery_voxels[i].x + 0) * (height + 1) + (artery_voxels[i].y + 1) +
				(artery_voxels[i].z + 1) * (width + 1) * (height + 1);

			for (int k = 0; k < 8; k++)
			{
				arterycorners_pts[artery_voxels[i].corners[k]] = Point_3(ps[k][0], ps[k][1], ps[k][2]);
			}
			artery_voxels[i].corners_pts[0] = ps[0][0]; artery_voxels[i].corners_pts[1] = ps[0][1]; artery_voxels[i].corners_pts[2] = ps[0][2];
			artery_voxels[i].corners_pts[3] = ps[3][0]; artery_voxels[i].corners_pts[4] = ps[3][1]; artery_voxels[i].corners_pts[5] = ps[3][2];
			artery_voxels[i].corners_pts[6] = ps[1][0]; artery_voxels[i].corners_pts[7] = ps[1][1]; artery_voxels[i].corners_pts[8] = ps[1][2];
			artery_voxels[i].corners_pts[9] = ps[2][0]; artery_voxels[i].corners_pts[10] = ps[2][1]; artery_voxels[i].corners_pts[11] = ps[2][2];
			artery_voxels[i].corners_pts[12] = ps[4][0]; artery_voxels[i].corners_pts[13] = ps[4][1]; artery_voxels[i].corners_pts[14] = ps[4][2];
			artery_voxels[i].corners_pts[15] = ps[7][0]; artery_voxels[i].corners_pts[16] = ps[7][1]; artery_voxels[i].corners_pts[17] = ps[7][2];
			artery_voxels[i].corners_pts[18] = ps[5][0]; artery_voxels[i].corners_pts[19] = ps[5][1]; artery_voxels[i].corners_pts[20] = ps[5][2];
			artery_voxels[i].corners_pts[21] = ps[6][0]; artery_voxels[i].corners_pts[22] = ps[6][1]; artery_voxels[i].corners_pts[23] = ps[6][2];

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

			std::vector<std::vector<double>> ps = {
				{ cen_x - voxel_size_x / 2,
				cen_y - voxel_size_y / 2,
				cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,
				cen_y - voxel_size_y / 2,
				cen_z - voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,
				cen_y + voxel_size_y / 2,
				cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,
				cen_y + voxel_size_y / 2,
				cen_z - voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,
				cen_y - voxel_size_y / 2,
				cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,
				cen_y - voxel_size_y / 2,
				cen_z + voxel_size_z / 2 },
				{ cen_x + voxel_size_x / 2,
				cen_y + voxel_size_y / 2,
				cen_z + voxel_size_z / 2 },
				{ cen_x - voxel_size_x / 2,
				cen_y + voxel_size_y / 2,
				cen_z + voxel_size_z / 2 } };

			micro_voxels[i].corners[0] = micro_voxels[i].x * (height + 1) + micro_voxels[i].y +
				micro_voxels[i].z * (width + 1) * (height + 1);
			micro_voxels[i].corners[1] = (micro_voxels[i].x + 1) * (height + 1) + micro_voxels[i].y +
				micro_voxels[i].z * (width + 1) * (height + 1);
			micro_voxels[i].corners[2] = (micro_voxels[i].x + 1) * (height + 1) + (micro_voxels[i].y + 1) +
				micro_voxels[i].z * (width + 1) * (height + 1);
			micro_voxels[i].corners[3] = (micro_voxels[i].x + 0) * (height + 1) + (micro_voxels[i].y + 1) +
				micro_voxels[i].z * (width + 1) * (height + 1);
			micro_voxels[i].corners[4] = micro_voxels[i].x * (height + 1) + micro_voxels[i].y +
				(micro_voxels[i].z + 1) * (width + 1) * (height + 1);
			micro_voxels[i].corners[5] = (micro_voxels[i].x + 1) * (height + 1) + micro_voxels[i].y +
				(micro_voxels[i].z + 1) * (width + 1) * (height + 1);
			micro_voxels[i].corners[6] = (micro_voxels[i].x + 1) * (height + 1) + (micro_voxels[i].y + 1) +
				(micro_voxels[i].z + 1) * (width + 1) * (height + 1);
			micro_voxels[i].corners[7] = (micro_voxels[i].x + 0) * (height + 1) + (micro_voxels[i].y + 1) +
				(micro_voxels[i].z + 1) * (width + 1) * (height + 1);

			for (int k = 0; k < 8; k++)
			{
				microcorners_pts[micro_voxels[i].corners[k]] = Point_3(ps[k][0], ps[k][1], ps[k][2]);
			}
			micro_voxels[i].corners_pts[0] = ps[0][0]; micro_voxels[i].corners_pts[1] = ps[0][1]; micro_voxels[i].corners_pts[2] = ps[0][2];
			micro_voxels[i].corners_pts[3] = ps[3][0]; micro_voxels[i].corners_pts[4] = ps[3][1]; micro_voxels[i].corners_pts[5] = ps[3][2];
			micro_voxels[i].corners_pts[6] = ps[1][0]; micro_voxels[i].corners_pts[7] = ps[1][1]; micro_voxels[i].corners_pts[8] = ps[1][2];
			micro_voxels[i].corners_pts[9] = ps[2][0]; micro_voxels[i].corners_pts[10] = ps[2][1]; micro_voxels[i].corners_pts[11] = ps[2][2];
			micro_voxels[i].corners_pts[12] = ps[4][0]; micro_voxels[i].corners_pts[13] = ps[4][1]; micro_voxels[i].corners_pts[14] = ps[4][2];
			micro_voxels[i].corners_pts[15] = ps[7][0]; micro_voxels[i].corners_pts[16] = ps[7][1]; micro_voxels[i].corners_pts[17] = ps[7][2];
			micro_voxels[i].corners_pts[18] = ps[5][0]; micro_voxels[i].corners_pts[19] = ps[5][1]; micro_voxels[i].corners_pts[20] = ps[5][2];
			micro_voxels[i].corners_pts[21] = ps[6][0]; micro_voxels[i].corners_pts[22] = ps[6][1]; micro_voxels[i].corners_pts[23] = ps[6][2];

			mapmicro_voxels[micro_voxels[i].index_] = micro_voxels[i];
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


	void visit_neighbor(int vox, std::map<int, PixelVessel>& voxels)
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
			if (voxels.find(xplus) != voxels.end())
			{
				voxels.at(vox).xfacet = 0;
				//visit_neighbor(xplus, voxels);
			}
		}
		//-x
		if (v.x > 0)
		{
			int xminus = vox - height;
			if (voxels.find(xminus) != voxels.end())
			{
				voxels.at(vox).x_facet = 0;
				//visit_neighbor(xminus, voxels);
			}
		}
		//y
		if (v.y < height - 1)
		{
			int yplus = vox + 1;
			if (voxels.find(yplus) != voxels.end())
			{
				voxels.at(vox).yfacet = 0;
				//visit_neighbor(yplus, voxels);
			}
		}
		//-y
		if (v.y > 0)
		{
			int yminus = vox - 1;
			if (voxels.find(yminus) != voxels.end())
			{
				voxels.at(vox).y_facet = 0;
				//visit_neighbor(yminus, voxels);
			}
		}
		//z
		if (v.z < slice - 1)
		{
			int zplus = vox + height * width;
			if (voxels.find(zplus) != voxels.end())
			{
				voxels.at(vox).zfacet = 0;
				//visit_neighbor(zplus,voxels);
			}
		}
		//-z
		if (v.z > 0)
		{
			int zminus = vox - height * width;
			if (voxels.find(zminus) != voxels.end())
			{
				voxels.at(vox).z_facet = 0;
				//visit_neighbor(zminus, voxels);
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

		std::cout << "convert and save done!!!" << std::endl;
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
		std::cout << "save done!!!" << std::endl;
	}

	void subdivision(std::map<int, Point_3>& corners_pts,
		std::vector<std::vector<int>>& faces)
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
			vs[id] = (Point_3(vi->point().x(), vi->point().y(), vi->point().z()));
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

	void remesh(std::map<int, Point_3>& corners_pts,
		std::vector<std::vector<int>>& faces_) {

		std::map<int, Point_3> vs;
		std::vector<std::vector<int>> fs;

		//remesh
		Meshu mesh;
		read_Meshu(corners_pts, faces_, mesh);

		double target_edge_length = IMAGEWIDTHSIZE / width;
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
			vs[n] = (mesh.point(v));
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
		Mesh_sub& mesh_)
	{
		Point_3* vertex = new Point_3[vertices.size()];
		int** face = new int* [faces.size()];

		for (int i = 0; i < faces.size(); i++)
		{
			face[i] = new int[3];
			for (int j = 0; j < 3; j++)
			{
				face[i][j] = faces[i][j];
			}
		}
		int i = 0;
		for (auto it = vertices.begin(); it != vertices.end(); it++, i++)
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
		std::cout << "load mesh done!!!" << std::endl;

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
		std::cout << "Vertex normals done:" << std::endl;

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

private:

	std::vector<PixelVessel> vein_voxels;
	std::vector<PixelVessel> artery_voxels;
	std::vector<PixelVessel> micro_voxels;

	std::map<int, PixelVessel> mapvein_voxels;
	std::map<int, PixelVessel> mapartery_voxels;
	std::map<int, PixelVessel> mapmicro_voxels;

	//surface
	std::map<int, Point_3> veincorners_pts;
	std::vector<std::vector<int>> vein_faces;
	std::vector<float> vein_smooth_faces;

	std::map<int, Point_3> arterycorners_pts;
	std::vector<std::vector<int>> artery_faces;
	std::vector<float> artery_smooth_faces;

	std::map<int, Point_3> microcorners_pts;
	std::vector<std::vector<int>> micro_faces;
	std::vector<float> micro_smooth_faces;

	//normals
	std::vector<Vector_3> vein_normals;
	std::vector<Vector_3> artery_normals;
	std::vector<Vector_3> micro_normals;
};

#endif