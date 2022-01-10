#pragma once
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh.h>
#include "vessel_mesh.h"

inline int save_data(std::string path, GPara *paras, const std::vector<std::vector<float>>& faces)
{
#pragma omp parallel for
	for (int i = 0; i < faces.size(); i++)
	{
		int from_, to_;
		if (i == paras->NCOMOBO) {
			from_ = 1;
			to_ = paras->Total_sclice;
		}
		else
		{
			from_ = i * (paras->SLICE_INTERNAL / 2) + 1;
			to_ = from_ + paras->SLICE_INTERNAL - 1;
		}

		if (faces[i].empty())
			continue;
		std::string filename = path + "Slice_" + std::to_string(from_)+"_"+ std::to_string(to_) + ".stl";
		std::ofstream fout(filename, std::ios::binary);
		if (!fout)
			continue;
		fout.precision(4);
		fout << "#write into " << filename << "\n";
		for (int j = 0; j < faces[i].size(); j += 18) {
			GEO::vec3 p1(faces[i][j + 3], faces[i][j + 4], faces[i][j + 5]);
			GEO::vec3 p2(faces[i][j + 9], faces[i][j + 10], faces[i][j + 11]);
			GEO::vec3 p3(faces[i][j + 15], faces[i][j + 16], faces[i][j + 17]);
			fout << "facet normal " << normalize(cross(p2 - p1, p3 - p1))
				<< std::endl;
			fout << "outer loop" << std::endl;
			fout << "vertex " << p1 << std::endl;
			fout << "vertex " << p2 << std::endl;
			fout << "vertex " << p3 << std::endl;
			fout << "endloop" << std::endl;
			fout << "endfacet" << std::endl;
		}
		fout.close();
	}
	return 1;
}

inline int save_data(std::string filename, const std::vector<std::vector<float>> &faces, 
	const std::vector<std::vector<PixelVessel>> &voxels)
{
	std::ofstream fout(filename,std::ios::binary);
	if (!fout || voxels.size() != faces.size())
		return -1;

	//ofs.write(reinterpret_cast<char*>(&model), sizeof(model));

	fout.precision(4);
	fout << "#write into " << filename << "\n";
	for (int i = 0; i < faces.size(); i++)
	{
		fout<<"mesh "<< faces[i].size()<<" ";
		for (int j = 0; j < faces[i].size(); j++) {
			fout << faces[i][j] << " ";
			//fout.write((char*)&faces[i][j], sizeof(char));
		}
#if SAVE_VOXELS
		fout << "voxels " << voxels[i].size() << " ";
		for (int j = 0; j < voxels[i].size(); j++) {
			for (int k = 0; k < 3; k++)
			{
				fout << voxels[i][j].center[k] << " ";
			}
			for (int k = 0; k < 24; k++)
			{
				fout << voxels[i][j].corners_pts[k] << " ";
			}
		}	
		fout << "\n";
#endif
	}
	fout << "End\n";
	fout.close();
	return 1;
}

inline int read_data(std::string filename, std::vector<std::vector<float>> &faces,
	std::vector<std::vector<PixelVessel>>& voxels)
{
	std::string model_path(filename);
	//read data from test_input.in file
	QString configure_path = QString::fromStdString(model_path);
	QFile file(configure_path);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QTextStream in(&file);
		QString line = in.readLine();
		if (line.isNull())
			return false;
		while (!line.isNull())
		{
			QStringList list = line.split(" ");
			if (!list.at(0).compare("End"))
			{
				line = in.readLine();
				break;
			}
			if (!list.at(0).compare("mesh"))
			{
				std::vector<float> fs;
				int size_ = list[1].toInt();
				for (int j = 2; j < list.size() - 1; j++)
				{
					float v_ = list[j].toFloat();
					fs.push_back(v_);
				}
				faces.push_back(fs);
			}
#if SAVE_VOXELS
			if (!list.at(0).compare("voxels"))
			{
				int size_ = list[1].toInt();
				std::vector<PixelVessel> vs;
				for (int j = 2; j < list.size() - 1;)
				{
					PixelVessel p;
					for (int k = 0; k < 3; k++)
					{
						float v_ = list[j].toFloat();
						p.center[k] = v_;
						j++;
					}
					for (int k = 0; k < 24; k++)
					{
						float v_ = list[j].toFloat();
						p.corners_pts[k] = v_;
						j++;
					}
					vs.push_back(p);
				}
				voxels.push_back(vs);
			}
#endif
			line = in.readLine();
		}
		file.close();
	}
}

class CompoundLayers
{
public:
	CompoundLayers(int expand_, GPara* paras_in) {
		expand_level = expand_;
		paras = paras_in;
		load_allimages();

		if (BComboSlice)
		{
			vein_smooth_faces.resize(paras->NCOMOBO + 1);
			artery_smooth_faces.resize(paras->NCOMOBO + 1);
			micro_smooth_faces.resize(paras->NCOMOBO + 1);
//#pragma omp parallel for
			for (int i = 0; i < paras->NCOMOBO + 1; i++)
			{
				if (paras->NCOMOBO == 1 && i == 0)
					continue;
				int Nslice = paras->SLICE_INTERNAL;
				if (i == paras->NCOMOBO)
				{
					Nslice = paras->slice;
				}
				std::vector<float> smooth_faces;
				if (!vein_voxels[i].empty()) {
					Vessel veinV(Nslice,paras, vein_voxels[i], smooth_faces);
					vein_smooth_faces[i] = (smooth_faces);
					smooth_faces.clear();
				}
				if (!artery_voxels[i].empty()) {
					Vessel arteryV(Nslice, paras, artery_voxels[i], smooth_faces);
					artery_smooth_faces[i] = (smooth_faces);
					smooth_faces.clear();
				}
				if (!micro_voxels[i].empty()) {
					Vessel microV(Nslice, paras, micro_voxels[i], smooth_faces);
					micro_smooth_faces[i] = (smooth_faces);
				}
				GEO::Logger::out("combo") << "ComboSlices: " << i << " Done!" << std::endl;
			}
		}
		else
		{
			int Nslice = paras->slice;
			std::vector<float> smooth_faces;
			Vessel veinV(Nslice, paras, vein_voxels[0], smooth_faces);
			vein_smooth_faces.push_back(smooth_faces);
			smooth_faces.clear();
			Vessel arteryV(Nslice, paras, artery_voxels[0], smooth_faces);
			artery_smooth_faces.push_back(smooth_faces);
			smooth_faces.clear();
			Vessel microV(Nslice, paras, micro_voxels[0], smooth_faces);
			micro_smooth_faces.push_back(smooth_faces);
			GEO::Logger::out("combo") << "ComboSlices: Done!" << std::endl;
		}
	}

	~CompoundLayers() {}

	std::vector<std::vector<PixelVessel>> get_vein()
	{
		return vein_voxels;
	}

	std::vector<std::vector<PixelVessel>> get_artery()
	{
		return artery_voxels;
	}

	std::vector<std::vector<PixelVessel>> get_micro()
	{
		return micro_voxels;
	}

	std::vector<std::vector<float>> get_vein_smooth_faces()
	{
		return vein_smooth_faces;
	}

	std::vector<std::vector<float>> get_artery_smooth_faces()
	{
		return artery_smooth_faces;
	}

	std::vector<std::vector<float>> get_micro_smooth_faces()
	{
		return micro_smooth_faces;
	}

	inline void load_allimages();

	void preprocess_vessel(std::vector<PixelVessel> &vein_slice,
	std::vector<PixelVessel> &artery_slice,std::vector<PixelVessel> &micro_slice) {

		std::map<int, ExpandVessel> map_vessel_voxels;
		for (auto it = micro_slice.begin(); it != micro_slice.end(); it++)
		{
			ExpandVessel ev; ev.index_ = it->index_;
			ev.vessel_type = MICRO; ev.x = it->x; ev.y = it->y; ev.z = it->z;
			map_vessel_voxels[it->index_] = ev;
		}
		for (auto it = vein_slice.begin();it != vein_slice.end();it++)
		{
			ExpandVessel ev; ev.index_ = it->index_;
			ev.vessel_type = VEIN; ev.x = it->x; ev.y = it->y; ev.z = it->z;
			map_vessel_voxels[it->index_] = ev;
		}
		for (auto it = artery_slice.begin(); it != artery_slice.end(); it++)
		{
			ExpandVessel ev; ev.index_ = it->index_;
			ev.vessel_type = ARTERY; ev.x = it->x; ev.y = it->y; ev.z = it->z;
			map_vessel_voxels[it->index_] = ev;
		}	

		std::vector<PixelVessel> vein_temp = vein_slice;
		std::vector<PixelVessel> arte_temp = artery_slice;
		for (int i = 0;i<expand_level;i++)
		{			
			expand_cuslice(vein_temp,map_vessel_voxels,VEIN);
			for (auto it = vein_temp.begin();it != vein_temp.end();it++)
			{
				vein_slice.push_back(*it);
			}
			expand_cuslice(arte_temp, map_vessel_voxels, ARTERY);
			for (auto it = arte_temp.begin(); it != arte_temp.end(); it++)
			{
				artery_slice.push_back(*it);
			}
		}
	}

	void expand_cuslice(std::vector<PixelVessel> &vessel_slice,
		std::map<int, ExpandVessel> &map_vessel_voxels,int type_) {

		if (vessel_slice.empty())
		{
			return;
		}

		int z_id = vessel_slice.begin()->z;
		std::set<std::pair<int,int>> new_added;
		for (auto it = vessel_slice.begin(); it != vessel_slice.end(); it++)
		{		
			int plusx = it->index_ + paras->height;
			auto res = map_vessel_voxels.find(plusx);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{			
				new_added.insert(std::pair<int,int>(it->x + 1, it->y));
				res->second.vessel_type = type_;
			}
			int plus_x = it->index_ - paras->height;
			res = map_vessel_voxels.find(plus_x);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{
				new_added.insert(std::pair<int, int>(it->x - 1, it->y));
				res->second.vessel_type = type_;
			}
			int plusy = it->index_ + 1;
			res = map_vessel_voxels.find(plusy);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{
				new_added.insert(std::pair<int, int>(it->x, it->y+1));
				res->second.vessel_type = type_;
			}
			int plus_y = it->index_ - 1;
			res = map_vessel_voxels.find(plus_y);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{
				new_added.insert(std::pair<int, int>(it->x, it->y - 1));
				res->second.vessel_type = type_;
			}
			int plusxy = it->index_ + paras->height + 1;
			res = map_vessel_voxels.find(plusxy);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{
				new_added.insert(std::pair<int, int>(it->x+1, it->y + 1));
				res->second.vessel_type = type_;
			}
			int plusx_y = it->index_ + paras->height - 1;
			res = map_vessel_voxels.find(plusx_y);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{
				new_added.insert(std::pair<int, int>(it->x + 1, it->y - 1));
				res->second.vessel_type = type_;
			}
			int plus_xy = it->index_ - paras->height + 1;
			res = map_vessel_voxels.find(plus_xy);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{
				new_added.insert(std::pair<int, int>(it->x - 1, it->y + 1));
				res->second.vessel_type = type_;
			}
			int plus_x_y = it->index_ - paras->height - 1;
			res = map_vessel_voxels.find(plus_x_y);
			if (res != map_vessel_voxels.end() && res->second.vessel_type == MICRO)
			{
				new_added.insert(std::pair<int, int>(it->x - 1, it->y - 1));
				res->second.vessel_type = type_;
			}			
		}
	
		vessel_slice.clear();
		for (auto it = new_added.begin();it != new_added.end();it++)
		{
			PixelVessel newp; newp.x = it->first; newp.y = it->second;
			newp.z = z_id;
			newp.index_ = newp.x* paras->height + newp.y + newp.z* paras->width* paras->height;
			vessel_slice.push_back(newp);
		}
	}

private:
	int expand_level;
	GPara* paras;
	std::vector<std::vector<PixelVessel>> vein_voxels;
	std::vector<std::vector<PixelVessel>> artery_voxels;
	std::vector<std::vector<PixelVessel>> micro_voxels;

	std::vector<std::vector<float>> vein_smooth_faces;
	std::vector<std::vector<float>> artery_smooth_faces;
	std::vector<std::vector<float>> micro_smooth_faces;
};

void CompoundLayers::load_allimages()
{
	int file_size_ = 0;
	//VEIN
	std::vector<std::string> veinmaskfile;
	std::vector<std::pair<QString, int>>	veinmask_files;
	GEO::FileSystem::get_files(paras->inpath+"vein", veinmaskfile);
	for (int i = 0; i < veinmaskfile.size(); i++)
	{
		QString filename_ = QString::fromStdString(veinmaskfile[i]);
		int index = filename_.split("/").last().split(".").at(0).split("_").last().toInt();
		veinmask_files.push_back(std::pair<QString, int>(filename_, index));
	}
	sort(veinmask_files.begin(), veinmask_files.end(), sort_qstringpair_secondgreater);

	//artery
	std::vector<std::string> arterymaskfile;
	std::vector<std::pair<QString, int>>	arterymask_files;
	GEO::FileSystem::get_files(paras->inpath + "artery", arterymaskfile);
	for (int i = 0; i < arterymaskfile.size(); i++)
	{
		QString filename_ = QString::fromStdString(arterymaskfile[i]);
		int index = filename_.split("/").last().split(".").at(0).split("_").last().toInt();
		arterymask_files.push_back(std::pair<QString, int>(filename_, index));
	}
	sort(arterymask_files.begin(), arterymask_files.end(), sort_qstringpair_secondgreater);

	//micro
	std::vector<std::string> micromaskfile;
	std::vector<std::pair<QString, int>>	micromask_files;
	GEO::FileSystem::get_files(paras->inpath + "micro", micromaskfile);
	for (int i = 0; i < micromaskfile.size(); i++)
	{
		QString filename_ = QString::fromStdString(micromaskfile[i]);
		int index = filename_.split("/").last().split(".").at(0).split("_").last().toInt();
		micromask_files.push_back(std::pair<QString, int>(filename_, index));
	}
	sort(micromask_files.begin(), micromask_files.end(), sort_qstringpair_secondgreater);

	//size
	if (veinmask_files.size()>0)
	{
		QImage im_;
		im_.load(veinmask_files[0].first);
		paras->width = std::max(paras->width,im_.width());
		paras->height = std::max(paras->height,im_.height());
		paras->slice = std::max(paras->slice, int(veinmask_files.size()));
	}
	if (arterymask_files.size() > 0)
	{
		QImage im_;
		im_.load(arterymask_files[0].first);
		paras->width = std::max(paras->width, im_.width());
		paras->height = std::max(paras->height, im_.height());
		paras->slice = std::max(paras->slice, int(arterymask_files.size()));
	}
	if (micromask_files.size() > 0)
	{
		QImage im_;
		im_.load(micromask_files[0].first);
		paras->width = std::max(paras->width, im_.width());
		paras->height = std::max(paras->height, im_.height());
		paras->slice = std::max(paras->slice, int(micromask_files.size()));
	}

	file_size_ = arterymask_files.size() > 0 ? arterymask_files.size() : file_size_;
	file_size_ = micromask_files.size() > file_size_ ? micromask_files.size() : file_size_;
	file_size_ = veinmask_files.size() > file_size_ ? veinmask_files.size() : file_size_;

	if (arterymask_files.size() != veinmask_files.size())
	{
		GEO::Logger::err("read") << "wrong size: artery != vein" << std::endl;
	}

	int min_x = 10000, max_x = 0, min_y = 10000, max_y = 0;
	std::vector<std::vector<PixelVessel>> vein_all(file_size_), artery_all(file_size_), micro_all(file_size_);
//#pragma omp parallel for
	for (int i = 0; i < file_size_; i++)
	{
		std::vector<PixelVessel> vein_now, artery_now, micro_now;

		if (file_size_ > veinmask_files.size() &&
			i>= paras->VA_FROM && i<= paras->VA_TO ||
			file_size_ == veinmask_files.size())
		{
			//vein
			if (!veinmask_files.empty()) {
				QImage im_;
				im_.load(veinmask_files[i - paras->VA_FROM].first);
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
							int index = wid * im_.height() + hei + i * im_.width() * im_.height();
							v.index_ = index;
							vein_now.push_back(v);
						}
					}
				}
			}
			//artery
			if (!arterymask_files.empty()) {
				QImage im_a;
				im_a.load(arterymask_files[i - paras->VA_FROM].first);
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
							artery_now.push_back(v);
						}
					}
				}
			}
		}	

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
					int index = wid*im_m.height() + hei + i*im_m.width()*im_m.height();
					v.index_ = index;
					micro_now.push_back(v);
				}
			}
		}

		//pre-process vessel
		std::vector<PixelVessel> vein_temp, artery_temp, micro_temp;
		vein_temp = vein_now;
		artery_temp = artery_now;
		micro_temp = micro_now;
		if (arterymask_files.size() == veinmask_files.size())
		{
			preprocess_vessel(vein_temp, artery_now, micro_now);
			for (auto it = vein_temp.begin(); it != vein_temp.end(); it++)
			{
				vein_now.push_back(*it);
			}
			for (auto it = artery_temp.begin(); it != artery_temp.end(); it++)
			{
				artery_now.push_back(*it);
			}
		}
		else
		{
			GEO::Logger::err("read") << "wrong: inequivalent size for vein-artery-micro\n";
		}
		vein_all[i] = vein_now;
		artery_all[i] = artery_now;
		micro_all[i] = micro_now;
	}
	paras->width = max_x - min_x + 1;
	paras->height = max_y - min_y + 1;

	std::vector<PixelVessel> vein_whole, artery_whole, micro_whole;
	for (int k = 0; k<vein_all.size(); k++)
	{
		for (int i = 0; i < vein_all[k].size(); i++)
		{
			PixelVessel newp = vein_all[k][i];
			newp.x -= min_x;
			newp.y -= min_y;
			newp.index_ = newp.x* paras->height + newp.y + newp.z* paras->width* paras->height;
			vein_all[k][i] = newp;
			vein_whole.push_back(newp);
		}
	}
	for (int k = 0; k < artery_all.size(); k++)
	{
		for (int i = 0; i < artery_all[k].size(); i++)
		{
			PixelVessel newp = artery_all[k][i];
			newp.x -= min_x;
			newp.y -= min_y;
			newp.index_ = newp.x * paras->height + newp.y + newp.z * paras->width * paras->height;
			artery_all[k][i] = newp;
			artery_whole.push_back(newp);
		}
	}
	for (int k = 0; k < micro_all.size(); k++)
	{
		for (int i = 0; i < micro_all[k].size(); i++)
		{
			PixelVessel newp = micro_all[k][i];
			newp.x -= min_x;
			newp.y -= min_y;
			newp.index_ = newp.x * paras->height + newp.y + newp.z * paras->width * paras->height;
			micro_all[k][i] = newp;
			micro_whole.push_back(newp);
		}
	}

	if (BComboSlice)
	{
		vein_voxels.resize(paras->NCOMOBO + 1);
		artery_voxels.resize(paras->NCOMOBO + 1);
		micro_voxels.resize(paras->NCOMOBO + 1);
		for (int i = 0; i < paras->NCOMOBO; i++)
		{
			{
				int internal_ = paras->SLICE_INTERNAL, break_ = paras->SLICE_INTERNAL/2;
				for (int k = i * break_; k < i * break_ + internal_; k++)
				{
					if (k < vein_all.size()) {
						for (auto it = vein_all[k].begin(); it != vein_all[k].end(); it++)
						{
							vein_voxels[i].push_back(*it);
						}
					}
					if (k < artery_all.size()) {
						for (auto it = artery_all[k].begin(); it != artery_all[k].end(); it++)
						{
							artery_voxels[i].push_back(*it);
						}
					}
					if (k < micro_all.size()) {
						for (auto it = micro_all[k].begin(); it != micro_all[k].end(); it++)
						{
							micro_voxels[i].push_back(*it);
						}
					}
				}
			}
		}
		vein_voxels[paras->NCOMOBO] = vein_whole;
		artery_voxels[paras->NCOMOBO] = artery_whole;
		micro_voxels[paras->NCOMOBO] = micro_whole;
	}
	else
	{
		vein_voxels.push_back(vein_whole);
		artery_voxels.push_back(artery_whole);
		micro_voxels.push_back(micro_whole);
	}

	GEO::Logger::out("I/O") << "load images done!!!" << std::endl;
}

