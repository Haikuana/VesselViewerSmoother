#include "viewer.h"

namespace GEO {
	int VesselViewer::current_comboslice = 0;
	extern int MaxCom = 0;

	VesselViewer::VesselViewer() : SimpleApplication("Vessel Viewer") {

		//current directory
		const std::string directory_= GEO::FileSystem::get_current_working_directory();
		std::vector<std::string> out_;
		String::split_string(directory_, '\\', out_);
		//out_.push_back("vessel");
		directory_model = String::join_strings(out_, "/");

		layers = NULL;
		vein_faces = NULL;
		artery_faces = NULL;
		micro_faces = NULL;

		mesh_ = false;
		point_size_ = 10.0f;
		shrink_ = 0.0f;
		smooth_ = true;

		balpha = false;
		micro_alpha = 0.3;

		do_draw_vein = true;
		do_draw_artery = true;
		do_draw_micro = true;

		vein_color_ = vec4f(0.0f, 0.0f, 0.8f, 1.0f);
		artery_color_ = vec4f(0.8f, 0.0f, 0.0f, 1.0f);
		micro_color_ = vec4f(0.8f, 0.62f, 0.13f, 1.0f);
		micro_backcolor_ = vec4f(0.963f, 0.581f, 0.704f, 1.0f);

		if (paras.height > 0) {
			image_wid = IMAGEWIDTHSIZE;
			image_hei = IMAGEWIDTHSIZE / paras.width * paras.height;
			image_sli = IMAGEWIDTHSIZE / paras.width * paras.slice * SCALEVOXEL;

			// Define the 3d region that we want to display
			// (xmin, ymin, zmin, xmax, ymax, zmax)
			set_region_of_interest(-image_wid, -image_hei, -image_sli, image_wid, image_hei, image_sli);
		}
		else
		{
			set_region_of_interest(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);
		}
		primitive_ = 1;
		//std::cout << GLUP::GEOMETRY_VERTICES_OUT << std::endl;

		add_key_func("x", decrement_comboId_callback, "- cells shrink");
		add_key_func("w", increment_comboId_callback, "+ cells shrink");
	}

	VesselViewer::~VesselViewer() {
		if (vein_faces)
		{
			delete vein_faces;
			vein_faces = NULL;
		}
		if (artery_faces)
		{
			delete artery_faces;
			artery_faces = NULL;
		}
		if (micro_faces)
		{
			delete micro_faces;
			micro_faces = NULL;
		}
		if (layers)
		{
			delete layers;
			layers = NULL;
		}
	}

	void VesselViewer::load_vessel(int type, std::string path_)
	{
		paras.inpath = path_;

		std::vector<std::string> micromaskfile;
		GEO::FileSystem::get_files(path_ + "micro", micromaskfile);
		std::vector<std::string> veinmaskfile;
		GEO::FileSystem::get_files(path_+"vein", veinmaskfile);
		std::vector<std::string> artmaskfile;
		GEO::FileSystem::get_files(path_+"artery", artmaskfile);

		if (veinmaskfile.size() != artmaskfile.size())
		{
			GEO::Logger::err("I/O") << "Size of vein must be same with artery!!" << std::endl;
			return;
		}
		paras.Total_sclice = micromaskfile.size();
		paras.VA_FROM = 0;
		paras.VA_TO = paras.Total_sclice - 1;
		if (veinmaskfile.size() != micromaskfile.size() && !veinmaskfile.empty())
		{
			paras.VA_FROM = (micromaskfile.size() - veinmaskfile.size()) / 2;
			paras.VA_TO = paras.VA_FROM + veinmaskfile.size() - 1;
		}

		if (paras.Total_sclice == 96)
		{
			paras.SLICE_INTERNAL = 24;
		}
		else if (paras.Total_sclice == 56)
		{
			paras.SLICE_INTERNAL = 14;
		}
		else if (paras.Total_sclice == 48)
		{
			paras.SLICE_INTERNAL = 12;
		}
		else if (paras.Total_sclice < 20)
		{
			paras.SLICE_INTERNAL = paras.Total_sclice;
		}
		else if (paras.Total_sclice < 35)
		{
			paras.SLICE_INTERNAL = paras.Total_sclice / 2;
		}
		else
			paras.SLICE_INTERNAL = paras.Total_sclice / 4;

		for (int t = 0; t < paras.Total_sclice - paras.SLICE_INTERNAL;)
		{
			paras.NCOMOBO++;
			t += paras.SLICE_INTERNAL / 2;
		}
		if (paras.Total_sclice > paras.SLICE_INTERNAL)
			paras.NCOMOBO++;
		MaxCom = paras.NCOMOBO;
		paras.type = type;

		clock_t t1 = clock();
		std::vector<std::vector<float>> vein_fs;
		std::vector<std::vector<float>> arte_fs;
		std::vector<std::vector<float>> mico_fs;
		if (type == 2)
		{
			std::string file_vein = path_ + "vein.data";
			std::string file_artery = path_ + "artery.data";
			std::string file_micro = path_ + "micro.data";
			read_data(file_vein, vein_fs, all_vein_voxels);
			read_data(file_artery, arte_fs, all_artery_voxels);
			read_data(file_micro, mico_fs, all_micro_voxels);
			clock_t t2 = clock();
			GEO::Logger::out("Load") << "time for loading: " << (t2 - t1) / 1000.0 << std::endl;
		}
		else
		{
			layers = new CompoundLayers(EXPANDLEVEL, &paras);
			all_vein_voxels = layers->get_vein();
			all_artery_voxels = layers->get_artery();
			all_micro_voxels = layers->get_micro();
			vein_fs = layers->get_vein_smooth_faces();
			arte_fs = layers->get_artery_smooth_faces();
			mico_fs = layers->get_micro_smooth_faces();

			clock_t t2 = clock();
			GEO::Logger::out("Load") << "time for calcultion: " << (t2 - t1) / 1000.0 << std::endl;

			if (type == 1) {
				if (!vein_fs.empty() && !all_vein_voxels.empty())
				{
					save_data(path_ + "vein_", &paras, vein_fs);
				}
				if (!arte_fs.empty() && !all_artery_voxels.empty())
				{
					save_data(path_ + "artery_", &paras, arte_fs);
				}
				if (!mico_fs.empty() && !all_micro_voxels.empty())
				{
					save_data(path_ + "micro_", &paras, mico_fs);
				}
				clock_t t3 = clock();
				GEO::Logger::out("Load") << "time for saving: " << (t2 - t1) / 1000.0 << std::endl;
			}
		}
		if (!vein_fs.empty()) {
			vein_faces = new float* [vein_fs.size()];
			for (int k = 0; k < vein_fs.size(); k++)
			{
				vein_faces_size.push_back(vein_fs[k].size());
				vein_faces[k] = new float[vein_faces_size[k]];
				std::copy(vein_fs[k].begin(), vein_fs[k].end(), vein_faces[k]);
			}
		}		
		if (!arte_fs.empty()) {
			artery_faces = new float* [arte_fs.size()];
			for (int k = 0; k < arte_fs.size(); k++)
			{
				artery_faces_size.push_back(arte_fs[k].size());
				artery_faces[k] = new float[artery_faces_size[k]];
				std::copy(arte_fs[k].begin(), arte_fs[k].end(), artery_faces[k]);
			}
		}
		if (!mico_fs.empty()) {
			micro_faces = new float* [mico_fs.size()];
			for (int k = 0; k < mico_fs.size(); k++)
			{
				micro_faces_size.push_back(mico_fs[k].size());
				micro_faces[k] = new float[micro_faces_size[k]];
				std::copy(mico_fs[k].begin(), mico_fs[k].end(), micro_faces[k]);
			}
		}
	}

	/**
	 * \copydoc SimpleApplication::GL_terminate()
	 */
	void VesselViewer::GL_terminate() {
		SimpleApplication::GL_terminate();
	}

	/**
	 * \brief Displays and handles the GUI for object properties.
	 * \details Overloads Application::draw_object_properties().
	 */
	void VesselViewer::draw_object_properties() {
		SimpleApplication::draw_object_properties();

		//int expand_level;
		//ImGui::SliderInt("Round", &expand_level, 1, 10);
		//if (ImGui::SimpleButton(icon_UTF8("redo-alt") + "Update"))
		//{
		//	//update_expand_level();
		//}
		//ImGui::Separator();

		if (BComboSlice)
		{
			int from_, to_;
			ImGui::SliderInt("", &current_comboslice, 0, paras.NCOMOBO, "");
			if (current_comboslice == paras.NCOMOBO) {
				from_ = 1;
				to_ = paras.Total_sclice;
			}
			else
			{
				from_ = current_comboslice * (paras.SLICE_INTERNAL / 2) + 1;
				to_ = from_ + paras.SLICE_INTERNAL - 1;
			}

			std::string slice_range = "Slice: " + std::to_string(from_) + " to " + std::to_string(to_);
			char* cslice_range = (char*)slice_range.c_str();
			ImGui::LabelText("", cslice_range);
		}

		ImGui::NewLine();
		ImGui::Checkbox("##Vein", &do_draw_vein);
		ImGui::SameLine();
		ImGui::ColorEdit3WithPalette("Vein", vein_color_.data());
		if (do_draw_vein)
		{
			vein_colors[0] = vein_color_.x;
			vein_colors[1] = vein_color_.y;
			vein_colors[2] = vein_color_.z;
			vein_colors[3] = 1.0f;
		}
		ImGui::Separator();
		ImGui::Checkbox("##Artery", &do_draw_artery);
		ImGui::SameLine();
		ImGui::ColorEdit3WithPalette("Artery", artery_color_.data());
		if (do_draw_artery)
		{
			artery_colors[0] = artery_color_.x;
			artery_colors[1] = artery_color_.y;
			artery_colors[2] = artery_color_.z;
			artery_colors[3] = 1.0f;
		}
		ImGui::Separator();
		ImGui::Checkbox("##Micro", &do_draw_micro);
		ImGui::SameLine();
		ImGui::ColorEdit3WithPalette("Micro", micro_color_.data());
		if (do_draw_micro)
		{
			micro_colors[0] = micro_color_.x;
			micro_colors[1] = micro_color_.y;
			micro_colors[2] = micro_color_.z;
			micro_colors[3] = 0.1f;
		}
		if (balpha)
		{
			//ImGui::Indent();
			ImGui::ColorEdit3WithPalette("Micro", micro_backcolor_.data());
			if (do_draw_micro)
			{
				micro_backcolors[0] = micro_backcolor_.x;
				micro_backcolors[1] = micro_backcolor_.y;
				micro_backcolors[2] = micro_backcolor_.z;
				micro_backcolors[3] = 1.0f;
			}
			//ImGui::Unindent();
		}

		ImGui::NewLine();
		/*ImGui::Combo(" ", &primitive_,
			"point\0surface\0volume\0\0"
		);*/

		ImGui::RadioButton("Points", &primitive_, 0);
		ImGui::SliderFloat("Size", &point_size_, 1.0f, 50.0f, "%.1f");
		ImGui::Separator();

		ImGui::RadioButton("Surface", &primitive_, 1);
		ImGui::Checkbox("Smooth", &smooth_);
		ImGui::Checkbox("Alpha", &balpha);
		ImGui::Separator();

		ImGui::RadioButton("Volume", &primitive_, 2);
		ImGui::SliderFloat("Shrk", &shrink_, 0.0f, 1.0f, "%.2f");
	}

	/**
	 * \brief Draws the scene according to currently set primitive and
	 *  drawing modes.
	 */
	void VesselViewer::draw_scene() {

		//glupSetSpecular(0.4f);

		// GLUP can have different colors for frontfacing and
		// backfacing polygons.
		glupSetColor3f(GLUP_FRONT_COLOR, 1.0f, 1.0f, 0.0f);
		glupSetColor3f(GLUP_BACK_COLOR, 1.0f, 0.0f, 1.0f);

		// Take into account the toggles from the Object pane:

		// Enable/disable individual per-vertex colors.
		glupDisable(GLUP_VERTEX_COLORS);

		// There is a global light switch. Note: facet normals are
		// automatically computed by GLUP, no need to specify
		// them ! (but you cannot have per-vertex normals).
		if (lighting_) {
			glupEnable(GLUP_LIGHTING);
		}
		else {
			glupDisable(GLUP_LIGHTING);
		}

		// Each facet can have a black outline displayed.
		if (mesh_) {
			glupEnable(GLUP_DRAW_MESH);
		}
		else {
			glupDisable(GLUP_DRAW_MESH);
		}

		// Texture mapping.
		glupDisable(GLUP_TEXTURING);

		if (primitive_ == 1) {
			if (smooth_) {
				glupEnable(GLUP_VERTEX_NORMALS);
			}
			else {
				glupDisable(GLUP_VERTEX_NORMALS);
			}
		}

		switch (primitive_) {

		case 0: {
			glupSetPointSize(point_size_);
			if (do_draw_vein && !all_vein_voxels.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, vein_colors);
				glupBegin(GLUP_POINTS);
				for (int i = 0; i < all_vein_voxels[current_comboslice].size(); i++)
				{
					glupVertex3f(all_vein_voxels[current_comboslice][i].center[0],
						all_vein_voxels[current_comboslice][i].center[1],
						all_vein_voxels[current_comboslice][i].center[2]);
				}
				glupEnd();
			}
			if (do_draw_artery && !all_artery_voxels.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, artery_colors);
				glupBegin(GLUP_POINTS);
				for (int i = 0; i < all_artery_voxels[current_comboslice].size(); i++)
				{
					glupVertex3f(all_artery_voxels[current_comboslice][i].center[0],
						all_artery_voxels[current_comboslice][i].center[1],
						all_artery_voxels[current_comboslice][i].center[2]);
				}
				glupEnd();
			}
			if (do_draw_micro && !all_micro_voxels.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, micro_colors);
				glupBegin(GLUP_POINTS);
				for (int i = 0; i < all_micro_voxels[current_comboslice].size(); i++)
				{
					glupVertex3f(all_micro_voxels[current_comboslice][i].center[0],
						all_micro_voxels[current_comboslice][i].center[1],
						all_micro_voxels[current_comboslice][i].center[2]);
				}
				glupEnd();
			}
		} break;


		case 1: {

			float specular_backup = glupGetSpecular();
			glupSetSpecular(0.4f);

			if (do_draw_vein && !vein_faces_size.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, vein_colors);
				glupBegin(GLUP_TRIANGLES);
				for (int i = 0; i < vein_faces_size[current_comboslice];)
				{
					glupNormal3d(vein_faces[current_comboslice][i],
						vein_faces[current_comboslice][i + 1], vein_faces[current_comboslice][i + 2]);
					i += 3;
					glupVertex3d(vein_faces[current_comboslice][i],
						vein_faces[current_comboslice][i + 1], vein_faces[current_comboslice][i + 2]);
					i += 3;
				}
				glupEnd();
			}
			if (do_draw_artery && !artery_faces_size.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, artery_colors);
				glupBegin(GLUP_TRIANGLES);
				for (int i = 0; i < artery_faces_size[current_comboslice];)
				{
					glupNormal3d(artery_faces[current_comboslice][i],
						artery_faces[current_comboslice][i + 1], artery_faces[current_comboslice][i + 2]);
					i += 3;
					glupVertex3d(artery_faces[current_comboslice][i],
						artery_faces[current_comboslice][i + 1], artery_faces[current_comboslice][i + 2]);
					i += 3;
				}
				glupEnd();
			}
			if (do_draw_micro && !micro_faces_size.empty())
			{
				if (balpha)
				{
					glupEnable(GLUP_ALPHA_DISCARD);
					//micro_colors[3] = micro_alpha;
				}

				glupSetColor4fv(GLUP_FRONT_COLOR, micro_colors);
				glupSetColor4fv(GLUP_BACK_COLOR, micro_backcolors);
				glupBegin(GLUP_TRIANGLES);
				for (int i = 0; i < micro_faces_size[current_comboslice];)
				{
					glupNormal3d(micro_faces[current_comboslice][i],
						micro_faces[current_comboslice][i + 1], micro_faces[current_comboslice][i + 2]);
					i += 3;
					glupVertex3d(micro_faces[current_comboslice][i],
						micro_faces[current_comboslice][i + 1], micro_faces[current_comboslice][i + 2]);
					i += 3;
				}
				glupEnd();

				glupDisable(GLUP_ALPHA_DISCARD);
			}

			glupSetSpecular(specular_backup);
		} break;

		case 2: {

			if (do_draw_vein && !all_vein_voxels.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, vein_colors);
				glupBegin(GLUP_HEXAHEDRA);
				for (int i = 0; i < all_vein_voxels[current_comboslice].size(); i++)
				{
					PixelVessel p = all_vein_voxels[current_comboslice][i];
					for (int k = 0; k < 8; k++)
					{
						glupVertex3f(p.corners_pts[k * 3], p.corners_pts[k * 3 + 1], p.corners_pts[k * 3 + 2]);
					}
				}
				glupEnd();
			}
			if (do_draw_artery && !all_artery_voxels.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, artery_colors);
				glupBegin(GLUP_HEXAHEDRA);
				for (int i = 0; i < all_artery_voxels[current_comboslice].size(); i++)
				{
					PixelVessel p = all_artery_voxels[current_comboslice][i];
					for (int k = 0; k < 8; k++)
					{
						glupVertex3f(p.corners_pts[k * 3], p.corners_pts[k * 3 + 1], p.corners_pts[k * 3 + 2]);
					}
				}
				glupEnd();
			}

			glupSetCellsShrink(shrink_);
			if (do_draw_micro && !all_micro_voxels.empty())
			{
				glupSetColor4fv(GLUP_FRONT_COLOR, micro_colors);
				glupBegin(GLUP_HEXAHEDRA);
				for (int i = 0; i < all_micro_voxels[current_comboslice].size(); i++)
				{
					PixelVessel p = all_micro_voxels[current_comboslice][i];
					for (int k = 0; k < 8; k++)
					{
						glupVertex3f(p.corners_pts[k * 3], p.corners_pts[k * 3 + 1], p.corners_pts[k * 3 + 2]);
					}
				}
				glupEnd();
			}
			glupSetCellsShrink(0.0);

		} break;

		default:
			break;
		}

		glupDisable(GLUP_VERTEX_NORMALS);
	}

	void VesselViewer::draw_menu_bar()
	{
		if (ImGui::BeginMainMenuBar()) {
			if (ImGui::BeginMenu("File")) {
				
				if (ImGui::SimpleButton(icon_UTF8("folder-open") + " Load...")) {
					ImGui::FileDialog("load_path",
						input_configure_path,
						geo_imgui_string_length, " ", directory_model);
					load_type = 0;
				}

				if (ImGui::SimpleButton(icon_UTF8("folder-open") + " Load&Save...")) {
					ImGui::FileDialog("load_path",
						input_configure_path,
						geo_imgui_string_length, " ", directory_model);
					load_type = 1;
				}
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}
		std::string inpath(input_configure_path);
		if (load_type != -1 && inpath != "")
		{
			std::vector<std::string> out_;
			String::split_string(inpath, '/', out_);
			if (out_.rbegin()->empty())
			{
				out_.pop_back();
			}
			inpath = String::join_strings(out_, "/") + "/";
			load_vessel(load_type, inpath);
			load_type = -1;
		}
	}

	/**
	 * \brief Creates the texture.
	 * \details This function overloads Application::init_graphics(). It
	 *  is called as soon as the OpenGL context is ready for rendering. It
	 *  is meant to initialize the graphic objects used by the application.
	 */
	void VesselViewer::GL_initialize() {
		SimpleApplication::GL_initialize();
	}

	void VesselViewer::decrement_comboId_callback()
	{
		current_comboslice = std::max(current_comboslice - 1, 0);
	}

	void VesselViewer::increment_comboId_callback()
	{
		current_comboslice = std::min(current_comboslice + 1, MaxCom);
	}


}