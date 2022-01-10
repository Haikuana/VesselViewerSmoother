/*
 *  Copyright (c) 2012-2014, Bruno Levy
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy
 *
 *     Bruno.Levy@inria.fr
 *     http://www.loria.fr/~levy
 *
 *     ALICE Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 */

#include <geogram_gfx/gui/simple_application.h>
#include <geogram_gfx/GLUP/GLUP_private.h>
#include "compound_layers.h"

namespace {

    using namespace GEO;

    /**
     * \brief An application that demonstrates both
     *  GLUP primitives and glup_viewer application
     *  framework.
     */
	class DemoGlupApplication : public SimpleApplication {
	public:

		/**
		 * \brief DemoGlupApplication constructor.
		 */
		DemoGlupApplication() : SimpleApplication("Demo GLUP") {

			layers = new CompoundLayers(EXPANDLEVEL);

			all_vein_voxels = layers->get_vein();
			all_artery_voxels = layers->get_artery();
			all_micro_voxels = layers->get_micro();
			
			std::vector<std::vector<float>> vein_fs = layers->get_vein_smooth_faces();
			std::vector<std::vector<float>> arte_fs = layers->get_artery_smooth_faces();
			std::vector<std::vector<float>> mico_fs = layers->get_micro_smooth_faces();

			vein_faces = new float*[vein_fs.size()];
			artery_faces = new float*[arte_fs.size()];
			micro_faces = new float*[mico_fs.size()];
			for (int k = 0;k<vein_fs.size();k++)
			{
				vein_faces_size.push_back(vein_fs[k].size());
				vein_faces[k] = new float[vein_faces_size[k]];
				std::copy(vein_fs[k].begin(), vein_fs[k].end(), vein_faces[k]);		

				artery_faces_size.push_back(arte_fs[k].size());
				artery_faces[k] = new float[artery_faces_size[k]];
				std::copy(arte_fs[k].begin(), arte_fs[k].end(), artery_faces[k]);

				micro_faces_size.push_back(mico_fs[k].size());
				micro_faces[k] = new float[micro_faces_size[k]];
				std::copy(mico_fs[k].begin(), mico_fs[k].end(), micro_faces[k]);
			}
			
			mesh_ = false;
			point_size_ = 10.0f;
			shrink_ = 0.0f;
			smooth_ = true;
			
			current_comboslice = 0;

			balpha = true;
			micro_alpha = 0.3;

			do_draw_vein = true;
			do_draw_artery = true;
			do_draw_micro = true;

			vein_color_ =  vec4f(0.0f, 0.0f, 0.8f, 1.0f);
			artery_color_ = vec4f(0.8f, 0.0f, 0.0f, 1.0f);
			micro_color_ = vec4f(0.8f, 0.62f, 0.13f, 1.0f);
			micro_backcolor_ = vec4f(0.963f, 0.581f, 0.704f, 1.0f);

			image_wid = IMAGEWIDTHSIZE;
			image_hei = IMAGEWIDTHSIZE / width*height;
			image_sli = IMAGEWIDTHSIZE / width*slice*SCALEVOXEL;

			// Define the 3d region that we want to display
			// (xmin, ymin, zmin, xmax, ymax, zmax)
			set_region_of_interest(-image_wid,-image_hei,-image_sli,image_wid,image_hei,image_sli);
			primitive_ = 1;
			//std::cout << GLUP::GEOMETRY_VERTICES_OUT << std::endl;

			
		}

		~DemoGlupApplication() {
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

		/**
		 * \copydoc SimpleApplication::GL_terminate()
		 */
		void GL_terminate() override {
			SimpleApplication::GL_terminate();
		}

		/**
		 * \brief Displays and handles the GUI for object properties.
		 * \details Overloads Application::draw_object_properties().
		 */
		void draw_object_properties() override {
			SimpleApplication::draw_object_properties();

			/*ImGui::SliderInt("Round", &expand_level, 1, 10);
			if (ImGui::SimpleButton(icon_UTF8("redo-alt") + "Update"))
			{
				update_expand_level();
			}
			ImGui::Separator();*/

			if (BComboSlice)
			{
				int from_, to_;
				if (SLICE_INTERNAL == 21)
				{
					ImGui::SliderInt("", &current_comboslice, 0, 6, "");
					if (current_comboslice < 5)
					{
						from_ = current_comboslice * 5;
						to_ = from_ + 20;
					}
					else if (current_comboslice == 5)
					{
						from_ = 27;
						to_ = 47;
					}
					else
					{
						from_ = 0;
						to_ = 47;
					}
				}
				else
				{
					//internal 11
					ImGui::SliderInt("", &current_comboslice, 0, 19, "");
					if (current_comboslice < 18)
					{
						from_ = current_comboslice * 2;
						to_ = from_ + 10;
					}
					else if (current_comboslice == 18)
					{
						from_ = 37;
						to_ = 47;
					}
					else
					{
						from_ = 0;
						to_ = 47;
					}
				}				
				std::string slice_range = "Slice: " + std::to_string(from_) + " to " + std::to_string(to_);
				char* cslice_range = (char*)slice_range.c_str();
				ImGui::LabelText("", cslice_range);
			}

			ImGui::NewLine();
			ImGui::NewLine();
			ImGui::Checkbox("##Vein", &do_draw_vein);
			ImGui::SameLine();
			ImGui::ColorEdit3WithPalette("Vein.", vein_color_.data());
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
				ImGui::Indent();
				ImGui::ColorEdit3WithPalette("MicroBack", micro_backcolor_.data());
				if (do_draw_micro)
				{
					micro_backcolors[0] = micro_backcolor_.x;
					micro_backcolors[1] = micro_backcolor_.y;
					micro_backcolors[2] = micro_backcolor_.z;
					micro_backcolors[3] = 1.0f;
				}
				ImGui::Unindent();
			}

			ImGui::NewLine();
			ImGui::NewLine();
			ImGui::Combo(" ", &primitive_,
				"point\0surface\0volume\0\0"
			);
			//ImGui::SliderInt("n", (int*)&n_, 4, 300);
			//ImGui::Text(
			//	"%s",
			//	("nb prim.: " + String::to_string(nb_primitives())).c_str()
			//);
			ImGui::Separator();
			if (primitive_ == 1) {
				ImGui::Checkbox("Smooth", &smooth_);
			}
			if (primitive_ == 0) {
				ImGui::SliderFloat("PtSz.", &point_size_, 1.0f, 50.0f, "%.1f");
			}
			if (primitive_ == 2) {
				ImGui::SliderFloat("Shrk.", &shrink_, 0.0f, 1.0f, "%.2f");
			}
			if (do_draw_micro)
			{
				if (primitive_ == 1) {
					ImGui::Checkbox("Alpha.", &balpha);
					//ImGui::SliderFloat("", &micro_alpha, 0.0f, 1.0f, "%.3f");
				}
			}
		}

		/**
		 * \brief Draws the scene according to currently set primitive and
		 *  drawing modes.
		 */
		void draw_scene() override {

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
				if (do_draw_vein)
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
				if (do_draw_artery)
				{
					glupSetColor4fv(GLUP_FRONT_COLOR, artery_colors);
					glupBegin(GLUP_POINTS);
					for (int i = 0; i <  all_artery_voxels[current_comboslice].size(); i++)
					{
						glupVertex3f(all_artery_voxels[current_comboslice][i].center[0],
							all_artery_voxels[current_comboslice][i].center[1],
							all_artery_voxels[current_comboslice][i].center[2]);
					}
					glupEnd();
				}
				if (do_draw_micro)
				{
					glupSetColor4fv(GLUP_FRONT_COLOR, micro_colors);
					glupBegin(GLUP_POINTS);
					for (int i = 0; i <  all_micro_voxels[current_comboslice].size(); i++)
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

				if (do_draw_vein)
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
				if (do_draw_artery)
				{
					glupSetColor4fv(GLUP_FRONT_COLOR, artery_colors);
					glupBegin(GLUP_TRIANGLES);
					for (int i = 0; i < artery_faces_size[current_comboslice];)
					{
						glupNormal3d(artery_faces[current_comboslice][i],
							artery_faces[current_comboslice][i+1], artery_faces[current_comboslice][i+2]);
						i += 3;
						glupVertex3d(artery_faces[current_comboslice][i],
							artery_faces[current_comboslice][i+1], artery_faces[current_comboslice][i+2]);
						i += 3;
					}
					glupEnd();
				}
				if (do_draw_micro)
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

				if (do_draw_vein)
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
				if (do_draw_artery)
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
				if (do_draw_micro)
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

		/**
		 * \brief Creates the texture.
		 * \details This function overloads Application::init_graphics(). It
		 *  is called as soon as the OpenGL context is ready for rendering. It
		 *  is meant to initialize the graphic objects used by the application.
		 */
		void GL_initialize() override {
			SimpleApplication::GL_initialize();
		}

		void update_expand_level()
		{

		}

	protected:

	

	private:
		bool mesh_;
		float point_size_;
		float shrink_;
		bool smooth_;
		int primitive_;

		bool balpha;
		float micro_alpha;

		bool do_draw_vein;
		bool do_draw_artery;
		bool do_draw_micro;

		vec4f vein_color_;
		float vein_colors[4];
		vec4f artery_color_;
		float artery_colors[4];
		vec4f micro_color_;
		float micro_colors[4];
		vec4f micro_backcolor_;
		float micro_backcolors[4];

		//int expand_level;
		int current_comboslice;

		double image_wid, image_hei, image_sli;

		CompoundLayers *layers;

		std::vector<std::vector<PixelVessel>> all_vein_voxels;
		std::vector<std::vector<PixelVessel>> all_artery_voxels;
		std::vector<std::vector<PixelVessel>> all_micro_voxels;

		float**			vein_faces;
		vector<int>		vein_faces_size;
		float**			artery_faces;
		vector<int>		artery_faces_size;
		float**			micro_faces;
		vector<int>		micro_faces_size;
	};
      
}

int main(int argc, char** argv) {
    DemoGlupApplication app;
    app.start(argc, argv);
    return 0;
}
