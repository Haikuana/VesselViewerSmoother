/*
 *  Copyright (c) 2012-2016, Bruno Levy
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

#include <geogram_gfx/gui/simple_mesh_application.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/basic/file_system.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/stopwatch.h>

namespace GEO {

    /**********************************************************************/

    SimpleMeshApplication::SimpleMeshApplication(
	const std::string& name
    ) : SimpleApplication(name) {
        std::vector<std::string> extensions;
        GEO::MeshIOHandlerFactory::list_creators(extensions);
        file_extensions_ = String::join_strings(extensions, ';');

		set_default_filename("out.meshb");

        anim_speed_ = 1.0f;
        anim_time_ = 0.0f;

		current_bc_id = 2;
		bcs.resize(8);
		//current directory

		const std::string directory_ = FileSystem::get_current_working_directory();
		std::vector<std::string> out_;
		String::split_string(directory_, '\\', out_);
		out_.push_back("Input");
		directory_new = String::join_strings(out_, "/");

        show_vertices_ = false;
        show_vertices_selection_ = true;
        vertices_size_ = 1.0f;
		vertices_color_ = vec4f(0.0f, 1.0f, 0.0f, 1.0f);
	
        show_surface_ = true;
		smooth_ = false;
        show_surface_sides_ = false;
        show_mesh_ = true;
		mesh_color_ = vec4f(0.0f, 0.0f, 0.0f, 1.0f);
		mesh_width_ = 0.1f;
	
        show_surface_borders_ = false;
		surface_color_ =   vec4f(0.5f, 0.5f, 1.0f, 1.0f);
		surface_color_2_ = vec4f(1.0f, 0.5f, 0.0f, 1.0f); 
	
        show_volume_ = false;
		volume_color_ = vec4f(0.9f, 0.9f, 0.9f, 1.0f);	
        cells_shrink_ = 0.0f;
        show_colored_cells_ = false;
        show_hexes_ = true;
		show_connectors_ = true;
		
		btensor_data = false;
		show_tensor_hex_ = false;
		tensor_color_ = vec4f(1.0f, 0.5f, 0.0f, 1.0f);
		tensor_stress_map_ = true;
		tensor_ellipsoid_ = false;
		tensor_mesh_or_smooth = false;

        show_attributes_ = false;
        current_colormap_texture_ = 0;
        attribute_min_ = 0.0f;
        attribute_max_ = 0.0f;
        attribute_ = "vertices.point_fp32[0]";
        attribute_name_ = "point_fp32[0]";
        attribute_subelements_ = MESH_VERTICES;

        add_key_toggle("p", &show_vertices_, "vertices");
        add_key_toggle("S", &show_surface_, "surface");
        add_key_toggle("c", &show_surface_sides_, "two-sided");
        add_key_toggle("B", &show_surface_borders_, "borders");
        add_key_toggle("m", &show_mesh_, "mesh");
        add_key_toggle("V", &show_volume_, "volume");
        add_key_toggle("j", &show_hexes_, "hexes");
        add_key_toggle("k", &show_connectors_, "connectors");
        add_key_toggle("C", &show_colored_cells_, "colored cells");

        add_key_func("r", decrement_anim_time_callback, "- anim speed");
        add_key_func("t", increment_anim_time_callback, "+ anim speed");
        add_key_func("x", decrement_cells_shrink_callback, "- cells shrink");
        add_key_func("w", increment_cells_shrink_callback, "+ cells shrink");
    }

    void SimpleMeshApplication::geogram_initialize(int argc, char** argv) {
	GEO::initialize();
        GEO::CmdLine::declare_arg(
            "attributes", true, "load mesh attributes"
        );

        GEO::CmdLine::declare_arg(
            "single_precision", true, "use single precision vertices (FP32)"
        );
	SimpleApplication::geogram_initialize(argc, argv);
    }
    
    std::string SimpleMeshApplication::supported_read_file_extensions() {
        return file_extensions_;
    }
    
    std::string SimpleMeshApplication::supported_write_file_extensions() {
        return file_extensions_;
    }

    void SimpleMeshApplication::autorange() {
        if(attribute_subelements_ != MESH_NONE) {
            attribute_min_ = 0.0;
            attribute_max_ = 0.0;
            const MeshSubElementsStore& subelements =
                mesh_.get_subelements_by_type(attribute_subelements_);
            ReadOnlyScalarAttributeAdapter attribute(
                subelements.attributes(), attribute_name_
            );
            if(attribute.is_bound()) {
                attribute_min_ = Numeric::max_float32();
                attribute_max_ = Numeric::min_float32();
                for(index_t i=0; i<subelements.nb(); ++i) {
                    attribute_min_ =
                        std::min(attribute_min_, float(attribute[i]));
                    attribute_max_ =
                        std::max(attribute_max_, float(attribute[i]));
                }
            } 
        }
    }

    std::string SimpleMeshApplication::attribute_names() {
        return mesh_.get_scalar_attributes();
    }

    void SimpleMeshApplication::set_attribute(const std::string& attribute) {
        attribute_ = attribute;
        std::string subelements_name;
        String::split_string(
            attribute_, '.',
            subelements_name,
            attribute_name_
        );
        attribute_subelements_ =
            mesh_.name_to_subelements_type(subelements_name);
        if(attribute_min_ == 0.0f && attribute_max_ == 0.0f) {
            autorange();
        } 
    }
    
	void SimpleMeshApplication::draw_object_properties() {
		SimpleApplication::draw_object_properties();
		float s = float(scaling());
		ImGui::Checkbox("attributes", &show_attributes_);
		if (show_attributes_) {
			if (attribute_min_ == 0.0f && attribute_max_ == 0.0f) {
				autorange();
			}
			if (ImGui::Button(
				(attribute_ + "##Attribute").c_str(), ImVec2(-1, 0))
				) {
				ImGui::OpenPopup("##Attributes");
			}
			if (ImGui::BeginPopup("##Attributes")) {
				std::vector<std::string> attributes;
				String::split_string(attribute_names(), ';', attributes);
				for (index_t i = 0; i < attributes.size(); ++i) {
					if (ImGui::Button(attributes[i].c_str())) {
						set_attribute(attributes[i]);
						autorange();
						ImGui::CloseCurrentPopup();
					}
				}
				ImGui::EndPopup();
			}
			ImGui::InputFloat("min", &attribute_min_);
			ImGui::InputFloat("max", &attribute_max_);
			if (ImGui::Button("autorange", ImVec2(-1, 0))) {
				autorange();
			}
			if (ImGui::ImageButton(
				convert_to_ImTextureID(current_colormap_texture_),
				ImVec2(115.0f*s, 8.0f*s))
				) {
				ImGui::OpenPopup("##Colormap");
			}
			if (ImGui::BeginPopup("##Colormap")) {
				for (index_t i = 0; i < colormaps_.size(); ++i) {
					if (ImGui::ImageButton(
						convert_to_ImTextureID(colormaps_[i].texture),
						ImVec2(100.0f*s, 8.0f*s))
						) {
						current_colormap_texture_ = colormaps_[i].texture;
						ImGui::CloseCurrentPopup();
					}
				}
				ImGui::EndPopup();
			}
		}

		if (mesh_.vertices.dimension() >= 6) {
			ImGui::Separator();
			ImGui::Checkbox(
				"Animate", animate_ptr()
			);
			ImGui::SliderFloat("spd.", &anim_speed_, 1.0f, 10.0f, "%.1f");
			ImGui::SliderFloat("t.", &anim_time_, 0.0f, 1.0f, "%.2f");
		}

		ImGui::Separator();
		ImGui::Checkbox("##VertOnOff", &show_vertices_);
		ImGui::SameLine();
		ImGui::ColorEdit3WithPalette("Vert.", vertices_color_.data());

		if (show_vertices_) {
			ImGui::Indent();
			ImGui::Checkbox("selection", &show_vertices_selection_);
			ImGui::SliderFloat("sz.", &vertices_size_, 0.1f, 5.0f, "%.1f");
			ImGui::Unindent();
		}

		if (mesh_.facets.nb() != 0) {
			ImGui::Separator();
			ImGui::Checkbox("##SurfOnOff", &show_surface_);
			ImGui::SameLine();
			ImGui::ColorEdit3WithPalette(
				"Surf.", surface_color_.data()
			);
			if (show_surface_) {
				ImGui::Indent();
				ImGui::Checkbox("##SidesOnOff", &show_surface_sides_);
				ImGui::SameLine();
				ImGui::ColorEdit3WithPalette(
					"2sided", surface_color_2_.data()
				);

				ImGui::Checkbox("##MeshOnOff", &show_mesh_);
				ImGui::SameLine();
				ImGui::ColorEdit3WithPalette("mesh", mesh_color_.data());

				if (show_mesh_) {
					ImGui::SliderFloat(
						"wid.", &mesh_width_, 0.1f, 2.0f, "%.1f"
					);
				}

				ImGui::Checkbox("smooth", &smooth_);
				ImGui::Checkbox("borders", &show_surface_borders_);
				ImGui::Unindent();
			}
			else
			{
				show_mesh_ = false;
			}
		}

		if (mesh_.cells.nb() != 0) {
			ImGui::Separator();
			ImGui::Checkbox("##VolumeOnOff", &show_volume_);
			ImGui::SameLine();
			ImGui::ColorEdit3WithPalette("Volume", volume_color_.data());

			if (show_volume_) {
				if (!mesh_.cells.are_simplices()) {
					ImGui::Checkbox("colored cells", &show_colored_cells_);
					ImGui::Checkbox("hexes", &show_hexes_);
				}
			}
		}

		if (btensor_data)
		{
			ImGui::Separator();
			ImGui::Checkbox("##TensorOnOff", &show_tensor_hex_);
			ImGui::SameLine();
			ImGui::ColorEdit3WithPalette("Tensor", tensor_color_.data());

			if (show_tensor_hex_) {
				ImGui::Indent();
				ImGui::Checkbox("ellipsoid", &tensor_ellipsoid_);
				ImGui::Checkbox("stress", &tensor_stress_map_);
				ImGui::Checkbox("mesh|smooth", &tensor_mesh_or_smooth);
				ImGui::Unindent();
			}
		}

		if (show_volume_ || show_tensor_hex_)
		{
			ImGui::Separator();
			ImGui::SliderFloat(
				"shrk.", &cells_shrink_, 0.0f, 1.0f, "%.2f"
			);
			ImGui::Separator();
		}
	}

	void SimpleMeshApplication::draw_stress_properties() {
		SimpleApplication::draw_stress_properties();

		//ImGui::NewLine();
		//ImGui::NewLine();
		float s = float(scaling());
		ImGui::PushItemWidth(100.0f*s);
		if (ImGui::TreeNode("Stress Computation"))
		{
			if (ImGui::TreeNode("Input Model"))
			{	
				if (ImGui::SimpleButton(icon_UTF8("folder-open") + "Load model"))
				{
					ImGui::FileDialog("load_model",
						model_filename_,
						geo_imgui_string_length,
						std::string("obj;stl").c_str(), directory_new);
				}		
				ImGui::PushItemWidth(300.0f*s);
				ImGui::InputText("path", model_filename_, geo_imgui_string_length);
				ImGui::PopItemWidth();
				ImGui::TreePop();
			}
			ImGui::Separator();
			if (ImGui::TreeNode("Boundary Conditions"))
			{
				for (int i= 0;i<current_bc_id;i++)
				{
					add_boundary_condition(i);
				}
				
				if(ImGui::SimpleButton(icon_UTF8("plus-circle") + "Add"))
				{
					if (current_bc_id<bcs.size())
					{
						current_bc_id++;
					}
				}
				ImGui::SameLine();
				if(ImGui::SimpleButton(icon_UTF8("minus-circle") + "Del"))
				{
					if (current_bc_id < bcs.size() &&
						current_bc_id>0)
					{
						current_bc_id--;
					}
				}
				ImGui::TreePop();
			}
			ImGui::Separator();
			if (ImGui::TreeNode("Tetrahedralize"))
			{
				ImGui::InputFloat("max element volume", &max_element_volume);
				ImGui::Checkbox("use stress info", &buse_stress_info);
				if (buse_stress_info)
				{
					ImGui::Indent();
					ImGui::InputFloat("min mesh size", &min_mesh_size);
					ImGui::InputFloat("max mesh size", &max_mesh_size);
					ImGui::Unindent();
				}		
				ImGui::TreePop();
			}
			ImGui::Separator();
			if (ImGui::TreeNode("Material Properties"))
			{
				ImGui::InputFloat("density", &density);
				ImGui::InputFloat("elastic modulus", &elastic_modulus);
				ImGui::InputFloat("poisson ratio", &poisson_ratio);
				ImGui::TreePop();
			}
			ImGui::Separator();
			if (ImGui::TreeNode("Environment Gravity"))
			{
				ImGui::InputDouble("x", &gravity[0]);
				ImGui::InputDouble("y", &gravity[1]);
				ImGui::InputDouble("z", &gravity[2]);
				ImGui::TreePop();
			}
			ImGui::Separator();
			if (ImGui::TreeNode("Save files"))
			{
				ImGui::InputText("tet file for Geogram", tet_file, geo_imgui_string_length);
				ImGui::InputText("mytet file for tensor", mytet_file, geo_imgui_string_length);
				ImGui::InputText("rbm file for Range", rbm_file, geo_imgui_string_length);
				ImGui::TreePop();
			}
			ImGui::Separator();
			if (ImGui::SimpleButton(icon_UTF8("play-circle") + "Start Solve"))
			{
				std::cout << "exec" << std::endl;
			}
			ImGui::SameLine();
			if (ImGui::SimpleButton(icon_UTF8("redo-alt") + "Restart Solve"))
			{
				std::cout << "exec1" << std::endl;
			}
			//ImGui::SameLine();
			//ImGui::SimpleButton();

			ImGui::TreePop();
		}
		ImGui::PopItemWidth();
		//ImGui::TreeNodeEx("input model");

	}

	void SimpleMeshApplication::add_boundary_condition(int id_)
	{
		float s = float(scaling());
		std::string id = "bc"+std::to_string(id_);
		if (ImGui::Button((id+" type: "+bcs[id_].type).c_str(), ImVec2(400.0f*s, 0.0))
			) {
			ImGui::OpenPopup(("type-"+id).c_str());
		}
		if (ImGui::BeginPopup(("type-"+id).c_str())) {
			std::vector<std::string> types_ = 
			{"displacement","force","force unit area","weight"};
			for (index_t i = 0; i < types_.size(); ++i) {
				if (ImGui::Button(types_[i].c_str())) {
					bcs[id_].type = types_[i];
					ImGui::CloseCurrentPopup();
				}
			}
			ImGui::EndPopup();
		}
		if (bcs[id_].type == "weight")
		{
			ImGui::InputFloat(("value-" + id).c_str(), &bcs[id_].scalar_data);
		}
		else
		{
			ImGui::InputFloat(("x-" + id).c_str(), &bcs[id_].vecdata[0]);
			ImGui::InputFloat(("y-" + id).c_str(), &bcs[id_].vecdata[1]);
			ImGui::InputFloat(("z-" + id).c_str(), &bcs[id_].vecdata[2]);
		}
		if (ImGui::SimpleButton(icon_UTF8("folder-open") + "Load faces " + id))
		{
			ImGui::FileDialog(("load_faces_list " + id).c_str(),
				bcs[id_].faces_file,
				geo_imgui_string_length, 
				std::string("obj;stl").c_str(), directory_new);
		}
		ImGui::PushItemWidth(300.0f*s);
		ImGui::InputText(("path-" + id).c_str(), bcs[id_].faces_file, geo_imgui_string_length);
		ImGui::PopItemWidth();
	}

    void SimpleMeshApplication::increment_anim_time_callback() {
        instance()->anim_time_ = std::min(
            instance()->anim_time_ + 0.05f, 1.0f
        );
    }
        
    void SimpleMeshApplication::decrement_anim_time_callback() {
        instance()->anim_time_ = std::max(
            instance()->anim_time_ - 0.05f, 0.0f
        );
    }

    void SimpleMeshApplication::increment_cells_shrink_callback() {
        instance()->cells_shrink_ = std::min(
            instance()->cells_shrink_ + 0.05f, 1.0f
        );
    }
        
    void SimpleMeshApplication::decrement_cells_shrink_callback() {
        instance()->cells_shrink_ = std::max(
            instance()->cells_shrink_ - 0.05f, 0.0f
        );
    }

    void SimpleMeshApplication::GL_initialize() {
        SimpleApplication::GL_initialize();
        current_colormap_texture_ = colormaps_[3].texture;
    }
    
	void SimpleMeshApplication::draw_scene() {

		if (mesh_gfx_.mesh() == nullptr) {
			return;
		}

		if (animate()) {
			anim_time_ = float(
				sin(double(anim_speed_) * GEO::SystemStopwatch::now())
				);
			anim_time_ = 0.5f * (anim_time_ + 1.0f);
		}

		mesh_gfx_.set_lighting(lighting_);
		mesh_gfx_.set_time(double(anim_time_));

		if (show_attributes_) {
			mesh_gfx_.set_scalar_attribute(
				attribute_subelements_, attribute_name_,
				double(attribute_min_), double(attribute_max_),
				current_colormap_texture_, 1
			);
		}
		else {
			mesh_gfx_.unset_scalar_attribute();
		}

		if (show_vertices_) {
			mesh_gfx_.set_points_color(
				vertices_color_.x, vertices_color_.y, vertices_color_.z
			);
			mesh_gfx_.set_points_size(vertices_size_);
			mesh_gfx_.draw_vertices();
		}

		if (show_vertices_selection_) {
			mesh_gfx_.set_points_color(1.0, 0.0, 0.0);
			mesh_gfx_.set_points_size(2.0f * vertices_size_);
			mesh_gfx_.set_vertices_selection("selection");
			mesh_gfx_.draw_vertices();
			mesh_gfx_.set_vertices_selection("");
		}

		mesh_gfx_.set_mesh_color(0.0, 0.0, 0.0);

		mesh_gfx_.set_surface_color(
			surface_color_.x, surface_color_.y, surface_color_.z
		);

		if (show_surface_sides_) {
			mesh_gfx_.set_backface_surface_color(
				surface_color_2_.x, surface_color_2_.y, surface_color_2_.z
			);
		}

		mesh_gfx_.set_show_mesh(show_mesh_);
		mesh_gfx_.set_mesh_color(mesh_color_.x, mesh_color_.y, mesh_color_.z);
		mesh_gfx_.set_mesh_width(index_t(mesh_width_*10.0f));

		if (show_surface_) {			

			float specular_backup = glupGetSpecular();
			glupSetSpecular(0.4f);
			mesh_gfx_.draw_surface();
			glupSetSpecular(specular_backup);
		}

		if (show_surface_borders_) {
			mesh_gfx_.draw_surface_borders();
		}

		if (show_mesh_) {
			mesh_gfx_.draw_edges();
		}

		if (show_volume_) {

			if (
				glupIsEnabled(GLUP_CLIPPING) &&
				glupGetClipMode() == GLUP_CLIP_SLICE_CELLS
				) {
				mesh_gfx_.set_lighting(false);
			}

			mesh_gfx_.set_shrink(double(cells_shrink_));
			mesh_gfx_.set_draw_cells(GEO::MESH_HEX, show_hexes_);
			mesh_gfx_.set_draw_cells(GEO::MESH_CONNECTOR, show_connectors_);

			if (show_colored_cells_) {
				mesh_gfx_.set_cells_colors_by_type();
			}
			else {
				mesh_gfx_.set_cells_color(
					volume_color_.x, volume_color_.y, volume_color_.z
				);
			}
			mesh_gfx_.draw_volume();

			mesh_gfx_.set_lighting(lighting_);
		}

		if (show_tensor_hex_)
		{
			if (tensor_ellipsoid_ && tensor_mesh_or_smooth) {
				glupEnable(GLUP_VERTEX_NORMALS);
			}
			else {
				glupDisable(GLUP_VERTEX_NORMALS);
			}

			// Enable/disable individual per-vertex colors.
			glupEnable(GLUP_VERTEX_COLORS);
			if (tensor_stress_map_) {			
				mesh_gfx_.set_use_stress_map(true);
			}
			else
			{
				mesh_gfx_.set_tensor_color(
					tensor_color_.x, tensor_color_.y, tensor_color_.z
				);
				mesh_gfx_.set_use_stress_map(false);
			}
				
			if (lighting_) {
				glupEnable(GLUP_LIGHTING);
			}
			else {
				glupDisable(GLUP_LIGHTING);
			}
			// Each facet can have a black outline displayed.
			if (tensor_mesh_or_smooth) {
				glupEnable(GLUP_DRAW_MESH);
			}
			else {
				glupDisable(GLUP_DRAW_MESH);
			}

			glupSetCellsShrink(cells_shrink_);

			//draw tensor hex
			mesh_gfx_.draw_tensor(tensor_ellipsoid_);

			glupDisable(GLUP_VERTEX_COLORS);
			glupDisable(GLUP_DRAW_MESH);
			glupDisable(GLUP_VERTEX_NORMALS);
		}
	}


    bool SimpleMeshApplication::load(const std::string& filename) {
        if(!FileSystem::is_file(filename)) {
            Logger::out("I/O") << "is not a file" << std::endl;
        }
        mesh_gfx_.set_mesh(nullptr);

        mesh_.clear(false,false);
        
        if(GEO::CmdLine::get_arg_bool("single_precision")) {
            mesh_.vertices.set_single_precision();
        }
        
        MeshIOFlags flags;
        if(CmdLine::get_arg_bool("attributes")) {
            flags.set_attribute(MESH_FACET_REGION);
            flags.set_attribute(MESH_CELL_REGION);            
        } 
        if(!mesh_load(filename, mesh_, flags)) {
            return false;
        }
		btensor_data = flags.data_type() > 0 ? true : false;

        if(
            FileSystem::extension(filename) == "obj6" ||
            FileSystem::extension(filename) == "tet6"
        ) {
            Logger::out("Vorpaview")
                << "Displaying mesh animation." << std::endl;

	    start_animation();
            
            mesh_gfx_.set_animate(true);
            double xyzmin[3];
            double xyzmax[3];
            get_bbox(mesh_, xyzmin, xyzmax, true);
            set_region_of_interest(
                xyzmin[0], xyzmin[1], xyzmin[2],
                xyzmax[0], xyzmax[1], xyzmax[2]
            );
        } else {
            mesh_gfx_.set_animate(false);            
            mesh_.vertices.set_dimension(3);
            double xyzmin[3];
            double xyzmax[3];
            get_bbox(mesh_, xyzmin, xyzmax, false);
            set_region_of_interest(
                xyzmin[0], xyzmin[1], xyzmin[2],
                xyzmax[0], xyzmax[1], xyzmax[2]
            );
        }

        show_vertices_ = (mesh_.facets.nb() == 0);
        mesh_gfx_.set_mesh(&mesh_);
		if (flags.data_type()>0)
		{
			mesh_gfx_.set_tensor_sparse_hexs();
			mesh_gfx_.set_tensor_ellipsoids();
		}	

	current_file_ = filename;
        return true;
    }

    bool SimpleMeshApplication::save(const std::string& filename) {
        MeshIOFlags flags;
        if(CmdLine::get_arg_bool("attributes")) {
            flags.set_attribute(MESH_FACET_REGION);
            flags.set_attribute(MESH_CELL_REGION);            
        }
	if(FileSystem::extension(filename) == "geogram") {
	    mesh_.vertices.set_double_precision();
	}
	bool result = true;
	if(mesh_save(mesh_, filename, flags)) {
	    current_file_ = filename;
	} else {
	    result = false;
	}
        if(GEO::CmdLine::get_arg_bool("single_precision")) {	
	    mesh_.vertices.set_single_precision();
	}
	return result;
    }
    
    void SimpleMeshApplication::get_bbox(
        const Mesh& M_in, double* xyzmin, double* xyzmax, bool animate
    ) {
        geo_assert(M_in.vertices.dimension() >= index_t(animate ? 6 : 3));
        for(index_t c = 0; c < 3; c++) {
            xyzmin[c] = Numeric::max_float64();
            xyzmax[c] = Numeric::min_float64();
        }

        for(index_t v = 0; v < M_in.vertices.nb(); ++v) {
            if(M_in.vertices.single_precision()) {
                const float* p = M_in.vertices.single_precision_point_ptr(v);
                for(coord_index_t c = 0; c < 3; ++c) {
                    xyzmin[c] = std::min(xyzmin[c], double(p[c]));
                    xyzmax[c] = std::max(xyzmax[c], double(p[c]));
                    if(animate) {
                        xyzmin[c] = std::min(xyzmin[c], double(p[c+3]));
                        xyzmax[c] = std::max(xyzmax[c], double(p[c+3]));
                    }
                }
            } else {
                const double* p = M_in.vertices.point_ptr(v);
                for(coord_index_t c = 0; c < 3; ++c) {
                    xyzmin[c] = std::min(xyzmin[c], p[c]);
                    xyzmax[c] = std::max(xyzmax[c], p[c]);
                    if(animate) {
                        xyzmin[c] = std::min(xyzmin[c], p[c+3]);
                        xyzmax[c] = std::max(xyzmax[c], p[c+3]);
                    }
                }
            }
        }
    }
}
