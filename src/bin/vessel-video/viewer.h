#pragma once

#include <geogram_gfx/gui/simple_application.h>
#include <geogram_gfx/GLUP/GLUP_private.h>
#include "compound_layers.h"

namespace GEO {

	class VesselViewer : public SimpleApplication {
	public:

		/**
		 * \brief DemoGlupApplication constructor.
		 */
		VesselViewer();

		~VesselViewer();

	private:
		void load_vessel(int type, std::string path_);

		/**
		 * \copydoc SimpleApplication::GL_terminate()
		 */
		void GL_terminate() override;

		/**
		 * \brief Displays and handles the GUI for object properties.
		 * \details Overloads Application::draw_object_properties().
		 */
		void draw_object_properties() override;

		/**
		 * \brief Draws the scene according to currently set primitive and
		 *  drawing modes.
		 */
		void draw_scene() override;
		void draw_menu_bar() override;

		/**
		 * \brief Creates the texture.
		 * \details This function overloads Application::init_graphics(). It
		 *  is called as soon as the OpenGL context is ready for rendering. It
		 *  is meant to initialize the graphic objects used by the application.
		 */
		void GL_initialize() override;

		void update_expand_level();

	protected:
		static void decrement_comboId_callback();
		static void increment_comboId_callback();


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
		static int current_comboslice;

		double image_wid, image_hei, image_sli;

		GPara			paras;
		CompoundLayers* layers;

		std::vector<std::vector<PixelVessel>> all_vein_voxels;
		std::vector<std::vector<PixelVessel>> all_artery_voxels;
		std::vector<std::vector<PixelVessel>> all_micro_voxels;

		float** vein_faces;
		vector<int>		vein_faces_size;
		float** artery_faces;
		vector<int>		artery_faces_size;
		float** micro_faces;
		vector<int>		micro_faces_size;

		char input_configure_path[geo_imgui_string_length] = "";
		std::string directory_model;
		int			load_type = -1;
	};

}
