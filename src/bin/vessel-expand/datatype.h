#pragma once

#include <vector>
#include <list>
#include <set>
#include <string>
#include <assert.h>
#include <fstream>
#include <algorithm>
#include <io.h>

#include <QDir>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QRgb>
#include <QImage>

#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/boost/graph/properties_Polyhedron_3.h>
#include <boost/function_output_iterator.hpp>
#include <CGAL/Subdivision_method_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/boost/graph/selection.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#define IMAGEWIDTHSIZE 0.5
#define SCALEVOXEL 5.5
#define M_PI 3.1415926

#define EXPANDLEVEL 3

#define BComboSlice 1
#define SLICE_INTERNAL 11//21 or 11
#define NCOMOBO 19//6 or 19

#define SAVE_FILES 0

int width = 0;
int height = 0;
int slice = 0;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                                          Point_2;
typedef K::Point_3                                          Point_3;
typedef K::Vector_2                                         Vector_2;
typedef K::Vector_3                                         Vector_3;

typedef CGAL::Simple_cartesian<double>     Kernels;
typedef CGAL::Polyhedron_3<Kernels>        Mesh_sub;
typedef Mesh_sub::HalfedgeDS HD;

typedef CGAL::Surface_mesh<K::Point_3>     Meshu;
typedef boost::graph_traits<Meshu>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Meshu>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<Meshu>::face_descriptor face_descriptor;
typedef boost::graph_traits<Meshu>::halfedge_descriptor halfedge_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

struct halfedge2edge
{
	halfedge2edge(const Meshu& m, std::vector<edge_descriptor>& edges)
		: m_mesh(m), m_edges(edges)
	{}
	void operator()(const halfedge_descriptor& h) const
	{
		m_edges.push_back(edge(h, m_mesh));
	}
	const Meshu& m_mesh;
	std::vector<edge_descriptor>& m_edges;
};

template <class HDS>
class Builder_obj : public CGAL::Modifier_base<HDS>
{
private:
	typedef typename HDS::Vertex::Point Point;
	typedef typename CGAL::Polyhedron_incremental_builder_3<HDS> Builder;
	Point_3 *vertex;
	int **face;
	int vnum;
	int fnum;
	int fvnum;

public:
	Builder_obj(Point_3 *v, int **f, int vn, int fn, int fv)
	{
		vertex = v;
		face = f;
		vnum = vn;
		fnum = fn;
		fvnum = fv;
	}
	~Builder_obj() {}

	void operator()(HDS& hds)
	{
		Builder builder(hds, true);
		if (fvnum == 4)
			builder.begin_surface(4, 1, 8);
		else
			builder.begin_surface(3, 1, 6);
		read_vertices(builder);
		read_facets(builder);
		builder.end_surface();
	}

private:
	// read vertex coordinates
	void read_vertices(Builder &builder)
	{
		for (int i = 0; i < vnum; i++)
			builder.add_vertex(Point(vertex[i].x(), vertex[i].y(), vertex[i].z()));
	}

	// read facets and uv coordinates per halfedge
	void read_facets(Builder &builder)
	{
		// create facet
		for (int i = 0; i < fnum; i++)
		{
			builder.begin_facet();
			for (int j = 0; j < fvnum; j++)
				builder.add_vertex_to_facet(face[i][j]);
			builder.end_facet();
		}
	}
};



bool getFiles(std::string file_path, std::vector<std::string>& file_names)
{
	intptr_t hFile = 0;
	_finddata_t fileInfo;
	hFile = _findfirst(file_path.c_str(), &fileInfo);
	if (hFile != -1) {
		do {
			if ((fileInfo.attrib &  _A_SUBDIR)) {
				continue;
			}
			else {
				file_names.push_back(fileInfo.name);
				//cout << fileInfo.name << endl;
			}

		} while (_findnext(hFile, &fileInfo) == 0);

		_findclose(hFile);
	}
	else
		return false;

	return true;
}

bool sort_qstringpair_secondgreater(std::pair<QString, int> p1, std::pair<QString, int> p2)
{
	return p1.second < p2.second;
}

enum VesselType
{
	VEIN, ARTERY, MICRO
};

struct ExpandVessel 
{
	int vessel_type; //VesselType
	int index_;
	int x;
	int y;
	int z;
};

struct PixelVessel
{
	int x;//width
	int y;//height
	int z;//slice

	int index_;

	double center[3] = { 0,0,0 };
	bool xfacet = 1;//normal x plane;
	bool x_facet = 1;//normal -xplane;
	bool yfacet = 1;//normal y plane;
	bool y_facet = 1;//normal -y plane;
	bool zfacet = 1;//normal z plane;
	bool z_facet = 1;//normal -z plane;
	bool bvisited = false;

	int corners[8];//-x-y-z:1; x-y-z:2; xyz:7; 
	float corners_pts[24];//-x-y-z:1; x-y-z:2; xyz:7; 
};