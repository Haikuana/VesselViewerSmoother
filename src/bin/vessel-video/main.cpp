#pragma once
#include "viewer.h"

int main(int argc, char** argv) {

	GEO::initialize();
	GEO::CmdLine::import_arg_group("standard");

    GEO::VesselViewer app;
    app.start(argc, argv);
    return 0;
}
