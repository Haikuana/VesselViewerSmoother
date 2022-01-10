// This file is part of fTetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2019 Yixin Hu <yixin.hu@nyu.edu>
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
#ifdef USE_FTETWILD_REMESHING

#include <CLI/CLI.hpp>

#ifdef FLOAT_TETWILD_USE_TBB
#include <tbb/task_scheduler_init.h>
#include <thread>
#endif

#include <floattetwild/Mesh.hpp>
#include <floattetwild/MeshIO.hpp>
#include <floattetwild/FloatTetDelaunay.h>
#include <floattetwild/LocalOperations.h>
#include <floattetwild/MeshImprovement.h>
#include <floattetwild/Simplification.h>
#include <floattetwild/AABBWrapper.h>
#include <floattetwild/Statistics.h>
#include <floattetwild/TriangleInsertion.h>
#include <floattetwild/CSGTreeParser.hpp>

#include <floattetwild/Logger.hpp>
#include <Eigen/Dense>

#include <igl/Timer.h>

#ifdef LIBIGL_WITH_TETGEN
#include <igl/copyleft/tetgen/tetrahedralize.h>
#endif

#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>

#include <geogram/mesh/mesh.h>

#include<bitset>

#define UI_OUT_MESSAGES true

class GeoLoggerForward: public GEO::LoggerClient {
    std::shared_ptr<spdlog::logger> logger_;

public:
    template<typename T>
    GeoLoggerForward(T logger) : logger_(logger) {}

private:
    std::string truncate(const std::string &msg) {
        static size_t prefix_len = GEO::CmdLine::ui_feature(" ", UI_OUT_MESSAGES).size();
        return msg.substr(prefix_len, msg.size() - 1 - prefix_len);
    }

protected:
    void div(const std::string &title) override {
        logger_->trace(title.substr(0, title.size() - 1));
    }

    void out(const std::string &str) override {
        logger_->info(truncate(str));
    }

    void warn(const std::string &str) override {
        logger_->warn(truncate(str));
    }

    void err(const std::string &str) override {
        logger_->error(truncate(str));
    }

    void status(const std::string &str) override {
        // Errors and warnings are also dispatched as status by geogram, but without
        // the "feature" header. We thus forward them as trace, to avoid duplicated
        // logger info...
        logger_->trace(str.substr(0, str.size() - 1));
    }
};

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <geogram/basic/geometry.h>
#include <floattetwild/Predicates.hpp>
//#include "floattetwild/LocalOperations.h"

void connect_2_meshes(std::string m1, std::string m2, std::string m);

//extern "C" void exactinit();
#include "load_vessel.h"

int FtetWild(SimpleMesh &mesh_, double ideal_edge_rel = 0.01,double grid_ = 1e-3);

#include <igl/readSTL.h>
#include <igl/writeSTL.h>
#include <igl/writeOFF.h>

#endif