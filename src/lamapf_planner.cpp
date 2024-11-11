#include <cstdio>
#include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "common_interfaces.h"


#include <gtest/gtest.h>
#include "common_interfaces.h"
#include "LA-MAPF/algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "LA-MAPF/algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"
#include "LA-MAPF/algorithm/LA-MAPF/CBS/large_agent_CBS.h"

//#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

void layeredLargeAgentMAPFTest(const std::string& file_path) {
    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << "map scale = " << dim[0] << "*" << dim[1] << std::endl;

    auto start_t = clock();

    LargeAgentMAPFInstanceDecompositionPtr<2> decomposer_ptr = nullptr;
    std::vector<std::vector<int> > grid_visit_count_table;

    auto instances = deserializer.getTestInstance({40}, 1);

    auto layered_paths = layeredLargeAgentMAPF<2>(instances.front().second,
                                                  instances.front().first,
                                                  dim, is_occupied,
                                                  CBS::LargeAgentCBS_func<2>,
                                                  grid_visit_count_table,
                                                  60, decomposer_ptr,
                                                  false);

    auto end_t = clock();

    double time_cost = ((double)end_t-start_t)/CLOCKS_PER_SEC;

    std::cout << (layered_paths.size() == instances.front().first.size() ? "success" : "failed")
              << " layered large agent mapf in " << time_cost << "s " << std::endl;
    std::cout << std::endl;

//    gettimeofday(&tv_pre, &tz);
//    CBS::LargeAgentCBS<2, CircleAgent<2> > solver(deserializer.getInstances(), deserializer.getAgents(),
//                                                  dim, is_occupied);
//    gettimeofday(&tv_after, &tz);
//    double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
//    std::vector<LAMAPF_Path> raw_path;
//    if(solver.solve(60)) {
//        raw_path = solver.getSolution();
//    }
//    std::cout << (raw_path.size() == deserializer.getAgents().size() ? "success" : "failed")
//              << " raw large agent mapf in " << time_cost1 << "ms " << std::endl;

    InstanceVisualization(instances.front().first, decomposer_ptr->getAllPoses(),
                          instances.front().second, layered_paths, grid_visit_count_table);
}

int main(int argc, char ** argv) {
    layeredLargeAgentMAPFTest(map_test_config.at("la_ins_path"));
    return 0;
}