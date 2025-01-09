#include <iostream>
#include <cstring>
#include <cstdlib>

#include <octomap/ScanGraph.h>
#include <superray_quadmap/ScanGraph.h>

void printUsage(char* self){
    std::cout << "USAGE: " << self << " [options]" << std::endl << std::endl;
    std::cout << "This tool converts a 3-D scan graph into a 2-D version." << std::endl;

    std::cout << "Options: " << std::endl;
    std::cout << " -i <InputFile.graph> (required)" << std::endl;
    std::cout << " -o <OutputFile.graph2d> (required)" << std::endl;
    std::cout << " -h <height[m]> (optional, default: height of origin)" << std::endl;
    std::cout << " -l <interval[m] (optional, default: 0.05m" << std::endl;

    exit(0);
}

int main(int argc, char** argv) {
    // default values
    bool use_origin_height = true;
    double height = 1.0;
    double interval = 0.05;
    std::string graph3DFilename = "";
    std::string graph2DFilename = "";

    int arg = 0;
    while (++arg < argc){
        if (!strcmp(argv[arg], "-i"))
            graph3DFilename = std::string(argv[++arg]);
        else if (!strcmp(argv[arg], "-o"))
            graph2DFilename = std::string(argv[++arg]);
        else if (!strcmp(argv[arg], "-h")){
            height = atof(argv[++arg]);
            use_origin_height = false;
        }
        else if (!strcmp(argv[arg], "-l"))
            interval = atof(argv[++arg]);
        else
            printUsage(argv[0]);
    }

    // Verify the input
    if (graph3DFilename == "" || graph2DFilename == "")
        printUsage(argv[0]);
    if (interval <= 0.0)
        printUsage(argv[0]);

    std::cout << "Converting the scan graph file (3D --> 2D) ===========================" << std::endl;;
    octomap::ScanGraph* graph3d = new octomap::ScanGraph();
    quadmap::ScanGraph* graph2d = new quadmap::ScanGraph();

    if (!graph3d->readBinary(graph3DFilename)){
        std::cout << "There is no graph file at " + graph3DFilename << std::endl;
        exit(1);
    }

    // Convert the 3-D scan nodes into the 2-D nodes
    std::cout << "Converting scan nodes" << std::endl;
    for (octomap::ScanGraph::iterator scan_it = graph3d->begin(); scan_it != graph3d->end(); scan_it++) {
        // Read a node of 3-D scan graph
        const octomap::pose6d frame_origin = (*scan_it)->pose;
        const octomap::Pointcloud* scan = (*scan_it)->scan;

        // Create a node of 2-D scan graph
        quadmap::pose3d frame_origin_2d(frame_origin.x(), frame_origin.y(), frame_origin.yaw());
        double scan_height = height;
        if(use_origin_height)
            scan_height = frame_origin.z();

        // Filter out the points out of the interval
        quadmap::Pointcloud* scan_2d = new quadmap::Pointcloud();
        for(unsigned int i = 0; i < scan->size(); i++){
            if((*scan)[i].z() < scan_height - interval || (*scan)[i].z() > scan_height + interval)
                continue;
            scan_2d->push_back((*scan)[i].x(), (*scan)[i].y());
        }

        // Add the new node into the 2-D scan graph
        graph2d->addNode(scan_2d, frame_origin_2d);

        std::cout << "(" << graph2d->size() << "/" << graph3d->size() << ") " << std::endl;
    }

    // Generate edges of 2-D scan graph
    size_t graph3d_edge_size = std::distance(graph3d->edges_begin(), graph3d->edges_end());
    std::cout << "Generating edges of 2-D scan graph" << std::endl;
    for(octomap::ScanGraph::edge_iterator edge_it = graph3d->edges_begin(); edge_it != graph3d->edges_end(); edge_it++) {
        graph2d->addEdge((*edge_it)->first->id, (*edge_it)->second->id);
        std::cout << "(" << graph2d->edge_size() << " / " << graph3d_edge_size << ")" << std::endl;
    }

    graph2d->writeBinary(graph2DFilename);

    delete graph3d;
    delete graph2d;

    return 0;
}
