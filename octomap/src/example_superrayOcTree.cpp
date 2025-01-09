#include <iostream>
#include <cstring>

#include <octomap/octomap_timing.h>
#include <superray_octomap/SuperRayOcTree.h>

void printUsage(char* self){
	std::cout << "USAGE: " << self << " [options]" << std::endl << std::endl;
	std::cout << "This tool inserts the data of a scan graph file (point clouds with poses)" << std::endl;
	std::cout << "into an octree using Super Ray based Updates method." << std::endl;
	std::cout << "The output is a compact maximum-likelihood binary octree file(.bt)" << std::endl;
	std::cout << "or a full occupancy representation file(.ot)." << std::endl;

	std::cout << "Options: " << std::endl;
	std::cout << " -i <InputFile.graph> (required)" << std::endl;
	std::cout << " -o <OutputFile.bt or OutputFile.ot> (required)" << std::endl;
	std::cout << " -res <resolution[m]> (optional, default 0.1m)" << std::endl;
	std::cout << " -thr <threshold> (optional, default 20)" << std::endl;

	exit(0);
}

int main(int argc, char** argv) {
	// default values
	double res = 0.1;
	int threshold = 20;
	std::string graphFilename = "";
	std::string treeFilename = "";

	timeval start;
	timeval stop;

	int arg = 0;
	while (++arg < argc){
		if (!strcmp(argv[arg], "-i"))
			graphFilename = std::string(argv[++arg]);
		else if (!strcmp(argv[arg], "-o"))
			treeFilename = std::string(argv[++arg]);
		else if (!strcmp(argv[arg], "-res") && argc - arg < 2)
			printUsage(argv[0]);
		else if (!strcmp(argv[arg], "-res"))
			res = atof(argv[++arg]);
		else if (!strcmp(argv[arg], "-thr"))
			threshold = atoi(argv[++arg]);
		else {
			printUsage(argv[0]);
		}
	}
	if (graphFilename == "" || treeFilename == "")
		printUsage(argv[0]);

	std::string file_extension = treeFilename.substr(treeFilename.length() - 3, treeFilename.length());
	if(file_extension != ".bt" && file_extension != ".ot")
		printUsage(argv[0]);

	// Verify input
	if (res <= 0.0){
		std::cout << "Resolution must be positive" << std::endl;
		exit(1);
	}
	if (threshold < 0){
		std::cout << "Threshold must be non negative" << std::endl;
		exit(1);
	}

	std::cout << "Reading Graph file ===========================" << std::endl;
	octomap::ScanGraph* graph = new octomap::ScanGraph();
	if (!graph->readBinary(graphFilename)){
		std::cout << "There is no graph file at " + graphFilename << std::endl;
		exit(1);
	}

	// transform pointclouds first, so we can directly operate on them later
	for (octomap::ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
		octomap::pose6d frame_origin = (*scan_it)->pose;
		octomap::point3d sensor_origin = frame_origin.inv().transform((*scan_it)->pose.trans());

		(*scan_it)->scan->transform(frame_origin);
		octomap::point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
		(*scan_it)->pose = octomap::pose6d(transformed_sensor_origin, octomath::Quaternion());
	}

	std::cout << "Creating tree ================================" << std::endl;
	octomap::SuperRayOcTree* tree = new octomap::SuperRayOcTree(res);

	double time_to_update = 0.0;	// sec
	size_t currentScan = 1;
	for (octomap::ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
		std::cout << "\r(" << currentScan << "/" << graph->size() << ")" << std::flush;

		// Generate Super Ray
		gettimeofday(&start, NULL);  // start timer
		tree->insertSuperRayCloudRays((*scan_it)->scan, (*scan_it)->pose.trans(), threshold);
		gettimeofday(&stop, NULL);  // stop timer
		time_to_update += (stop.tv_sec - start.tv_sec) + 1.0e-6 * (stop.tv_usec - start.tv_usec);

		currentScan++;
	}

	std::cout << "\rDone building tree: (" << (currentScan - 1) << "/" << graph->size() << ")" << std::endl;
	std::cout << "Time to insert scans: " << time_to_update << " [sec]" << std::endl;
	std::cout << "Time to insert 100K points took: " << time_to_update / ((double)graph->getNumPoints() / 100000) << " [sec] (avg)" << std::endl << std::endl;

    if(file_extension == ".bt") {
        tree->writeBinary(treeFilename);
    }
    else if(file_extension == ".ot") {
        tree->write(treeFilename);
    }

	delete graph;
	delete tree;

	return 0;
}
