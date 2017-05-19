#include <iostream>
#include <cstring>

#include <gridmap2D/gridmap2D_timing.h>
#include <gridmap2D_superray/SuperRayGrid2D.h>

// For experiments
// #include <../../gridmap3D/include/gridmap3D/Grid3D.h>
// #include <../../gridmap3D/include/gridmap3D/math/Quaternion.h>

void printUsage(char* self){
	std::cout << "USAGE: " << self << " [options]" << std::endl << std::endl;
	std::cout << "This tool inserts the data of a scan graph file (point clouds with poses)" << std::endl;
	std::cout << "into an octree using Super Ray based Updates method." << std::endl;
	std::cout << "The output is a compact maximum-likelihood binary octree file(.bt, bonsai tree)." << std::endl;

	std::cout << "Options: " << std::endl;
	std::cout << " -i <InputFile.graph> (required)" << std::endl;
	std::cout << " -o <OutputFile.bt> (required)" << std::endl;
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

	// Verify input
	if (res <= 0.0){
		std::cout << "Resolution must be positive" << std::endl;
		exit(1);
	}
	if (threshold < 0){
		std::cout << "Threshold must be non negative" << std::endl;
		exit(1);
	}

	std::cout << "Reading Graph file===========================" << std::endl;
	gridmap2D::ScanGraph* graph = new gridmap2D::ScanGraph();
	if (!graph->readBinary(graphFilename)){
		std::cout << "There is no graph file at " + graphFilename << std::endl;
		exit(1);
	}

	// transform pointclouds first, so we can directly operate on them later
	for (gridmap2D::ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
		gridmap2D::pose3d frame_origin = (*scan_it)->pose;
		gridmap2D::point2d sensor_origin = frame_origin.inv().transform((*scan_it)->pose.trans());

		(*scan_it)->scan->transform(frame_origin);
		gridmap2D::point2d transformed_sensor_origin = frame_origin.transform(sensor_origin);
		(*scan_it)->pose = gridmap2D::pose3d(transformed_sensor_origin, 0);
	}

	std::cout << "\nCreating tree\n===========================\n";
	gridmap2D::SuperRayGrid2D* grid = new gridmap2D::SuperRayGrid2D(res);

	double time_to_update = 0.0;	// sec
	size_t currentScan = 1;
	for (gridmap2D::ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
		std::cout << "(" << currentScan << "/" << graph->size() << ") " << std::endl;

		// Generate Super Ray
		gettimeofday(&start, NULL);  // start timer
		grid->insertSuperRayCloudRays((*scan_it)->scan, (*scan_it)->pose.trans(), threshold);
		gettimeofday(&stop, NULL);  // stop timer
		time_to_update += (stop.tv_sec - start.tv_sec) + 1.0e-6 * (stop.tv_usec - start.tv_usec);;

		currentScan++;
	}

	std::cout << "Done building tree." << std::endl;
	std::cout << "Time to insert scans (SR): " << time_to_update << " [sec]" << std::endl;
	std::cout << "Time to insert 100.000 points took (SR): " << time_to_update / ((double)graph->getNumPoints() / 100000) << " [sec] (avg)" << std::endl << std::endl;

//	grid->writeBinary(treeFilename);

	delete graph;
	delete grid;

	return 0;

/*	gridmap3D::ScanGraph* graph3D = new gridmap3D::ScanGraph();
	if (!graph3D->readBinary(graphFilename)){
		std::cout << "There is no graph file at " + graphFilename << std::endl;
		exit(1);
	}
	for (gridmap3D::ScanGraph::iterator scan_it = graph3D->begin(); scan_it != graph3D->end(); scan_it++) {
		gridmap3D::pose6d frame_origin = (*scan_it)->pose;
		gridmap3D::point3d sensor_origin = frame_origin.inv().transform((*scan_it)->pose.trans());

		(*scan_it)->scan->transform(frame_origin);
		gridmap3D::point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
		(*scan_it)->pose = gridmap3D::pose6d(transformed_sensor_origin, gridmath3D::Quaternion());
	}

	std::cout << "\nCreating tree\n===========================\n";
	gridmap2D::SuperRayGrid2D* grid = new gridmap2D::SuperRayGrid2D(res);

	double time_to_update = 0.0;	// sec
	size_t currentScan = 1;
	for (gridmap3D::ScanGraph::iterator scan_it = graph3D->begin(); scan_it != graph3D->end(); scan_it++) {
		std::cout << "(" << currentScan << "/" << graph3D->size() << ") " << std::endl;

		gridmap2D::Pointcloud pc2D;
		gridmap3D::Pointcloud& pc3D = *(*scan_it)->scan;
		for (unsigned int i = 0; i < pc3D.size(); i++){
			pc2D.push_back(pc3D[i].x(), pc3D[i].y());
		}
		gridmap2D::point2d origin((*scan_it)->pose.trans().x(), (*scan_it)->pose.trans().y());

		// Generate Super Ray
		gettimeofday(&start, NULL);  // start timer
		grid->insertSuperRayCloudRays(pc2D, origin, threshold);
		gettimeofday(&stop, NULL);  // stop timer
		time_to_update += (stop.tv_sec - start.tv_sec) + 1.0e-6 * (stop.tv_usec - start.tv_usec);;

		currentScan++;
	}

	std::cout << "Done building tree." << std::endl;
	std::cout << "Time to insert scans (SR): " << time_to_update << " [sec]" << std::endl;
	std::cout << "Time to insert 100.000 points took (SR): " << time_to_update / ((double)graph3D->getNumPoints() / 100000) << " [sec] (avg)" << std::endl << std::endl;

	//	grid->writeBinary(treeFilename);

	delete graph3D;
	delete grid;

	return 0;*/
}
