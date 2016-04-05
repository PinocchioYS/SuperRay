#include <iostream>
#include <fstream>

#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>

void printUsage(char* self){
	std::cout << "USAGE: " << self << " [options]" << std::endl << std::endl;
	std::cout << "This tool inserts the data of a scan graph file (point clouds with poses)" << std::endl;
	std::cout << "into an octree using Super Ray based Updates method." << std::endl;
	std::cout << "The output is a compact maximum-likelihood binary octree file(.bt, bonsai tree)." << std::endl;

	std::cout << "Options: " << std::endl;
	std::cout << " -i <InputFile.graph> (required)" << std::endl;
	std::cout << " -o <OutputFile.bt> (required)" << std::endl;
	std::cout << " -res <resolution[m]> (optional, default 0.1m)" << std::endl;
	std::cout << " -log (enable a detailed log file)" << std::endl;
	std::cout << " -ben (enable benchmarking with octree without super rays)" << std::endl;

	exit(0);
}

double compareOccupancy(octomap::OcTree* _base, octomap::OcTree* _compared){
	int numNodes = 0;
	double difference = 0.0;
	for (octomap::OcTree::leaf_iterator it = _base->begin_leafs(), end = _base->end_leafs(); it != end; it++){
		octomap::OcTreeNode* node = _compared->search(it.getKey());
		if (!node){
			difference += 1.0;
		}
		else{
			double baseOccupancy = it->getOccupancy();
			double comparedOccupancy = node->getOccupancy();
			double diff = abs(baseOccupancy - comparedOccupancy);

			difference += (diff * diff);

		}
		numNodes++;
	}
	difference = sqrt(difference / numNodes);

	return difference;
}

int main(int argc, char** argv) {
	// default values
	double res = 0.1;
	std::string graphFilename = "";
	std::string treeFilename = "";
	bool detailedLog = false;
	bool benchmarking = false;

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
		else if (!strcmp(argv[arg], "-log"))
			detailedLog = true;
		else if (!strcmp(argv[arg], "-ben"))
			benchmarking = true;
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

	std::cout << "Reading Graph file===========================" << std::endl;;
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

	std::ofstream logfile;
	if (detailedLog){
		logfile.open((treeFilename + ".log").c_str());
		logfile << "Speed of update " << graphFilename << " over time" << std::endl;
		logfile << "Resolution: " << res << std::endl;
		logfile << "Scan endpoints: " << graph->getNumPoints() << std::endl;
		logfile << "Benchmark: ";
		if (benchmarking)
			logfile << "true" << std::endl;
		else
			logfile << "false" << std::endl;
		logfile << "[scan number]\t[time to gen SR]\t[time to insert SR]\t[time to insert scan(SR)]";
		if (benchmarking){
			logfile << "\t[time to insert scan(PC)]\t[MSE]";
		}
		logfile << std::endl;
	}

	std::cout << "\nCreating tree\n===========================\n";
	octomap::OcTree* tree = new octomap::OcTree(res);
	octomap::OcTree* base = NULL;
	if (benchmarking){
		base = new octomap::OcTree(res);
	}

	double time_to_generate_sr = 0.0;	// sec
	double time_to_insert_sr = 0.0;		// sec
	double time_to_insert_pc = 0.0;		// sec
	size_t numScans = graph->size();
	size_t currentScan = 1;
	for (octomap::ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
		std::cout << "(" << currentScan << "/" << graph->size() << ") " << std::endl;

		// Generate Super Ray
		gettimeofday(&start, NULL);  // start timer
		octomap::SuperRayCloud srcloud;
		octomap::SuperRayGenerator srgenerator(tree->getResolution(), tree->getTreeMaxVal(), 20);
		srgenerator.GenerateSuperRay((*scan_it)->scan, (*scan_it)->pose.trans(), srcloud);
		gettimeofday(&stop, NULL);  // stop timer
		double tgenSR = (stop.tv_sec - start.tv_sec) + 1.0e-6 * (stop.tv_usec - start.tv_usec);
		time_to_generate_sr += tgenSR;

		// Insert super ray
		gettimeofday(&start, NULL);  // start timer
		tree->insertSuperRayCloud(srcloud, srcloud.origin, true);
		gettimeofday(&stop, NULL);  // stop timer
		double tinsSR = (stop.tv_sec - start.tv_sec) + 1.0e-6 * (stop.tv_usec - start.tv_usec);
		time_to_insert_sr += tinsSR;

		if (detailedLog){
			logfile << currentScan << "\t" << tgenSR << "\t" << tinsSR << "\t" << tgenSR + tinsSR;
		}
		
		if (benchmarking){
			double tinsPC = 0.0;
			gettimeofday(&start, NULL);  // start timer
			base->insertPointCloudRays((*scan_it)->scan, (*scan_it)->pose.trans(), true);
			gettimeofday(&stop, NULL);  // stop timer
			tinsPC = (stop.tv_sec - start.tv_sec) + 1.0e-6 * (stop.tv_usec - start.tv_usec);
			time_to_insert_pc += tinsPC;
			if (detailedLog){
				logfile << "\t" << tinsPC << "\t" << compareOccupancy(base, tree);
			}
		}

		if (detailedLog){
			logfile << std::endl;
		}

		currentScan++;
	}

	std::cout << "Done building tree." << std::endl;
	std::cout << "Time to generate super rays: " << time_to_generate_sr << " [sec]" << std::endl;
	std::cout << "Time to insert super rays: " << time_to_insert_sr << " [sec]" << std::endl;
	std::cout << "Time to insert scans (SR): " << time_to_generate_sr + time_to_insert_sr << " [sec]" << std::endl;
	std::cout << "Time to insert 100.000 points took (SR): " << time_to_insert_sr / ((double)graph->getNumPoints() / 100000) << " sec (avg)" << std::endl << std::endl;

	if (benchmarking){
		std::cout << "time to insert scans (PC): " << time_to_insert_pc << " [sec]" << std::endl;
		std::cout << "time to insert 100.000 points took (PC): " << time_to_insert_pc / ((double)graph->getNumPoints() / 100000) << " sec (avg)" << std::endl << std::endl;
		std::cout << "MSE between two octrees: " << compareOccupancy(base, tree) << std::endl;
	}

	tree->writeBinary(treeFilename);

	delete graph;
	delete tree;
	if (benchmarking){
		delete base;
	}

	if (logfile.is_open())
		logfile.close();

	return 0;
}
