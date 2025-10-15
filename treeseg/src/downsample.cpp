#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <iostream>

int main(int argc, char **argv)
{
	// Test message to verify deployment
	std::cout << "============================================" << std::endl;
	std::cout << "  DOWNSAMPLE EXECUTABLE - DEPLOYMENT TEST  " << std::endl;
	std::cout << "  Version: 1.0 - Deployed successfully!    " << std::endl;
	std::cout << "============================================" << std::endl;
	
	std::vector<std::string> args(argv+1,argv+argc);
	
	// Check if arguments provided
	if(args.size() < 2) {
		std::cout << "\n[ERROR] Not enough arguments!" << std::endl;
		std::cout << "Usage: ./downsample <edgelength> <input.pcd> [input2.pcd ...]" << std::endl;
		std::cout << "Example: ./downsample 0.05 myfile.pcd" << std::endl;
		return 1;
	}
	
	float edgelength = std::stof(args[0]);
	std::cout << "\n[INFO] Starting downsample with edge length: " << edgelength << std::endl;
	
	bool otree = true;
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<PointTreeseg>::Ptr original(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr downsampled(new pcl::PointCloud<PointTreeseg>);
	
	for(int i=1;i<args.size();i++)
	{
		std::cout << "[INFO] Processing file " << i << "/" << (args.size()-1) << ": " << args[i] << std::endl;
		
		std::vector<std::string> id = getFileID(args[i]);
		std::stringstream ss;
		ss << id[0] << ".tileheyy.downsample." << id[1] << ".pcd";
		
		reader.read(args[i],*original);
		downsample(original,edgelength,downsampled,otree);
		writer.write(ss.str(),*downsampled,true);
		
		std::cout << "[SUCCESS] Created: " << ss.str() << std::endl;
		
		original->clear();
		downsampled->clear();
	}
	
	std::cout << "\n============================================" << std::endl;
	std::cout << "  DOWNSAMPLE COMPLETED SUCCESSFULLY!       " << std::endl;
	std::cout << "============================================" << std::endl;
	
	return 0;
}