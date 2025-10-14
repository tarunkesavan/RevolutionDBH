#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

int main(int argc, char **argv)
{
    if(argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <smoothness> <stem_cloud> <plot_cloud> <min_diameter>" << std::endl;
        return 1;
    }

	std::vector<std::string> args(argv+1,argv+argc);

	// Parse minimum diameter and convert to radius
    double minDiameter = std::stod(argv[argc-1]);
    double minRadius = minDiameter / 2.0;

	//
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	//
	std::cout << "Reading plot-level cloud: " << std::flush;
	pcl::PointCloud<PointTreeseg>::Ptr plot(new pcl::PointCloud<PointTreeseg>);
	readTiles(args,plot);
	std::cout << "complete" << std::endl;
	//
	for(int i=1;i<getTilesStartIdx(args);i++)
	{
		std::cout << "----------: " << args[i] << std::endl;		
		std::vector<std::string> id = getFileID(args[i]);
		pcl::PointCloud<PointTreeseg>::Ptr foundstem(new pcl::PointCloud<PointTreeseg>);
		reader.read(args[i],*foundstem);
		//
		std::cout << "RANSAC cylinder fit: " << std::flush;
		int nnearest = 60;
		cylinder cyl;
		fitCylinder(foundstem,nnearest,false,false,cyl);
		if(cyl.rad < minRadius) cyl.rad = minRadius;
		std::cout << cyl.rad << std::endl;	
		//
		std::cout << "Segmenting extended cylinder: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr volume(new pcl::PointCloud<PointTreeseg>);
		// Creates a wider cylinder than the actual stem to ensure capturing all relevant points
		// Acts as a buffer zone around the detected stem
		// The factor of 12 means the cylinder volume will be 12x wider than the fitted stem radius
		float expansionfactor = 8; // MARTIN was 12
		cyl.rad = cyl.rad*expansionfactor;
		spatial3DCylinderFilter(plot,cyl,volume);
		ss.str("");
		ss << id[0] << ".c." << id[1] << ".pcd";
		writer.write(ss.str(),*volume,true);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Segmenting ground returns: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr bottom(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr top(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr vnoground(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		std::sort(volume->points.begin(),volume->points.end(),sortCloudByZ);
		int groundidx = (2.5/100) * volume->points.size();
		float ground = volume->points[groundidx].z;
		Eigen::Vector4f min, max;
		pcl::getMinMax3D(*volume,min,max);
		spatial1DFilter(volume,"z",ground-1,ground+2.2,bottom); // MARTIN was 3.5m
		spatial1DFilter(volume,"z",ground+2.2,max[2],top); // MARTIN was 3.5m
		estimateNormals(bottom,250.0,normals);
		fitPlane(bottom,normals,0.5,inliers,0.75,30);
		extractIndices(bottom,inliers,true,vnoground);
		*vnoground += *top;
		ss.str("");
		ss << id[0] << ".c.ng." << id[1] << ".pcd";
		writer.write(ss.str(),*vnoground,true);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Region-based segmentation: " << std::flush; 
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
		normals->clear();
		estimateNormals(vnoground,50.0,normals);
		float smoothness = std::stof(args[0]);
		regionSegmentation(vnoground,normals,250,3,std::numeric_limits<int>::max(),smoothness,1,regions); // MARTIN original 250
		ss.str("");
		ss << id[0] << ".c.ng.r." << id[1] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Correcting stem: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr stem(new pcl::PointCloud<PointTreeseg>);
		int idx = findClosestIdx(foundstem,regions,true);
		*stem += *regions[idx];
		//nnearest = 60;
		//float zdelta = 0.75;
		//float zstart = 5;
		//float stepcovmax = 0.05;
		//float radchangemin = 0.9;
		//correctStem(regions[idx],nnearest,zstart,zdelta,stepcovmax,radchangemin,stem);
		ss.str("");
		ss << id[0] << ".stem." << id[1] << ".pcd";
		writer.write(ss.str(),*stem,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
