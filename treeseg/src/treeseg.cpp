/*
* treeseg.cpp
*
* MIT License
*
* Copyright 2017 Andrew Burt
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

// treeseg has been developed using:
// Point Cloud Library (http://www.pointclouds.org)

#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/octree/octree.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <iomanip>
#include <map>

//File IO

std::vector<std::string> getFileID(std::string filename)
{
	//Inflexible naming convention:
	//From rxp2pcd: ./DIR/PLOT.tile.number.pcd (plot can include hyphen e.g., FGC01 or FGC-01)
	//From downsample/thin: ./DIR/PLOT.tile.downsample(thin).number.pcd
	//Onwards: ./DIR/PLOT.X.number.pcd
	std::string pid,tid;
	std::vector<std::string> info;
	std::vector<std::string> tmp1,tmp2,tmp3;
	boost::split(tmp1,filename,boost::is_any_of("/"));
	boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
	if(tmp2[1] == "tile")
	{
		pid = tmp2[0];
		if(tmp2[2] == "downsample" || tmp2[2] == "thin") tid = tmp2[3];
		else tid = tmp2[2];
		info.push_back(pid);
		info.push_back(tid);
		return info;
	}
	else
	{
		pid = tmp2[0];
		tid = tmp2[tmp2.size()-2];
		info.push_back(pid);
		info.push_back(tid);
		return info;
	}
}

void readTiles(const std::vector<std::string> &args, pcl::PointCloud<PointTreeseg>::Ptr &cloud)
{
	pcl::PCDReader reader;
	pcl::PointCloud<PointTreeseg>::Ptr tmp(new pcl::PointCloud<PointTreeseg>);
	for(int i=0;i<args.size();i++)
	{
		std::string filename = args[i];
		std::vector<std::string> tmp1,tmp2;
		boost::split(tmp1,filename,boost::is_any_of("/"));
		boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
		if(tmp2.size() > 1)
		{
			if(tmp2[1] == "tile")
			{
				reader.read(args[i],*tmp);
				*cloud += *tmp;
				tmp->clear();	
			}
		}
	}
}

int getTilesStartIdx(const std::vector<std::string> &args)
{
	int start_tiles;
	for(int i=0;i<args.size();i++)
	{
		std::string filename = args[i];
		std::vector<std::string> tmp1,tmp2;
		boost::split(tmp1,filename,boost::is_any_of("/"));
		boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
		if(tmp2.size() > 1)
		{
			if(tmp2[1] == "tile")
			{
				start_tiles = i;
				break;
			}

		}
	}
	return start_tiles;
}

void writeClouds(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, std::string fname, bool doPCA)
{
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=0;i<clouds.size();i++)
	{
		int r = rand()%256;
		int g = rand()%256;
		int b = rand()%256;
		for(int j=0;j<clouds[i]->points.size();j++)
		{
			pcl::PointXYZRGB point;
			point.x = clouds[i]->points[j].x;
			point.y = clouds[i]->points[j].y;
			point.z = clouds[i]->points[j].z;
			point.r = r;
			point.g = g;
			point.b = b;
			out->insert(out->end(),point);
		}
	}
	writer.write(fname,*out,true);
}

std::vector<pcl::PointCloud<PointTreeseg>::Ptr> readClouds(const std::string &fname) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    
    // Add error checking for file read
    if (reader.read(fname, *inputCloud) == -1) {
        std::cerr << "Error: Could not read file " << fname << std::endl;
        return std::vector<pcl::PointCloud<PointTreeseg>::Ptr>();
    }

    // Debug output
    std::cout << "Read " << inputCloud->size() << " elements from file" << std::endl;

    std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clouds;
    std::map<std::tuple<int, int, int>, pcl::PointCloud<PointTreeseg>::Ptr> colorMap;

    // Validate input cloud
    if (inputCloud->empty()) {
        std::cerr << "Error: Input cloud is empty" << std::endl;
        return clouds;
    }

    // Process points
    for (const auto& point : inputCloud->points) {
        std::tuple<int, int, int> color(point.r, point.g, point.b);
        
        if (colorMap.find(color) == colorMap.end()) {
            pcl::PointCloud<PointTreeseg>::Ptr newCloud(new pcl::PointCloud<PointTreeseg>);
            colorMap[color] = newCloud;
            clouds.push_back(newCloud);
        }
        
        PointTreeseg newPoint;
        newPoint.x = point.x;
        newPoint.y = point.y;
        newPoint.z = point.z;
        colorMap[color]->points.push_back(newPoint);
    }

    // Debug output
    // std::cout << "Created " << clouds.size() << " separate clouds" << std::endl;
    // for (size_t i = 0; i < clouds.size(); i++) {
    //     std::cout << "Cloud " << i << " has " << clouds[i]->points.size() << " points" << std::endl;
    // }

    return clouds;
}

//Nearest neighbour analysis

std::vector<float> dNN(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest)
{
	std::vector<float> dist;
	pcl::KdTreeFLANN<PointTreeseg> tree;
	tree.setInputCloud(cloud);
	int k = nnearest + 1;
	for(pcl::PointCloud<PointTreeseg>::iterator it=cloud->begin();it!=cloud->end();it++)
	{
		std::vector<float> p_dist;
		PointTreeseg searchPoint;
		searchPoint.x = it->x;
		searchPoint.y = it->y;
		searchPoint.z = it->z;
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		tree.nearestKSearch(searchPoint,k,pointIdxNKNSearch,pointNKNSquaredDistance);
		for(int i=1;i<pointIdxNKNSearch.size();i++)
		{
			p_dist.push_back(sqrt(pointNKNSquaredDistance[i]));
		}
		float p_dist_sum = std::accumulate(p_dist.begin(),p_dist.end(),0.0);
		float p_dist_mean = p_dist_sum/p_dist.size();
		dist.push_back(p_dist_mean);
	}
	float dist_sum = std::accumulate(dist.begin(),dist.end(),0.0);
	float dist_mean = dist_sum/dist.size();
	float sq_sum = std::inner_product(dist.begin(), dist.end(), dist.begin(), 0.0);
	float stddev = std::sqrt(sq_sum / dist.size() - dist_mean * dist_mean);
	std::vector<float> results;
	results.push_back(dist_mean);
	results.push_back(stddev);
	return results;
}

std::vector<std::vector<float>> dNNz(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep)
{
	std::vector<std::vector<float>> results;
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	pcl::PointCloud<PointTreeseg>::Ptr tmp(new pcl::PointCloud<PointTreeseg>);
	for(float z=min[2];z<max[2];z+=zstep)
	{
		spatial1DFilter(cloud,"z",z,z+zstep,tmp);
		if(tmp->points.size() > nnearest)
		{
			std::vector<float> nn = dNN(tmp,nnearest);
			//float pos = z - min[2];
			float pos = z + zstep;
			std::vector<float> r;
			r.push_back(pos);
			r.push_back(nn[0]);
			results.push_back(r);
		}
		tmp->clear();
	}
	return results;
}

float interpolatedNNZ(float x, const std::vector<std::vector<float>> &nndata, bool extrapolate)
{
	std::vector<float> xData;
	std::vector<float> yData;
	for(int m=0;m<nndata.size();m++)
	{
		xData.push_back(nndata[m][0]);
		yData.push_back(nndata[m][1]);
	}
	int size = xData.size();
	int i = 0;
	if(x >= xData[size-2]) i = size - 2;
	else
	{
		while ( x > xData[i+1] ) i++;
	}
	double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
	if(!extrapolate)
	{
		if ( x < xL ) yR = yL;
		if ( x > xR ) yL = yR;
	}
	double dydx = ( yR - yL ) / ( xR - xL );
	return yL + dydx * ( x - xL );
}

//Cloud metrics

void getCloudMetrics(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, cloudmetrics metrics)
{
	Eigen::Vector4f min3D;
	Eigen::Vector4f max3D;
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariancematrix;
	Eigen::Matrix3f eigenvectors;
	Eigen::Vector3f eigenvalues;
	pcl::getMinMax3D(*cloud,min3D,max3D);
	computePCA(cloud,centroid,covariancematrix,eigenvectors,eigenvalues);
	float length = getCloudLength(cloud,centroid,eigenvectors);
	Eigen::Vector3f vector3D(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2));
	metrics.count = cloud->points.size();
	metrics.min3D = min3D;
	metrics.max3D = max3D;
	metrics.centroid = centroid;
	metrics.covariancematrix = covariancematrix;
	metrics.eigenvectors = eigenvectors;
	metrics.eigenvalues = eigenvalues;
	metrics.vector3D = vector3D; 
	metrics.length = length;
}

float getCloudLength(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const Eigen::Vector4f &centroid, const Eigen::Matrix3f &eigenvectors)
{
	Eigen::Vector3f point(centroid[0],centroid[1],centroid[2]);
	Eigen::Vector3f direction(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2));
	Eigen::Affine3f transform;
	Eigen::Vector3f world(0,direction[2],-direction[1]);
	direction.normalize();
	pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform);
	pcl::PointCloud<PointTreeseg>::Ptr transformedcloud(new pcl::PointCloud<PointTreeseg>);
	pcl::transformPointCloud(*cloud,*transformedcloud,transform);
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*transformedcloud,min,max);
	float length = max[2]-min[2];
	return length;
}

void getBasicCloudMetrics(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, basiccloudmetrics metrics)
{
	Eigen::Vector4f min3D;
	Eigen::Vector4f max3D;
	pcl::getMinMax3D(*cloud,min3D,max3D);
	metrics.count = cloud->points.size();
	metrics.min3D = min3D;
	metrics.max3D = max3D;
}

//Downsampling

void downsample(const pcl::PointCloud<PointTreeseg>::Ptr &original, float edgelength, pcl::PointCloud<PointTreeseg>::Ptr &filtered, bool octree)
{
	if(octree == false)
	{
		pcl::VoxelGrid<PointTreeseg> downsample;
		downsample.setInputCloud(original);
		downsample.setLeafSize(edgelength,edgelength,edgelength);
		downsample.filter(*filtered);
	}
	else
	{
		pcl::octree::OctreePointCloudSearch<PointTreeseg> octree(edgelength);
		octree.setInputCloud(original);
		octree.defineBoundingBox();
		octree.addPointsFromInputCloud();
		pcl::PointCloud<PointTreeseg>::VectorType voxelcentres;
		octree.getOccupiedVoxelCenters(voxelcentres);
		for(int i=0;i<voxelcentres.size();i++)
		{
			std::vector<int> voxelinliersidx;
			octree.voxelSearch(voxelcentres[i],voxelinliersidx);
			pcl::PointCloud<PointTreeseg>::Ptr voxelinliers(new pcl::PointCloud<PointTreeseg>);
			for(int j=0;j<voxelinliersidx.size();j++) voxelinliers->insert(voxelinliers->end(),original->points[voxelinliersidx[j]]);
			PointTreeseg centroid;
			pcl::computeCentroid(*voxelinliers,centroid);
			filtered->insert(filtered->end(),centroid);
		}
	}
}

void thin(const pcl::PointCloud<PointTreeseg>::Ptr &original, float edgelength, pcl::PointCloud<PointTreeseg>::Ptr &filtered, bool preservePointClosestToVoxelCentroid)
{
	pcl::octree::OctreePointCloudSearch<PointTreeseg> octree(edgelength);
	octree.setInputCloud(original);
	octree.defineBoundingBox();
	octree.addPointsFromInputCloud();
	pcl::PointCloud<PointTreeseg>::VectorType voxelcentres;
	octree.getOccupiedVoxelCenters(voxelcentres);
	for(int i=0;i<voxelcentres.size();i++)
	{
		std::vector<int> voxelinliersidx;
		octree.voxelSearch(voxelcentres[i],voxelinliersidx);
		pcl::PointCloud<PointTreeseg>::Ptr voxelinliers(new pcl::PointCloud<PointTreeseg>);
		for(int j=0;j<voxelinliersidx.size();j++) voxelinliers->insert(voxelinliers->end(),original->points[voxelinliersidx[j]]);
		PointTreeseg centroid;
		pcl::computeCentroid(*voxelinliers,centroid);
		pcl::KdTreeFLANN<PointTreeseg> kdtree;
		kdtree.setInputCloud(voxelinliers);
		int K = voxelinliers->points.size();
		std::vector<int> pointIdx(K);
		std::vector<float> pointDist(K);
		kdtree.nearestKSearch(centroid,K,pointIdx,pointDist);
		#if XYZRRDRS == true
			if(preservePointClosestToVoxelCentroid == true)
			{
				filtered->insert(filtered->end(),voxelinliers->points[pointIdx[0]]);
			}
			else
			{
				pcl::PointCloud<PointTreeseg>::Ptr sortedvoxelinliers(new pcl::PointCloud<PointTreeseg>);
				for(int k=0;k<pointIdx.size();k++) sortedvoxelinliers->insert(sortedvoxelinliers->end(),voxelinliers->points[pointIdx[k]]);
				std::sort(sortedvoxelinliers->points.begin(),sortedvoxelinliers->points.end(),sortCloudByDeviation); //change this for any of RRDRS
				filtered->insert(filtered->end(),sortedvoxelinliers->points[0]);
			}
		#else
			filtered->insert(filtered->end(),voxelinliers->points[pointIdx[0]]);
		#endif
	}
}

//Spatial Filters

void spatial1DFilter(const pcl::PointCloud<PointTreeseg>::Ptr &original, std::string dimension, float min, float max, pcl::PointCloud<PointTreeseg>::Ptr &filtered)
{
	pcl::PassThrough<PointTreeseg> pass;
	pass.setInputCloud(original);
	pass.setFilterFieldName(dimension);
	pass.setFilterLimits(min,max);
	pass.filter(*filtered);
}

void spatial3DCylinderFilter(const pcl::PointCloud<PointTreeseg>::Ptr &original, cylinder cyl, pcl::PointCloud<PointTreeseg>::Ptr &filtered)
{
	float len = 1000;
	Eigen::Vector3f cp1(cyl.x+len*cyl.dx,cyl.y+len*cyl.dy,cyl.z+len*cyl.dz);
	Eigen::Vector3f cp2(cyl.x-len*cyl.dx,cyl.y-len*cyl.dy,cyl.z-len*cyl.dz);
	Eigen::Vector3f cn1 = cp2 - cp1;
	cn1.normalize();
	Eigen::Vector3f cn2 = -cn1;
	for(int i=0;i<original->points.size();i++)
	{
		Eigen::Vector3f p(original->points[i].x,original->points[i].y,original->points[i].z);
		float dot1 = cn1.dot(p-cp1);
		if(dot1 > 0)
		{
			float dot2 = cn2.dot(p-cp2);
			if(dot2 > 0)
			{
				Eigen::Vector3f mp = p - (cn1 * cn1.dot(p-cp1));
				float dist = sqrt(pow(mp(0)-cp1(0),2)+pow(mp(1)-cp1(1),2)+pow(mp(2)-cp1(2),2));
				if(dist <= cyl.rad)
				{
					filtered->insert(filtered->end(),original->points[i]);
				}
			}
		}
	}
}

//Clustering

void euclideanClustering(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, float dmax, int nmin, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters)
{
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>);
	tree->setInputCloud(cloud);
	pcl::EuclideanClusterExtraction<PointTreeseg> ec;
	ec.setClusterTolerance(dmax);
	ec.setMinClusterSize(nmin);
	ec.setMaxClusterSize(std::numeric_limits<int>().max());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{
		pcl::PointCloud<PointTreeseg>::Ptr tmpcloud(new pcl::PointCloud<PointTreeseg>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{
			tmpcloud->insert(tmpcloud->end(),cloud->points[*pit]);
		}
		clusters.push_back(tmpcloud);
	}
}

//Principal component analysis

void computePCA(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, Eigen::Vector4f &centroid, Eigen::Matrix3f &covariancematrix, Eigen::Matrix3f &eigenvectors, Eigen::Vector3f &eigenvalues)
{
	pcl::compute3DCentroid(*cloud,centroid);
	pcl::computeCovarianceMatrix(*cloud,centroid,covariancematrix);
	pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
}

//Surface normals

void estimateNormals(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, float searchValue, pcl::PointCloud<pcl::Normal>::Ptr &normals, bool useRadiusSearch)
{
    pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>());
    tree->setInputCloud(cloud);
    pcl::NormalEstimation<PointTreeseg,pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    
    if (useRadiusSearch) {
        // Use radius search
        ne.setRadiusSearch(searchValue);
    } else {
        // Use k-nearest neighbor search (default)
        ne.setKSearch(static_cast<int>(searchValue));
    }
    
    ne.compute(*normals);
}

//Segmentation

void regionSegmentation(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, int nneighbours, int nmin, int nmax, float smoothness, float curvature, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions)
{
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg> ());
	tree->setInputCloud(cloud);
	pcl::RegionGrowing<PointTreeseg,pcl::Normal> reg;
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(nneighbours);
	reg.setMinClusterSize(nmin);
	reg.setMaxClusterSize(nmax);
	reg.setSmoothnessThreshold(smoothness * (M_PI / 180.0));
	reg.setCurvatureThreshold(curvature);
	reg.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{       
		pcl::PointCloud<PointTreeseg>::Ptr tmpcloud(new pcl::PointCloud<PointTreeseg>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{       
			tmpcloud->insert(tmpcloud->end(),cloud->points[*pit]);
		}
		regions.push_back(tmpcloud);
	}
}

//Shape fitting

void fitPlane(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, float dthreshold, pcl::PointIndices::Ptr &inliers, float nweight, float angle, Eigen::Vector3f axis)
{
	pcl::SACSegmentationFromNormals<PointTreeseg,pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.setOptimizeCoefficients(true);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(dthreshold);
	seg.setNormalDistanceWeight(nweight);
	seg.setAxis(axis);
	seg.setEpsAngle(angle * (M_PI / 180.0));
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.segment(*inliers,*coefficients);	
}

std::vector<float> fitCircle(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest)
{
	std::vector<float> nndata;
	nndata = dNN(cloud,nnearest);
	float nndist = nndata[0];
	pcl::PointIndices inliers;
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentation<PointTreeseg> seg;
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000000);
	seg.setModelType(pcl::SACMODEL_CIRCLE2D);
	seg.setDistanceThreshold(nndist);
	seg.setInputCloud(cloud);
	seg.segment(inliers,coefficients);
	std::vector<float> results;
	results.push_back(coefficients.values[0]);
	results.push_back(coefficients.values[1]);
	results.push_back(coefficients.values[2]);
	return results;
}

void fitCylinder(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl)
{
	cyl.ismodel = false;
	if(cloud->points.size() >= 10)
	{
		std::vector<float> nndata;
		nndata  = dNN(cloud,nnearest);
		float nndist = nndata[0];
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		estimateNormals(cloud,nnearest,normals);
		pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>());
		tree->setInputCloud(cloud);
		pcl::PointIndices indices;
		pcl::ModelCoefficients coeff;
		pcl::SACSegmentationFromNormals<PointTreeseg, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000000);
		seg.setDistanceThreshold(nndist);
		seg.setInputCloud(cloud);
		seg.setInputNormals(normals);
		seg.segment(indices,coeff);
		if(indices.indices.size() > 0)
		{
			pcl::PointCloud<PointTreeseg>::Ptr inliers(new pcl::PointCloud<PointTreeseg>);
			for(std::vector<int>::iterator pit=indices.indices.begin();pit!=indices.indices.end();pit++)
			{
				inliers->insert(inliers->end(),cloud->points[*pit]);
			}

			// Get axis direction and normalize it
			Eigen::Vector3f axis(coeff.values[3], coeff.values[4], coeff.values[5]);
			axis.normalize();
			
			// Get arbitrary point on axis from RANSAC
			Eigen::Vector3f point_on_axis(coeff.values[0], coeff.values[1], coeff.values[2]);
			
			// Find center point of inliers
			Eigen::Vector4f inlier_centroid;
			pcl::compute3DCentroid(*inliers, inlier_centroid);
			Eigen::Vector3f center_point(inlier_centroid[0], inlier_centroid[1], inlier_centroid[2]);
			
			// Project center point onto axis to get base point for cylinder
			Eigen::Vector3f v = center_point - point_on_axis;
			float dist_along_axis = v.dot(axis);
			Eigen::Vector3f base_point = point_on_axis + (dist_along_axis * axis);
			
			// Set cylinder parameters using the base point, center of inliers is best guess for that
			cyl.x = base_point[0];
			cyl.y = base_point[1];
			cyl.z = base_point[2];
			cyl.dx = axis[0];
			cyl.dy = axis[1];
			cyl.dz = axis[2];
			cyl.rad = coeff.values[6];
				
			cyl.steprad = 0;
			cyl.stepcov = 0;
			cyl.radratio = 0;
				
			cyl.cloud = cloud;
			cyl.inliers = inliers;

			if(cyl.rad > 0 && finite == false && diagnostics == false) cyl.ismodel = true;
			if(cyl.rad > 0 && finite == true)
			{
				pcl::PointCloud<PointTreeseg>::Ptr inliers_transformed(new pcl::PointCloud<PointTreeseg>);
				Eigen::Vector3f point(coeff.values[0],coeff.values[1],coeff.values[2]);
				Eigen::Vector3f direction(coeff.values[3],coeff.values[4],coeff.values[5]);
				Eigen::Affine3f transform;
				Eigen::Vector3f world(0,direction[2],-direction[1]);
				direction.normalize();
				pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform);
				pcl::transformPointCloud(*inliers,*inliers_transformed,transform);
				Eigen::Vector4f min,max;
				pcl::getMinMax3D(*inliers_transformed,min,max);
				cyl.len = max[2]-min[2];
				if(cyl.len > 0) cyl.ismodel = true;
			}
			if(cyl.rad > 0 && diagnostics == true)
			{
				cylinderDiagnostics(cyl,nnearest);
				if(cyl.steprad > 0) cyl.ismodel = true;
			}
		}
	}
}

void fitCylinder_b(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl)
{
	cyl.ismodel = false;
	if(cloud->points.size() >= 10)
	{
		std::vector<float> nndata;
		nndata  = dNN(cloud,nnearest);
		float nndist = nndata[0];
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		estimateNormals(cloud,nnearest,normals);
		pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>());
		tree->setInputCloud(cloud);
		pcl::PointIndices indices;
		pcl::ModelCoefficients coeff;
		pcl::SACSegmentationFromNormals<PointTreeseg, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000000);
		seg.setDistanceThreshold(nndist);
		seg.setInputCloud(cloud);
		seg.setInputNormals(normals);
		seg.segment(indices,coeff);
		if(indices.indices.size() > 0)
		{
			pcl::PointCloud<PointTreeseg>::Ptr inliers(new pcl::PointCloud<PointTreeseg>);
			for(std::vector<int>::iterator pit=indices.indices.begin();pit!=indices.indices.end();pit++)
			{
				inliers->insert(inliers->end(),cloud->points[*pit]);
			}

			// Get axis direction and normalize it
			Eigen::Vector3f axis(coeff.values[3], coeff.values[4], coeff.values[5]);
			axis.normalize();
			
			// Get arbitrary point on axis from RANSAC
			Eigen::Vector3f point_on_axis(coeff.values[0], coeff.values[1], coeff.values[2]);
			
			// Find center point of inliers
			Eigen::Vector4f inlier_centroid;
			pcl::compute3DCentroid(*inliers, inlier_centroid);
			Eigen::Vector3f center_point(inlier_centroid[0], inlier_centroid[1], inlier_centroid[2]);
			
			// Project center point onto axis to get base point for cylinder
			Eigen::Vector3f v = center_point - point_on_axis;
			float dist_along_axis = v.dot(axis);
			Eigen::Vector3f base_point = point_on_axis + (dist_along_axis * axis);
			
			// Set cylinder parameters using the base point, center of inliers is best guess for that
			cyl.x = base_point[0];
			cyl.y = base_point[1];
			cyl.z = base_point[2];
			cyl.dx = axis[0];
			cyl.dy = axis[1];
			cyl.dz = axis[2];
			cyl.rad = coeff.values[6];
				
			cyl.steprad = 0;
			cyl.stepcov = 0;
			cyl.radratio = 0;
				
			cyl.cloud = cloud;
			cyl.inliers = inliers;

			if(cyl.rad > 0 && finite == false && diagnostics == false) cyl.ismodel = true;
			if(cyl.rad > 0 && finite == true)
			{
				pcl::PointCloud<PointTreeseg>::Ptr inliers_transformed(new pcl::PointCloud<PointTreeseg>);
				Eigen::Vector3f point(coeff.values[0],coeff.values[1],coeff.values[2]);
				Eigen::Vector3f direction(coeff.values[3],coeff.values[4],coeff.values[5]);
				Eigen::Affine3f transform;
				Eigen::Vector3f world(0,direction[2],-direction[1]);
				direction.normalize();
				pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform);
				pcl::transformPointCloud(*inliers,*inliers_transformed,transform);
				Eigen::Vector4f min,max;
				pcl::getMinMax3D(*inliers_transformed,min,max);
				cyl.len = max[2]-min[2];
				if(cyl.len > 0) cyl.ismodel = true;
				// float length = max[2] - min[2];
				// if (length > 0.3 && length <= 0.5) {
				// 	cyl.ismodel = true;
				// }
			}
			// if(cyl.rad > 0 && diagnostics == true)
			// {
			// 	cylinderDiagnostics(cyl,nnearest);
			// 	if(cyl.steprad > 0) cyl.ismodel = true;
			// }
		}
	}
}

void fitCylinder_changed_bck(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl)
{
    cyl.ismodel = false;

    if (cloud->points.size() >= 10)
    {
        std::vector<float> nndata = dNN(cloud, nnearest);
        float nndist = nndata[0];
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        estimateNormals(cloud, nnearest, normals);
        pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>());
        tree->setInputCloud(cloud);

        int maxTries = 1000; // limit attempts
        bool found = false;
        for (int tries = 0; tries < maxTries && !found; ++tries)
        {
            pcl::PointIndices indices;
            pcl::ModelCoefficients coeff;
            pcl::SACSegmentationFromNormals<PointTreeseg, pcl::Normal> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER);
            seg.setNormalDistanceWeight(0.1);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(nndist);
            seg.setInputCloud(cloud);
            seg.setInputNormals(normals);
            seg.segment(indices, coeff);

            if (indices.indices.size() == 0)
                continue;

            pcl::PointCloud<PointTreeseg>::Ptr inliers(new pcl::PointCloud<PointTreeseg>);
            for (int idx : indices.indices)
                inliers->push_back(cloud->points[idx]);

            // Cylinder axis and point
            Eigen::Vector3f axis(coeff.values[3], coeff.values[4], coeff.values[5]);
            axis.normalize();
            Eigen::Vector3f point(coeff.values[0], coeff.values[1], coeff.values[2]);

            // Transform inliers
            Eigen::Affine3f transform;
            Eigen::Vector3f world(0, axis[2], -axis[1]);
            pcl::getTransformationFromTwoUnitVectorsAndOrigin(world, axis, point, transform);

            pcl::PointCloud<PointTreeseg>::Ptr inliers_transformed(new pcl::PointCloud<PointTreeseg>);
            pcl::transformPointCloud(*inliers, *inliers_transformed, transform);

            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*inliers_transformed, min, max);
            float length = max[2] - min[2];

            // Check if length is close to 0.5m (±5cm)
            if (std::abs(length - 0.5f) <= 0.05f)
            {
                // Set cylinder
                cyl.x = point[0];
                cyl.y = point[1];
                cyl.z = point[2];
                cyl.dx = axis[0];
                cyl.dy = axis[1];
                cyl.dz = axis[2];
                cyl.rad = coeff.values[6];
                cyl.len = length;
                cyl.cloud = cloud;
                cyl.inliers = inliers;
                cyl.steprad = 0;
                cyl.stepcov = 0;
                cyl.radratio = 0;
                cyl.ismodel = true;

                if (diagnostics)
                {
                    cylinderDiagnostics(cyl, nnearest);
                    cyl.ismodel = (cyl.steprad > 0);
                }

                found = true; // Stop after finding one good fit
            }
        }
    }
}

void cylinderDiagnostics(cylinder &cyl, int nnearest)
{
	int NSTEP = 6;
	pcl::PointCloud<PointTreeseg>::Ptr inliers_transformed(new pcl::PointCloud<PointTreeseg>);
	Eigen::Vector3f point(cyl.x,cyl.y,cyl.z);
	Eigen::Vector3f direction(cyl.dx,cyl.dy,cyl.dz);
	Eigen::Affine3f transform;
	Eigen::Vector3f world(0,direction[2],-direction[1]);
	direction.normalize();
	pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform);
	pcl::transformPointCloud(*cyl.inliers,*inliers_transformed,transform);
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*inliers_transformed,min,max);
	float zstep = (max[2]-min[2]) / NSTEP;
	std::vector<float> zrads;
	for(int i=0;i<NSTEP;i++)
	{
		float zmin = min[2] + i * zstep;
		float zmax = min[2] + (i+1) * zstep;
		pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
		spatial1DFilter(inliers_transformed,"z",zmin,zmax,slice);
		cylinder zcyl;
		fitCylinder(slice,nnearest,false,false,zcyl);
		if(zcyl.ismodel == true) zrads.push_back(zcyl.rad);
	}
	if(zrads.size() >= NSTEP - 2)
	{
		float sum = std::accumulate(zrads.begin(),zrads.end(),0.0);
		float mean = sum/zrads.size();
		std::vector<float> diff(zrads.size());
		// std::transform(zrads.begin(),zrads.end(),diff.begin(),std::bind2nd(std::minus<float>(),mean));
		std::bind(std::minus<float>(), std::placeholders::_1, mean);
		float stddev = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / zrads.size());
		cyl.steprad = mean;
		cyl.stepcov = stddev/mean;
		cyl.radratio = std::min(cyl.rad,cyl.steprad)/std::max(cyl.rad,cyl.steprad);
	}
	// bring back inliers to original frame
	Eigen::Affine3f transform_inverse = transform.inverse();
	pcl::transformPointCloud(*inliers_transformed, *cyl.inliers, transform_inverse);
}

void writeCylinderParameters(const std::vector<cylinder>& cylinders, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile) {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }
    
    size_t numCylinders = cylinders.size();
    outFile.write(reinterpret_cast<const char*>(&numCylinders), sizeof(numCylinders));
    
    for (const auto& cyl : cylinders) {
        outFile.write(reinterpret_cast<const char*>(&cyl.ismodel), sizeof(cyl.ismodel));
        outFile.write(reinterpret_cast<const char*>(&cyl.rad), sizeof(cyl.rad));
        outFile.write(reinterpret_cast<const char*>(&cyl.len), sizeof(cyl.len));
        outFile.write(reinterpret_cast<const char*>(&cyl.stepcov), sizeof(cyl.stepcov));
        outFile.write(reinterpret_cast<const char*>(&cyl.radratio), sizeof(cyl.radratio));
        outFile.write(reinterpret_cast<const char*>(&cyl.x), sizeof(cyl.x));
        outFile.write(reinterpret_cast<const char*>(&cyl.y), sizeof(cyl.y));
        outFile.write(reinterpret_cast<const char*>(&cyl.z), sizeof(cyl.z));
        outFile.write(reinterpret_cast<const char*>(&cyl.dx), sizeof(cyl.dx));
        outFile.write(reinterpret_cast<const char*>(&cyl.dy), sizeof(cyl.dy));
        outFile.write(reinterpret_cast<const char*>(&cyl.dz), sizeof(cyl.dz));
    }
    outFile.close();
}

std::vector<cylinder> readCylinderParameters(const std::string& filename) {
    std::ifstream inFile(filename, std::ios::binary);
    if (!inFile) {
        throw std::runtime_error("Could not open file for reading: " + filename);
    }
    
    size_t numCylinders;
    inFile.read(reinterpret_cast<char*>(&numCylinders), sizeof(numCylinders));
    
    std::vector<cylinder> cylinders;
    cylinders.reserve(numCylinders);
    
    for (size_t i = 0; i < numCylinders; i++) {
        cylinder cyl;
        inFile.read(reinterpret_cast<char*>(&cyl.ismodel), sizeof(cyl.ismodel));
        inFile.read(reinterpret_cast<char*>(&cyl.rad), sizeof(cyl.rad));
        inFile.read(reinterpret_cast<char*>(&cyl.len), sizeof(cyl.len));
        inFile.read(reinterpret_cast<char*>(&cyl.stepcov), sizeof(cyl.stepcov));
        inFile.read(reinterpret_cast<char*>(&cyl.radratio), sizeof(cyl.radratio));
        inFile.read(reinterpret_cast<char*>(&cyl.x), sizeof(cyl.x));
        inFile.read(reinterpret_cast<char*>(&cyl.y), sizeof(cyl.y));
        inFile.read(reinterpret_cast<char*>(&cyl.z), sizeof(cyl.z));
        inFile.read(reinterpret_cast<char*>(&cyl.dx), sizeof(cyl.dx));
        inFile.read(reinterpret_cast<char*>(&cyl.dy), sizeof(cyl.dy));
        inFile.read(reinterpret_cast<char*>(&cyl.dz), sizeof(cyl.dz));
        cylinders.push_back(cyl);
    }
    inFile.close();
    return cylinders;
}

//Generic

bool sortCloudByX(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.x < p2.x;}
bool sortCloudByY(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.y < p2.y;}
bool sortCloudByZ(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.z < p2.z;}
bool sortCloudByDescEuclidDist(const PointTreeseg &p1, const PointTreeseg &p2)
{
	if (p1.x != p2.x) return p1.x > p2.x;
	else if (p1.y != p2.y) return p1.y > p2.y;
	else return p1.z > p2.z;
}
#if XYZRRDRS == true
bool sortCloudByRange(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.range < p2.range;}
bool sortCloudByReflectance(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.reflectance < p2.reflectance;}
bool sortCloudByDeviation(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.deviation < p2.deviation;}
bool sortCloudByReturnNumber(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.return_number < p2.return_number;}
bool sortCloudByScanNumber(const PointTreeseg &p1, const PointTreeseg &p2) {return p1.scan_number < p2.scan_number;}
#endif

bool equalPoint(const PointTreeseg &p1, const PointTreeseg &p2)
{
	if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) return true;
	return false;
}

bool sort2DFloatVectorByCol1(const std::vector<float> &v1, const std::vector<float> &v2) {return v1[1] < v2[1];}
bool sort2DFloatVectorByCol2(const std::vector<float> &v1, const std::vector<float> &v2) {return v1[2] < v2[2];}

int findClosestIdx(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, bool biggest)
{
	int idx = 0;
	std::vector<std::vector<float>> data;
	for(int i=0;i<clouds.size();i++)
	{
		float d;
		if(cloud->points.size() > clouds[i]->points.size()) d = minDistBetweenClouds(cloud,clouds[i]);
		else d = minDistBetweenClouds(clouds[i],cloud);
		std::vector<float> tmp;
		tmp.push_back(d);
		tmp.push_back(clouds[i]->points.size());
		data.push_back(tmp);
	}
	float mindist = std::numeric_limits<float>::infinity();
	for(int j=0;j<data.size();j++)
	{
		if(data[j][0] < mindist)
		{
			mindist = data[j][0];
			idx = j;
		}
	}
	if(biggest == true)
	{		
		float tolerance = 1;
		mindist = mindist + (1.0 * tolerance);
		int size = 0;
		for(int k=0;k<data.size();k++)
		{
			if(data[k][0] < mindist && int(data[k][1]) > size)
			{
				size = int(data[k][1]);
				idx = k;
			}
		}
	}
	return idx;
}

int findPrincipalCloudIdx(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds)
{
	std::vector<std::vector<float>> info;
	for(int i=0;i<clouds.size();i++)
	{
		Eigen::Vector4f min,max;
		pcl::getMinMax3D(*clouds[i],min,max);
		std::vector<float> tmp;
		tmp.push_back(i);
		tmp.push_back(clouds[i]->points.size());
		tmp.push_back(min[2]);
		info.push_back(tmp);
	}
	std::sort(info.begin(),info.end(),sort2DFloatVectorByCol2);
	float zrange = abs(info[info.size()-1][2]-info[0][2]);
	float zpercentile = (5.0/100.0)*zrange;
	float zmax = info[0][2]+zpercentile;
	int idx = 0;
	int pcount = 0;
	for(int i=0;i<info.size();i++)
	{
		if(info[i][1] > pcount)
		{
			if(info[i][2] < zmax)
			{
				idx = info[i][0];
				pcount = info[i][1];
			}
		}
	}
	return idx;
}

void extractIndices(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers, bool invert, pcl::PointCloud<PointTreeseg>::Ptr &filtered)
{
	pcl::ExtractIndices<PointTreeseg> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(invert);
	extract.filter(*filtered);
}

float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b)
{
	pcl::KdTreeFLANN<PointTreeseg> kdtree;
	kdtree.setInputCloud(a);
	return minDistBetweenClouds(a,b,kdtree);
}

float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b, const pcl::KdTreeFLANN<PointTreeseg> &kdtree)
{
	//assuming a is the larger of the two clouds
	float distance = std::numeric_limits<float>::infinity();
	int K = 1;
	for(pcl::PointCloud<PointTreeseg>::iterator it=b->begin();it!=b->end();it++)
	{
		std::vector<float> p_dist;
		PointTreeseg searchPoint;
		searchPoint.x = it->x;
		searchPoint.y = it->y;
		searchPoint.z = it->z;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);
		float d = sqrt(pointNKNSquaredDistance[0]);
		if(d < distance) distance = d;
	}
	return distance;
}

bool intersectionTest3DBox(const Eigen::Vector4f &amin, const Eigen::Vector4f &amax, const Eigen::Vector4f &bmin, const Eigen::Vector4f &bmax)
{
	bool intersection = false;
	if(amin[0] <= bmax[0] && amax[0] >= bmin[0])
	{
		if(amin[1] <= bmax[1] && amax[1] >= bmin[1])
		{
			if(amin[2] <= bmax[2] && amax[2] >= bmin[2])
			{
				intersection = true;
			}
		}
	}
	return intersection;
}

void catIntersectingClouds(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds)
{
	//this is poorly optimised and currently the primary bottleneck of findstems.
	bool donesomething = true;
	while(donesomething == true)
	{
		int idx;
		std::vector<float> duplicates;
		for(int i=0;i<clouds.size();i++)
		{
			Eigen::Vector4f amin,amax;
			pcl::getMinMax3D(*clouds[i],amin,amax);
			for(int j=0;j<clouds.size();j++)
			{
				if(j != i)
				{
					Eigen::Vector4f bmin,bmax;
					pcl::getMinMax3D(*clouds[j],bmin,bmax);
					bool intersects = intersectionTest3DBox(amin,amax,bmin,bmax);
					if(intersects == true) duplicates.push_back(j);
				}
			}
			if(duplicates.size() > 0)
			{
				idx = i;
				break;
			}
		}
		if(duplicates.size() > 0) 
		{
			std::sort(duplicates.begin(),duplicates.end(),std::greater<int>());
			for(int k=0;k<duplicates.size();k++)
			{
				*clouds[idx] += *clouds[duplicates[k]];				
				clouds.erase(clouds.begin()+duplicates[k]);
			}
		}
		else donesomething = false;
	}
}

void removeDuplicatePoints(pcl::PointCloud<PointTreeseg>::Ptr &cloud)
{
	std::sort(cloud->points.begin(),cloud->points.end(),sortCloudByDescEuclidDist);
	auto unique = std::unique(cloud->points.begin(),cloud->points.end(),equalPoint);
	cloud->erase(unique,cloud->end());
}

//treeseg specific

std::vector<std::vector<float>> getDtmAndSlice(const pcl::PointCloud<PointTreeseg>::Ptr &plot, float resolution, float percentile, float zmin, float zmax, pcl::PointCloud<PointTreeseg>::Ptr &slice)
{
	std::vector<std::vector<float>> dem;
	std::vector<float> result;
	pcl::PointCloud<PointTreeseg>::Ptr tmpcloud(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr tile(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr tileslice(new pcl::PointCloud<PointTreeseg>);
	Eigen::Vector4f plotmin,plotmax;
	pcl::getMinMax3D(*plot,plotmin,plotmax);
	for(float x=plotmin[0];x<plotmax[0];x+=resolution)
	{
		spatial1DFilter(plot,"x",x,x+resolution,tmpcloud);
		for(float y=plotmin[1];y<plotmax[1];y+=resolution)
		{
			spatial1DFilter(tmpcloud,"y",y,y+resolution,tile);
			std::sort(tile->points.begin(),tile->points.end(),sortCloudByZ);
			int idx = (percentile / 100) * tile->points.size();
			float ground = tile->points[idx].z;
			result.push_back(x);
			result.push_back(y);
			result.push_back(ground);
			dem.push_back(result);
			spatial1DFilter(tile,"z",ground+zmin,ground+zmax,tileslice);
			*slice += *tileslice;
			result.clear();
			tile->clear();
			tileslice->clear();
		}
		tmpcloud->clear();
	}
	return dem;
}

void correctStem(const pcl::PointCloud<PointTreeseg>::Ptr &stem, float nnearest, float zstart, float zstep, float stepcovmax, float radchangemin, pcl::PointCloud<PointTreeseg>::Ptr &corrected)
{
	Eigen::Vector4f min, max;
	pcl::getMinMax3D(*stem,min,max);
	float zstop;
	bool broken = false;
	for(float z=min[2]+zstart;z<max[2];z+=zstep)
	{
		pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);	
		spatial1DFilter(stem,"z",z,z+zstep,slice);
		std::vector<float> circle = fitCircle(slice,nnearest);
		Eigen::Vector4f zmin,zmax;
		pcl::getMinMax3D(*slice,zmin,zmax);
		int steps = 5;
		float dz = (zmax[2]-zmin[2])/float(steps);
		std::vector<float> zrad;
		for(int i=0;i<steps;i++)
		{
			pcl::PointCloud<PointTreeseg>::Ptr zslice(new pcl::PointCloud<PointTreeseg>);	
			spatial1DFilter(slice,"z",z+i*dz,z+(i+1)*dz,zslice);
			if(zslice->points.size() > 10)
			{
				std::vector<float> zcircle = fitCircle(zslice,nnearest);
				zrad.push_back(zcircle[2]);
			}
		}
		float sum = std::accumulate(zrad.begin(),zrad.end(),0.0);
		float mean = sum/zrad.size();
		std::vector<float> diff(zrad.size());
		// std::transform(zrad.begin(),zrad.end(),diff.begin(),std::bind2nd(std::minus<float>(),mean));
		std::bind(std::minus<float>(), std::placeholders::_1, mean);
		float stdev = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / zrad.size());
		float cov = stdev / mean;
		float radchange = std::min(circle[2],mean) / std::max(circle[2],mean);
		//std::cout << circle[2] << " " << mean << " " << cov << " " << radchange << std::endl;
		if(cov > stepcovmax || radchange < radchangemin)
		{
			zstop = z-zstep*1.5;
			broken = true;
			break;
		}
	}
	if(broken == true) spatial1DFilter(stem,"z",min[2],zstop,corrected);
	else spatial1DFilter(stem,"z",min[2],max[2]-zstep,corrected);
}

void removeFarRegions(float dmin, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions)
{
	std::vector<pcl::KdTreeFLANN<PointTreeseg>> kdtrees;
	for(int i=0;i<regions.size();i++)
	{
		pcl::KdTreeFLANN<PointTreeseg> tree;
		tree.setInputCloud(regions[i]);
		kdtrees.push_back(tree);
	}
	int principalidx = findPrincipalCloudIdx(regions);
	Eigen::Vector4f pmin,pmax;
	pcl::getMinMax3D(*regions[principalidx],pmin,pmax);
	int mincount = static_cast<int>(static_cast<float>(regions[principalidx]->points.size() * 0.25));
	std::vector<int> remove_list;
	for(int i=0;i<regions.size();i++)
	{
		if(i != principalidx && regions[i]->points.size() > mincount)
		{
			Eigen::Vector4f cmin,cmax;
			pcl::getMinMax3D(*regions[i],cmin,cmax);
			if(cmin[2] < pmax[2])
			{
				float d = 0;
				if(regions[principalidx]->points.size() >= regions[i]->points.size()) d = minDistBetweenClouds(regions[principalidx],regions[i],kdtrees[principalidx]);
				else d = minDistBetweenClouds(regions[i],regions[principalidx],kdtrees[i]);
				if(d > dmin) remove_list.push_back(i);
			}
		}

	}
	std::sort(remove_list.begin(),remove_list.end(),std::greater<int>());
	for(int k=0;k<remove_list.size();k++)
	{
		regions.erase(regions.begin()+remove_list[k]);
	}
}

void precalculateIntersections(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions, std::vector<std::vector<bool>> &intersections, float expansion)
{
	std::vector<Eigen::Vector4f> bbmin,bbmax;
	for(int i=0;i<regions.size();i++)
	{
		Eigen::Vector4f min,max;
		pcl::getMinMax3D(*regions[i],min,max);
		min[0] -= expansion;
		min[1] -= expansion;
		min[2] -= expansion;
		max[0] += expansion;
		max[1] += expansion;
		max[2] += expansion;
		bbmin.push_back(min);
		bbmax.push_back(max);
	}
	for(int i=0;i<regions.size();i++)
	{
		std::vector<bool> intersects;
		for(int j=0;j<regions.size();j++)
		{
			bool intersect = intersectionTest3DBox(bbmin[i],bbmax[i],bbmin[j],bbmax[j]);
			intersects.push_back(intersect);
		}
		intersections.push_back(intersects);
	}
}

void buildTree(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions, int cyclecount, int firstcount, float firstdist, int nnearest,  float seconddist, pcl::PointCloud<PointTreeseg>::Ptr &tree)
{
	std::vector<int> unallocated;
	std::vector<int> allocated;
	std::vector<int> previouslyallocated;
	std::vector<int> newlyallocated;	
	std::vector<pcl::KdTreeFLANN<PointTreeseg>> kdtrees;
	for(int i=0;i<regions.size();i++)
	{
		unallocated.push_back(i);
		pcl::KdTreeFLANN<PointTreeseg> tree;
		tree.setInputCloud(regions[i]);
		kdtrees.push_back(tree);
	}
	std::vector<std::vector<bool>> intersections;
	precalculateIntersections(regions,intersections,seconddist/2);
	std::cout << "." << std::flush;
	int idx = findPrincipalCloudIdx(regions);
	allocated.push_back(unallocated[idx]);
	previouslyallocated.push_back(allocated[0]);
	unallocated.erase(std::remove(unallocated.begin(),unallocated.end(),allocated[0]),unallocated.end());
	bool donesomething = true;
	int count = 0;
	while(donesomething == true && count < cyclecount)
	{
		for(int i=0;i<previouslyallocated.size();i++)
		{
			std::vector<std::vector<float>> dinfo;
			for(int j=0;j<unallocated.size();j++)
			{
				float d = std::numeric_limits<int>::max();
				if(intersections[previouslyallocated[i]][unallocated[j]] == true)
				{
					if(regions[previouslyallocated[i]]->points.size() >= regions[unallocated[j]]->points.size())
					{
						d = minDistBetweenClouds(regions[previouslyallocated[i]],regions[unallocated[j]],kdtrees[previouslyallocated[i]]);
					}	
					else
					{
						d = minDistBetweenClouds(regions[unallocated[j]],regions[previouslyallocated[i]],kdtrees[unallocated[j]]);
					}
				}
				std::vector<float> tmp;
				tmp.push_back(unallocated[j]);
				tmp.push_back(d);
				dinfo.push_back(tmp);
			}
			if(dinfo.size() > 0)
			{
				std::sort(dinfo.begin(),dinfo.end(),sort2DFloatVectorByCol1);
				if(count < firstcount)
				{
					for(int j=0;j<dinfo.size();j++)
					{
						if(dinfo[j][1] <= firstdist)
						{
							int tmpidx = static_cast<int>(dinfo[j][0]);
							newlyallocated.push_back(tmpidx);
						}						
					}
				}
				else
				{
					for(int j=0;j<dinfo.size();j++)
					{
						if(j == nnearest) break;
						else
						{
							if(dinfo[j][1] <= seconddist)
							{
								int tmpidx = static_cast<int>(dinfo[j][0]);
								newlyallocated.push_back(tmpidx);
							}
						}
					}
				}
			}
		}
		previouslyallocated.clear();
		if(newlyallocated.size() > 0)
		{
			std::sort(newlyallocated.begin(),newlyallocated.end());
			newlyallocated.erase(std::unique(newlyallocated.begin(),newlyallocated.end()),newlyallocated.end());
			for(int i=0;i<newlyallocated.size();i++)
			{
				allocated.push_back(newlyallocated[i]);
				previouslyallocated.push_back(newlyallocated[i]);
				unallocated.erase(std::remove(unallocated.begin(),unallocated.end(),newlyallocated[i]),unallocated.end());
			}
			newlyallocated.clear();
		}
		else donesomething = false;
		count++;
		std::cout << "." << std::flush;
	}
	for(int i=0;i<allocated.size();i++) *tree += *regions[allocated[i]];
}

treeparams getTreeParams(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep, float diffmax)
{       
	treeparams params;
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	float z = min[2] + 1.3;
	int i=0;
	bool stable = false;
	while(stable == false)
	{
		if(z >= max[2]-zstep*1.5) break;
		else
		{
			pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
			pcl::PointCloud<PointTreeseg>::Ptr bslice(new pcl::PointCloud<PointTreeseg>);
			pcl::PointCloud<PointTreeseg>::Ptr fslice(new pcl::PointCloud<PointTreeseg>);
			spatial1DFilter(cloud,"z",z-zstep/2,z+zstep/2,slice);
			spatial1DFilter(cloud,"z",z-zstep*1.5,z-zstep/2,bslice);
			spatial1DFilter(cloud,"z",z+zstep/2,z+zstep*1.5,fslice);
			if(i == 0)
			{
				Eigen::Vector4f smin,smax;
				pcl::getMinMax3D(*slice,smin,smax);
				params.x = (smax[0] + smin[0]) / 2;
				params.y = (smax[1] + smin[1]) / 2;
				params.h = max[2]-min[2];
        			params.c = sqrt(pow(max[0]-min[0],2)+pow(max[1]-min[1],2));
			}
			cylinder cyl,bcyl,fcyl;
			fitCylinder(slice,nnearest,false,true,cyl);
			fitCylinder(bslice,nnearest,false,false,bcyl);
			fitCylinder(fslice,nnearest,false,false,fcyl);
			float d = (cyl.rad + bcyl.rad + fcyl.rad) / 3 * 2;
			float bdiff = fabs(cyl.rad - bcyl.rad) / cyl.rad;
			float fdiff =  fabs(cyl.rad - fcyl.rad) / cyl.rad;
			float diff = (bdiff + fdiff) / 2;
			if(cyl.ismodel == true && diff <= diffmax)
			{
				params.d = d;
				stable = true;
			}
		}
		z += 0.1;
		i++;
	}
	return params;
}

std::vector<pcl::PointCloud<PointTreeseg>::Ptr> classifyVerticalRegions(
    const std::vector<pcl::PointCloud<PointTreeseg>::Ptr>& regions,
    float maxAltitudeAngle,
    float minHorizontalRatio,
    bool verbose) 
{
    std::vector<pcl::PointCloud<PointTreeseg>::Ptr> filteredRegions;
    int totalRegions = regions.size();
    
    // Parameters for surface area analysis
    const float searchRadius = 0.1;

    for(int i = 0; i < regions.size(); i++) {
        if (verbose) {
            printProgressBar(i, regions.size());
        }

        // Skip empty regions
        if (regions[i]->empty()) continue;

        // Compute surface normals (each normal represents a small surface area)
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        estimateNormals(regions[i], searchRadius, normals, true);  // Use radius search

        // Analyze surface areas' altitude angles
        int totalValidSurfaces = 0;
        int horizontalSurfaces = 0;

        for(int j = 0; j < normals->size(); j++) {
            // Skip invalid normals
            if (!pcl::isFinite(normals->points[j])) continue;

            float nx = normals->points[j].normal_x;
            float ny = normals->points[j].normal_y;
            float nz = normals->points[j].normal_z;

            // Calculate angle between normal and vertical axis (0,0,1)
            float angleToVertical = std::acos(std::abs(nz)) * (180.0f/M_PI);

            // For stems, we want normals that are perpendicular to the vertical axis
            // This means the angle should be close to 90 degrees
            if (std::abs(angleToVertical - 90.0f) <= maxAltitudeAngle) {
                horizontalSurfaces++;
            }
            totalValidSurfaces++;
        }

        if (totalValidSurfaces == 0) continue;

        // Calculate ratio of horizontal surfaces
        float horizontalRatio = static_cast<float>(horizontalSurfaces) / totalValidSurfaces;

        // Apply filtering criterion - high ratio means most surfaces are perpendicular to vertical (stem-like)
        if (horizontalRatio >= minHorizontalRatio) {
            filteredRegions.push_back(regions[i]);
        }
    }

    return filteredRegions;
}

void printProgressBar(int current, int total) {
    const int barWidth = 50;  // Reduced width for better visibility
    float progress = float(current) / total;
    
    // Clear the current line
    std::cout << "\r";
    
    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << "% (" 
              << current << "/" << total << ")";
    std::cout.flush();  // Ensure immediate output
}

void printFilteringParameters(float dmin, float dmax, float lmin, float stepcovmax, float radratiomin, float xmin, float xmax, float ymin, float ymax) {
    std::cout << "Filtering Parameters:" << std::endl;
    std::cout << std::left
              << std::setw(15) << "Parameter"
              << std::setw(10) << "dmin"
              << std::setw(10) << "dmax"
              << std::setw(10) << "lmin"
              << std::setw(15) << "stepcovmax"
              << std::setw(15) << "radratiomin"
              << std::setw(10) << "xmin"
              << std::setw(10) << "xmax"
              << std::setw(10) << "ymin"
              << std::setw(10) << "ymax" << std::endl;
    std::cout << std::string(100, '-') << std::endl;
    std::cout << std::left
              << std::setw(15) << "Value"
              << std::setw(10) << dmin
              << std::setw(10) << dmax
              << std::setw(10) << lmin
              << std::setw(15) << stepcovmax
              << std::setw(15) << radratiomin
              << std::setw(10) << xmin
              << std::setw(10) << xmax
              << std::setw(10) << ymin
              << std::setw(10) << ymax << std::endl;
}

void printFilteringStatistics(const std::map<std::string, int>& filterStats, const std::map<std::string, float>& filterSums) {
    std::cout << "Filtering Statistics:" << std::endl;
    
    // Print header row with all criteria
    std::cout << std::left << std::setw(12) << "Metric";
    for (const auto& stat : filterStats) {
        std::cout << std::setw(12) << stat.first;
    }
    std::cout << std::endl;
    std::cout << std::string(12 + filterStats.size() * 12, '-') << std::endl;
    
    // Print count row
    std::cout << std::left << std::setw(12) << "Count";
    for (const auto& stat : filterStats) {
        std::cout << std::setw(12) << stat.second;
    }
    std::cout << std::endl;
    
    // Print average row
    std::cout << std::left << std::setw(12) << "Avg Value";
    for (const auto& stat : filterStats) {
        float avg = (stat.second > 0) ? (filterSums.at(stat.first) / stat.second) : 0.0f;
        std::cout << std::setw(12) << std::fixed << std::setprecision(1) << avg;
    }
    std::cout << std::endl;
    
    std::cout << std::string(12 + filterStats.size() * 12, '-') << std::endl;
}

void printFittingResults(const cylinder &cyl, int i, const pcl::PointCloud<PointTreeseg>::Ptr &region) {
    // Calculate region centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*region, centroid);
    
    // Print header only for first iteration
    if (i == 0) {
        std::cout << std::left
            << std::setw(8) << "Model"
            << std::setw(10) << "Radius"
            << std::setw(10) << "Length"
            << std::setw(10) << "StepCov"
            << std::setw(10) << "RadRatio"
            << std::setw(10) << "X"
            << std::setw(10) << "Y"
            << std::setw(10) << "RegX"
            << std::setw(10) << "RegY"
            << std::endl;
        std::cout << std::string(88, '-') << std::endl;
    }

    // Print values
    std::cout << std::left
        << std::setw(8) << (cyl.ismodel ? "true" : "false")
        << std::setw(10) << std::fixed << std::setprecision(3) << cyl.rad
        << std::setw(10) << cyl.len
        << std::setw(10) << cyl.stepcov
        << std::setw(10) << cyl.radratio
        << std::setw(10) << cyl.x
        << std::setw(10) << cyl.y
        << std::setw(10) << centroid[0]
        << std::setw(10) << centroid[1]
        << std::endl;
}
