#include "treeseg.h"
#include <pcl/io/pcd_io.h>
#include <iostream>

int main(int argc, char **argv)
{
    // --- 1. Check argument count ---
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0] 
                  << " <resolution> <percentile> <zmin> <zmax> <input_file>" 
                  << std::endl;
        return 1;
    }

    std::vector<std::string> args(argv + 1, argv + argc);

    // --- 2. Convert arguments safely ---
    float resolution, percentile, zmin, zmax;
    try {
        resolution = std::stof(args[0]);
        percentile = std::stof(args[1]);
        zmin = std::stof(args[2]);
        zmax = std::stof(args[3]);
    } catch (const std::exception &e) {
        std::cerr << "Error converting arguments to float: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "[DEBUG] resolution=" << resolution 
              << ", percentile=" << percentile 
              << ", zmin=" << zmin 
              << ", zmax=" << zmax << std::endl;

    // --- 3. Initialize point cloud ---
    pcl::PointCloud<PointTreeseg>::Ptr plotcloud(new pcl::PointCloud<PointTreeseg>);
    if (!plotcloud) {
        std::cerr << "Failed to allocate plotcloud!" << std::endl;
        return 1;
    }

    pcl::PCDWriter writer;

    // --- 4. Get file ID ---
    std::vector<std::string> id;
    try {
        id = getFileID(args[4]);
    } catch (const std::exception &e) {
        std::cerr << "Error in getFileID: " << e.what() << std::endl;
        return 1;
    }
    if (id.empty()) {
        std::cerr << "getFileID returned empty list!" << std::endl;
        return 1;
    }
    std::cout << "[DEBUG] id[0] = " << id[0] << std::endl;

    // --- 5. Read tiles ---
    try {
        readTiles(args, plotcloud);
    } catch (const std::exception &e) {
        std::cerr << "Error in readTiles: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "[DEBUG] plotcloud size = " << plotcloud->size() << std::endl;

    // --- 6. DTM and slicing ---
    std::stringstream ss;
    ss << id[0] << ".slice.pcd";
    pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
    std::vector<std::vector<float>> dem;

    try {
        dem = getDtmAndSlice(plotcloud, resolution, percentile, zmin, zmax, slice);
    } catch (const std::exception &e) {
        std::cerr << "Error in getDtmAndSlice: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "[DEBUG] DTM/Slice generated, size = " << dem.size() << std::endl;
    for (int j = 0; j < dem.size(); j++) {
        if (dem[j].size() >= 3)
            std::cout << dem[j][0] << " " << dem[j][1] << " " << dem[j][2] << std::endl;
        else
            std::cout << "[WARNING] dem[" << j << "] has insufficient data!" << std::endl;
    }

    // --- 7. Write slice PCD file ---
    try {
        writer.write(ss.str(), *slice, true);
        std::cout << "[DEBUG] Wrote slice to " << ss.str() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error writing PCD: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
