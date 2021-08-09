#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <vector>

#include "cwipc_codec/api.h"

#define COUNT 50

std::vector<int> all_octree_bits { 6, 9 };
std::vector<int> all_jpeg_quality { 85 };
std::vector<int> all_tilenumber { 1, 2, 3, 4 };

int main(int argc, char** argv)
{
	uint64_t timestamp = 0LL;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " pointcloudfile.ply" << std::endl;
        return 2;
    }
    //
    // Read pointcloud file
    //
    char *errorMessage = NULL;
    cwipc *pc = cwipc_read(argv[1], 0LL, &errorMessage, CWIPC_API_VERSION);

    if (pc == NULL || errorMessage) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[1] << ": " << errorMessage << std::endl;
        return 1;
    }
    std::cerr << argv[0] << ": Read pointcloud, " << pc->count() << " points, " << pc->get_uncompressed_size() << " bytes (uncompressed)" << std::endl;
    //
    // Compress
    //
    cwipc_encoder_params param;
	param.do_inter_frame = false;
	param.gop_size = 1;
	param.exp_factor = 1.0;
	param.octree_bits = 0;
	param.jpeg_quality = 0;
	param.macroblock_size = 16;
	param.tilenumber = 0;
    param.voxelsize = 0;

	char *errorString;
    cwipc_encodergroup *encodergroup = cwipc_new_encodergroup(&errorString, CWIPC_API_VERSION);
    if (encodergroup == NULL) {
        std::cerr << argv[0] << ": Could not create encodergroup: " << errorString << std::endl;
        return 1;
    }
    std::vector<cwipc_encoder *> encoders;
    for(int octree_bits : all_octree_bits) {
        for(int jpeg_quality : all_jpeg_quality) {
            for(int tilenumber : all_tilenumber) {
                param.octree_bits = octree_bits;
                param.jpeg_quality = jpeg_quality;
                param.tilenumber = tilenumber;
                cwipc_encoder *e = encodergroup->addencoder(CWIPC_ENCODER_PARAM_VERSION, &param, &errorString);
                if (e == NULL) {
                    std::cerr << argv[0] << ": Could not create encoder: " << errorString << std::endl;
                    return 1;
                }
                encoders.push_back(e);
            }
        }
    }
    
    auto t0_cpu = std::clock();
    auto t0_wall = std::chrono::high_resolution_clock::now();
    size_t totalBufSize = 0;
    int totalCount = 0;
    
    for (int i=0; i<COUNT; i++) {
    	encodergroup->feed(pc);
        for(cwipc_encoder *e: encoders) {
            bool ok = e->available(true);
            if (!ok) {
                std::cerr << argv[0] << ": Encoder did not create compressed data" << std::endl;
                return 1;
            }
            size_t bufSize = e->get_encoded_size();
            totalBufSize += bufSize;
            totalCount++;
            char *buffer = (char *)malloc(bufSize);
            if (buffer == NULL) {
                std::cerr << argv[0] << ": Could not allocate " << bufSize << " bytes." << std::endl;
                return 1;
            }
            ok = e->copy_data(buffer, bufSize);
            if (!ok) {
                std::cerr << argv[0] << ": Encoder could not copy compressed data" << std::endl;
                return 1;
            }
            free(buffer);
        }
    }
    
    auto t1_cpu = std::clock();
    auto t1_wall = std::chrono::high_resolution_clock::now();
    
    double delta_cpu = (double)(t1_cpu - t0_cpu) *1000.0 / CLOCKS_PER_SEC;
    double delta_wall = std::chrono::duration<double, std::milli>(t1_wall - t0_wall).count();
	delta_cpu /= COUNT;
	delta_wall /= COUNT;
    std::cerr << argv[0] << ": Compressed " << COUNT << " times, output: " << totalBufSize/COUNT << " bytes per input cloud, " << totalBufSize/totalCount << " average per output packet" << std::endl;
    std::cerr << std::fixed << std::setprecision(2) << argv[0] << ": per iteration: cpu: " <<  delta_cpu << " ms, real: " << delta_wall << "ms" << std::endl;

    pc->free();	// After feeding the pointcloud into the encoder we can free it.
    encodergroup->free(); // We don't need the encoder anymore.
    return 0;
}

