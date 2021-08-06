#include <iostream>
#include <fstream>
#include <chrono>

#include "cwipc_codec/api.h"

#define COUNT 10
#define OCTREE_BITS 9
#define JPEG_QUALITY 85
#define TILENUMBER -1

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
	param.octree_bits = OCTREE_BITS;
	param.jpeg_quality = JPEG_QUALITY;
	param.macroblock_size = 16;
	param.tilenumber = TILENUMBER;

	char *errorString;
    cwipc_encoder *encoder = cwipc_new_encoder(CWIPC_ENCODER_PARAM_VERSION, &param, &errorString, CWIPC_API_VERSION);
    if (encoder == NULL) {
    	std::cerr << argv[0] << ": Could not create encoder: " << errorString << std::endl;
    	return 1;
    }
    
    auto t0_cpu = std::clock();
    auto t0_wall = std::chrono::high_resolution_clock::now();
    size_t bufSize;
    
    for (int i=0; i<COUNT; i++) {
    	encoder->feed(pc);
		bool ok = encoder->available(true);
		if (!ok) {
			std::cerr << argv[0] << ": Encoder did not create compressed data" << std::endl;
			return 1;
		}
        bufSize = encoder->get_encoded_size();
		char *buffer = (char *)malloc(bufSize);
		if (buffer == NULL) {
			std::cerr << argv[0] << ": Could not allocate " << bufSize << " bytes." << std::endl;
			return 1;
		}
		ok = encoder->copy_data(buffer, bufSize);
		if (!ok) {
			std::cerr << argv[0] << ": Encoder could not copy compressed data" << std::endl;
			return 1;
		}
		free(buffer);
    }
    
    auto t1_cpu = std::clock();
    auto t1_wall = std::chrono::high_resolution_clock::now();
    
    double delta_cpu = (double)(t1_cpu - t0_cpu) *1000.0 / CLOCKS_PER_SEC;
    double delta_wall = std::chrono::duration<double, std::milli>(t1_wall - t0_wall).count();
	delta_cpu /= COUNT;
	delta_wall /= COUNT;
    std::cerr << argv[0] << ": Compressed " << COUNT << " times, output: " << bufSize << " bytes" << std::endl;
    std::cerr << std::fixed << std::setprecision(2) << argv[0] << ": per iteration: cpu: " <<  delta_cpu << " ms, real: " << delta_wall << "ms" << std::endl;

    pc->free();	// After feeding the pointcloud into the encoder we can free it.
    encoder->free(); // We don't need the encoder anymore.
    return 0;
}

