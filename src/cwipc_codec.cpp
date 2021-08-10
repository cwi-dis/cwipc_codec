// cwi_encode.cpp : Defines the exported functions for the DLL application.
//
#include <cstdint>
#include <chrono>
#include <sstream>
#include <thread>
#include <condition_variable>
#ifdef WIN32
#define _CWIPC_CODEC_EXPORT __declspec(dllexport)
#else
#define _CWIPC_CODEC_EXPORT
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_codec/api.h"
#include "readerwriterqueue.h"
#include <pcl/point_cloud.h>
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>

// xxxjack using namespace std;

// Some parameters that will eventually be customizable again
const int num_threads = 1;
const bool downdownsampling = false;
const int iframerate = 1;

class cwipc_encoder_impl : public cwipc_encoder
{
public:
    cwipc_encoder_impl(cwipc_encoder_params *_params)
    :	m_encoder(NULL),
    	m_params(*_params),
        m_result(NULL),
        m_result_size(0),
        m_remaining_frames_in_gop(0),
        m_alive(true),
        m_queue_tilefilter(1),
        m_queue_encoder(1)
    {

	}
    
    ~cwipc_encoder_impl() {}
    
    void free() {
        if (m_result) ::free(m_result);
        m_encoder = NULL;
        m_result = NULL;
        m_result_size = 0;
        m_alive = false;
        m_result_cv.notify_all();
    };
    
    void close() {
        m_alive = false;
        m_result_cv.notify_all();
    }
    
    bool eof() { return !m_alive; };
    
    bool available(bool wait) {
    	std::unique_lock<std::mutex> lock(m_result_mutex);
    	if (wait) m_result_cv.wait(lock, [this]{return m_result != NULL || !m_alive; });
    	return m_result != NULL; 
    };
    
    bool at_gop_boundary() { return m_remaining_frames_in_gop <= 0; };

    void feed(cwipc *pc) {
        if (!m_alive) {
            std::cerr << "cwipc_encoder: feed() after close()" << std::endl;
            return;
        }
        m_queue_tilefilter.enqueue(pc);
        bool ok = _run_single_tilefilter();
        assert(ok);
        ok = _run_single_encode();
        assert(ok);
    };
    
    size_t get_encoded_size() { 
    	// xxxjack note that this is not thread-safe: before the copy_data()
    	// call another thread could have grabbed the data.
    	return m_result_size; 
    };
    
    bool copy_data(void *buffer, size_t bufferSize) {
    	std::lock_guard<std::mutex> lock(m_result_mutex);
        if (m_result == NULL || bufferSize < m_result_size) return false;
        memcpy(buffer, m_result, m_result_size);
        ::free(m_result);
        m_result = NULL;
        m_result_size = 0;
        return true;
    };
    
private:
	void alloc_encoder() {
		// xxxjack note that feed() and alloc_encoder() are not thread-safe.
        double point_resolution = std::pow ( 2.0, -1.0 * m_params.octree_bits);
        double octree_resolution = std::pow ( 2.0, -1.0 * m_params.octree_bits);
        m_encoder = NULL;
        m_encoder = pcl::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > (
            new pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> (
                  pcl::io::MANUAL_CONFIGURATION,
                  false,
                  point_resolution,
                  octree_resolution,
                  downdownsampling, // no intra voxel coding in this first version of the codec
                  iframerate, // i_frame_rate,
                  true, // do color encoding
                  8, // color bits
                  1, // color coding type
                  false, // centroid computation - expensive with little added value
                  false, // scalable bitstream - not implemented
                  false, // do_connectivity_coding_ not implemented
                  m_params.jpeg_quality,
                  num_threads
                  ));
        m_encoder->setMacroblockSize(m_params.macroblock_size);
        m_remaining_frames_in_gop = m_params.gop_size;
	}
    
    bool _run_single_tilefilter() {
        cwipc *pc = nullptr;
        m_queue_tilefilter.wait_dequeue(pc);
        if (pc == nullptr) return false;
        cwipc *newpc = nullptr;
        // Apply tile filtering, if needed
        if (m_params.tilenumber) {
            newpc = cwipc_tilefilter(pc, m_params.tilenumber);
            if (newpc == NULL) {
                std::cerr << "cwipc_encoder: tilefilter failed" << std::endl;
                return;
            }
        }
        else {
            // Make shallow clone of pc
            newpc = cwipc_from_pcl(pc->access_pcl_pointcloud(), pc->timestamp(), nullptr, CWIPC_API_VERSION);
        }
        pc = nullptr; // Ensure we don't access this anymore.
        m_queue_encoder.enqueue(newpc);
        return true;
    }
	
    bool _run_single_encode() {
        cwipc *newpc = nullptr;
        m_queue_encoder.wait_dequeue(newpc);
        if (newpc == nullptr) return false;
        std::stringstream comp_frame;
        // Allocate an encoder if none is available (we are at the beginning of a GOP)
        if (m_encoder == NULL)
            alloc_encoder();
        bool start_new_gop = m_remaining_frames_in_gop <= 0;
        m_remaining_frames_in_gop = m_params.do_inter_frame ? m_params.gop_size : 1;

        cwipc_pcl_pointcloud pcl_pc = newpc->access_pcl_pointcloud();
        if (pcl_pc->size() == 0) {
            // Special case: if the point cloud is empty we compress a point cloud with a single black point at 0,0,0
            pcl_pc = new_cwipc_pcl_pointcloud();
            cwipc_pcl_point dummyPoint;
            pcl_pc->push_back(dummyPoint);
        }
        // Note by Jack: this is a hack. If the point granularity of the pointcloud
        // is larger (more coarse) than the octree_resolution in the encoder we adapt
        // the encoder parameter, so a reasonable value is transmitted in the output
        // file or packet.
        float cellsize = newpc->cellsize();
        if (cellsize > m_encoder->octreeResolution) {
            m_encoder->octreeResolution = cellsize;
        }
        m_encoder->encodePointCloud(pcl_pc, comp_frame, newpc->timestamp());
        m_remaining_frames_in_gop--;
#ifdef delete_encoder_at_end_of_gop
        /* Free the encoder if we are at the end of the GOP */
        if (m_remaining_frames_in_gop <= 0) {
            m_encoder = NULL;
        }
#endif
        /* Store the result */
        std::lock_guard<std::mutex> lock(m_result_mutex);
        if (m_result) {
            ::free(m_result);
            m_result = NULL;
            m_result_size = 0;
        }
        m_result_size = comp_frame.str().length();
        m_result = (void *)malloc(m_result_size);
        comp_frame.str().copy((char *)m_result, m_result_size);
        m_result_cv.notify_one();
        newpc->free();
        return true;
    }
    
	pcl::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > m_encoder;
    cwipc_encoder_params m_params;
    void *m_result;
    size_t m_result_size;
    std::mutex m_result_mutex;
    std::condition_variable m_result_cv;
    int m_remaining_frames_in_gop;
    bool m_alive;
    moodycamel::BlockingReaderWriterQueue<cwipc *> m_queue_tilefilter;
    moodycamel::BlockingReaderWriterQueue<cwipc *> m_queue_encoder;
};

class cwipc_encodergroup_impl : public cwipc_encodergroup
{
public:
    cwipc_encodergroup_impl()
    :	m_voxelsize(-1)
    {
	}

    ~cwipc_encodergroup_impl() {}

    void free() {
    	for (auto enc: m_encoders) {
    		enc->free();
		}
		m_encoders.clear();
    };
    
    void close() {
        for (auto enc : m_encoders) enc->close();
    }

	void feed(cwipc *pc) {
		cwipc *newpc = NULL;
        if (m_voxelsize > 0) {
            newpc = cwipc_downsample(pc, m_voxelsize);
            if (newpc == NULL) {
                std::cerr << "cwipc_encodergroup: cwipc_downsample failed" << std::endl;
                return;
            }
        } else {
            // Make shallow clone of pc
            newpc = cwipc_from_pcl(pc->access_pcl_pointcloud(), pc->timestamp(), nullptr, CWIPC_API_VERSION);
        }
        pc = nullptr; // Ensure we don't access this anymore.
        if (m_encoders.size() == 1) {
            m_encoders[0]->feed(newpc);
        } else {
            // Fire up threads for all decoders in parallel, and wait for all of them.
            // Very simple to implement, may be good enough.
            std::vector<std::thread*> threads;
            for (auto enc : m_encoders) {
                std::thread* thr = new std::thread([enc, newpc] { enc->feed(newpc); });
                threads.push_back(thr);
            }
            for (std::thread* thr : threads) {
                thr->join();
            }
        }
		newpc->free();
	};

	cwipc_encoder *addencoder(int version, cwipc_encoder_params* params, char **errorMessage) {
		if (m_voxelsize >= 0 && m_voxelsize != params->voxelsize) {
			*errorMessage = (char *)"cwipc_encodergroup: voxelsize must be the same for all encoders";
			return NULL;
		}
		m_voxelsize = params->voxelsize;
		cwipc_encoder_impl *newEncoder = (cwipc_encoder_impl *)cwipc_new_encoder(version, params, errorMessage, CWIPC_API_VERSION);
		if (newEncoder == NULL) return NULL;
		m_encoders.push_back(newEncoder);
		return newEncoder;
	};
private:
	std::vector<cwipc_encoder_impl *> m_encoders;
	float m_voxelsize;
};

class cwipc_decoder_impl : public cwipc_decoder
{
public:
    cwipc_decoder_impl() 
    : m_result(NULL),
      m_alive(true)
    {}
    
    ~cwipc_decoder_impl() {}
    
    void free() {
    	m_alive = false;
    	m_result_cv.notify_all();
    };
    
    void close() { m_alive = false; }
    
    bool eof() { return !m_alive; };
    
    bool available(bool wait) {
    	std::unique_lock<std::mutex> lock(m_result_mutex);
    	if (wait) m_result_cv.wait(lock, [this]{return m_result != NULL || !m_alive; });
    	return m_result != NULL; 
    };
    
    void feed(void *buffer, size_t bufferSize) {
        if (!m_alive) {
            std::cerr << "cwipc_decoder: feed() called after close()" << std::endl;
            return;
        }
        cwipc_encoder_params par;
        //Default codec parameter values set in signals
        par.do_inter_frame = false;
        par.gop_size = 1;
        par.exp_factor = 1.0;
        par.octree_bits = 8;
        par.jpeg_quality = 85;
        par.macroblock_size = 16;
        std::stringstream compfr;
        //Convert buffer to stringstream for encoding
        std::string str((char *)buffer, bufferSize);
        std::stringstream istream(str);
        pcl::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > decoder_V2_;
        decoder_V2_ = pcl::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > (
            new pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> (
                pcl::io::MANUAL_CONFIGURATION,
                false,
                std::pow ( 2.0, -1.0 * par.octree_bits ),
                std::pow ( 2.0, -1.0 * par.octree_bits),
                downdownsampling, // no intra voxel coding in this first version of the codec
                iframerate, // i_frame_rate,
                true,
                8,
                1,
                false,
                false, // not implemented
                false, // do_connectivity_coding_, not implemented
                par.jpeg_quality,
                num_threads
                ));
        cwipc_pcl_pointcloud decpc = new_cwipc_pcl_pointcloud();
        uint64_t timeStamp = 0;
        bool ok = decoder_V2_->decodePointCloud(istream, decpc, timeStamp);
        if (decpc->size() == 1) {
            // Special case: single point (0,0,0,0,0,0) signals an empty pointcloud
            cwipc_pcl_point& pt(decpc->at(0));
            if (abs(pt.x) < 0.01 && abs(pt.y) < 0.01 && abs(pt.z) < 0.01
                && pt.r < 2 && pt.g < 2 && pt.b < 2
                ) {
                decpc->clear();
            }
        }
        std::lock_guard<std::mutex> lock(m_result_mutex);
        if (ok) {
            m_result = cwipc_from_pcl(decpc, timeStamp, NULL, CWIPC_API_VERSION);
            m_result->_set_cellsize(decoder_V2_->octreeResolution);
        } else {
            m_result = NULL;
        }
        m_result_cv.notify_one();
    };
    
    cwipc *get() {
        std::lock_guard<std::mutex> lock(m_result_mutex);
        cwipc *rv = m_result;
        m_result = NULL;
        return rv;
    };
private:
    cwipc *m_result;
    std::mutex m_result_mutex;
    std::condition_variable m_result_cv;
    bool m_alive;
};

cwipc_encoder* cwipc_new_encoder(int version, cwipc_encoder_params *params, char **errorMessage, uint64_t apiVersion) {
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_synthetic: incorrect apiVersion";
		}
		return NULL;
	}
    if (version != CWIPC_ENCODER_PARAM_VERSION) {
    	*errorMessage = (char *)"cwpic_bew_encoder: incorrect encoder param version";
        return NULL;
    }
    if (params == NULL) {
    	static cwipc_encoder_params dft = {false, 1, 1.0, 9, 85, 16, 0, 0};
    	params = &dft;
	}
	// Check parameters for this release
	if (params->do_inter_frame) {
		*errorMessage = (char *)"cwipc_new_encoder: do_inter_frame must be false for this version";
		return NULL;
	}
	if (params->gop_size != 1) {
		*errorMessage = (char *)"cwipc_new_encoder: gop_size must be 1 for this version";
		return NULL;
	}
    return new cwipc_encoder_impl(params);
}

void cwipc_encoder_free(cwipc_encoder *obj) {
    obj->free();
}

bool cwipc_encoder_available(cwipc_encoder *obj, bool wait) {
    return obj->available(wait);
}

bool cwipc_encoder_eof(cwipc_encoder *obj) {
    return obj->eof();
}

void cwipc_encoder_feed(cwipc_encoder *obj, cwipc* pc) {
    obj->feed(pc);
}

void cwipc_encoder_close(cwipc_encoder *obj) {
    obj->close();
}

size_t cwipc_encoder_get_encoded_size(cwipc_encoder *obj) {
    return obj->get_encoded_size();
}

bool cwipc_encoder_copy_data(cwipc_encoder *obj, void *buffer, size_t bufferSize) {
    return obj->copy_data(buffer, bufferSize);
}

bool cwipc_encoder_at_gop_boundary(cwipc_encoder *obj) {
    return obj->at_gop_boundary();
}

cwipc_encodergroup *cwipc_new_encodergroup(char **errorMessage, uint64_t apiVersion) {
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_new_encodergroup: incorrect apiVersion";
		}
		return NULL;
	}
	return new cwipc_encodergroup_impl();
};

void cwipc_encodergroup_free(cwipc_encodergroup *obj) {
    obj->free();
}

cwipc_encoder *cwipc_encodergroup_addencoder(cwipc_encodergroup *obj, int version, cwipc_encoder_params* params, char **errorMessage) {
	return obj->addencoder(version, params, errorMessage);
}

void cwipc_encodergroup_feed(cwipc_encodergroup *obj, cwipc* pc) {
	return obj->feed(pc);
}

void cwipc_encodergroup_close(cwipc_encodergroup *obj) {
    obj->close();
}

cwipc_decoder* cwipc_new_decoder(char **errorMessage, uint64_t apiVersion) {
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_synthetic: incorrect apiVersion";
		}
		return NULL;
	}
    return new cwipc_decoder_impl();
}

void cwipc_decoder_feed(cwipc_decoder *obj, void *buffer, size_t bufferSize) {
    obj->feed(buffer, bufferSize);
}

void cwipc_decoder_close(cwipc_decoder *obj) {
    obj->close();
}
