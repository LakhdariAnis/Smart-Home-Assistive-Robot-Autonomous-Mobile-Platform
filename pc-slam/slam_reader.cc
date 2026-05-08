/**
 * slam_reader.cc — POSIX shared memory consumer → ORB-SLAM3 monocular tracking
 *
 * Reads BGR frames written by bridge.py from shared memory "orbframe":
 *   [0:8]   uint64  sequence number (little-endian)
 *   [8:16]  double  timestamp (seconds since epoch, little-endian)
 *   [16:]   uint8   raw BGR frame  (640 × 480 × 3 = 921 600 bytes)
 *
 * Build:  see CMakeLists_slam_reader.txt
 * Run:    ./slam_reader <ORBvoc.txt> <picam.yaml> [--no-viewer]
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#ifdef WITH_ZMQ
#include <zmq.h>
#endif

// ORB-SLAM3 v1.0+ returns Sophus::SE3f from TrackMonocular (not cv::Mat)
#include <sophus/se3.hpp>
#include "System.h"

// ── Shared memory constants (must match bridge.py) ────────────────────────
static constexpr int    WIDTH        = 640;
static constexpr int    HEIGHT       = 480;
static constexpr int    CHANNELS     = 3;
static constexpr size_t FRAME_BYTES  = WIDTH * HEIGHT * CHANNELS;
static constexpr size_t HEADER_BYTES = 8 + 8;
static constexpr size_t SHM_SIZE     = HEADER_BYTES + FRAME_BYTES;
static const char*      SHM_NAME     = "/orbframe";

// ── Helpers ───────────────────────────────────────────────────────────────

static inline uint64_t read_u64(const uint8_t* p)
{
    uint64_t v = 0; std::memcpy(&v, p, 8); return v;
}

static inline double read_f64(const uint8_t* p)
{
    double v = 0.0; std::memcpy(&v, p, 8); return v;
}

// Camera centre in world frame from a world→camera SE3
static Eigen::Vector3f cam_position(const Sophus::SE3f& Tcw)
{
    return Tcw.inverse().translation();
}

static std::string pose_to_json(const Sophus::SE3f& Tcw, double ts,
                                uint64_t seq, bool ok)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "{\"seq\":" << seq << ",\"ts\":" << ts
       << ",\"ok\":"  << (ok ? "true" : "false");
    if (ok) {
        Eigen::Vector3f t = cam_position(Tcw);
        Eigen::Matrix3f R = Tcw.inverse().rotationMatrix();
        ss << ",\"x\":" << t.x() << ",\"y\":" << t.y() << ",\"z\":" << t.z();
        ss << ",\"R\":[";
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                ss << R(r,c); if (r*3+c < 8) ss << ",";
            }
        ss << "]";
    } else {
        ss << ",\"x\":null,\"y\":null,\"z\":null,\"R\":null";
    }
    ss << "}";
    return ss.str();
}

// ── Main ──────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <ORBvoc.txt> <picam.yaml> [--no-viewer]\n";
        return 1;
    }
    const std::string vocab_path  = argv[1];
    const std::string config_path = argv[2];
    bool use_viewer = true;
    for (int i = 3; i < argc; ++i)
        if (std::string(argv[i]) == "--no-viewer") use_viewer = false;

    // ── Open shared memory ─────────────────────────────────────────────────
    std::cout << "[slam_reader] Waiting for shared memory '" << SHM_NAME << "' ...\n";
    int shm_fd = -1;
    for (int attempt = 0; attempt < 300; ++attempt) {
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0);
        if (shm_fd >= 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (shm_fd < 0) { perror("shm_open"); return 1; }

    uint8_t* shm_ptr = static_cast<uint8_t*>(
        mmap(nullptr, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0));
    if (shm_ptr == MAP_FAILED) { perror("mmap"); close(shm_fd); return 1; }
    std::cout << "[slam_reader] Shared memory mapped OK\n";

    // ── ZMQ publisher ──────────────────────────────────────────────────────
#ifdef WITH_ZMQ
    void* zmq_ctx = zmq_ctx_new();
    void* zmq_pub = zmq_socket(zmq_ctx, ZMQ_PUB);
    zmq_bind(zmq_pub, "tcp://*:5557");
    std::cout << "[slam_reader] ZMQ PUB on port 5557\n";
#endif

    // ── Init ORB-SLAM3 ─────────────────────────────────────────────────────
    std::cout << "[slam_reader] Loading ORB-SLAM3 ...\n";
    ORB_SLAM3::System SLAM(vocab_path, config_path,
                           ORB_SLAM3::System::MONOCULAR, use_viewer);
    std::cout << "[slam_reader] Ready.\n";

    // ── Tracking loop ──────────────────────────────────────────────────────
    uint64_t last_seq = UINT64_MAX, frames_tracked = 0, frames_lost = 0;
    auto t_start = std::chrono::steady_clock::now();

    while (true) {
        uint64_t seq, seq2;
        double   ts;
        cv::Mat frame(HEIGHT, WIDTH, CV_8UC3);

        do {
            seq = read_u64(shm_ptr);
            ts  = read_f64(shm_ptr + 8);
            std::memcpy(frame.data, shm_ptr + HEADER_BYTES, FRAME_BYTES);
            seq2 = read_u64(shm_ptr);
        } while (seq != seq2);

        if (seq == last_seq) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        last_seq = seq;

        // bridge.py writes BGR; yaml Camera.RGB:1 expects RGB
        cv::Mat frame_rgb;
        cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);

        // TrackMonocular returns Sophus::SE3f (world→camera)
        Sophus::SE3f Tcw = SLAM.TrackMonocular(frame_rgb, ts);

        // Tracking lost = SE3 is identity (no map points matched)
        Eigen::Vector3f t = cam_position(Tcw);
        bool ok = (SLAM.GetTrackingState() == 2);

        if (ok) {
            ++frames_tracked;
            std::cout << std::fixed << std::setprecision(4)
                      << "[SLAM] seq=" << seq << "  ts=" << ts
                      << "  x=" << t.x() << "  y=" << t.y() << "  z=" << t.z()
                      << "\n";
        } else {
            ++frames_lost;
            std::cout << "[SLAM] seq=" << seq << "  TRACKING LOST\n";
        }

#ifdef WITH_ZMQ
        std::string json = pose_to_json(Tcw, ts, seq, ok);
        zmq_send(zmq_pub, json.c_str(), json.size(), ZMQ_NOBLOCK);
#endif

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t_start).count();
        if (elapsed > 5.0) {
            std::cout << "[slam_reader] "
                      << frames_tracked << " tracked  "
                      << frames_lost    << " lost  "
                      << std::setprecision(1)
                      << (frames_tracked + frames_lost) / elapsed << " fps\n";
            frames_tracked = frames_lost = 0;
            t_start = now;
        }
    }

    SLAM.Shutdown();
    munmap(shm_ptr, SHM_SIZE);
    close(shm_fd);
#ifdef WITH_ZMQ
    zmq_close(zmq_pub); zmq_ctx_destroy(zmq_ctx);
#endif
    return 0;
}
