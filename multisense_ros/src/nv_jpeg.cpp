/**
 * @file nv_jpeg.cpp
 *
 * NVJPG through libnvmm_jpeg, as the SEAL OrbbecSDK_ROS2 fork's gpu/nv_jpeg.cpp
 * reaches it beside OpenCV in the Orbbec's container; the encode follows the
 * Jetson Multimedia API's NvJpegEncoder::encodeFromFd. Built only where that API
 * is installed, against its own libjpeg 9 headers, which no other file sees.
 **/

#include <multisense_ros/aux_jpeg.h>

#include <dlfcn.h>

#include <csetjmp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>

// After the system headers: libjpeg's headers need FILE and size_t.
#include <jpeglib.h>
#include <nvbufsurface.h>

namespace multisense_ros {

namespace {

// Opened local and called through this table, libnvmm_jpeg serves only this file;
// its DT_SYMBOLIC keeps its own internal calls inside it. Every other jpeg_* caller in
// the process keeps libjpeg-turbo.
struct LibNvJpeg
{
    decltype(&jpeg_std_error) std_error = nullptr;
    decltype(&jpeg_CreateCompress) create_compress = nullptr;
    decltype(&jpeg_destroy_compress) destroy_compress = nullptr;
    decltype(&jpeg_suppress_tables) suppress_tables = nullptr;
    decltype(&jpeg_mem_dest) mem_dest = nullptr;
    decltype(&jpeg_set_defaults) set_defaults = nullptr;
    decltype(&jpeg_set_quality) set_quality = nullptr;
    decltype(&jpeg_set_hardware_acceleration_parameters_enc) set_hw_enc = nullptr;
    decltype(&jpeg_start_compress) start_compress = nullptr;
    decltype(&jpeg_write_raw_data) write_raw_data = nullptr;
    decltype(&jpeg_finish_compress) finish_compress = nullptr;

    static const LibNvJpeg *get(std::string *error)
    {
        static std::once_flag once;
        static LibNvJpeg lib;
        static std::string load_error;
        std::call_once(once, [] { load_error = lib.load(); });
        if (!load_error.empty())
        {
            *error = load_error;
            return nullptr;
        }
        return &lib;
    }

private:
    template <typename F>
    static bool bind(void *handle, const char *name, F *fn, std::string *missing)
    {
        *fn = reinterpret_cast<F>(dlsym(handle, name));
        if (*fn == nullptr)
        {
            *missing = name;
        }
        return *fn != nullptr;
    }

    std::string load()
    {
        void *handle = dlopen("libnvmm_jpeg.so", RTLD_NOW | RTLD_LOCAL);
        if (handle == nullptr)
        {
            const char *reason = dlerror();
            return std::string{"dlopen libnvmm_jpeg.so: "} + (reason ? reason : "unknown error");
        }
        std::string missing;
        const bool ok = bind(handle, "jpeg_std_error", &std_error, &missing) &&
                        bind(handle, "jpeg_CreateCompress", &create_compress, &missing) &&
                        bind(handle, "jpeg_destroy_compress", &destroy_compress, &missing) &&
                        bind(handle, "jpeg_suppress_tables", &suppress_tables, &missing) &&
                        bind(handle, "jpeg_mem_dest", &mem_dest, &missing) &&
                        bind(handle, "jpeg_set_defaults", &set_defaults, &missing) &&
                        bind(handle, "jpeg_set_quality", &set_quality, &missing) &&
                        bind(handle, "jpeg_set_hardware_acceleration_parameters_enc", &set_hw_enc, &missing) &&
                        bind(handle, "jpeg_start_compress", &start_compress, &missing) &&
                        bind(handle, "jpeg_write_raw_data", &write_raw_data, &missing) &&
                        bind(handle, "jpeg_finish_compress", &finish_compress, &missing);
        return ok ? std::string{} : "libnvmm_jpeg.so lacks " + missing;
    }
};

struct ErrorManager
{
    jpeg_error_mgr pub;
    std::jmp_buf jump;
    char message[JMSG_LENGTH_MAX];
};

// libjpeg's error_exit must not return, and C++ exceptions cannot cross its C frames.
void error_exit(j_common_ptr cinfo)
{
    auto *err = reinterpret_cast<ErrorManager *>(cinfo->err);
    (*cinfo->err->format_message)(cinfo, err->message);
    std::longjmp(err->jump, 1);
}

// Warnings would go to stderr; a failure reaches the caller through error_exit.
void quiet(j_common_ptr, int)
{
}

}

struct NvJpegEncoder::Impl
{
    const LibNvJpeg *lib = nullptr;
    int quality = 0;
    int width = 0;
    int height = 0;
    NvBufSurface *surface = nullptr;
    bool mapped = false;

    jpeg_compress_struct cinfo{};
    ErrorManager err{};
    bool encoder_ready = false;

    std::vector<unsigned char> out_buffer;
    // Members, not locals: they must hold their values across a longjmp out of libjpeg.
    unsigned char *out = nullptr;
    size_t out_size = 0;

    ~Impl()
    {
        if (encoder_ready)
        {
            lib->destroy_compress(&cinfo);
        }
        if (mapped)
        {
            NvBufSurfaceUnMap(surface, 0, -1);
        }
        if (surface != nullptr)
        {
            NvBufSurfaceDestroy(surface);
        }
    }

    bool create_encoder()
    {
        std::memset(&cinfo, 0, sizeof(cinfo));
        std::memset(&err, 0, sizeof(err));
        cinfo.err = lib->std_error(&err.pub);
        err.pub.error_exit = error_exit;
        err.pub.emit_message = quiet;
        if (setjmp(err.jump) != 0)
        {
            return false;
        }
        lib->create_compress(&cinfo, JPEG_LIB_VERSION, sizeof(cinfo));
        lib->suppress_tables(&cinfo, TRUE);
        encoder_ready = true;
        return true;
    }

    void release_out()
    {
        // libjpeg moves to a buffer of its own when ours is too small.
        if (out != nullptr && out != out_buffer.data())
        {
            std::free(out);
        }
        out = nullptr;
    }

    // The surface's frame into jpeg; false with err.message set when libjpeg fails.
    bool encode_surface(std::vector<uint8_t> *jpeg)
    {
        if (!encoder_ready && !create_encoder())
        {
            return false;
        }
        out = out_buffer.data();
        out_size = out_buffer.size();
        err.message[0] = '\0';
        if (setjmp(err.jump) != 0)
        {
            // The object is unusable after a longjmp out of libjpeg.
            lib->destroy_compress(&cinfo);
            encoder_ready = false;
            release_out();
            return false;
        }
        // libnvmm_jpeg can report a failed start through msg_code alone, without error_exit,
        // as the Multimedia API's encodeFromFd checks.
        err.pub.msg_code = 0;
        lib->mem_dest(&cinfo, &out, &out_size);
        cinfo.fd = static_cast<int>(surface->surfaceList[0].bufferDesc);
        cinfo.IsVendorbuf = TRUE;
        cinfo.raw_data_in = TRUE;
        cinfo.in_color_space = JCS_YCbCr;
        lib->set_defaults(&cinfo);
        lib->set_quality(&cinfo, quality, TRUE);
        lib->set_hw_enc(&cinfo, TRUE, static_cast<unsigned int>(out_size), 0, 0);
        cinfo.in_color_space = JCS_YCbCr;
        lib->start_compress(&cinfo, FALSE);
        if (err.pub.msg_code != 0)
        {
            (*err.pub.format_message)(reinterpret_cast<j_common_ptr>(&cinfo), err.message);
            lib->destroy_compress(&cinfo);
            encoder_ready = false;
            release_out();
            return false;
        }
        lib->write_raw_data(&cinfo, nullptr, 0);
        lib->finish_compress(&cinfo);
        try
        {
            jpeg->assign(out, out + out_size);
        }
        catch (...)
        {
            release_out();
            throw;
        }
        release_out();
        return true;
    }
};

NvJpegEncoder::NvJpegEncoder(std::unique_ptr<Impl> impl):
    impl_(std::move(impl))
{
}

NvJpegEncoder::~NvJpegEncoder() = default;

std::unique_ptr<NvJpegEncoder> NvJpegEncoder::create(int quality, int width, int height, std::string *error)
{
    if (quality < 1 || quality > 100)
    {
        *error = "quality must be 1-100";
        return nullptr;
    }
    if (width < 2 || height < 2 || width % 2 != 0 || height % 2 != 0)
    {
        *error = "4:2:0 needs an even width and height, got " + std::to_string(width) + "x" + std::to_string(height);
        return nullptr;
    }
    const LibNvJpeg *lib = LibNvJpeg::get(error);
    if (lib == nullptr)
    {
        return nullptr;
    }
    auto impl = std::make_unique<Impl>();
    impl->lib = lib;
    impl->quality = quality;
    impl->width = width;
    impl->height = height;

    // Full-range YCbCr, as JPEG is: the planes go in as they are, with no range conversion.
    NvBufSurfaceCreateParams params{};
    params.gpuId = 0;
    params.width = static_cast<uint32_t>(width);
    params.height = static_cast<uint32_t>(height);
    params.colorFormat = NVBUF_COLOR_FORMAT_YUV420_ER;
    params.layout = NVBUF_LAYOUT_PITCH;
    params.memType = NVBUF_MEM_SURFACE_ARRAY;
    if (NvBufSurfaceCreate(&impl->surface, 1, &params) != 0)
    {
        impl->surface = nullptr;
        *error = "NvBufSurfaceCreate failed";
        return nullptr;
    }
    impl->surface->numFilled = 1;
    const NvBufSurfacePlaneParams &planes = impl->surface->surfaceList[0].planeParams;
    if (planes.num_planes != 3 || planes.width[0] != params.width || planes.height[0] != params.height ||
        planes.width[1] != params.width / 2 || planes.height[1] != params.height / 2 ||
        planes.width[2] != params.width / 2 || planes.height[2] != params.height / 2)
    {
        *error = "the hardware surface is not three planes of a 4:2:0 frame this size";
        return nullptr;
    }
    // Mapped once and kept: the CPU writes each frame straight into the engine's surface.
    if (NvBufSurfaceMap(impl->surface, 0, -1, NVBUF_MAP_WRITE) != 0)
    {
        *error = "NvBufSurfaceMap failed";
        return nullptr;
    }
    impl->mapped = true;
    if (!impl->create_encoder())
    {
        *error = std::string{"NVJPG encoder init: "} + impl->err.message;
        return nullptr;
    }
    impl->out_buffer.resize(static_cast<size_t>(width) * height * 3 / 2);
    return std::unique_ptr<NvJpegEncoder>(new NvJpegEncoder(std::move(impl)));
}

std::shared_ptr<std::vector<uint8_t>> NvJpegEncoder::encode(const uint8_t *luma,
                                                            const uint8_t *chroma,
                                                            int width,
                                                            int height)
{
    Impl &s = *impl_;
    if (width != s.width || height != s.height)
    {
        throw std::invalid_argument("NVJPG encoder made for " + std::to_string(s.width) + "x" +
                                    std::to_string(s.height) + ", handed " + std::to_string(width) + "x" +
                                    std::to_string(height));
    }
    NvBufSurfaceParams &surface = s.surface->surfaceList[0];
    fill_i420(luma, chroma, width, height,
              static_cast<uint8_t *>(surface.mappedAddr.addr[0]), surface.planeParams.pitch[0],
              static_cast<uint8_t *>(surface.mappedAddr.addr[1]), surface.planeParams.pitch[1],
              static_cast<uint8_t *>(surface.mappedAddr.addr[2]), surface.planeParams.pitch[2]);
    if (NvBufSurfaceSyncForDevice(s.surface, 0, -1) != 0)
    {
        throw std::runtime_error("NVJPG: NvBufSurfaceSyncForDevice failed");
    }
    auto jpeg = std::make_shared<std::vector<uint8_t>>();
    if (!s.encode_surface(jpeg.get()))
    {
        throw std::runtime_error(std::string{"NVJPG: "} + s.err.message);
    }
    return jpeg;
}

}
