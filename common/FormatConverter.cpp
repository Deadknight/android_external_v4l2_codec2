// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//#define LOG_NDEBUG 0
#define LOG_TAG "FormatConverter"

#include <v4l2_codec2/common/FormatConverter.h>


#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <mutex>
#include <utils/SystemClock.h>

#include <inttypes.h>

#include <memory>
#include <string>

#include <C2AllocatorGralloc.h>
#include <C2PlatformSupport.h>
#include <android/hardware/graphics/common/1.0/types.h>
#include <inttypes.h>
#include <libyuv.h>
#include <ui/GraphicBuffer.h>
#include <utils/Log.h>

#include <v4l2_codec2/common/VideoTypes.h>  // for HalPixelFormat

using android::hardware::graphics::common::V1_0::BufferUsage;

namespace android {

namespace {
// The constant expression of mapping the pixel format conversion pair (src, dst) to a unique
// integer.
constexpr int convertMap(VideoPixelFormat src, VideoPixelFormat dst) {
    return static_cast<int>(src) * (static_cast<int>(VideoPixelFormat::UNKNOWN) + 1) +
           static_cast<int>(dst);
}

// The helper function to copy a plane pixel by pixel. It assumes bytesPerPixel is 1.
void copyPlaneByPixel(const uint8_t* src, int srcStride, int srcColInc, uint8_t* dst, int dstStride,
                      int dstColInc, int width, int height) {
    for (int row = 0; row < height; row++) {
        const uint8_t* srcRow = src;
        uint8_t* dstRow = dst;
        for (int col = 0; col < width; col++) {
            memcpy(dstRow, srcRow, 1);
            srcRow += srcColInc;
            dstRow += dstColInc;
        }
        src += srcStride;
        dst += dstStride;
    }
}

// ARDA_H264_FULL_DMABUF_PRODUCTION_V1
class ArdaIspFullDmabufConverter {
public:
    ~ArdaIspFullDmabufConverter() { reset(); }

    bool convert(uint64_t frameIndex, int inputDmabufFd, int srcStride,
                 int outputDmabufFd, size_t outputDmabufSize,
                 int width, int height) {
        std::lock_guard<std::mutex> lock(mLock);

        if (inputDmabufFd < 0 || outputDmabufFd < 0 ||
            srcStride < width * 4 || outputDmabufSize == 0 ||
            width != 1920 || height != 1080) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 invalid args input_fd=%d output_fd=%d "
                  "srcStride=%d outputSize=%zu size=%dx%d",
                  inputDmabufFd, outputDmabufFd, srcStride,
                  outputDmabufSize, width, height);
            return false;
        }

        if (!ensureInitialized(width, height, srcStride)) return false;
        if (outputDmabufSize < mCaptureSizeImage) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 output DMABUF too small size=%zu required=%zu",
                  outputDmabufSize, mCaptureSizeImage);
            return false;
        }

        const size_t inputBytesUsed = static_cast<size_t>(srcStride) * height;

        if (!queueCapture(outputDmabufFd, outputDmabufSize) ||
            !queueOutput(inputDmabufFd, inputBytesUsed)) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 frame=%" PRIu64 " QBUF failed", frameIndex);
            reset();
            return false;
        }

        if (!mStreaming) {
            if (!streamOn(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ||
                !streamOn(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)) {
                ALOGE("ARDA_FULL_DMABUF_ISP_V1 STREAMON failed");
                reset();
                return false;
            }
            mStreaming = true;
        }

        if (!waitForBothDequeues()) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 frame=%" PRIu64 " DQBUF timeout/failure",
                  frameIndex);
            reset();
            return false;
        }

        return true;
    }

private:
    static int xioctl(int fd, unsigned long request, void* arg) {
        int ret;
        do {
            ret = ioctl(fd, request, arg);
        } while (ret < 0 && errno == EINTR);
        return ret;
    }

    bool setFormat(v4l2_buf_type type, uint32_t pixfmt, int width, int height,
                   uint32_t requestedBpl, uint32_t* negotiatedBpl,
                   size_t* sizeImage) {
        struct v4l2_format fmt = {};
        fmt.type = type;
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = pixfmt;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
        if (requestedBpl) fmt.fmt.pix_mp.plane_fmt[0].bytesperline = requestedBpl;

        if (xioctl(mFd, VIDIOC_S_FMT, &fmt) != 0) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 VIDIOC_S_FMT type=%u pixfmt=0x%08x "
                  "failed errno=%d (%s)", type, pixfmt, errno, strerror(errno));
            return false;
        }
        if (fmt.fmt.pix_mp.pixelformat != pixfmt || fmt.fmt.pix_mp.num_planes != 1) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 unexpected format type=%u "
                  "requested=0x%08x got=0x%08x planes=%u",
                  type, pixfmt, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.num_planes);
            return false;
        }

        *negotiatedBpl = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
        *sizeImage = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
        return true;
    }

    bool requestDmabuf(v4l2_buf_type type) {
        struct v4l2_requestbuffers req = {};
        req.count = 1;
        req.type = type;
        req.memory = V4L2_MEMORY_DMABUF;
        if (xioctl(mFd, VIDIOC_REQBUFS, &req) != 0 || req.count < 1) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 VIDIOC_REQBUFS type=%u DMABUF failed "
                  "errno=%d (%s)", type, errno, strerror(errno));
            return false;
        }
        return true;
    }

    bool ensureInitialized(int width, int height, int srcStride) {
        if (mFd >= 0 && mWidth == width && mHeight == height &&
            mSrcStride == srcStride) {
            return true;
        }

        reset();
        mFd = open("/dev/video12", O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (mFd < 0) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 open /dev/video12 failed errno=%d (%s)",
                  errno, strerror(errno));
            return false;
        }

#ifdef V4L2_PIX_FMT_BGR32
        const uint32_t rgbFormat = V4L2_PIX_FMT_BGR32;
#else
        ALOGE("ARDA_FULL_DMABUF_ISP_V1 build lacks V4L2_PIX_FMT_BGR32");
        reset();
        return false;
#endif

        if (!setFormat(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, rgbFormat,
                       width, height, static_cast<uint32_t>(srcStride),
                       &mOutputBytesPerLine, &mOutputSizeImage) ||
            !setFormat(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_PIX_FMT_NV12,
                       width, height, width,
                       &mCaptureBytesPerLine, &mCaptureSizeImage) ||
            !requestDmabuf(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) ||
            !requestDmabuf(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)) {
            reset();
            return false;
        }

        if (mOutputBytesPerLine != static_cast<uint32_t>(srcStride) ||
            mCaptureBytesPerLine != static_cast<uint32_t>(width)) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 invalid stride src=%d output=%u capture=%u",
                  srcStride, mOutputBytesPerLine, mCaptureBytesPerLine);
            reset();
            return false;
        }

        mWidth = width;
        mHeight = height;
        mSrcStride = srcStride;
        ALOGI("ARDA_FULL_DMABUF_ISP_INIT_V1 device=/dev/video12 input=DMABUF_BGR4 "
              "output=DMABUF_NV12 size=%dx%d input_bpl=%u input_size=%zu "
              "capture_bpl=%u capture_size=%zu",
              width, height, mOutputBytesPerLine, mOutputSizeImage,
              mCaptureBytesPerLine, mCaptureSizeImage);
        return true;
    }

    bool queueCapture(int dmabufFd, size_t dmabufSize) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane plane = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_DMABUF;
        buf.index = 0;
        buf.length = 1;
        buf.m.planes = &plane;
        plane.m.fd = dmabufFd;
        plane.length = static_cast<uint32_t>(dmabufSize);
        plane.bytesused = 0;
        plane.data_offset = 0;
        if (xioctl(mFd, VIDIOC_QBUF, &buf) != 0) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 VIDIOC_QBUF CAPTURE fd=%d length=%u "
                  "failed errno=%d (%s)",
                  dmabufFd, plane.length, errno, strerror(errno));
            return false;
        }
        return true;
    }

    bool queueOutput(int dmabufFd, size_t bytesUsed) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane plane = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        buf.memory = V4L2_MEMORY_DMABUF;
        buf.index = 0;
        buf.length = 1;
        buf.m.planes = &plane;
        plane.m.fd = dmabufFd;
        plane.length = static_cast<uint32_t>(std::max(mOutputSizeImage, bytesUsed));
        plane.bytesused = static_cast<uint32_t>(bytesUsed);
        plane.data_offset = 0;
        if (xioctl(mFd, VIDIOC_QBUF, &buf) != 0) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 VIDIOC_QBUF OUTPUT fd=%d length=%u "
                  "bytesused=%zu failed errno=%d (%s)",
                  dmabufFd, plane.length, bytesUsed, errno, strerror(errno));
            return false;
        }
        return true;
    }

    bool streamOn(v4l2_buf_type type) {
        int t = static_cast<int>(type);
        if (xioctl(mFd, VIDIOC_STREAMON, &t) != 0) {
            ALOGE("ARDA_FULL_DMABUF_ISP_V1 VIDIOC_STREAMON type=%u failed errno=%d (%s)",
                  type, errno, strerror(errno));
            return false;
        }
        return true;
    }

    bool dequeueOne(v4l2_buf_type type, bool* done) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane plane = {};
        buf.type = type;
        buf.memory = V4L2_MEMORY_DMABUF;
        buf.length = 1;
        buf.m.planes = &plane;

        if (xioctl(mFd, VIDIOC_DQBUF, &buf) == 0) {
            *done = true;
            return true;
        }
        if (errno == EAGAIN) return true;

        ALOGE("ARDA_FULL_DMABUF_ISP_V1 VIDIOC_DQBUF type=%u failed errno=%d (%s)",
              type, errno, strerror(errno));
        return false;
    }

    bool waitForBothDequeues() {
        bool outputDone = false;
        bool captureDone = false;
        const int64_t deadlineNs =
                android::elapsedRealtimeNano() + 1000LL * 1000LL * 1000LL;

        while (!outputDone || !captureDone) {
            if (!outputDone &&
                !dequeueOne(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, &outputDone)) {
                return false;
            }
            if (!captureDone &&
                !dequeueOne(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, &captureDone)) {
                return false;
            }
            if (outputDone && captureDone) return true;

            const int64_t nowNs = android::elapsedRealtimeNano();
            if (nowNs >= deadlineNs) return false;
            int timeoutMs = static_cast<int>((deadlineNs - nowNs) / 1000000LL);
            if (timeoutMs < 1) timeoutMs = 1;

            struct pollfd pfd = {};
            pfd.fd = mFd;
            pfd.events = POLLIN | POLLOUT | POLLPRI | POLLERR;

            int ret;
            do {
                ret = poll(&pfd, 1, timeoutMs);
            } while (ret < 0 && errno == EINTR);

            if (ret < 0) {
                ALOGE("ARDA_FULL_DMABUF_ISP_V1 poll failed errno=%d (%s)",
                      errno, strerror(errno));
                return false;
            }
            if (ret == 0) return false;
            if (pfd.revents & POLLERR) {
                ALOGE("ARDA_FULL_DMABUF_ISP_V1 poll returned POLLERR");
                return false;
            }
        }
        return true;
    }

    void reset() {
        if (mFd >= 0 && mStreaming) {
            int capture = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            int output = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            xioctl(mFd, VIDIOC_STREAMOFF, &capture);
            xioctl(mFd, VIDIOC_STREAMOFF, &output);
        }
        mStreaming = false;

        if (mFd >= 0) {
            close(mFd);
            mFd = -1;
        }

        mWidth = 0;
        mHeight = 0;
        mSrcStride = 0;
        mOutputBytesPerLine = 0;
        mOutputSizeImage = 0;
        mCaptureBytesPerLine = 0;
        mCaptureSizeImage = 0;
    }

    std::mutex mLock;
    int mFd = -1;
    int mWidth = 0;
    int mHeight = 0;
    int mSrcStride = 0;
    bool mStreaming = false;
    uint32_t mOutputBytesPerLine = 0;
    size_t mOutputSizeImage = 0;
    uint32_t mCaptureBytesPerLine = 0;
    size_t mCaptureSizeImage = 0;
};

ArdaIspFullDmabufConverter gArdaIspFullDmabufConverter;

}  // namespace

ImplDefinedToRGBXMap::ImplDefinedToRGBXMap(sp<GraphicBuffer> buf, uint8_t* addr, int rowInc)
      : mBuffer(std::move(buf)), mAddr(addr), mRowInc(rowInc) {}

ImplDefinedToRGBXMap::~ImplDefinedToRGBXMap() {
    mBuffer->unlock();
}

// static
std::unique_ptr<ImplDefinedToRGBXMap> ImplDefinedToRGBXMap::Create(
        const C2ConstGraphicBlock& block) {
    uint32_t width, height, format, stride, igbpSlot, generation;
    uint64_t usage, igbpId;
    android::_UnwrapNativeCodec2GrallocMetadata(block.handle(), &width, &height, &format, &usage,
                                                &stride, &generation, &igbpId, &igbpSlot);

    if (format != HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) {
        ALOGE("The original format (=%u) is not IMPLEMENTATION_DEFINED", format);
        return nullptr;
    }

    native_handle_t* grallocHandle = android::UnwrapNativeCodec2GrallocHandle(block.handle());
    sp<GraphicBuffer> buf = new GraphicBuffer(grallocHandle, GraphicBuffer::CLONE_HANDLE, width,
                                              height, format, 1, usage, stride);
    native_handle_delete(grallocHandle);

    void* pointer = nullptr;
    int32_t status = buf->lock(GRALLOC_USAGE_SW_READ_OFTEN, &pointer);
    if (status != OK) {
        ALOGE("Failed to lock buffer as IMPLEMENTATION_DEFINED format");
        return nullptr;
    }

    uint8_t* addr = reinterpret_cast<uint8_t*>(pointer);
    int rowInc = static_cast<int>(stride * 4);  // RGBX 4-byte data per pixel
    ALOGD("Parsed input format IMPLEMENTATION_DEFINED to RGBX_8888");
    return std::unique_ptr<ImplDefinedToRGBXMap>(
            new ImplDefinedToRGBXMap(std::move(buf), addr, rowInc));
}

// static
std::unique_ptr<FormatConverter> FormatConverter::Create(VideoPixelFormat outFormat,
                                                         const ui::Size& visibleSize,
                                                         uint32_t inputCount,
                                                         const ui::Size& codedSize) {
    if (outFormat != VideoPixelFormat::I420 && outFormat != VideoPixelFormat::NV12) {
        ALOGE("Unsupported output format: %d", static_cast<int32_t>(outFormat));
        return nullptr;
    }

    std::unique_ptr<FormatConverter> converter(new FormatConverter);
    if (converter->initialize(outFormat, visibleSize, inputCount, codedSize) != C2_OK) {
        ALOGE("Failed to initialize FormatConverter");
        return nullptr;
    }
    return converter;
}

c2_status_t FormatConverter::initialize(VideoPixelFormat outFormat, const ui::Size& visibleSize,
                                        uint32_t inputCount, const ui::Size& codedSize) {
    ALOGV("initialize(out_format=%s, visible_size=%dx%d, input_count=%u, coded_size=%dx%d)",
          videoPixelFormatToString(outFormat).c_str(), visibleSize.width, visibleSize.height,
          inputCount, codedSize.width, codedSize.height);

    std::shared_ptr<C2BlockPool> pool;
    c2_status_t status = GetCodec2BlockPool(C2BlockPool::BASIC_GRAPHIC, nullptr, &pool);
    if (status != C2_OK) {
        ALOGE("Failed to get basic graphic block pool (err=%d)", status);
        return status;
    }

    HalPixelFormat halFormat;
    if (outFormat == VideoPixelFormat::I420) {
        // Android HAL format doesn't have I420, we use YV12 instead and swap U and V data while
        // conversion to perform I420.
        halFormat = HalPixelFormat::YV12;
    } else {
        halFormat = HalPixelFormat::YCBCR_420_888;  // will allocate NV12 by minigbm.
    }

    uint32_t bufferCount = std::max(inputCount, kMinInputBufferCount);
    for (uint32_t i = 0; i < bufferCount; i++) {
        std::shared_ptr<C2GraphicBlock> block;
        status = pool->fetchGraphicBlock(codedSize.width, codedSize.height,
                                         static_cast<uint32_t>(halFormat),
                                         {(C2MemoryUsage::CPU_READ | C2MemoryUsage::CPU_WRITE),
                                          static_cast<uint64_t>(BufferUsage::VIDEO_ENCODER)},
                                         &block);
        if (status != C2_OK) {
            ALOGE("Failed to fetch graphic block (err=%d)", status);
            return status;
        }
        mGraphicBlocks.emplace_back(new BlockEntry(std::move(block)));
        mAvailableQueue.push(mGraphicBlocks.back().get());
    }

    mOutFormat = outFormat;
    mVisibleSize = visibleSize;

    mTempPlaneU =
            std::unique_ptr<uint8_t[]>(new uint8_t[mVisibleSize.width * mVisibleSize.height / 4]);
    mTempPlaneV =
            std::unique_ptr<uint8_t[]>(new uint8_t[mVisibleSize.width * mVisibleSize.height / 4]);

    return C2_OK;
}

C2ConstGraphicBlock FormatConverter::convertBlock(uint64_t frameIndex,
                                                  const C2ConstGraphicBlock& inputBlock,
                                                  c2_status_t* status) {
    if (!isReady()) {
        ALOGV("There is no available block for conversion");
        *status = C2_NO_MEMORY;
        return inputBlock;  // This is actually redundant and should not be used.
    }

    BlockEntry* entry = mAvailableQueue.front();
    std::shared_ptr<C2GraphicBlock> outputBlock = entry->mBlock;

    // ARDA_H264_FULL_DMABUF_PRODUCTION_V1:
    // Proven 1920x1080 HWC/RGBA input DMA-BUF -> video12 BGR4 -> NV12
    // directly into the existing C2 output graphic-block DMA-BUF.
    // Any mismatch/import failure falls through to AOSP's CPU converter.
    if (mOutFormat == VideoPixelFormat::NV12 &&
        mVisibleSize.width == 1920 && mVisibleSize.height == 1080) {
        uint32_t metaWidth = 0, metaHeight = 0, metaFormat = 0, metaStride = 0;
        uint32_t metaGeneration = 0, metaSlot = 0;
        uint64_t metaUsage = 0, metaIgbpId = 0;
        android::_UnwrapNativeCodec2GrallocMetadata(
                inputBlock.handle(), &metaWidth, &metaHeight, &metaFormat,
                &metaUsage, &metaStride, &metaGeneration, &metaIgbpId, &metaSlot);

        const bool directCandidate =
                metaFormat == HAL_PIXEL_FORMAT_RGBA_8888 ||
                metaFormat == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;

        if (directCandidate && metaWidth == 1920 && metaHeight == 1080 &&
            metaStride >= 1920) {
            native_handle_t* inputHandle =
                    android::UnwrapNativeCodec2GrallocHandle(inputBlock.handle());
            native_handle_t* outputHandle =
                    android::UnwrapNativeCodec2GrallocHandle(outputBlock->handle());

            bool fullDmabufOk = false;
            if (inputHandle && outputHandle &&
                inputHandle->numFds >= 1 && outputHandle->numFds >= 1 &&
                inputHandle->data[0] >= 0 && outputHandle->data[0] >= 0) {
                uint32_t outWidth = 0, outHeight = 0, outFormat = 0, outStride = 0;
                uint32_t outGeneration = 0, outSlot = 0;
                uint64_t outUsage = 0, outIgbpId = 0;
                android::_UnwrapNativeCodec2GrallocMetadata(
                        outputBlock->handle(), &outWidth, &outHeight, &outFormat,
                        &outUsage, &outStride, &outGeneration, &outIgbpId, &outSlot);

                struct stat outputStat = {};
                const int outputFd = outputHandle->data[0];
                if (fstat(outputFd, &outputStat) == 0 && outputStat.st_size > 0 &&
                    outWidth == 1920 && outHeight == 1080 &&
                    outFormat == HAL_PIXEL_FORMAT_YCBCR_420_888 &&
                    outStride >= 1920) {
                    fullDmabufOk = gArdaIspFullDmabufConverter.convert(
                            frameIndex, inputHandle->data[0],
                            static_cast<int>(metaStride * 4u), outputFd,
                            static_cast<size_t>(outputStat.st_size),
                            mVisibleSize.width, mVisibleSize.height);
                }
            }

            if (inputHandle) native_handle_delete(inputHandle);
            if (outputHandle) native_handle_delete(outputHandle);

            if (fullDmabufOk) {
                *status = C2_OK;
                entry->mAssociatedFrameIndex = frameIndex;
                mAvailableQueue.pop();
                return outputBlock->share(
                        C2Rect(mVisibleSize.width, mVisibleSize.height), C2Fence());
            }
        }
    }

    const C2GraphicView& inputView = inputBlock.map().get();
    C2PlanarLayout inputLayout = inputView.layout();

    // The above layout() cannot fill layout information and memset 0 instead if the input format is
    // IMPLEMENTATION_DEFINED and its backed format is RGB. We fill the layout by using
    // ImplDefinedToRGBXMap in the case.
    std::unique_ptr<ImplDefinedToRGBXMap> idMap;
    if (static_cast<uint32_t>(inputLayout.type) == 0u) {
        idMap = ImplDefinedToRGBXMap::Create(inputBlock);
        if (idMap == nullptr) {
            ALOGE("Unable to parse RGBX_8888 from IMPLEMENTATION_DEFINED");
            *status = C2_CORRUPTED;
            return inputBlock;  // This is actually redundant and should not be used.
        }
        inputLayout.type = C2PlanarLayout::TYPE_RGB;
    }

    C2GraphicView outputView = outputBlock->map().get();
    C2PlanarLayout outputLayout = outputView.layout();
    uint8_t* dstY = outputView.data()[C2PlanarLayout::PLANE_Y];
    uint8_t* dstU = outputView.data()[C2PlanarLayout::PLANE_V];   // only for I420
    uint8_t* dstV = outputView.data()[C2PlanarLayout::PLANE_U];   // only for I420
    uint8_t* dstUV = outputView.data()[C2PlanarLayout::PLANE_U];  // only for NV12
    const int dstStrideY = outputLayout.planes[C2PlanarLayout::PLANE_Y].rowInc;
    const int dstStrideU = outputLayout.planes[C2PlanarLayout::PLANE_V].rowInc;   // only for I420
    const int dstStrideV = outputLayout.planes[C2PlanarLayout::PLANE_U].rowInc;   // only for I420
    const int dstStrideUV = outputLayout.planes[C2PlanarLayout::PLANE_U].rowInc;  // only for NV12

    VideoPixelFormat inputFormat = VideoPixelFormat::UNKNOWN;
    *status = C2_OK;
    if (inputLayout.type == C2PlanarLayout::TYPE_YUV) {
        const uint8_t* srcY = inputView.data()[C2PlanarLayout::PLANE_Y];
        const uint8_t* srcU = inputView.data()[C2PlanarLayout::PLANE_U];
        const uint8_t* srcV = inputView.data()[C2PlanarLayout::PLANE_V];
        const int srcStrideY = inputLayout.planes[C2PlanarLayout::PLANE_Y].rowInc;
        const int srcStrideU = inputLayout.planes[C2PlanarLayout::PLANE_U].rowInc;
        const int srcStrideV = inputLayout.planes[C2PlanarLayout::PLANE_V].rowInc;
        if (inputLayout.rootPlanes == 3) {
            inputFormat = VideoPixelFormat::YV12;
        } else if (inputLayout.rootPlanes == 2) {
            inputFormat = (srcV > srcU) ? VideoPixelFormat::NV12 : VideoPixelFormat::NV21;
        }

        if (inputFormat == mOutFormat) {
            ALOGV("Zero-Copy is applied");
            mGraphicBlocks.emplace_back(new BlockEntry(frameIndex));
            return inputBlock;
        }

        switch (convertMap(inputFormat, mOutFormat)) {
        case convertMap(VideoPixelFormat::YV12, VideoPixelFormat::I420):
            libyuv::I420Copy(srcY, srcStrideY, srcU, srcStrideU, srcV, srcStrideV, dstY, dstStrideY,
                             dstU, dstStrideU, dstV, dstStrideV, mVisibleSize.width,
                             mVisibleSize.height);
            break;
        case convertMap(VideoPixelFormat::YV12, VideoPixelFormat::NV12):
            libyuv::I420ToNV12(srcY, srcStrideY, srcU, srcStrideU, srcV, srcStrideV, dstY,
                               dstStrideY, dstUV, dstStrideUV, mVisibleSize.width,
                               mVisibleSize.height);
            break;
        case convertMap(VideoPixelFormat::NV12, VideoPixelFormat::I420):
            libyuv::NV12ToI420(srcY, srcStrideY, srcU, srcStrideU, dstY, dstStrideY, dstU,
                               dstStrideU, dstV, dstStrideV, mVisibleSize.width,
                               mVisibleSize.height);
            break;
        case convertMap(VideoPixelFormat::NV21, VideoPixelFormat::I420):
            libyuv::NV21ToI420(srcY, srcStrideY, srcV, srcStrideV, dstY, dstStrideY, dstU,
                               dstStrideU, dstV, dstStrideV, mVisibleSize.width,
                               mVisibleSize.height);
            break;
        case convertMap(VideoPixelFormat::NV21, VideoPixelFormat::NV12):
            ALOGV("%s(): Converting PIXEL_FORMAT_NV21 -> PIXEL_FORMAT_NV12", __func__);
            libyuv::CopyPlane(srcY, srcStrideY, dstY, dstStrideY, mVisibleSize.width,
                              mVisibleSize.height);
            copyPlaneByPixel(srcU, srcStrideU, 2, dstUV, dstStrideUV, 2, mVisibleSize.width / 2,
                             mVisibleSize.height / 2);
            copyPlaneByPixel(srcV, srcStrideV, 2, dstUV + 1, dstStrideUV, 2, mVisibleSize.width / 2,
                             mVisibleSize.height / 2);
            break;
        default:
            ALOGE("Unsupported pixel format conversion from %s to %s",
                  videoPixelFormatToString(inputFormat).c_str(),
                  videoPixelFormatToString(mOutFormat).c_str());
            *status = C2_CORRUPTED;
            return inputBlock;  // This is actually redundant and should not be used.
        }
    } else if (inputLayout.type == C2PlanarLayout::TYPE_RGB) {
        // There is only RGBA_8888 specified in C2AllocationGralloc::map(), no BGRA_8888. Maybe
        // BGRA_8888 is not used now?
        inputFormat = VideoPixelFormat::ABGR;

        const uint8_t* srcRGB = (idMap) ? idMap->addr() : inputView.data()[C2PlanarLayout::PLANE_R];
        const int srcStrideRGB =
                (idMap) ? idMap->rowInc() : inputLayout.planes[C2PlanarLayout::PLANE_R].rowInc;

        switch (convertMap(inputFormat, mOutFormat)) {
        case convertMap(VideoPixelFormat::ABGR, VideoPixelFormat::I420):
            libyuv::ABGRToI420(srcRGB, srcStrideRGB, dstY, dstStrideY, dstU, dstStrideU, dstV,
                               dstStrideV, mVisibleSize.width, mVisibleSize.height);
            break;
        case convertMap(VideoPixelFormat::ABGR, VideoPixelFormat::NV12): {
            // There is no libyuv function to convert ABGR to NV12. Therefore, we first convert to
            // I420 on dst-Y plane and temporary U/V plane. Then we copy U and V pixels from
            // temporary planes to dst-UV interleavedly.
            const int tempStride = mVisibleSize.width / 2;
            libyuv::ABGRToI420(srcRGB, srcStrideRGB, dstY, dstStrideY, mTempPlaneU.get(),
                               tempStride, mTempPlaneV.get(), tempStride, mVisibleSize.width,
                               mVisibleSize.height);
            libyuv::MergeUVPlane(mTempPlaneU.get(), tempStride, mTempPlaneV.get(), tempStride,
                                 dstUV, dstStrideUV, mVisibleSize.width / 2,
                                 mVisibleSize.height / 2);
            break;
        }
        default:
            ALOGE("Unsupported pixel format conversion from %s to %s",
                  videoPixelFormatToString(inputFormat).c_str(),
                  videoPixelFormatToString(mOutFormat).c_str());
            *status = C2_CORRUPTED;
            return inputBlock;  // This is actually redundant and should not be used.
        }
    } else {
        ALOGE("Unsupported input layout type");
        *status = C2_CORRUPTED;
        return inputBlock;  // This is actually redundant and should not be used.
    }

    ALOGV("convertBlock(frame_index=%" PRIu64 ", format=%s)", frameIndex,
          videoPixelFormatToString(inputFormat).c_str());
    entry->mAssociatedFrameIndex = frameIndex;
    mAvailableQueue.pop();
    return outputBlock->share(C2Rect(mVisibleSize.width, mVisibleSize.height), C2Fence());
}

c2_status_t FormatConverter::returnBlock(uint64_t frameIndex) {
    ALOGV("returnBlock(frame_index=%" PRIu64 ")", frameIndex);

    auto iter = std::find_if(mGraphicBlocks.begin(), mGraphicBlocks.end(),
                             [frameIndex](const std::unique_ptr<BlockEntry>& be) {
                                 return be->mAssociatedFrameIndex == frameIndex;
                             });
    if (iter == mGraphicBlocks.end()) {
        ALOGE("Failed to find graphic block by converted/zero-copied frame index: %" PRIu64 "",
              frameIndex);
        return C2_BAD_INDEX;
    }

    if ((*iter)->mBlock) {
        // Returned block is format converted.
        (*iter)->mAssociatedFrameIndex = kNoFrameAssociated;
        mAvailableQueue.push(iter->get());
    } else {
        // Returned block is zero-copied.
        mGraphicBlocks.erase(iter);
    }
    return C2_OK;
}

}  // namespace android
