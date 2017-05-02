#ifndef __WRAPPED_MMAL_BUFFER_H__
#define __RPI_VIDEO_CAPTURER_H__

#include <mutex>
#include <condition_variable>

extern "C" {
    #include "mmal_encoder.h"
}

#include "webrtc/api/video/video_frame_buffer.h"
#include "webrtc/base/callback.h"
#include "webrtc/base/scoped_ref_ptr.h"

namespace webrtc {

class WrappedMMALBuffer : public webrtc::VideoFrameBuffer {
public:
    WrappedMMALBuffer(
        int width,
        int height,
        MMAL_BUFFER_HEADER_T *mmal_buffer,
        std::function<void()> release
    );
    ~WrappedMMALBuffer();

    void Done();
    void Wait();

    int width() const override;
    int height() const override;

    const uint8_t* DataY() const override;
    const uint8_t* DataU() const override;
    const uint8_t* DataV() const override;
    int StrideY() const override;
    int StrideU() const override;
    int StrideV() const override;

    void* native_handle() const override;

    rtc::scoped_refptr<VideoFrameBuffer> NativeToI420Buffer() override;

private:
    const int width_;
    const int height_;
    MMAL_BUFFER_HEADER_T *mmal_buffer_;
    std::function<void()> release_;

    std::mutex m_;
    std::condition_variable cv_;
    bool done_;
};

}

#endif // __RPI_VIDEO_CAPTURER_H__
