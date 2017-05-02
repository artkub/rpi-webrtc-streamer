
#include "wrapped_mmal_buffer.h"

#include "webrtc/base/checks.h"
#include "webrtc/api/video/i420_buffer.h"
#include "webrtc/base/scoped_ref_ptr.h"

namespace webrtc {

WrappedMMALBuffer::WrappedMMALBuffer(
        int width,
        int height,
        MMAL_BUFFER_HEADER_T *mmal_buffer,
        std::function<void()> release)
    : width_(width),
      height_(height),
      mmal_buffer_(mmal_buffer),
      release_(release),
      done_(false) {
}

WrappedMMALBuffer::~WrappedMMALBuffer() {
    release_();
}

void WrappedMMALBuffer::Done() {
    std::lock_guard<std::mutex> guard(m_);
    done_ = true;
    cv_.notify_all();
}

void WrappedMMALBuffer::Wait() {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&]{ return done_; });
}

int WrappedMMALBuffer::width() const {
    return width_;
}

int WrappedMMALBuffer::height() const {
    return height_;
}

const uint8_t* WrappedMMALBuffer::DataY() const {
    return nullptr;
    //return mmal_buffer_->data;
}
const uint8_t* WrappedMMALBuffer::DataU() const {
    return nullptr;
    //return mmal_buffer_->data + (height_ * width_);
}
const uint8_t* WrappedMMALBuffer::DataV() const {
    return nullptr;
    //return mmal_buffer_->data + (height_ * width_ * 1.25);
}

int WrappedMMALBuffer::StrideY() const {
    return width_;
}
int WrappedMMALBuffer::StrideU() const {
    return width_ / 2;
}
int WrappedMMALBuffer::StrideV() const {
    return width_ / 2;
}

void* WrappedMMALBuffer::native_handle() const {
    return mmal_buffer_;
}

rtc::scoped_refptr<VideoFrameBuffer> WrappedMMALBuffer::NativeToI420Buffer() {
    rtc::scoped_refptr<I420Buffer> buffer = I420Buffer::Create(width_, height_);
    I420Buffer::SetBlack(buffer);
    return nullptr;
}

}
