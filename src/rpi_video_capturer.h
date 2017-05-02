/*
Copyright (c) 2016, rpi-webrtc-streamer Lyu,KeunChang

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __RPI_VIDEO_CAPTURER_H__
#define __RPI_VIDEO_CAPTURER_H__

#include "webrtc/base/criticalsection.h"
#include "webrtc/media/base/videocapturer.h"

extern "C" {
    #include "mmal_encoder.h"
}

namespace webrtc {

#define RASPI_CAM_MIN_WIDTH 320
#define RASPI_CAM_MAX_WIDTH 1920
//#define RASPI_CAM_MAX_WIDTH 1280

class RPiVideoCapturer :
    //public FrameQueue,
    public cricket::VideoCapturer
{

public:

    RPiVideoCapturer();
    ~RPiVideoCapturer();

    // public cricket::VideoCapturer overrides
    cricket::CaptureState Start(const cricket::VideoFormat& format) override;
    void Stop() override;
    bool IsScreencast() const override { return false; };
    bool IsRunning() override { return this->capture_state() == cricket::CS_RUNNING; }

    void Init();

    int getWidth(void);
    int getHeight(void);
    bool SetFrame(int width, int height);
    bool SetRate(int framerate/*, int bitrate*/);
    bool InitCamera(int width, int height, int framerate);
    //bool ReinitCamera(int width, int height, int framerate);
    bool UninitCamera(void);
    //bool ForceKeyFrame(void);
    //bool IsKeyFrame(void);
    bool StartCapture(void);
    bool StopCapture(void);
    void ProcessBuffer(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

    // Callback Functions
    void OnBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
    static void BufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

    void ReleaseBuffer(MMAL_BUFFER_HEADER_T *buffer);

protected:
    // protected cricket::VideoCapturer overrides
    bool GetPreferredFourccs(std::vector<unsigned int>* fourccs) override;
    void OnSinkWantsChanged(const rtc::VideoSinkWants& wants) override;

private:
    MMAL_PORT_T *camera_preview_port_;
    MMAL_PORT_T *camera_video_port_;
    MMAL_PORT_T *resizer_input_port_;
    MMAL_PORT_T *resizer_output_port_;
    MMAL_PORT_T *camera_still_port_;
    //MMAL_PORT_T *preview_input_port_;
    //MMAL_PORT_T *encoder_input_port_;
    //MMAL_PORT_T *encoder_output_port_;

    RASPIVID_STATE state_;

    rtc::CriticalSection crit_sect_;

    bool Resize(int width, int height);
};

// singleton wrappper
RPiVideoCapturer* getRPiVideoCapturer(void);

}	// namespace webrtc

#endif // __RPI_VIDEO_CAPTURER_H__
