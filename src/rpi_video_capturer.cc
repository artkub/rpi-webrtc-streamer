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

#include <cmath>
#include <stdio.h>
#include <string.h>

#include "webrtc/base/checks.h"
#include "webrtc/base/logging.h"
#include "webrtc/base/stringutils.h"
#include "webrtc/base/criticalsection.h"
#include "webrtc/base/bind.h"

#include "rpi_video_capturer.h"
#include "wrapped_mmal_buffer.h"


namespace webrtc {

// RPi Video Capturer singleton reference
RPiVideoCapturer* getRPiVideoCapturer() {
    static RPiVideoCapturer rpi_video_capturer_;
    return &rpi_video_capturer_;
}

struct resolution {
    int width, height, pixelCount;
};
static const resolution resolutions[] = {
    { 640, 360, 640*360 },
    //{ 720, 405, 720*405 },
    { 854, 480, 854*480 },
    { 960, 540, 960*540 },
    //{ 1024, 576, 1024*576 },
    { 1280, 720, 1280*720 },
    //{ 1366, 768, 1366*768 },
    //{ 1600, 900, 1600*900 },
    { 1920, 1080, 1920*1080 }
};

RPiVideoCapturer::RPiVideoCapturer()
{

    camera_preview_port_ 	= nullptr;
    camera_video_port_ 		= nullptr;
    camera_still_port_		= nullptr;
    //preview_input_port_		= nullptr;

    bcm_host_init();

    // Register our application with the logging system
    vcos_log_register("rpi_video_capturer", VCOS_LOG_CATEGORY);

    // reset encoder setting to default state
    default_status(&state_);
}

RPiVideoCapturer::~RPiVideoCapturer() {
}

int RPiVideoCapturer::getWidth( void )
{
    return state_.width;
}

int RPiVideoCapturer::getHeight( void )
{
    return state_.height;
}

void RPiVideoCapturer::Init()
{
    std::vector<cricket::VideoFormat> supported;

    for (unsigned int i = 0; i < sizeof(resolutions)/sizeof(resolutions[0]); i++ ) {
        resolution res = resolutions[i];

        cricket::VideoFormat format;

        format.fourcc = cricket::FOURCC_I420;
        format.width = res.width;
        format.height = res.height;
        format.interval = cricket::VideoFormat::FpsToInterval(30);

        supported.push_back(format);
    }

    SetSupportedFormats(supported);
}

cricket::CaptureState RPiVideoCapturer::Start(const cricket::VideoFormat& format)
{
    const int fps = cricket::VideoFormat::IntervalToFps(format.interval);
    SetCaptureFormat(&format);
    SetCaptureState(cricket::CS_RUNNING);

    bool ok = InitCamera(format.width, format.height, fps);
    if (!ok) return cricket::CS_FAILED;
    ok = StartCapture();
    if (!ok) return cricket::CS_FAILED;

    return cricket::CS_RUNNING;
}

void RPiVideoCapturer::Stop()
{
    UninitCamera();
    StopCapture();

    SetCaptureFormat(NULL);
    SetCaptureState(cricket::CS_STOPPED);
}

bool RPiVideoCapturer::GetPreferredFourccs(std::vector<unsigned int>* fourccs)
{
    fourccs->push_back(cricket::FOURCC_I420);
    return true;
}

//void RPiVideoCapturer::OnSinkWantsChanged(const rtc::VideoSinkWants& wants)
//{
//    int pixel_count = 0;
//
//    if (wants.max_pixel_count_step_up) {
//        pixel_count = wants.max_pixel_count_step_up.value_or(0) * 4 / 3;
//    } else if (wants.max_pixel_count) {
//        pixel_count = wants.max_pixel_count.value_or(0);
//    }
//
//    if (pixel_count > 0) {
//        int width, height;
//
//        width = std::sqrt(pixel_count * 16 / 9);
//        if (width < RASPI_CAM_MIN_WIDTH) width = RASPI_CAM_MIN_WIDTH;
//        if (width > RASPI_CAM_MAX_WIDTH) width = RASPI_CAM_MAX_WIDTH;
//        height = width * 9 / 16;
//
//        if (width == state_.width && height == state_.height) {
//            LOG(INFO) << "Keeping current resolution";
//        } else {
//            Resize(width, height);
//        }
//    }
//}

void RPiVideoCapturer::OnSinkWantsChanged(const rtc::VideoSinkWants& wants)
{
    if (wants.max_pixel_count_step_up) {
        int curr_pixel_count = wants.max_pixel_count_step_up.value_or(0);
        for (unsigned int i = 0; i < sizeof(resolutions)/sizeof(resolutions[0]); i++ ) {
            resolution res = resolutions[i];

            // Pick lowest resolution with a higher pixel count
            if (res.pixelCount > curr_pixel_count) {
                Resize(res.width, res.height);
                return;
            }
        }

        LOG(INFO) << "Already at highest resolution";
    } else if (wants.max_pixel_count) {
        int max_pixel_count = wants.max_pixel_count.value_or(0);
        int i = sizeof(resolutions)/sizeof(resolutions[0]);
        while (i--) {
            resolution res = resolutions[i];

            // Pick highest resolution with a lower or equal pixel count
            if (res.pixelCount <= max_pixel_count) {
                Resize(res.width, res.height);
                return;
            }
        }

        LOG(INFO) << "Already at lowest resolution";
    }
}

bool RPiVideoCapturer::Resize(int width, int height)
{
    LOG(INFO) << "Changing size to " << width << "x" << height;

    bool success = true;


    // Disable camera
    check_disable_port(camera_video_port_);
    mmal_port_pool_destroy(camera_video_port_, state_.camera_pool);
    if (mmal_component_disable(state_.camera_component) != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Failed to disable camera";
        return false;
    }

    // Set new width/height
    state_.width = width;
    state_.height = height;
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
        {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
            .max_stills_w = width,
            .max_stills_h = height,
            .stills_yuv422 = 0,
            .one_shot_stills = 0,
            .max_preview_video_w = width,
            .max_preview_video_h = height,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
        };
        mmal_port_parameter_set(state_.camera_component->control, &cam_config.hdr);
    }
    camera_video_port_->format->es->video.width = VCOS_ALIGN_UP(width, 32);
    camera_video_port_->format->es->video.height = VCOS_ALIGN_UP(height, 16);
    camera_video_port_->format->es->video.crop.width = width;
    camera_video_port_->format->es->video.crop.height = height;
    if (mmal_port_format_commit(camera_video_port_) != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Failed to commit camera output format";
        success = false;
    }

    // Enable camera
    if (mmal_component_enable(state_.camera_component) != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Failed to enable camera";
        return false;
    }
    state_.camera_pool = mmal_port_pool_create(camera_video_port_, camera_video_port_->buffer_num, camera_video_port_->buffer_size);
    if (!state_.camera_pool) {
        LOG(LS_ERROR) << "Failed to create buffer header pool for camera video port";
        return false;
    }
    if (mmal_port_enable(camera_video_port_, BufferCallback) != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Failed to enable video port";
        return false;
    }
    StartCapture();

    return success;
}


bool RPiVideoCapturer::InitCamera(int width, int height, int framerate)
{
    MMAL_STATUS_T status = MMAL_SUCCESS;

    LOG(INFO) << "Start initialize the MMAL encode wrapper."
              << width << "x" << height << "@" << framerate;
    rtc::CritScope cs(&crit_sect_);

    //state_.width =  width;
    //state_.height =  height;
    state_.width =  1920;
    state_.height =  1080;

    state_.framerate =  framerate;
    //state_.bitrate =  bitrate * 1000;

    if ((status = create_camera_component(&state_)) != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Failed to create camera component";
        return false;
    }
    //else if ((status = create_resizer_component(&state_)) != MMAL_SUCCESS) {
    //    LOG(LS_ERROR) << "Failed to create resizer component";
    //    destroy_camera_component(&state_);
    //    return false;
    //}
    else {
        if (state_.verbose)
            LOG(INFO) << "Starting component connection stage";

        camera_preview_port_ = state_.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
        camera_video_port_   = state_.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        //resizer_input_port_  = state_.resizer_component->input[0];
        //resizer_output_port_ = state_.resizer_component->output[0];

        camera_still_port_   = state_.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        //preview_input_port_  = state_.preview_parameters.preview_component->input[0];

        //LOG(INFO) << "camera_video_port_->format: " << camera_video_port_->format->type << ", " << camera_video_port_->format->encoding << ", " << camera_video_port_->format->encoding_variant << ", " << camera_video_port_->format->es << ", " << camera_video_port_->format->bitrate << ", " << camera_video_port_->format->flags << ", " << camera_video_port_->format->extradata_size << ", " << camera_video_port_->format->extradata;
        //LOG(INFO) << "camera_video_port_->format->es->video: " << camera_video_port_->format->es->video.width << ", " << camera_video_port_->format->es->video.height << ", " << camera_video_port_->format->es->video.color_space;
        //LOG(INFO) << "camera_video_port_->format->es->video.crop: " << camera_video_port_->format->es->video.crop.x << ", " << camera_video_port_->format->es->video.crop.y << ", " << camera_video_port_->format->es->video.crop.width << ", " << camera_video_port_->format->es->video.crop.height;
        //LOG(INFO) << "camera_video_port_->format->es->video.frame_rate: " << camera_video_port_->format->es->video.frame_rate.num << ", " << camera_video_port_->format->es->video.frame_rate.den;
        //LOG(INFO) << "camera_video_port_->format->es->video.par: " << camera_video_port_->format->es->video.par.num << ", " << camera_video_port_->format->es->video.par.den;


        //status = connect_ports(camera_video_port_, resizer_input_port_, &state_.resizer_connection);
        //if (status != MMAL_SUCCESS) {
        //    state_.resizer_connection = nullptr;
        //    return false;
        //}

        // Set up our userdata - this is passed though to the callback where we need the information.
        state_.callback_data.pstate = &state_;
        state_.callback_data.abort = 0;

        camera_video_port_->userdata = (struct MMAL_PORT_USERDATA_T *)this;
        //resizer_output_port_->userdata = (struct MMAL_PORT_USERDATA_T *)this;
        if (state_.verbose)
            LOG(INFO) << "Enabling video port";

        // Enable the encoder output port and tell it its callback function
        status = mmal_port_enable(camera_video_port_, BufferCallback);
        //status = mmal_port_enable(resizer_output_port_, BufferCallback);
        if (status != MMAL_SUCCESS) {
            LOG(LS_ERROR) << "Failed to enable video port";
            return false;
        }

        //dump_all_mmal_component(&state_);

        return true;
    }

    LOG(LS_ERROR) << "Should not be reached";
    return false;
}

void RPiVideoCapturer::BufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    reinterpret_cast<RPiVideoCapturer *> (port->userdata)-> OnBufferCallback(port, buffer);
}


bool RPiVideoCapturer::UninitCamera(void) {
    rtc::CritScope cs(&crit_sect_);

    LOG(INFO) << "unitialize the MMAL encode wrapper.";

    // Disable all our ports that are not handled by connections
    check_disable_port(camera_video_port_);
    check_disable_port(camera_still_port_);

    // destroy pools
    mmal_port_pool_destroy(camera_video_port_, state_.camera_pool);

    //if (state_.resizer_connection)
    //    mmal_connection_destroy(state_.resizer_connection);

    if (state_.camera_component)
        mmal_component_disable(state_.camera_component);

    //if (state_.resizer_component)
    //    mmal_component_disable(state_.resizer_component);

    //destroy_encoder_component(&state_);
    raspipreview_destroy(&state_.preview_parameters);
    destroy_camera_component(&state_);
    //destroy_resizer_component(&state_);

    return true;
}

void RPiVideoCapturer::OnBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    if (!port->is_enabled) {
        mmal_buffer_header_release(buffer);
        return;
    }

    // We pass our file handle and other stuff in via the userdata field.
    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

    if (pData) {
        ProcessBuffer(port, buffer);
    } else {
        vcos_log_error("Received a encoder buffer callback with no state");
    }
}

void RPiVideoCapturer::ProcessBuffer( MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer ) {
    int width = state_.width;
    int height = state_.height;

    rtc::scoped_refptr<webrtc::VideoFrameBuffer> frame_buffer =
        new rtc::RefCountedObject<webrtc::WrappedMMALBuffer>(
            width, height, buffer,
            rtc::Bind(&RPiVideoCapturer::ReleaseBuffer, this, buffer));

    webrtc::VideoFrame frame(frame_buffer, webrtc::kVideoRotation_0, int64_t(buffer->pts));

    buffer->user_data = frame_buffer.get();

    // call VideoCapturer OnFrame to signal new frame
    OnFrame(frame, width, height);
}

void RPiVideoCapturer::ReleaseBuffer( MMAL_BUFFER_HEADER_T *buffer ) {
    mmal_buffer_header_release(buffer);

    if (camera_video_port_->is_enabled) {
        MMAL_STATUS_T status;

        MMAL_BUFFER_HEADER_T *new_buffer = mmal_queue_get(state_.camera_pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(camera_video_port_, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            LOG(LS_ERROR) << "Unable to return a buffer to the encoder port";
    }
}

bool RPiVideoCapturer::StartCapture( void ) {
    // Send all the buffers to the encoder output port
    int num = mmal_queue_length(state_.camera_pool->queue);
    //int num = mmal_queue_length(state_.resizer_pool->queue);

    for (int q=0; q<num; q++) {
        MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state_.camera_pool->queue);
        //MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state_.resizer_pool->queue);

        if (!buffer)
            LOG(LS_ERROR) << "Unable to get a required buffer " << q << " from pool queue";

        if (mmal_port_send_buffer(camera_video_port_, buffer)!= MMAL_SUCCESS)
            LOG(LS_ERROR) << "Unable to send a buffer to video output port (" << q << ").";
        //if (mmal_port_send_buffer(resizer_output_port_, buffer)!= MMAL_SUCCESS)
        //    LOG(LS_ERROR) << "Unable to send a buffer to resizer output port (" << q << ").";
    }

    if (mmal_port_parameter_set_boolean(camera_video_port_, MMAL_PARAMETER_CAPTURE, MMAL_TRUE)
            != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Unable to set capture start";
        return false;
    }
    LOG(INFO) << "capture started.";

    return true;
}


bool RPiVideoCapturer::StopCapture( void ) {
    if (mmal_port_parameter_set_boolean(camera_video_port_, MMAL_PARAMETER_CAPTURE, MMAL_FALSE)
            != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Unable to unset capture start";
        return false;
    }
    LOG(INFO) << "capture stopped.";
    return true;
}


//
bool RPiVideoCapturer::SetFrame(int width, int height ) {
    if ( state_.width != width || state_.height != height ) {
        rtc::CritScope cs(&crit_sect_);

        LOG(INFO) << "MMAL frame encoding parameters changed:  "
                  << width << "x" << height << "@" << state_.framerate;

        state_.width = width;
        state_.height = height;
    }
    return true;
}


//
bool RPiVideoCapturer::SetRate(int framerate/*, int bitrate*/) {
    if ( state_.framerate != framerate/* || state_.bitrate != bitrate*1000 */) {
        rtc::CritScope cs(&crit_sect_);
        MMAL_STATUS_T status;

        if( state_.framerate != framerate ) {   // frame rate
            LOG(INFO) << "MMAL frame encoding framerate changed : "  << framerate << " fps";
            MMAL_PARAMETER_FRAME_RATE_T param;
            param.hdr.id = MMAL_PARAMETER_FRAME_RATE;
            param.hdr.size = sizeof(param);
            param.frame_rate.num = framerate;
            param.frame_rate.den = 1;

            status = mmal_port_parameter_set(camera_video_port_, &param.hdr);
            if (status != MMAL_SUCCESS) {
                LOG(LS_ERROR) << "Unable to set framerate";
            }

        }
        //state_.bitrate = bitrate*1000;
        state_.framerate = framerate;

    }
    return true;
}

}	// webrtc namespace

