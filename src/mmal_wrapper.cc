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

#include <stdio.h>
#include <string.h>

#include "webrtc/base/checks.h"
#include "webrtc/base/logging.h"
#include "webrtc/base/stringutils.h"
#include "webrtc/base/criticalsection.h"

#include "mmal_wrapper.h"
#include "wrapped_mmal_buffer.h"


namespace webrtc {

// MMAL Encoder singleton reference
MMALEncoderWrapper* getMMALEncoderWrapper() {
    static MMALEncoderWrapper encode_wrapper_;
    return &encode_wrapper_;
}

MMALEncoderWrapper::MMALEncoderWrapper()
{

    //camera_preview_port_ = nullptr;
    //camera_video_port_ = nullptr;
    //camera_still_port_ = nullptr;
    //preview_input_port_ = nullptr;
    encoder_input_port_ = nullptr;
    encoder_output_port_ = nullptr;

    frame_pool_ = nullptr;
    curr_frame_ = nullptr;
    curr_frame_len_ = 0;

    bcm_host_init();

    // Register our application with the logging system
    vcos_log_register("mmal_wrapper", VCOS_LOG_CATEGORY);

    // reset encoder setting to default state
    default_status(&state_);
    state_.verbose = true;

    //FrameQueue();
}

MMALEncoderWrapper::~MMALEncoderWrapper() {
}

int MMALEncoderWrapper::getWidth( void )
{
    return state_.width;
}

int MMALEncoderWrapper::getHeight( void )
{
    return state_.height;
}


bool MMALEncoderWrapper::InitEncoder(int width, int height, int framerate, int bitrate, std::function<void(MMAL_BUFFER_HEADER_T *frame)> frame_callback)
{
    frame_callback_ = frame_callback;
    MMAL_STATUS_T status = MMAL_SUCCESS;

    LOG(INFO) << "Start initialize the MMAL encode wrapper. @"
              << framerate << ", " << bitrate << "kbps";
    rtc::CritScope cs(&crit_sect_);

    state_.width =  width;
    state_.height =  height;
    state_.framerate =  framerate;
    state_.bitrate =  bitrate * 1000;

    if ((status = create_encoder_component(&state_)) != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Failed to create encode component";
        raspipreview_destroy(&state_.preview_parameters);
        destroy_camera_component(&state_);
        return false;
    }
    else {
        if (state_.verbose)
            LOG(INFO) << "Starting component connection stage";

        encoder_input_port_  = state_.encoder_component->input[0];
        encoder_output_port_ = state_.encoder_component->output[0];


        // Set encoder input port format
        //encoder_input_port_->format->encoding = MMAL_ENCODING_I420;
        encoder_input_port_->format->encoding = MMAL_ENCODING_OPAQUE;
        encoder_input_port_->format->encoding_variant = MMAL_ENCODING_I420;
        encoder_input_port_->format->es->video.width = width;
        encoder_input_port_->format->es->video.height = height;
        encoder_input_port_->format->es->video.crop.width = width;
        encoder_input_port_->format->es->video.crop.height = height;

        if (mmal_port_format_commit(encoder_input_port_) != MMAL_SUCCESS) {
            LOG(LS_ERROR) << "Failed to commit encoder input format";
            return false;
        }

        // Set up our userdata - this is passed though to the callback where we need the information.
        state_.callback_data.pstate = &state_;
        state_.callback_data.abort = 0;

        encoder_output_port_->userdata = (struct MMAL_PORT_USERDATA_T *)this;
        if (state_.verbose)
            LOG(INFO) << "Enabling encoder output port";

        frame_pool_ = mmal_pool_create( 10, width * height );
        if(frame_pool_ == NULL) {
            LOG(LS_ERROR) << "Frame pool creation failed";
            return false;
        };

        // Enable the encoder output port and tell it its callback function
        status = mmal_port_enable(encoder_input_port_, InputCallback);
        if (status != MMAL_SUCCESS) {
            LOG(LS_ERROR) << "Failed to setup encoder input";
            return false;
        }

        // Enable the encoder output port and tell it its callback function
        status = mmal_port_enable(encoder_output_port_, BufferCallback);
        if (status != MMAL_SUCCESS) {
            LOG(LS_ERROR) << "Failed to setup encoder output";
            return false;
        }

        //dump_all_mmal_component(&state_);

        // Send all the buffers to the encoder output port
        int num = mmal_queue_length(state_.encoder_pool->queue);

        for (int q=0; q<num; q++) {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state_.encoder_pool->queue);

            if (!buffer)
                LOG(LS_ERROR) << "Unable to get a required buffer " << q << " from pool queue";

            if (mmal_port_send_buffer(encoder_output_port_, buffer)!= MMAL_SUCCESS)
                LOG(LS_ERROR) << "Unable to send a buffer to encoder output port (" << q << ").";
        }

        return true;
    }

    LOG(LS_ERROR) << "Not Reached";
    return false;
}

bool MMALEncoderWrapper::Encode(const VideoFrame& frame) {
    rtc::scoped_refptr<const VideoFrameBuffer> frame_buf = frame.video_frame_buffer();

    int width, height, length;
    width = frame_buf->width();
    height = frame_buf->height();
    //length = width * height * 1.5;
    length = 128;

    if (width == 1 && height == 1) {
        return false;
    }

    MMAL_BUFFER_HEADER_T *buffer = static_cast<MMAL_BUFFER_HEADER_T *>(frame_buf->native_handle());

    if (!buffer) {
        // if frame_buf is not an mmal buffer; skip
        return false;
    }

    if (mmal_port_send_buffer(encoder_input_port_, buffer)!= MMAL_SUCCESS) {
        LOG(LS_ERROR) << "Unable to send a buffer to encoder input port.";
        mmal_buffer_header_release(buffer);
        return false;
    }

    WrappedMMALBuffer * wrapped_buffer = static_cast<WrappedMMALBuffer*>(buffer->user_data);
    if (wrapped_buffer) {
        wrapped_buffer->Wait();
    }

    return true;
}

void MMALEncoderWrapper::InputCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    WrappedMMALBuffer * wrapped_buffer = static_cast<WrappedMMALBuffer*>(buffer->user_data);
    buffer->user_data = NULL;
    wrapped_buffer->Done();
}

void MMALEncoderWrapper::BufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    reinterpret_cast<MMALEncoderWrapper *> (port->userdata)-> OnBufferCallback(port, buffer);
}

bool MMALEncoderWrapper::UninitEncoder(void) {
    rtc::CritScope cs(&crit_sect_);

    LOG(INFO) << "unitialize the MMAL encode wrapper.";

    // Disable all our ports that are not handled by connections
    check_disable_port(encoder_input_port_);
    check_disable_port(encoder_output_port_);

    mmal_pool_destroy(frame_pool_);

    /* Disable components */
    if (state_.encoder_component)
        mmal_component_disable(state_.encoder_component);

    destroy_encoder_component(&state_);
    return true;
}

void MMALEncoderWrapper::OnBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    MMAL_BUFFER_HEADER_T *new_buffer = nullptr;
    //static int64_t base_time =  -1;
    //static int64_t last_second = -1;
    //int64_t current_time = vcos_getmicrosecs64()/1000;

    // All our segment times based on the receipt of the first encoder callback
    //if (base_time == -1)
    //    base_time = vcos_getmicrosecs64()/1000;

    // We pass our file handle and other stuff in via the userdata field.
    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

    if (pData) {
        StitchFrame(buffer);
        //ProcessBuffer(buffer);
    }
    else {
        vcos_log_error("Received a encoder buffer callback with no state");
    }

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(state_.encoder_pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            LOG(LS_ERROR) << "Unable to return a buffer to the encoder port";
    }

    // See if the second count has changed and we need to update any annotation
    //if (current_time/1000 != last_second) {
    //    update_annotation_data(&state_);
    //    last_second = current_time/1000;
    //}
}

void MMALEncoderWrapper::StitchFrame( MMAL_BUFFER_HEADER_T *buffer ) {
    //RTC_DCHECK(inited_);
    RTC_DCHECK(buffer != NULL);

    if (curr_frame_len_ == -1 || buffer->length == 0) {
        LOG(INFO) << "Encoder buffer discarded (length: " << buffer->length << ")";
        return;
    }

    if (!curr_frame_) {
        // Get new frame buffer
        curr_frame_ = mmal_queue_get(frame_pool_->queue);

        if ( !curr_frame_ ) { // frame buffer is available
            curr_frame_len_ = -1;
            LOG(INFO) << "could not get frame from pool, dropping frame";
            return;
        }
    }

    // copy data into current frame
    if (curr_frame_len_ + buffer->length > curr_frame_->alloc_size) {
        LOG(INFO) << "ERROR: alloc_size: " << curr_frame_->alloc_size
                  << ", frame_size: " << curr_frame_len_ + buffer->length;
    };
    RTC_DCHECK(curr_frame_len_ + buffer->length <= curr_frame_->alloc_size);

    mmal_buffer_header_mem_lock(buffer);
    memcpy(curr_frame_->data + curr_frame_len_,  buffer->data, buffer->length);
    curr_frame_len_ += buffer->length;
    mmal_buffer_header_mem_unlock(buffer);

    // end of frame marked
    if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
        // frame buffer is available (not skipping)
        if ( curr_frame_ ) {
            // copying meta data to frame
            mmal_buffer_header_mem_lock(buffer);
            mmal_buffer_header_copy_header(curr_frame_, buffer);
            curr_frame_->length = curr_frame_len_;
            mmal_buffer_header_mem_unlock(buffer);
        }

        OnFrame(curr_frame_);

        // reset current frame
        curr_frame_ = NULL;
        curr_frame_len_ = 0;
    }
}

void MMALEncoderWrapper::OnFrame( MMAL_BUFFER_HEADER_T *frame ) {
    frame_callback_(frame);
}

void MMALEncoderWrapper::ReturnToPool(MMAL_BUFFER_HEADER_T *buffer) {
    mmal_buffer_header_release(buffer);
}

//
bool MMALEncoderWrapper::SetFrame(int width, int height ) {
    if( state_.width != width || state_.height != height ) {
        rtc::CritScope cs(&crit_sect_);

        LOG(INFO) << "MMAL frame encoding parameters changed:  "
                  << width << "x" << height << "@" << state_.framerate << ", " << state_.bitrate << "kbps";

        state_.width = width;
        state_.height = height;
    }
    return true;
}


//
bool MMALEncoderWrapper::SetRate(int framerate, int bitrate) {
    if( state_.framerate != framerate || state_.bitrate != bitrate*1000 ) {
        rtc::CritScope cs(&crit_sect_);
        MMAL_STATUS_T status;

        // LOG(INFO) << "MMAL frame encoding rate changed : "
        // 	<< state_.width << "x" << state_.height << "@" << framerate << ", " << bitrate << "kbps";
        if( state_.bitrate != bitrate * 1000 ) {
            LOG(LS_ERROR) << "changing bitrate to " << bitrate << "kbps";
            MMAL_PARAMETER_UINT32_T param =
            {{ MMAL_PARAMETER_VIDEO_BIT_RATE, sizeof(param)}, (uint32_t)bitrate*1000};
            // {{ MMAL_PARAMETER_VIDEO_ENCODE_PEAK_RATE, sizeof(param)}, (uint32_t)bitrate*1000};
            status = mmal_port_parameter_set(encoder_output_port_, &param.hdr);
            if (status != MMAL_SUCCESS) {
                LOG(LS_ERROR) << "Unable to set bitrate";
            }
        }

        //if( state_.framerate != framerate ) {   // frame rate
        //    LOG(INFO) << "MMAL frame encoding framerate changed : "  << framerate << " fps";
        //    MMAL_PARAMETER_FRAME_RATE_T param;
        //    param.hdr.id = MMAL_PARAMETER_FRAME_RATE;
        //    param.hdr.size = sizeof(param);
        //    param.frame_rate.num	= framerate;
        //    param.frame_rate.den	= 1;

        //    // status = mmal_port_parameter_set(encoder_output_port_, &param.hdr);
        //    status = mmal_port_parameter_set(camera_video_port_, &param.hdr);
        //    if (status != MMAL_SUCCESS) {
        //        LOG(LS_ERROR) << "Unable to set framerate";
        //    }

        //}
        state_.bitrate = bitrate*1000;
        state_.framerate = framerate;

    }
    return true;
}


//
bool MMALEncoderWrapper::ForceKeyFrame(void) {
    rtc::CritScope cs(&crit_sect_);
    LOG(INFO) << "MMAL force key frame encoding";

    if (mmal_port_parameter_set_boolean(encoder_output_port_,
                MMAL_PARAMETER_VIDEO_REQUEST_I_FRAME, MMAL_TRUE) != MMAL_SUCCESS) {
        LOG(LS_ERROR) << "failed to request KeyFrame";
        return false;
    }
    return true;
}

}	// webrtc namespace

