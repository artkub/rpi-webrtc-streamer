/*
 *  Copyright (c) 2016, rpi-webrtc-streamer  Lyu,KeunChang
 *
 * streamer.cc
 *
 * Modified version of webrtc/src/webrtc/examples/peer/client/peer_connection.cc in WebRTC source tree
 * The origianl copyright info below.
 */

/*
 *  Copyright 2012 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <memory>
#include <utility>
#include <vector>

#include "webrtc/base/json.h"
#include "webrtc/base/checks.h"
#include "webrtc/base/logging.h"
#include "webrtc/api/mediaconstraintsinterface.h"
#include "webrtc/media/engine/webrtcvideodecoderfactory.h"
#include "webrtc/media/engine/webrtcvideoencoderfactory.h"
#include "webrtc/media/engine/webrtcvideocapturerfactory.h"
#include "webrtc/modules/video_capture/video_capture_factory.h"
#include "webrtc/modules/video_coding/codecs/h264/include/h264.h"

#include "webrtc/system_wrappers/include/trace.h"

#include "clientconstraints.h"

#include "webrtc/api/test/fakeconstraints.h"
//#include "webrtc/media/base/fakevideocapturer.h"
#include "raspi_encoder.h"
#include "rpi_video_capturer.h"

#include "streamer.h"
#include "streamer_observer.h"
#include "streamer_config.h"



using webrtc::PeerConnectionInterface;


// Names used for SDP label
const char kAudioLabel[] = "audio_label";
const char kVideoLabel[] = "video_label";
const char kStreamLabel[] = "stream_label";

// Names used for a Android Direct IceCandidate JSON object.
const char kCandidateSdpMidName[] = "id";
const char kCandidateSdpMlineIndexName[] = "label";
const char kCandidateSdpName[] = "candidate";
const char kCandidateType[] = "type";

// Names used for a SessionDescription JSON object.
const char kSessionDescriptionTypeName[] = "type";
const char kSessionDescriptionSdpName[] = "sdp";

// DTLS enable flags
#define DTLS_ON  true
#define DTLS_OFF false

const bool dtls_enable = DTLS_ON;

class DummySetSessionDescriptionObserver 
    : public webrtc::SetSessionDescriptionObserver {
public:
    static DummySetSessionDescriptionObserver* Create() {
        return
            new rtc::RefCountedObject<DummySetSessionDescriptionObserver>();
    }
    virtual void OnSuccess() {
        LOG(INFO) << __FUNCTION__;
    }
    virtual void OnFailure(const std::string& error) {
        LOG(INFO) << __FUNCTION__ << " " << error;
    }

protected:
    DummySetSessionDescriptionObserver() {}
    ~DummySetSessionDescriptionObserver() {}
};

Streamer::Streamer(SocketServerObserver *session, StreamerConfig *config)
    : peer_id_(-1), dtls_enable_(true) {
    RTC_DCHECK(session != nullptr);
    session_ = session;
    session->RegisterObserver(this);
    streamer_config_ = config;
}

Streamer::~Streamer() {
    RTC_DCHECK(peer_connection_.get() == nullptr);
}

bool Streamer::connection_active() const {
    return peer_connection_.get() != nullptr;
}

void Streamer::Close() {
    // Reset session active session
    DeletePeerConnection();
}

bool Streamer::InitializePeerConnection() {
    // Disable internal H264 codec
    webrtc::DisableRtcUseH264();

    RTC_DCHECK(peer_connection_factory_.get() == nullptr);
    RTC_DCHECK(peer_connection_.get() == nullptr);

    rtc::ThreadManager::Instance()->WrapCurrentThread();
    webrtc::Trace::CreateTrace();

    network_thread_ = rtc::Thread::CreateWithSocketServer();
    network_thread_->SetName("network_thread", nullptr);
    RTC_CHECK(network_thread_->Start()) << "Failed to start network thread";

    worker_thread_ = rtc::Thread::Create();
    worker_thread_->SetName("worker_thread", nullptr);
    RTC_CHECK(worker_thread_->Start()) << "Failed to start worker thread";

    signaling_thread_ = rtc::Thread::Create();
    signaling_thread_->SetName("signaling_thread", nullptr);
    RTC_CHECK(signaling_thread_->Start()) << "Failed to start signaling thread";

    cricket::WebRtcVideoEncoderFactory* encoder_factory = nullptr;
    encoder_factory = new webrtc::MMALVideoEncoderFactory();

    //peer_connection_factory_ = webrtc::CreatePeerConnectionFactory()
    peer_connection_factory_  = webrtc::CreatePeerConnectionFactory(
            network_thread_.get(), worker_thread_.get(),
            signaling_thread_.get(), nullptr,
            encoder_factory, nullptr);

    if (!peer_connection_factory_.get()) {
        LOG(LS_ERROR) << __FUNCTION__ << "Failed to initialize PeerConnectionFactory";
        DeletePeerConnection();
        return false;
    }

    if (!CreatePeerConnection()) {
        LOG(LS_ERROR) << __FUNCTION__ << "CreatePeerConnection failed";
        DeletePeerConnection();
    }
    AddStreams();
    return peer_connection_.get() != nullptr;
}

bool Streamer::CreatePeerConnection() {
    RTC_DCHECK(peer_connection_factory_.get() != nullptr);
    RTC_DCHECK(peer_connection_.get() == nullptr);

    webrtc::PeerConnectionInterface::RTCConfiguration config;

    webrtc::PeerConnectionInterface::IceServer stun;
    streamer_config_->GetStunServer(stun.uri);
    config.servers.push_back(stun);

    webrtc::PeerConnectionInterface::IceServer turn;
    turn.uri = "turn:beta.volons.fr:3478";
    turn.username = "art";
    turn.password = "123";
    config.servers.push_back(turn);

    webrtc::FakeConstraints constraints;
    if (dtls_enable) {
        constraints.AddOptional(webrtc::MediaConstraintsInterface::kEnableDtlsSrtp,
                "true");
    } else {
        constraints.AddOptional(webrtc::MediaConstraintsInterface::kEnableDtlsSrtp,
                "false");
    }
    peer_connection_ = peer_connection_factory_->CreatePeerConnection(
            config, &constraints, nullptr, nullptr, this);

    // RTC_CHECK(peer_connection_.get() == nullptr) << "PeerConnection creation failed.";
    return peer_connection_.get() != nullptr;
}

void Streamer::DeletePeerConnection() {
    // Reset active stream session
    peer_connection_ = nullptr;
    active_streams_.clear();
    peer_connection_factory_ = nullptr;
    peer_id_ = -1;
}

//
// PeerConnectionObserver implementation.
//

// Called when a remote stream is added
void Streamer::OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {
    LOG(INFO) << __FUNCTION__ << " " << stream->label();
    LOG(LS_ERROR) << __FUNCTION__ << "Remote Stream added!" << stream->label();
}

void Streamer::OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {
    LOG(INFO) << __FUNCTION__ << " " << stream->label();
    LOG(LS_ERROR) << __FUNCTION__ << "Remote Stream removed!" << stream->label();
}

void Streamer::OnIceCandidate(const webrtc::IceCandidateInterface* candidate) {
    LOG(INFO) << __FUNCTION__ << " " << candidate->sdp_mline_index();

    Json::StyledWriter writer;
    Json::Value jmessage;

    // Names used for a Android Direct IceCandidate JSON object.
    jmessage[kCandidateType] = kCandidateSdpName;
    jmessage[kCandidateSdpMidName] = candidate->sdp_mid();
    jmessage[kCandidateSdpMlineIndexName] = candidate->sdp_mline_index();
    std::string sdp;
    if (!candidate->ToString(&sdp)) {
        LOG(LS_ERROR) << "Failed to serialize candidate";
        return;
    }
    jmessage[kCandidateSdpName] = sdp;
    SendMessage(writer.write(jmessage));
}

//
// StreamerObserver implementation.
//
void Streamer::OnPeerConnected(int peer_id, const std::string& name) {
    RTC_DCHECK(peer_id_ == -1);
    RTC_DCHECK(peer_id != -1);
    webrtc::Trace::CreateTrace();
    LOG(INFO) << __FUNCTION__ << "Peer " << peer_id  << ", " << name.c_str()
              << " connected, trying to initialize streamer instance";

    if (peer_connection_.get()) {
        LOG(LS_ERROR) << "We only support connecting to one peer at a time";
        return;
    }

    if (InitializePeerConnection()) {
        peer_id_ = peer_id;
        peer_connection_->CreateOffer(this, nullptr);
    } else {
        LOG(LS_ERROR) << "Failed to initialize PeerConnection";
    }
}

void Streamer::OnPeerDisconnected(int peer_id) {
    LOG(INFO) << __FUNCTION__;
    if (peer_id == peer_id_ && peer_connection_.get()) {
        LOG(INFO) << "Peer disconnected";
        DeletePeerConnection();
    }
}

void Streamer::OnMessageFromPeer(int peer_id, const std::string& message) {
    RTC_CHECK(peer_id_ == peer_id || peer_id_ == -1);
    RTC_CHECK(!message.empty());

    if (!peer_connection_.get()) {
        RTC_CHECK(peer_id_ == -1);
        peer_id_ = peer_id;

        if (!InitializePeerConnection()) {
            LOG(LS_ERROR) << "Failed to initialize our PeerConnection instance";
            // TODO
            // Reset the active stream session
            // client_->SignOut();
            return;
        }
    } else if (peer_id != peer_id_) {
        RTC_DCHECK(peer_id_ != -1);
        LOG(WARNING) << "Received a message from unknown peer while already in a "
                     "conversation with a different peer.";
        return;
    }

    Json::Reader reader;
    Json::Value jmessage;
    if (!reader.parse(message, jmessage)) {
        LOG(WARNING) << "Received unknown message. " << message;
        return;
    }
    std::string sdp;
    std::string json_object;

    rtc::GetStringFromJsonObject(jmessage, kSessionDescriptionSdpName, &sdp);
    if (!sdp.empty()) {
        std::string sdp;
        if (!rtc::GetStringFromJsonObject(jmessage, kSessionDescriptionSdpName,
                                          &sdp)) {
            LOG(WARNING) << "Can't parse received session description message.";
            LOG(WARNING) << ":" << message << ":";
            return;
        }
        std::string type;
        rtc::GetStringFromJsonObject(jmessage, kSessionDescriptionTypeName, &type);

        webrtc::SdpParseError error;
        webrtc::SessionDescriptionInterface* session_description(
            webrtc::CreateSessionDescription(type, sdp, &error));
        if (!session_description) {
            LOG(WARNING) << "Can't parse received session description message. "
                         << "SdpParseError was: " << error.description;
            return;
        }
        LOG(INFO) << " Received session description : \"" << message << "\"";
        peer_connection_->SetRemoteDescription(
            DummySetSessionDescriptionObserver::Create(), session_description);

        webrtc::ClientConstraints sdpConstraints;
        sdpConstraints.SetMandatoryReceiveAudio(false);
        sdpConstraints.SetMandatoryReceiveVideo(false);
        // TODO(kclyu) invalid sdp negotiation makes session_description 
        // null, need to figure it out how to fix it.
        if (session_description->type() ==
                webrtc::SessionDescriptionInterface::kOffer) {
            peer_connection_->CreateAnswer(this, &sdpConstraints);
        }

        return;
    } else {
        std::string sdp_mid;
        int sdp_mlineindex = 0;
        std::string sdp;

        if (!rtc::GetStringFromJsonObject(jmessage, kCandidateSdpMidName,
                                          &sdp_mid) ||
                !rtc::GetIntFromJsonObject(jmessage, kCandidateSdpMlineIndexName,
                                           &sdp_mlineindex) ||
                !rtc::GetStringFromJsonObject(jmessage, kCandidateSdpName, &sdp)) {
            LOG(WARNING) << "Can't parse received message.";
            return;
        }

        webrtc::SdpParseError error;
        std::unique_ptr<webrtc::IceCandidateInterface> candidate(
            webrtc::CreateIceCandidate(sdp_mid, sdp_mlineindex, sdp, &error));
        if (!candidate.get()) {
            LOG(WARNING) << "Can't parse received candidate message. "
                         << "SdpParseError was: " << error.description;
            return;
        }
        if (!peer_connection_->AddIceCandidate(candidate.get())) {
            LOG(WARNING) << "Failed to apply the received candidate";
            return;
        }
        LOG(INFO) << " Received candidate :" << message;
        return;
    }
}

void Streamer::OnMessageSent(int err) {
    LOG(INFO) << __FUNCTION__ << "Message Sent result : " << err;
}

cricket::VideoCapturer * Streamer::OpenVideoCaptureDevice() {
    webrtc::RPiVideoCapturer* capturer = new webrtc::RPiVideoCapturer();
    capturer->Init();
    return (cricket::VideoCapturer*)capturer;
}

void Streamer::AddStreams() {
    if (active_streams_.find(kStreamLabel) != active_streams_.end())
        return;  // Already added.

    webrtc::ClientConstraints audioConstraints;
    audioConstraints.SetMandatoryEchoCancellation(false);
    audioConstraints.SetMandatoryAudoGainControl(false);
    audioConstraints.SetMandatoryNoiseSuppression(false);
    audioConstraints.SetMandatoryHighpassFilter(false);

    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
        peer_connection_factory_->CreateAudioTrack(
            kAudioLabel, peer_connection_factory_->CreateAudioSource(&audioConstraints)));

    rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
        peer_connection_factory_->CreateVideoTrack(
            kVideoLabel,
            peer_connection_factory_->CreateVideoSource(OpenVideoCaptureDevice(),
                    nullptr)));

    rtc::scoped_refptr<webrtc::MediaStreamInterface> stream =
        peer_connection_factory_->CreateLocalMediaStream(kStreamLabel);

    //stream->AddTrack(audio_track);

    stream->AddTrack(video_track);
    if (!peer_connection_->AddStream(stream)) {
        LOG(LS_ERROR) << "Adding stream to PeerConnection failed";
    }
    typedef std::pair<std::string,
            rtc::scoped_refptr<webrtc::MediaStreamInterface> >
            MediaStreamPair;
    active_streams_.insert(MediaStreamPair(stream->label(), stream));
}


void Streamer::OnSuccess(webrtc::SessionDescriptionInterface* desc) {
    peer_connection_->SetLocalDescription(
        DummySetSessionDescriptionObserver::Create(), desc);

    std::string sdp;
    desc->ToString(&sdp);

    Json::StyledWriter writer;
    Json::Value jmessage;
    jmessage[kSessionDescriptionTypeName] = desc->type();
    jmessage[kSessionDescriptionSdpName] = sdp;
    SendMessage(writer.write(jmessage));
}

void Streamer::OnFailure(const std::string& error) {
    LOG(LERROR) << error;
}

void Streamer::SendMessage(const std::string& json_object) {
    LOG(INFO) << "Sending Message to Peer: " << json_object.c_str();
    session_->SendMessageToPeer( peer_id_, json_object );
}


