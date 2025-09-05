import gi
import json
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GObject

Gst.init(None)

def on_offer_created(promise):
    reply = promise.get_reply()
    offer = reply.get_value("offer")
    # Send SDP offer to Unity (via signaling)
    print(json.dumps({'sdp': offer.sdp.as_text(), 'type': 'offer'}))

def on_ice_candidate(sdp_mline_index, candidate):
    # Send ICE candidate to Unity
    print(json.dumps({'candidate': candidate}))

pipeline = Gst.parse_launch("v4l2src ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! webrtcbin name=sendrecv")
webrtc = pipeline.get_by_name("sendrecv")

promise = Gst.Promise.new_with_change_func(on_offer_created)
webrtc.emit("create-offer", None, promise)

pipeline.set_state(Gst.State.PLAYING)