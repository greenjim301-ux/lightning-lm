// Emscripten entry point: connects to UiWireServer (src/ui/ui_wire_server.h) over a browser
// WebSocket and drives PangolinUiScene (src/ui/pangolin_ui_scene.h) with the decoded messages —
// the wasm-side mirror of what PangolinWindowImpl's Update* methods do natively.
//
// Not part of the native (colcon/ROS2) build: only meant to be compiled by an Emscripten
// toolchain, which is why it isn't listed in src/CMakeLists.txt. Built as an ES6 module by
// src/ui/wasm/CMakeLists.txt and consumed from web/ (React/Vite host page), which sets
// window.LIGHTNING_WS_URL before instantiating the module.
#include <emscripten.h>
#include <emscripten/websocket.h>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ui/pangolin_ui_scene.h"
#include "ui/ui_wire_protocol.h"

using namespace lightning;
using namespace lightning::ui;

namespace {

class WasmUiClient {
   public:
    void Init(const char* ws_url) {
        scene_.Init();
        scene_.CreateMenu();
        scene_.CreateDisplayLayout();
        Connect(ws_url);
    }

    void Tick() {
        scene_.BeginFrame();
        scene_.PollMenu();
        scene_.RenderFrame();
        scene_.FinishFrame();
    }

    void HandleMessage(const uint8_t* data, size_t size) {
        wire::WireReader r(data, size);
        switch (r.type()) {
            case wire::MsgType::kFrontendPose: {
                SE3 pose = r.ReadPose();
                bool is_traj_point = r.ReadPod<uint8_t>() != 0;
                scene_.SetFrontendPose(pose);
                if (is_traj_point) {
                    scene_.AddFrontendTrajPt(pose);
                }
                break;
            }
            case wire::MsgType::kScan: {
                SE3 pose = r.ReadPose();
                auto points = r.ReadPoints();
                scene_.PushCurrentScan(points, pose);
                scene_.TrimScanQueue();
                break;
            }
            case wire::MsgType::kMapChunk: {
                int32_t id = r.ReadI32();
                bool is_dynamic = r.ReadPod<uint8_t>() != 0;
                auto points = r.ReadPoints();
                auto& chunks = is_dynamic ? dynamic_chunks_ : global_chunks_;
                chunks[id] = std::move(points);
                SyncChunks(is_dynamic);
                break;
            }
            case wire::MsgType::kMapChunkActiveSet: {
                bool is_dynamic = r.ReadPod<uint8_t>() != 0;
                uint32_t count = r.ReadU32();
                std::set<int32_t> active;
                for (uint32_t i = 0; i < count; ++i) active.insert(r.ReadI32());
                auto& chunks = is_dynamic ? dynamic_chunks_ : global_chunks_;
                for (auto it = chunks.begin(); it != chunks.end();) {
                    if (active.find(it->first) == active.end()) {
                        it = chunks.erase(it);
                    } else {
                        ++it;
                    }
                }
                SyncChunks(is_dynamic);
                break;
            }
            case wire::MsgType::kKeyframeAppend: {
                uint64_t id = r.ReadU64();
                SE3 pose = r.ReadPose();
                r.ReadPoints();  // 关键帧点云目前渲染端不需要（闭环线只用位姿），读掉即可
                keyframe_poses_[id] = pose;
                UpdateLoopLine();
                break;
            }
            case wire::MsgType::kKeyframePoseSync: {
                uint32_t count = r.ReadU32();
                for (uint32_t i = 0; i < count; ++i) {
                    uint64_t id = r.ReadU64();
                    SE3 pose = r.ReadPose();
                    keyframe_poses_[id] = pose;
                }
                UpdateLoopLine();
                break;
            }
        }
    }

   private:
    void Connect(const char* url) {
        if (!emscripten_websocket_is_supported()) {
            printf("[wasm-ui] WebSockets not supported by this browser\n");
            return;
        }
        EmscriptenWebSocketCreateAttributes attrs;
        emscripten_websocket_init_create_attributes(&attrs);
        attrs.url = url;
        ws_ = emscripten_websocket_new(&attrs);

        emscripten_websocket_set_onopen_callback(ws_, this, &WasmUiClient::OnOpen);
        emscripten_websocket_set_onmessage_callback(ws_, this, &WasmUiClient::OnMessage);
        emscripten_websocket_set_onerror_callback(ws_, this, &WasmUiClient::OnError);
        emscripten_websocket_set_onclose_callback(ws_, this, &WasmUiClient::OnClose);
    }

    void SyncChunks(bool is_dynamic) {
        if (is_dynamic) {
            scene_.SyncDynamicMap(dynamic_chunks_);
        } else {
            scene_.SyncGlobalMap(global_chunks_);
        }
    }

    // 闭环线：把已知关键帧按id(==插入顺序)连成折线交给scene，跟原生端每帧从all_keyframes_
    // 重新取位姿的逻辑对应——这里同理，每次位姿有更新(append或者周期性pose sync)都重新给一次。
    void UpdateLoopLine() {
        std::vector<Vec3f> pts;
        pts.reserve(keyframe_poses_.size());
        for (auto& kv : keyframe_poses_) {
            pts.push_back(kv.second.translation().cast<float>());
        }
        scene_.SetLoopLinePoints(pts);
    }

    static bool OnOpen(int /*event_type*/, const EmscriptenWebSocketOpenEvent* /*e*/, void* /*user_data*/) {
        printf("[wasm-ui] websocket connected\n");
        return true;
    }

    static bool OnMessage(int /*event_type*/, const EmscriptenWebSocketMessageEvent* e, void* user_data) {
        auto* self = static_cast<WasmUiClient*>(user_data);
        if (e->isText) {
            return true;
        }
        try {
            self->HandleMessage(e->data, e->numBytes);
        } catch (const std::exception& ex) {
            printf("[wasm-ui] failed to decode wire message (%u bytes): %s\n", e->numBytes, ex.what());
        }
        return true;
    }

    static bool OnError(int /*event_type*/, const EmscriptenWebSocketErrorEvent* /*e*/, void* /*user_data*/) {
        printf("[wasm-ui] websocket error\n");
        return true;
    }

    static bool OnClose(int /*event_type*/, const EmscriptenWebSocketCloseEvent* e, void* /*user_data*/) {
        printf("[wasm-ui] websocket closed: code=%d reason=%s\n", e->code, e->reason);
        return true;
    }

    PangolinUiScene scene_;
    EMSCRIPTEN_WEBSOCKET_T ws_ = 0;
    std::map<int32_t, std::vector<UiPoint>> global_chunks_;
    std::map<int32_t, std::vector<UiPoint>> dynamic_chunks_;
    std::map<uint64_t, SE3> keyframe_poses_;
};

WasmUiClient* g_client = nullptr;

void MainLoopStep() {
    if (g_client) {
        g_client->Tick();
    }
}

}  // namespace

// 宿主页面(web/)在加载本模块前把ws地址写到window.LIGHTNING_WS_URL上；不设置的话退回
// localhost，方便脱离React页面单独调试。
EM_JS(char*, GetWsUrlFromHostPage, (), {
    var url = (typeof window !== 'undefined' && window.LIGHTNING_WS_URL) || 'ws://localhost:9877';
    var len = lengthBytesUTF8(url) + 1;
    var buf = _malloc(len);
    stringToUTF8(url, buf, len);
    return buf;
});

int main(int argc, char** argv) {
    static WasmUiClient client;
    g_client = &client;

    char* ws_url = GetWsUrlFromHostPage();
    client.Init(ws_url);
    free(ws_url);

    emscripten_set_main_loop(MainLoopStep, 0, 1);
    return 0;
}
