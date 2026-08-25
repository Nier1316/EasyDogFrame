#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

/**
 * @file    xbox_controller.h
 * @brief   SDL2 Xbox 手柄输入封装
 *
 * 通过 SDL2 GameController API 读取 Xbox 手柄状态，
 * 提供归一化后的轴值和按键状态，供 Example21 使用。
 */

#include <SDL2/SDL.h>
#include <cstdint>
#include <cmath>

struct XboxState {
    // 左摇杆: -1.0 ~ +1.0
    float left_stick_x;
    float left_stick_y;

    // 右摇杆: -1.0 ~ +1.0
    float right_stick_x;
    float right_stick_y;

    // 扳机: 0.0 (松开) ~ 1.0 (完全按下)
    float left_trigger;
    float right_trigger;

    // 按键: true = 按下
    bool a, b, x, y;
    bool lb, rb;
    bool back, start;
    bool ls, rs;
    bool dpad_up, dpad_down, dpad_left, dpad_right;
};

class XboxController {
public:
    XboxController() = default;
    ~XboxController() { Shutdown(); }

    // 初始化 SDL2 手柄子系统，打开第一个可用手柄
    bool Initialize();

    // 关闭手柄，关闭 SDL2
    void Shutdown();

    // 轮询 SDL2 事件，更新内部状态（每控制周期调用一次）
    void Poll();

    // 获取最新状态
    const XboxState& GetState() const { return state_; }

    // 手柄是否已连接
    bool IsConnected() const { return controller_ != nullptr; }

    // 设置摇杆死区（归一化 0~1，默认 0.20 = 20%）。死区内的摇杆输入归零，
    // 用于过滤摇杆回中不准 / 轻微漂移造成的指令抖动。
    void SetDeadzone(float dz) { deadzone_ = dz; }
    float GetDeadzone() const { return deadzone_; }

private:
    SDL_GameController* controller_ = nullptr;
    XboxState state_ = {};
    float deadzone_ = 0.20f;   // 摇杆死区（20%，用户实测调大）
};

inline bool XboxController::Initialize() {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        printf("[XboxController] SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    // 蓝牙手柄在 SDL_Init 之后可能还没被枚举到，重试等待最多 ~3 秒。
    // SDL 通过事件队列发现热插拔设备，需 PumpEvents 驱动枚举。
    const int  kMaxTries  = 30;
    const int  kDelayMs   = 100;
    for (int attempt = 0; attempt < kMaxTries; attempt++) {
        SDL_PumpEvents();
        int num_joysticks = SDL_NumJoysticks();

        for (int i = 0; i < num_joysticks; i++) {
            if (SDL_IsGameController(i)) {
                controller_ = SDL_GameControllerOpen(i);
                if (controller_) {
                    const char* name = SDL_GameControllerName(controller_);
                    printf("[XboxController] Connected: %s (等待 %d ms)\n",
                           name ? name : "Unknown", attempt * kDelayMs);
                    return true;
                }
            }
        }

        if (attempt == 0) {
            printf("[XboxController] 等待手柄就绪...(检测到 %d 个 joystick)\n", num_joysticks);
        }
        SDL_Delay(kDelayMs);
    }

    printf("[XboxController] No compatible game controller found（超时）\n");
    return false;
}

inline void XboxController::Shutdown() {
    if (controller_) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }
    SDL_Quit();
}

inline void XboxController::Poll() {
    if (!controller_) return;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_CONTROLLERAXISMOTION: {
                int16_t raw = event.caxis.value;
                float val;

                switch (event.caxis.axis) {
                    case SDL_CONTROLLER_AXIS_LEFTX:
                    case SDL_CONTROLLER_AXIS_LEFTY:
                    case SDL_CONTROLLER_AXIS_RIGHTX:
                    case SDL_CONTROLLER_AXIS_RIGHTY:
                        // 摇杆死区（默认 20%，可 SetDeadzone 调整）：死区内归零
                        if (std::abs(raw) < (int)(SDL_JOYSTICK_AXIS_MAX * deadzone_)) {
                            val = 0.0f;
                        } else {
                            val = (float)raw / SDL_JOYSTICK_AXIS_MAX;
                        }
                        break;
                    case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                    case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                        // 扳机: 5% 死区，归一化到 [0, 1]
                        if (raw < SDL_JOYSTICK_AXIS_MAX / 20) {
                            val = 0.0f;
                        } else {
                            val = (float)raw / SDL_JOYSTICK_AXIS_MAX;
                        }
                        break;
                    default:
                        continue;
                }

                switch (event.caxis.axis) {
                    case SDL_CONTROLLER_AXIS_LEFTX:        state_.left_stick_x  = val; break;
                    case SDL_CONTROLLER_AXIS_LEFTY:        state_.left_stick_y  = val; break;
                    case SDL_CONTROLLER_AXIS_RIGHTX:       state_.right_stick_x = val; break;
                    case SDL_CONTROLLER_AXIS_RIGHTY:       state_.right_stick_y = val; break;
                    case SDL_CONTROLLER_AXIS_TRIGGERLEFT:  state_.left_trigger  = val; break;
                    case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: state_.right_trigger = val; break;
                }
                break;
            }

            case SDL_CONTROLLERBUTTONDOWN:
            case SDL_CONTROLLERBUTTONUP: {
                bool pressed = (event.type == SDL_CONTROLLERBUTTONDOWN);
                switch (event.cbutton.button) {
                    case SDL_CONTROLLER_BUTTON_A:             state_.a = pressed; break;
                    case SDL_CONTROLLER_BUTTON_B:             state_.b = pressed; break;
                    case SDL_CONTROLLER_BUTTON_X:             state_.x = pressed; break;
                    case SDL_CONTROLLER_BUTTON_Y:             state_.y = pressed; break;
                    case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:  state_.lb = pressed; break;
                    case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: state_.rb = pressed; break;
                    case SDL_CONTROLLER_BUTTON_BACK:          state_.back = pressed; break;
                    case SDL_CONTROLLER_BUTTON_START:         state_.start = pressed; break;
                    case SDL_CONTROLLER_BUTTON_LEFTSTICK:     state_.ls = pressed; break;
                    case SDL_CONTROLLER_BUTTON_RIGHTSTICK:    state_.rs = pressed; break;
                    case SDL_CONTROLLER_BUTTON_DPAD_UP:       state_.dpad_up    = pressed; break;
                    case SDL_CONTROLLER_BUTTON_DPAD_DOWN:     state_.dpad_down  = pressed; break;
                    case SDL_CONTROLLER_BUTTON_DPAD_LEFT:     state_.dpad_left  = pressed; break;
                    case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:    state_.dpad_right = pressed; break;
                }
                break;
            }

            case SDL_CONTROLLERDEVICEREMOVED:
                printf("[XboxController] Controller disconnected\n");
                SDL_GameControllerClose(controller_);
                controller_ = nullptr;
                break;
        }
    }
}

#endif // XBOX_CONTROLLER_H
