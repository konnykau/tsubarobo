// robomaster_ros2_ctrl.cpp  (JSON廃止 + カスタムmsg + 型別PIDゲイン)

#include <rclcpp/rclcpp.hpp>
#include "robomas_driver/msg/motor_cmd.hpp"
#include "robomas_driver/msg/motor_cmd_array.hpp"
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <atomic>
#include <array>
#include <string>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

// SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cerrno>

// --------- 設定 ----------
static constexpr int   NUM_MOTORS    = 8;
static constexpr char  CAN_IFACE[]   = "can0";
static constexpr int   TX_HZ         = 500;          // 送信周期(Hz)
static constexpr float CTRL_HZ       = 500.0f;       // 計算周期(Hz)
static constexpr float DT            = 1.0f / CTRL_HZ;

static constexpr int   CAN_ID_TX_1_4 = 0x200;        // ID1-4へ電流コマンド
static constexpr int   CAN_ID_TX_5_8 = 0x1FF;        // ID5-8へ電流コマンド
// フィードバックは 0x201..0x208

static constexpr int16_t CMD_CURR_MAX = 15000;      // 安全上限
static constexpr float AMP_TO_CMD = 800.0f;         // 1A -> 約800cmd（要実機調整）

static inline float rpm_to_rev(float rpm) { return rpm / 60.0f; }

// --------- 型定義 ----------
enum class MotorType { M2006, M3508 };
enum class CtrlMode  : uint8_t { CURRENT=0, SPEED=1, POSITION=2 };

struct Pid {
    float kp{0}, ki{0}, kd{0};
    float i_sum{0}, prev_e{0};
    float out_min{-1e9f}, out_max{1e9f};

    float step(float e, float dt) {
        i_sum += e * dt;
        float d = (e - prev_e) / dt;
        prev_e = e;
        float u = kp*e + ki*i_sum + kd*d;
        if (u > out_max) u = out_max;
        if (u < out_min) u = out_min;
        return u;
    }
    void reset() { i_sum=0; prev_e=0; }
};

struct MotorState {
    std::atomic<MotorType> type{MotorType::M3508};
    std::atomic<CtrlMode>  mode{CtrlMode::CURRENT};

    std::atomic<float> target{0.0f};     // 指令値（mode依存）

    // 計測
    std::atomic<float> meas_rpm{0.0f};
    std::atomic<int>   meas_enc{0};      // 0..8191
    std::atomic<float> est_rev{0.0f};    // 推定位置[rev]（速度積分）

    // PID（カスケード）
    Pid pid_pos;   // e[rev] -> target rpm
    Pid pid_spd;   // e[rpm] -> target current[A]

    std::atomic<int16_t> cmd_curr{0}; // 送信用コマンド値
};

static std::array<MotorState, NUM_MOTORS> M;
static std::atomic<bool> running{true};

// ---- 型別ゲイン ----
struct Gains {
    struct { float kp, ki, kd, out_min, out_max; } pos; // [rev]->rpm
    struct { float kp, ki, kd, out_min, out_max; } spd; // [rpm]->A
};

static inline void setup_pid(MotorState& m, const Gains& g) {
    m.pid_pos.kp = g.pos.kp;  m.pid_pos.ki = g.pos.ki;  m.pid_pos.kd = g.pos.kd;
    m.pid_pos.out_min = g.pos.out_min; m.pid_pos.out_max = g.pos.out_max; m.pid_pos.reset();

    m.pid_spd.kp = g.spd.kp;  m.pid_spd.ki = g.spd.ki;  m.pid_spd.kd = g.spd.kd;
    m.pid_spd.out_min = g.spd.out_min; m.pid_spd.out_max = g.spd.out_max; m.pid_spd.reset();
}

static inline Gains default_gains_for(MotorType t) {
    Gains g{};
    if (t == MotorType::M3508) {
        g.pos = { 300.f, 0.0f, 10.f,  -3000.f, 3000.f };
        g.spd = { 0.0200f, 0.0030f, 0.0f,  -18.f, 18.f };
    } else { // M2006
        g.pos = { 300.f, 0.0f, 10.f,  -10000.f, 10000.f };
        g.spd = { 0.0200f, 0.0030f, 0.0f,  -12.f, 12.f };
    }
    return g;
}

// ---- SocketCAN ----
static int open_can(const char* ifname) {
    int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return -1; }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); ::close(s); return -1; }

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); ::close(s); return -1; }

    // 受信タイムアウト（Ctrl+Cで止まりやすく）
    timeval tv{}; tv.tv_sec = 0; tv.tv_usec = 50 * 1000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return s;
}

// ---- RX ----
void rx_thread_func(int s) {
    while (running.load()) {
        struct can_frame fr{};
        int n = ::read(s, &fr, sizeof(fr));
        if (n < 0) {
            if (!running.load()) break;
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (fr.can_dlc < 7) continue;

        if (fr.can_id >= 0x201 && fr.can_id <= 0x208) {
            int idx = (int)fr.can_id - 0x201; // 0..7
            if (idx >= 0 && idx < NUM_MOTORS) {
                uint8_t* d = fr.data;
                int angle = (int)((d[0] << 8) | d[1]) & 0x3FFF; // 0..8191
                int16_t rpm = (int16_t)((d[2] << 8) | d[3]);
                M[idx].meas_enc.store(angle);
                M[idx].meas_rpm.store((float)rpm);

                // 位置推定（速度積分）
                float prev = M[idx].est_rev.load();
                float next = prev + rpm_to_rev((float)rpm) * (1.0f / TX_HZ);
                M[idx].est_rev.store(next);
            }
        }
    }
}

// ---- TX ----
static inline void pack_currents(int16_t c1, int16_t c2, int16_t c3, int16_t c4, can_frame& fr) {
    fr.can_dlc = 8;
    fr.data[0] = (uint8_t)((c1 >> 8) & 0xFF);
    fr.data[1] = (uint8_t)(c1 & 0xFF);
    fr.data[2] = (uint8_t)((c2 >> 8) & 0xFF);
    fr.data[3] = (uint8_t)(c2 & 0xFF);
    fr.data[4] = (uint8_t)((c3 >> 8) & 0xFF);
    fr.data[5] = (uint8_t)(c3 & 0xFF);
    fr.data[6] = (uint8_t)((c4 >> 8) & 0xFF);
    fr.data[7] = (uint8_t)(c4 & 0xFF);
}

void tx_thread_func(int s) {
    using namespace std::chrono;
    auto period = milliseconds(1000 / TX_HZ);
    while (running.load()) {
        auto t0 = steady_clock::now();

        // 1-4
        {
            struct can_frame fr{};
            fr.can_id = CAN_ID_TX_1_4;
            int16_t c1 = M[0].cmd_curr.load();
            int16_t c2 = M[1].cmd_curr.load();
            int16_t c3 = M[2].cmd_curr.load();
            int16_t c4 = M[3].cmd_curr.load();
            pack_currents(c1, c2, c3, c4, fr);
            ::write(s, &fr, sizeof(fr));
        }
        // 5-8
        {
            struct can_frame fr{};
            fr.can_id = CAN_ID_TX_5_8;
            int16_t c5 = M[4].cmd_curr.load();
            int16_t c6 = M[5].cmd_curr.load();
            int16_t c7 = M[6].cmd_curr.load();
            int16_t c8 = M[7].cmd_curr.load();
            pack_currents(c5, c6, c7, c8, fr);
            ::write(s, &fr, sizeof(fr));
        }

        auto t1 = steady_clock::now();
        auto dt = duration_cast<milliseconds>(t1 - t0);
        if (dt < period) std::this_thread::sleep_for(period - dt);
    }
}

// ---- 制御計算 ----
void compute_thread_func() {
    using namespace std::chrono;
    auto period = duration<double>(1.0 / CTRL_HZ);
    while (running.load()) {
        auto t0 = steady_clock::now();

        for (int i = 0; i < NUM_MOTORS; ++i) {
            auto mode = M[i].mode.load();
            float tgt  = M[i].target.load();
            float rpm  = M[i].meas_rpm.load();
            float rev  = M[i].est_rev.load();

            float out_current_A = 0.0f;

            switch (mode) {
            case CtrlMode::CURRENT:
                out_current_A = tgt; // A
                break;

            case CtrlMode::SPEED: {
                float e_rpm = tgt - rpm;
                out_current_A = M[i].pid_spd.step(e_rpm, DT);
            } break;

            case CtrlMode::POSITION: {
                float e_rev = tgt - rev;                     // 位置誤差[rev]
                float tgt_rpm = M[i].pid_pos.step(e_rev, DT);// 位置PID -> 目標rpm
                float e_rpm = tgt_rpm - rpm;                 // 速度誤差
                out_current_A = M[i].pid_spd.step(e_rpm, DT);
            } break;
            }

            int cmd = (int)std::lround(out_current_A * AMP_TO_CMD);
            if (cmd >  CMD_CURR_MAX) cmd =  CMD_CURR_MAX;
            if (cmd < -CMD_CURR_MAX) cmd = -CMD_CURR_MAX;
            M[i].cmd_curr.store((int16_t)cmd);
        }

        auto t1 = steady_clock::now();
        auto dt = t1 - t0;
        if (dt < period) std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(period - dt));
    }
}

// ---- ユーティリティ ----
static inline bool ieq(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i=0;i<a.size();++i) if (std::tolower(a[i]) != std::tolower(b[i])) return false;
    return true;
}

static inline std::optional<MotorType> motor_type_from(const std::string& s) {
    if (ieq(s,"M3508")) return MotorType::M3508;
    if (ieq(s,"M2006")) return MotorType::M2006;
    return std::nullopt;
}

static inline std::optional<CtrlMode> ctrl_mode_from(uint8_t m) {
    if (m==0) return CtrlMode::CURRENT;
    if (m==1) return CtrlMode::SPEED;
    if (m==2) return CtrlMode::POSITION;
    return std::nullopt;
}

// ---- ROS2 ノード ----
class RmNode : public rclcpp::Node {
public:
    RmNode() : Node("rm_ctrl_node") {
        sub_ = this->create_subscription<robomas_driver::msg::MotorCmdArray>(
            "/rm_cmd_array", 10,
            std::bind(&RmNode::on_cmd_array, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /rm_cmd_array (MotorCmdArray)");
    }

    Gains gains_m3508_{ default_gains_for(MotorType::M3508) };
    Gains gains_m2006_{ default_gains_for(MotorType::M2006) };

private:
    void on_cmd_array(const robomas_driver::msg::MotorCmdArray::SharedPtr msg) {
        for (const auto& c : msg->cmds) {
            if (c.id < 1 || c.id > NUM_MOTORS) {
                RCLCPP_WARN(this->get_logger(), "Invalid id=%d", c.id);
                continue;
            }
            int idx = c.id - 1;

            auto mt = motor_type_from(c.type);
            if (!mt) {
                RCLCPP_WARN(this->get_logger(), "Unknown type=%s", c.type.c_str());
                continue;
            }
            M[idx].type.store(*mt);

            auto md = ctrl_mode_from(c.mode);
            if (!md) {
                RCLCPP_WARN(this->get_logger(), "Unknown mode=%u", c.mode);
                continue;
            }
            M[idx].mode.store(*md);

            // タイプ変更時：対応ゲインを即反映（積分もリセット）
            setup_pid(M[idx], (*mt == MotorType::M3508) ? gains_m3508_ : gains_m2006_);

            M[idx].target.store(c.value);
        }
    }

    rclcpp::Subscription<robomas_driver::msg::MotorCmdArray>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    // 初期化（安全側）
    for (int i=0;i<NUM_MOTORS;++i) {
        M[i].mode.store(CtrlMode::CURRENT);
        M[i].target.store(0.0f);
        M[i].cmd_curr.store(0);
        M[i].pid_pos.reset();
        M[i].pid_spd.reset();
        // 既定タイプに対してゲイン適用（初期はM3508）
        auto t = M[i].type.load();
        setup_pid(M[i], default_gains_for(t));
    }

    // CAN
    int s = open_can(CAN_IFACE);
    if (s < 0) {
        std::cerr << "Failed to open CAN. Is " << CAN_IFACE << " up?\n";
        return 1;
    }

    // ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RmNode>();

    // シャットダウンフック（Ctrl+Cで停止しやすく）
    rclcpp::on_shutdown([&](){ running.store(false); });

    // スレッド起動
    std::thread th_rx(rx_thread_func, s);
    std::thread th_tx(tx_thread_func, s);
    std::thread th_cp(compute_thread_func);

    rclcpp::spin(node);

    // 終了
    th_rx.join();
    th_tx.join();
    th_cp.join();
    ::close(s);
    rclcpp::shutdown();
    return 0;
}
