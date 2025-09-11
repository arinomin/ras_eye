#include <iostream>
#include <chrono> // 時間計測用
#include <thread>   // sleep用
#include <pigpio.h> // pigpioライブラリ

// --- グローバル定数 ---
const int TRIG_PIN = 23;    // 超音波センサーのTrigピンのGPIO番号 (要確認)
const int ECHO_PIN = 24;    // 超音波センサーのEchoピンのGPIO番号 (要確認)
const int LED_PIN = 27;     // 警告用LEDのGPIO番号 (要確認)

const float WARNING_DISTANCE_CM = 45.0; // 警告を発する距離のしきい値 (cm)
const float SOUND_SPEED_CM_PER_S = 34300.0; // 音速 (cm/s)

// --- 関数: 超音波センサーで距離を測定する ---
float get_distance_ultrasonic() {
    // TrigピンをLowにして安定させる (念のため)
    gpioWrite(TRIG_PIN, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(2));

    // TrigピンをHighに10マイクロ秒間設定
    gpioWrite(TRIG_PIN, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    gpioWrite(TRIG_PIN, 0); // HighからLowに戻す

    // EchoピンがHighになるのを待つ
    // タイムアウトを追加し、無限ループを防ぐ
    auto start_time_us = gpioTick(); // pigpioのマイクロ秒単位のタイムスタンプ
    while (gpioRead(ECHO_PIN) == 0) {
        if (gpioTick() - start_time_us > 50000) { // 50msタイムアウト (4m先まで測るには十分)
            std::cerr << "DEBUG: Echo low timeout.\n";
            return -1.0; // 測定失敗
        }
        start_time_us = gpioTick(); // Highになる直前の時間を更新
    }
    long start_tick = start_time_us; // Highになった時刻

    // EchoピンがLowになるのを待つ
    auto end_time_us = gpioTick(); // pigpioのマイクロ秒単位のタイムスタンプ
    while (gpioRead(ECHO_PIN) == 1) {
        if (gpioTick() - end_time_us > 50000) { // 50msタイムアウト
            std::cerr << "DEBUG: Echo high timeout.\n";
            return -1.0; // 測定失敗
        }
        end_time_us = gpioTick(); // Lowになる直前の時間を更新
    }
    long end_tick = end_time_us; // Lowになった時刻

    // パルスの持続時間を計算 (マイクロ秒)
    long pulse_duration_us = end_tick - start_tick;

    // 距離を計算 (cm)
    // 距離 = (時間 * 音速) / 2 (往復のため)
    // 時間はマイクロ秒なので、秒に変換 ( / 1,000,000)
    float distance_cm = (pulse_duration_us / 1000000.0) * SOUND_SPEED_CM_PER_S / 2.0;

    // 物理的にありえない値や異常値をフィルタリング
    if (distance_cm < 0.0 || distance_cm > 400.0) { // 例: 4m以上は無効
        return -1.0; // 測定失敗
    }

    return distance_cm;
}

// --- 関数: LEDを制御する ---
void set_warning_led(bool on) {
    gpioWrite(LED_PIN, on ? 1 : 0);
    std::cout << "LED State: " << (on ? "ON" : "OFF") << std::endl;
}

// --- メイン関数 ---
int main() {
    std::cout << "--- Ultrasonic Sensor & LED Test Program ---" << std::endl;

    // 1. pigpioの初期化
    if (gpioInitialise() < 0) {
        std::cerr << "ERROR: pigpio initialisation failed\n";
        return 1;
    }
    std::cout << "DEBUG: pigpio initialized." << std::endl;

    // 2. GPIOピンモード設定
    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
    gpioSetMode(LED_PIN, PI_OUTPUT);

    // 初期化: TrigをLow, LEDをOff
    gpioWrite(TRIG_PIN, 0);
    gpioWrite(LED_PIN, 0);
    std::cout << "DEBUG: GPIO pin modes set and initialized." << std::endl;

    // 3. メインループ
    while (true) {
        float distance = get_distance_ultrasonic();

        if (distance > 0) { // 距離が正常に測定できた場合
            std::cout << "Measured Distance: " << std::fixed << std::setprecision(1) << distance << " cm" << std::endl;
            if (distance <= WARNING_DISTANCE_CM) {
                set_warning_led(true); // 警告距離以下ならLED点灯
            } else {
                set_warning_led(false); // 警告距離より遠いならLED消灯
            }
        } else { // 測定失敗した場合
            std::cout << "Distance measurement failed." << std::endl;
            set_warning_led(false); // LEDを消灯
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5秒ごとに測定
    }

    // 4. 終了処理 (通常到達しない)
    gpioTerminate(); // pigpioの終了
    std::cout << "Program terminated." << std::endl;
    return 0;
}
