#include <iostream>
#include <vector>
#include <chrono>   // 時間計測用
#include <thread>   // sleep用
#include <cmath>    // abs用
#include <iomanip>  // 距離表示の小数点制御用

// --- OpenCV関連 ---
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.h> // Haar Cascade用

// --- Raspberry Pi GPIO制御関連 ---
#include <pigpio.h>

// --- グローバル定数と調整パラメータ ---
// GPIOピン番号の定義
const int PAN_SERVO_PIN = 17;   // パン用サーボモーターのGPIOピン番号
const int TILT_SERVO_PIN = 18;  // チルト用サーボモーターのGPIOピン番号
const int TRIG_PIN = 23;        // 超音波センサーのTrigピンのGPIOピン番号
const int ECHO_PIN = 24;        // 超音波センサーのEchoピンのGPIOピン番号
const int LED_PIN = 27;         // 警告用LEDのGPIOピン番号

// カメラ設定
const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
const int CAMERA_CENTER_X = CAMERA_WIDTH / 2;
const int CAMERA_CENTER_Y = CAMERA_HEIGHT / 2;

// サーボ制御パラメータ (要調整)
const float Kp_PAN = 0.005;     // パン用サーボのP制御比例定数
const float Kp_TILT = 0.005;    // チルト用サーボのP制御比例定数
const int DEAD_ZONE = 15;       // 中心から±DEAD_ZONEピクセルは無視

// 警告設定
const float DISTANCE_THRESHOLD = 40.0; // 警告を発する距離のしきい値 (cm)

// --- グローバル変数 (状態保持用) ---
// 現在のサーボ角度 (PWM値)
float g_current_pan_angle = 1500;
float g_current_tilt_angle = 1500;

// OpenCV 顔検出器
cv::CascadeClassifier g_face_cascade;
const std::string FACE_CASCADE_PATH = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml"; // 明日、正確なパスを確認！

// --- 関数宣言 (プロトタイプ) ---
void setup_gpio();
void setup_opencv(cv::VideoCapture& cap);
cv::Point find_nose(const cv::Mat& frame);
void control_pan_tilt(int nose_x, int nose_y);
float get_distance_ultrasonic();
void set_warning_led(bool on);

// --- 関数定義 ---

// GPIO初期設定 (Bさん, Cさん担当箇所)
void setup_gpio() {
    if (gpioInitialise() < 0) {
        std::cerr << "ERROR: pigpio initialisation failed\n";
        exit(1);
    }

    gpioSetMode(PAN_SERVO_PIN, PI_OUTPUT);
    gpioSetMode(TILT_SERVO_PIN, PI_OUTPUT);
    gpioServo(PAN_SERVO_PIN, static_cast<unsigned int>(g_current_pan_angle));
    gpioServo(TILT_SERVO_PIN, static_cast<unsigned int>(g_current_tilt_angle));

    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
    gpioWrite(TRIG_PIN, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(2));

    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioWrite(LED_PIN, 0);
}

// OpenCV初期設定 (Aさん担当箇所)
void setup_opencv(cv::VideoCapture& cap) {
    if (!g_face_cascade.load(FACE_CASCADE_PATH)) {
        std::cerr << "ERROR: Could not load face cascade classifier [" << FACE_CASCADE_PATH << "]\n";
        gpioTerminate();
        exit(1);
    }

    cap.open(0); // 0は通常USBカメラまたはCSIカメラ
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open camera\n";
        gpioTerminate();
        exit(1);
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
}

// 顔検出 (Aさん担当箇所)
// 検出できなかった場合は x=-1, y=-1 を持つPointを返す
cv::Point find_nose(const cv::Mat& frame) {
    std::vector<cv::Rect> faces;
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray_frame, gray_frame);
    g_face_cascade.detectMultiScale(gray_frame, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    if (!faces.empty()) {
        size_t largest_face_idx = 0;
        for (size_t i = 1; i < faces.size(); ++i) {
            if (faces[i].area() > faces[largest_face_idx].area()) {
                largest_face_idx = i;
            }
        }
        cv::Rect largest_face = faces[largest_face_idx];
        return cv::Point(largest_face.x + largest_face.width / 2, largest_face.y + largest_face.height / 2);
    }
    return cv::Point(-1, -1); // 検出できなかった
}

// パン・チルト制御 (あなた担当箇所)
void control_pan_tilt(int nose_x, int nose_y) {
    if (nose_x == -1 || nose_y == -1) { // 鼻が検出されていない場合は動かさない
        // 現在位置を維持 (サーボは動かさない)
        // gpioServo(PAN_SERVO_PIN, static_cast<unsigned int>(g_current_pan_angle));
        // gpioServo(TILT_SERVO_PIN, static_cast<unsigned int>(g_current_tilt_angle));
        return;
    }

    int error_x = nose_x - CAMERA_CENTER_X;
    int error_y = nose_y - CAMERA_CENTER_Y;

    if (std::abs(error_x) > DEAD_ZONE) {
        g_current_pan_angle -= Kp_PAN * error_x; // 符号は要調整 (カメラとサーボの向きによる)
        if (g_current_pan_angle < 1000) g_current_pan_angle = 1000;
        if (g_current_pan_angle > 2000) g_current_pan_angle = 2000;
        gpioServo(PAN_SERVO_PIN, static_cast<unsigned int>(g_current_pan_angle));
    }

    if (std::abs(error_y) > DEAD_ZONE) {
        g_current_tilt_angle += Kp_TILT * error_y; // 符号は要調整
        if (g_current_tilt_angle < 1000) g_current_tilt_angle = 1000;
        if (g_current_tilt_angle > 2000) g_current_tilt_angle = 2000;
        gpioServo(TILT_SERVO_PIN, static_cast<unsigned int>(g_current_tilt_angle));
    }
}

// 超音波センサーによる距離測定 (Bさん担当箇所)
float get_distance_ultrasonic() {
    // TrigをHighにしてパルスを送る
    gpioWrite(TRIG_PIN, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(10)); // 10us待つ
    gpioWrite(TRIG_PIN, 0);

    auto pulse_start_time = std::chrono::high_resolution_clock::now();
    auto pulse_end_time = std::chrono::high_resolution_clock::now();

    // EchoピンがHighになるのを待つ (タイムアウト追加)
    // タイムアウトを短く設定しすぎると、距離が遠い場合に間に合わない可能性があります
    // 例えば、4m先を測るなら約23msかかる
    auto timeout_start = std::chrono::high_resolution_clock::now();
    while (gpioRead(ECHO_PIN) == 0) {
        pulse_start_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(pulse_start_time - timeout_start).count() > 0.1) return 999.0; // 100msタイムアウト
    }
    
    // EchoピンがLowになるのを待つ (タイムアウト追加)
    timeout_start = std::chrono::high_resolution_clock::now(); // タイムアウト開始時間をリセット
    while (gpioRead(ECHO_PIN) == 1) {
        pulse_end_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(pulse_end_time - timeout_start).count() > 0.1) return 999.0; // 100msタイムアウト
    }

    std::chrono::duration<double> pulse_duration = pulse_end_time - pulse_start_time;
    float distance_cm = pulse_duration.count() * 34300.0 / 2.0; // 音速 34300 cm/s

    // 異常値のフィルタリング (物理的にありえない値)
    if (distance_cm < 0.0 || distance_cm > 400.0) { // 400cm (4m) 以上は無効と判断
        return 999.0; // 無効な値を示す
    }
    return distance_cm;
}

// 警告LEDの制御 (Cさん担当箇所)
void set_warning_led(bool on) {
    gpioWrite(LED_PIN, on ? 1 : 0);
}

// --- メイン関数 (すべての機能を呼び出す中心) ---
int main() {
    // 1. 全体の初期設定
    setup_gpio();
    cv::VideoCapture cap;
    setup_opencv(cap);

    // デバッグ用表示ウィンドウ (必要に応じてコメントアウト)
    // cv::namedWindow("Ras-Eye Frame", cv::WINDOW_AUTOSIZE);

    // 2. メインループ
    while (true) {
        // 2-1. カメラからのフレーム取得
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Failed to capture frame. Exiting.\n";
            break;
        }

        // 2-2. 顔検出と鼻の座標取得
        cv::Point nose_position = find_nose(frame);

        // 2-3. パン・チルト制御
        control_pan_tilt(nose_position.x, nose_position.y);

        // 2-4. サーボの動きが安定するまで少し待つ
        if (nose_position.x != -1) { // 顔が検出されている場合のみ待機
             std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 2-5. 超音波センサーによる距離測定
        float distance_cm = get_distance_ultrasonic();

        // 2-6. LEDによるフィードバック
        if (distance_cm != 999.0 && distance_cm < DISTANCE_THRESHOLD) { // 距離が有効で、しきい値より近い場合
            set_warning_led(true); // LED点灯
        } else {
            set_warning_led(false); // LED消灯
        }

        // デバッグ用: コンソールに距離を表示 (必要に応じてコメントアウト)
        if (distance_cm != 999.0) {
            std::cout << "Distance: " << std::fixed << std::setprecision(1) << distance_cm << " cm" << std::endl;
        } else {
            std::cout << "Distance: Out of range / Error" << std::endl;
        }
        // 顔検出のデバッグ表示 (必要に応じてコメントアウト)
        // if (nose_position.x != -1) {
        //     cv::circle(frame, nose_position, 5, cv::Scalar(0, 0, 255), -1);
        // }
        // cv::imshow("Ras-Eye Frame", frame);
        // if (cv::waitKey(1) == 'q') break; // 'q'で終了

        // ループの最後に少し待機 (CPU負荷軽減と処理間隔調整)
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    // 3. 終了処理
    cap.release();
    gpioTerminate(); // pigpioの終了
    return 0;
}
