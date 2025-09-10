#include <iostream>
#include <vector>
#include <chrono> // 時間計測用
#include <thread>   // sleep用
#include <cmath>    // abs用

// --- OpenCV関連 ---
// OpenCVのヘッダーは、OpenCVがインストールされている環境で利用可能
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> // カメラ操作用
#include <opencv2/imgproc.hpp> // 画像処理用
#include <opencv2/objdetect.hpp> // 顔検出用

// --- Raspberry Pi GPIO制御関連 ---
// pigpioライブラリのヘッダー
// Raspberry Pi上でpigpioデーモンが起動している必要があります
#include <pigpio.h>

// --- グローバル変数 (部品のピン番号など) ---
// GPIOピン番号の定義
const int PAN_SERVO_PIN = 17; // パン用サーボモーターのGPIOピン番号
const int TILT_SERVO_PIN = 18; // チルト用サーボモーターのGPIOピン番号
const int TRIG_PIN = 23;      // 超音波センサーのTrigピンのGPIOピン番号
const int ECHO_PIN = 24;      // 超音波センサーのEchoピンのGPIOピン番号
const int LED_PIN = 27;       // 警告用LEDのGPIOピン番号

// 初期サーボ角度 (中心付近の値に調整してください)
float current_pan_angle = 1500; // PWM値 (1000-2000が一般的, 1500で中央)
float current_tilt_angle = 1500; // PWM値

// --- OpenCV 顔検出器 (グローバルまたはmain関数内で初期化) ---
cv::CascadeClassifier face_cascade;
// Haar Cascadeファイルのパス (Raspberry Pi上のパスを指定)
// 明日、このパスを正確に入力する必要があります！
// 通常は /usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml など
const std::string FACE_CASCADE_PATH = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml";

// --- 定数と調整パラメータ ---
const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
const int CAMERA_CENTER_X = CAMERA_WIDTH / 2;
const int CAMERA_CENTER_Y = CAMERA_HEIGHT / 2;

const float Kp_PAN = 0.005;   // パン用サーボのP制御比例定数 (要調整)
const float Kp_TILT = 0.005;  // チルト用サーボのP制御比例定数 (要調整)
const int DEAD_ZONE = 15;     // 中心から±DEAD_ZONEピクセルは無視 (要調整)

const float DISTANCE_THRESHOLD = 40.0; // 警告を発する距離のしきい値 (cm)

// --- メイン関数 ---
int main() {
    // 1. pigpioの初期化 (GPIO制御担当: Bさん, Cさん)
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialisation failed\n";
        return 1;
    }

    // 2. GPIOピンモード設定
    // サーボモーター (PWM出力)
    gpioSetMode(PAN_SERVO_PIN, PI_OUTPUT);
    gpioSetMode(TILT_SERVO_PIN, PI_OUTPUT);
    gpioServo(PAN_SERVO_PIN, current_pan_angle);  // 初期角度設定
    gpioServo(TILT_SERVO_PIN, current_tilt_angle); // 初期角度設定

    // 超音波センサー (Trig:出力, Echo:入力)
    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
    gpioWrite(TRIG_PIN, 0); // TrigピンをLowに初期化
    std::this_thread::sleep_for(std::chrono::microseconds(2)); // 安定化

    // LED (出力)
    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioWrite(LED_PIN, 0); // LEDをオフに初期化

    // 3. OpenCVの初期化 (画像解析担当: Aさん)
    // Haar Cascadeファイルの読み込み
    if (!face_cascade.load(FACE_CASCADE_PATH)) {
        std::cerr << "Error: Could not load face cascade classifier [" << FACE_CASCADE_PATH << "]\n";
        gpioTerminate();
        return -1;
    }

    // カメラの初期化
    cv::VideoCapture cap(0); // 0は通常USBカメラまたはCSIカメラ
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera\n";
        gpioTerminate();
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    // 4. メインループ (全担当)
    while (true) {
        cv::Mat frame;
        cap >> frame; // カメラからフレーム取得
        if (frame.empty()) {
            std::cerr << "Error: No frame captured\n";
            break;
        }

        // 4-1. 顔検出 (画像解析担当: Aさん)
        std::vector<cv::Rect> faces;
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray_frame, gray_frame);
        face_cascade.detectMultiScale(gray_frame, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

        // 鼻の座標 (最も大きい顔の中心を鼻と仮定)
        int nose_x = -1, nose_y = -1;
        if (!faces.empty()) {
            // 最も大きい顔を検出
            size_t largest_face_idx = 0;
            for (size_t i = 1; i < faces.size(); ++i) {
                if (faces[i].area() > faces[largest_face_idx].area()) {
                    largest_face_idx = i;
                }
            }
            cv::Rect largest_face = faces[largest_face_idx];
            nose_x = largest_face.x + largest_face.width / 2;
            nose_y = largest_face.y + largest_face.height / 2;

            // デバッグ用: 検出した顔と鼻の印
            // cv::rectangle(frame, largest_face, cv::Scalar(0, 255, 0), 2);
            // cv::circle(frame, cv::Point(nose_x, nose_y), 5, cv::Scalar(0, 0, 255), -1);
        }

        // 4-2. パン・チルト制御 (あなた)
        if (nose_x != -1 && nose_y != -1) { // 鼻が検出できた場合のみ
            int error_x = nose_x - CAMERA_CENTER_X;
            int error_y = nose_y - CAMERA_CENTER_Y;

            if (std::abs(error_x) > DEAD_ZONE) {
                current_pan_angle -= Kp_PAN * error_x; // 符号は要調整
                // 角度の範囲制限 (例: 1000-2000us)
                if (current_pan_angle < 1000) current_pan_angle = 1000;
                if (current_pan_angle > 2000) current_pan_angle = 2000;
                gpioServo(PAN_SERVO_PIN, static_cast<unsigned int>(current_pan_angle));
            }

            if (std::abs(error_y) > DEAD_ZONE) {
                current_tilt_angle += Kp_TILT * error_y; // 符号は要調整
                // 角度の範囲制限
                if (current_tilt_angle < 1000) current_tilt_angle = 1000;
                if (current_tilt_angle > 2000) current_tilt_angle = 2000;
                gpioServo(TILT_SERVO_PIN, static_cast<unsigned int>(current_tilt_angle));
            }

            // サーボが動くのを少し待つ
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
            
            // 4-3. 超音波センサーによる距離測定 (Bさん)
            gpioWrite(TRIG_PIN, 1); // TrigをHighに
            std::this_thread::sleep_for(std::chrono::microseconds(10)); // 10us待つ
            gpioWrite(TRIG_PIN, 0); // TrigをLowに

            auto pulse_start_time = std::chrono::high_resolution_clock::now();
            auto pulse_end_time = std::chrono::high_resolution_clock::now();

            // EchoピンがHighになるのを待つ
            while (gpioRead(ECHO_PIN) == 0 && std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - pulse_start_time).count() < 0.05) {
                pulse_start_time = std::chrono::high_resolution_clock::now();
            }
            // EchoピンがLowになるのを待つ
            while (gpioRead(ECHO_PIN) == 1 && std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - pulse_end_time).count() < 0.05) {
                pulse_end_time = std::chrono::high_resolution_clock::now();
            }

            // タイムスタンプをdurationに変換
            std::chrono::duration<double> pulse_duration = pulse_end_time - pulse_start_time;
            double duration_s = pulse_duration.count();

            // 距離を計算 (音速 34300 cm/s, 往復なので /2)
            float distance_cm = duration_s * 34300 / 2;

            // 異常値のフィルタリング (例: 0cm以下または400cm以上は無視)
            if (distance_cm < 0 || distance_cm > 400) {
                 distance_cm = 999.0; // 無効な値として扱う
            }
            
            // 4-4. LEDによるフィードバック (Cさん)
            if (distance_cm < DISTANCE_THRESHOLD) {
                gpioWrite(LED_PIN, 1); // LED点灯
            } else {
                gpioWrite(LED_PIN, 0); // LED消灯
            }

            // デバッグ用: 距離をコンソールに表示
            // std::cout << "Distance: " << std::fixed << std::setprecision(1) << distance_cm << " cm" << std::endl;

        } else { // 鼻が検出できなかった場合
            gpioServo(PAN_SERVO_PIN, static_cast<unsigned int>(current_pan_angle)); // 現在位置維持
            gpioServo(TILT_SERVO_PIN, static_cast<unsigned int>(current_tilt_angle)); // 現在位置維持
            gpioWrite(LED_PIN, 0); // LED消灯
            // std::cout << "No face detected." << std::endl;
        }

        // ループの最後に少し待機 (CPU負荷軽減と表示更新間隔調整)
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms待機 = 10FPS
    }

    // 5. 終了処理 (到達しない可能性が高いが念のため)
    cap.release();
    gpioTerminate();
    return 0;
}
