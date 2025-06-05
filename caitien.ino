#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// Khởi tạo đối tượng PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Khởi tạo đối tượng PS2X
PS2X ps2x;

// Định nghĩa kênh cho Motor trên PCA9685
#define LEFT_MOTOR_CHANNEL_A 8
#define LEFT_MOTOR_CHANNEL_B 9
#define RIGHT_MOTOR_CHANNEL_A 14
#define RIGHT_MOTOR_CHANNEL_B 15

// Định nghĩa chân cho PS2 Controller (ESP32 pins)
#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS (ATT)
#define PS2_CLK 14 // SCK

// Cài đặt cho PS2 Controller
#define X_JOY_CALIB 127 

// Tốc độ
#define TOP_SPEED 4095
#define NORM_SPEED 2048
#define DPAD_GENTLE_TURN_SPEED_BASE 1000   // Giảm tốc độ rẽ bằng D-Pad
#define ANALOG_TURN_MAX_SPEED_BASE 1500 // Giảm tốc độ rẽ bằng Joystick
#define ANALOG_STICK_DEADZONE 25      // Tăng nhẹ deadzone nếu cần

// Cài đặt cho xoay 180 độ
#define ROTATE_180_SPEED_BASE 1800         // Tốc độ cơ sở khi xoay 180 độ
#define ROTATE_180_DURATION_MS 800      // THỜI GIAN (ms) ĐỂ XOAY 180 ĐỘ - CẦN CĂN CHỈNH THỰC TẾ

// Cài đặt điều chỉnh tốc độ động cơ
#define SPEED_ADJUST_STEP 0.03f         // Bước thay đổi tốc độ (nhỏ hơn để mịn hơn)
#define MIN_SPEED_MODIFIER 0.2f        // Hệ số tốc độ tối thiểu
#define MAX_SPEED_MODIFIER 1.5f        // Hệ số tốc độ tối đa

// Biến toàn cục
bool isPerforming180Turn = false;
unsigned long turn180StartTime = 0;
float overallSpeedModifier = 1.0f; // Bắt đầu ở tốc độ bình thường

// Bật/tắt debug qua Serial
#define DEBUG_CTRL

// Hàm điều khiển tốc độ và chiều cho 2 motor
void setMotorSpeed_Robot(int speedLeft, int speedRight) {
  // Giới hạn tốc độ trong khoảng cho phép của PWM (-TOP_SPEED đến TOP_SPEED)
  speedLeft = constrain(speedLeft, -TOP_SPEED, TOP_SPEED);
  speedRight = constrain(speedRight, -TOP_SPEED, TOP_SPEED);

  if (speedLeft >= 0) {
    pwm.setPin(LEFT_MOTOR_CHANNEL_A, speedLeft);
    pwm.setPin(LEFT_MOTOR_CHANNEL_B, 0);
  } else {
    pwm.setPin(LEFT_MOTOR_CHANNEL_A, 0);
    pwm.setPin(LEFT_MOTOR_CHANNEL_B, abs(speedLeft));
  }

  if (speedRight >= 0) {
    pwm.setPin(RIGHT_MOTOR_CHANNEL_A, speedRight);
    pwm.setPin(RIGHT_MOTOR_CHANNEL_B, 0);
  } else {
    pwm.setPin(RIGHT_MOTOR_CHANNEL_A, 0);
    pwm.setPin(RIGHT_MOTOR_CHANNEL_B, abs(speedRight));
  }
}

// Hàm cài đặt PS2 Controller
void setupPS2controller() {
  int err = -1;
  Serial.println("Configuring PS2 Controller...");
  unsigned long startTime = millis();
  while (err != 0) {
    err = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
    if (millis() - startTime > 3000 && err != 0) {
      Serial.println("Cannot connect to PS2 controller. Check wiring. Retrying...");
      startTime = millis(); 
    }
    delay(100);
  }
  Serial.println("PS2 Controller Configured!");
}

// Hàm xử lý điều khiển từ PS2
void PS2control() {
  int finalSpeedLeft = 0;
  int finalSpeedRight = 0;
  
  // 1. Xử lý tự động xoay 180 độ nếu đang thực hiện
  if (isPerforming180Turn) {
    if (millis() - turn180StartTime < ROTATE_180_DURATION_MS) {
      int currentRotateSpeed = static_cast<int>(ROTATE_180_SPEED_BASE * overallSpeedModifier);
      currentRotateSpeed = constrain(currentRotateSpeed, 0, TOP_SPEED); // Chỉ lấy giá trị dương cho tốc độ cơ sở

      finalSpeedLeft = currentRotateSpeed;  // Xoay sang phải (ví dụ)
      finalSpeedRight = -currentRotateSpeed;
      
      setMotorSpeed_Robot(finalSpeedLeft, finalSpeedRight);
#ifdef DEBUG_CTRL
      Serial.print("Performing 180 Turn - L: "); Serial.print(finalSpeedLeft); Serial.print(" R: "); Serial.println(finalSpeedRight);
#endif
      return; // Bỏ qua các điều khiển khác
    } else {
      isPerforming180Turn = false; // Hoàn thành xoay
      setMotorSpeed_Robot(0, 0);   // Dừng hẳn sau khi xoay
#ifdef DEBUG_CTRL
      Serial.println("180 Turn Completed.");
#endif
      // Không return ở đây để các lệnh khác có thể được xử lý ngay nếu có
    }
  }

  // 2. Kiểm tra nút nhấn PSB_R3 để bắt đầu xoay 180 độ
  if (ps2x.ButtonPressed(PSB_R3) && !isPerforming180Turn) {
    isPerforming180Turn = true;
    turn180StartTime = millis();
#ifdef DEBUG_CTRL
    Serial.println("PSB_R3 Pressed - Initiating 180 Turn.");
#endif
    // Việc đặt tốc độ sẽ được xử lý ở đầu vòng lặp tiếp theo khi isPerforming180Turn là true
    return; // Ưu tiên hành động này, xử lý ở vòng lặp sau
  }

  // 3. Điều chỉnh tốc độ chung bằng L1/L2
  if (ps2x.Button(PSB_L1)) {
    overallSpeedModifier -= SPEED_ADJUST_STEP;
    if (overallSpeedModifier < MIN_SPEED_MODIFIER) {
      overallSpeedModifier = MIN_SPEED_MODIFIER;
    }
#ifdef DEBUG_CTRL
    Serial.print("L1 Held - Speed Modifier: "); Serial.println(overallSpeedModifier);
#endif
  }
  if (ps2x.Button(PSB_L2)) {
    overallSpeedModifier += SPEED_ADJUST_STEP;
    if (overallSpeedModifier > MAX_SPEED_MODIFIER) {
      overallSpeedModifier = MAX_SPEED_MODIFIER;
    }
#ifdef DEBUG_CTRL
    Serial.print("L2 Held - Speed Modifier: "); Serial.println(overallSpeedModifier);
#endif
  }

  
  // === LOGIC ĐIỀU KHIỂN CHÍNH ===
  bool dpad_command_active = false;
  bool turbo_mode = ps2x.Button(PSB_R2);

  // 4. Ưu tiên xử lý D-Pad
  if (ps2x.Button(PSB_PAD_UP)) { // Trước đây là TIẾN
    int move_speed_base = turbo_mode ? TOP_SPEED : NORM_SPEED;
    // ĐẢO NGƯỢC CHO ROBOT BỊ LẬT: Giờ PSB_PAD_UP sẽ là LÙI (so với chiều motor cũ)
    finalSpeedLeft = -move_speed_base;
    finalSpeedRight = -move_speed_base;
    dpad_command_active = true;
#ifdef DEBUG_CTRL
    Serial.println("D-Pad: UP (Inverted Logic - Backward)");
#endif
  } else if (ps2x.Button(PSB_PAD_DOWN)) { // Trước đây là LÙI
    int move_speed_base = turbo_mode ? TOP_SPEED : NORM_SPEED;
    // ĐẢO NGƯỢC CHO ROBOT BỊ LẬT: Giờ PSB_PAD_DOWN sẽ là TIẾN (so với chiều motor cũ)
    finalSpeedLeft = move_speed_base;
    finalSpeedRight = move_speed_base;
    dpad_command_active = true;
#ifdef DEBUG_CTRL
    Serial.println("D-Pad: DOWN (Inverted Logic - Forward)");
#endif
  } else if (ps2x.Button(PSB_PAD_LEFT)) { // Rẽ trái - Giữ nguyên logic
    finalSpeedLeft = -DPAD_GENTLE_TURN_SPEED_BASE;
    finalSpeedRight = DPAD_GENTLE_TURN_SPEED_BASE;
    dpad_command_active = true;
#ifdef DEBUG_CTRL
    Serial.println("D-Pad: LEFT (Turning Logic Unchanged)");
#endif
  } else if (ps2x.Button(PSB_PAD_RIGHT)) { // Rẽ phải - Giữ nguyên logic
    finalSpeedLeft = DPAD_GENTLE_TURN_SPEED_BASE;
    finalSpeedRight = -DPAD_GENTLE_TURN_SPEED_BASE;
    dpad_command_active = true;
#ifdef DEBUG_CTRL
    Serial.println("D-Pad: RIGHT (Turning Logic Unchanged)");
#endif
  }

  // 5. Nếu không có lệnh D-Pad, xử lý Joystick Phải (PSS_RX) để xoay - Giữ nguyên logic
  if (!dpad_command_active) {
    int joyRX_val = ps2x.Analog(PSS_RX); 

    if (joyRX_val > X_JOY_CALIB + ANALOG_STICK_DEADZONE) { // Xoay phải
      int turn_speed_base = map(joyRX_val, X_JOY_CALIB + ANALOG_STICK_DEADZONE + 1, 255, 0, ANALOG_TURN_MAX_SPEED_BASE);
      finalSpeedLeft = turn_speed_base;
      finalSpeedRight = -turn_speed_base;
    } else if (joyRX_val < X_JOY_CALIB - ANALOG_STICK_DEADZONE) { // Xoay trái
      int turn_speed_base = map(joyRX_val, X_JOY_CALIB - ANALOG_STICK_DEADZONE - 1, 0, 0, ANALOG_TURN_MAX_SPEED_BASE);
      finalSpeedLeft = -turn_speed_base;
      finalSpeedRight = turn_speed_base;
    }
  }
  
// ... (phần còn lại của PS2control() như áp dụng overallSpeedModifier và setMotorSpeed_Robot giữ nguyên)
  // 6. Áp dụng hệ số điều chỉnh tốc độ chung và giới hạn
  finalSpeedLeft = static_cast<int>(finalSpeedLeft * overallSpeedModifier);
  finalSpeedRight = static_cast<int>(finalSpeedRight * overallSpeedModifier);
  
  // Việc giới hạn cuối cùng được thực hiện trong setMotorSpeed_Robot

#ifdef DEBUG_CTRL
  Serial.print(F("Base L: ")); Serial.print(static_cast<int>(finalSpeedLeft / overallSpeedModifier)); // Hiển thị tốc độ trước khi nhân modifier
  Serial.print(F(" Base R: ")); Serial.print(static_cast<int>(finalSpeedRight / overallSpeedModifier));
  Serial.print(F(" | Modifier: ")); Serial.print(overallSpeedModifier, 2); // In modifier với 2 chữ số thập phân
  Serial.print(F(" | Final L: ")); Serial.print(finalSpeedLeft);
  Serial.print(F(" Final R: ")); Serial.println(finalSpeedRight);
  if (dpad_command_active) Serial.print("Dpad Active | ");
  if (finalSpeedLeft == 0 && finalSpeedRight == 0 && !isPerforming180Turn) Serial.println("Motors Stopped (or no input)");
  Serial.println("--------------------------------------");
#endif

  setMotorSpeed_Robot(finalSpeedLeft, finalSpeedRight);
}


void setup() {
  Serial.begin(115200);
  Serial.println("Robot PS2 Control - Advanced - Maker Playground");

  pwm.begin();
  pwm.setPWMFreq(50); // Hoặc 1000Hz / 1600Hz cho motor DC

  setMotorSpeed_Robot(0, 0); 

  setupPS2controller();

  Serial.println("Setup complete. Ready for PS2 control.");
}

void loop() {
  ps2x.read_gamepad(false, false); 

  PS2control(); 

  delay(30); // Giảm delay một chút để phản hồi nhanh hơn với L1/L2 giữ
}