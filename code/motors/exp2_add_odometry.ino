#define LEFT_MOTOR_A 27     // D0 Цифровой выход (левый мотор). Если 0 - едем вперед
#define LEFT_MOTOR_B 26     // D1 Цифровой выход (левый мотор). Если 0 - едем назад
#define RIGHT_MOTOR_A 33    // D2 Цифровой выход (правый мотор). Если 0 - едем вперед
#define RIGHT_MOTOR_B 25    // D3 Цифровой выход (правый мотор). Если 0 - едем назад

#define LEFT_ENCODER_A 34    // Цифровой вход, канал A энкодера левого колеса
#define LEFT_ENCODER_B 39    // Цифровой вход, канал B энкодера левого колеса 
#define RIGHT_ENCODER_A 35   // Цифровой вход, канал A энкодера правого колеса
#define RIGHT_ENCODER_B 32   // Цифровой вход, канал B энкодера правого колеса

/**
 * Глобальные переменные для хранения текущего счёта тиков (импульсов) энкодеров.
 * Используется тип long, т.к. значения могут быть достаточно большими при длительной работе.
 * Ключевое слово volatile указывает, что данные могут изменяться асинхронно (в прерываниях).
 */
 volatile long left_encoder_value = 0; 
 volatile long right_encoder_value = 0;
 long last_left_encoder = 0;
 long last_right_encoder = 0;
 
 
 long last_speed_left_encoder = 0, last_speed_right_encoder = 0;
 float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;
 float vlSpeedFiltered = 0.0, vrSpeedFiltered = 0.0;
 uint32_t lastTimeL = 0, lastTimeR = 0;
 
 float xPos = 0.0, yPos = 0.0, theta = 0.0;
 
 const float WHEEL_DIAMETER = 69.0;
 const float WHEEL_BASE = 185.0;
 const float ENCODER_RESOLUTION = 330.0;
 const float TICKS_TO_MM = (PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
 const float ALPHA = 0.2; // Коэффициент фильтрации скорости
 
 void left_interrupt() {digitalRead(LEFT_ENCODER_B)?left_encoder_value++:left_encoder_value--;}
 void right_interrupt() {digitalRead(RIGHT_ENCODER_B)?right_encoder_value++:right_encoder_value--;}
 
 
 void updateOdometry() {
   long deltaLeft = left_encoder_value - last_left_encoder;
   long deltaRight = right_encoder_value - last_right_encoder;
   last_left_encoder = left_encoder_value;
   last_right_encoder = right_encoder_value;
 
   float distLeft = deltaLeft * TICKS_TO_MM;
   float distRight = deltaRight * TICKS_TO_MM;
   float deltaS = (distLeft + distRight) / 2.0;
   float deltaTheta = (distRight - distLeft) / WHEEL_BASE;
 
   theta += deltaTheta;
   xPos += deltaS * cos(theta);
   yPos += deltaS * sin(theta);
 }
 void computeSpeed() {
   uint32_t now = micros();
 
   if (lastTimeL == 0 || lastTimeR == 0) { // Первая инициализация таймера
     lastTimeL = now;
     lastTimeR = now;
     last_speed_left_encoder = left_encoder_value;
     last_speed_right_encoder = right_encoder_value;
     return;
   }
 
   float dtL = (now - lastTimeL) * 0.000001;
   float dtR = (now - lastTimeR) * 0.000001;
 
   if (dtL > 0 && left_encoder_value != last_speed_left_encoder) {
     long deltaLeft = left_encoder_value - last_speed_left_encoder;
     float wlSpeed = ((float)deltaLeft / ENCODER_RESOLUTION) * 360.0f / dtL;
     float vlSpeed = (wlSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
     vlSpeedFiltered = ALPHA * vlSpeed + (1.0f - ALPHA) * vlSpeedFiltered;
     last_speed_left_encoder = left_encoder_value;
     lastTimeL = now;
 
     //Serial.printf("Left: ΔEnc=%ld, dt=%.6f, Speed=%.2f mm/s\n", deltaLeft, dtL, vlSpeedFiltered);
   } else if (dtL > 0.005 && left_encoder_value == last_speed_left_encoder){
     vlSpeedFiltered = 0;
   }
 
   if (dtR > 0 && right_encoder_value != last_speed_right_encoder) {
     long deltaRight = right_encoder_value - last_speed_right_encoder;
     float wrSpeed = ((float)deltaRight / ENCODER_RESOLUTION) * 360.0f / dtR;
     float vrSpeed = (wrSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
     vrSpeedFiltered = ALPHA * vrSpeed + (1.0f - ALPHA) * vrSpeedFiltered;
     last_speed_right_encoder = right_encoder_value;
     lastTimeR = now;
 
     //Serial.printf("Right: ΔEnc=%ld, dt=%.6f, Speed=%.2f mm/s\n", deltaRight, dtR, vrSpeedFiltered);
   } else if (dtR > 0.005 && right_encoder_value == last_speed_right_encoder){
     vrSpeedFiltered = 0;
   }
 }
 
 
 
 void setup() {
   Serial.begin(115200);
   delay(1000);
   Serial.println("System started");
 
   /**
    * Подключение функций-прерываний (interrupt service routines, ISR) к выводам энкодеров.
    * digitalPinToInterrupt(ПИН) преобразует номер пина в номер прерывания, которое мы хотим отлавливать.
    * Событие RISING означает вызов прерывания при переходе сигнала с LOW на HIGH.
    */
   attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), left_interrupt, RISING);
   attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), right_interrupt, RISING);
   
   // Настройка выводов для управления моторами как выходы (OUTPUT).
   pinMode(LEFT_MOTOR_A,OUTPUT);
   pinMode(LEFT_MOTOR_B,OUTPUT);
   pinMode(RIGHT_MOTOR_A,OUTPUT);
   pinMode(RIGHT_MOTOR_B,OUTPUT);
   
   // Настройка выводов энкодеров как входы (INPUT).
   pinMode(LEFT_ENCODER_A, INPUT);
   pinMode(LEFT_ENCODER_B, INPUT);
   pinMode(RIGHT_ENCODER_A, INPUT);
   pinMode(RIGHT_ENCODER_B, INPUT);
   
 
 }
 
 void loop() {
   if (Serial.available()) {
     String command = Serial.readStringUntil('\n');
     processCommand(command);
   }
   computeSpeed();
   updateOdometry(); 
 
   static uint32_t printTimer = 0;
   if (millis() - printTimer > 200) {
     Serial.printf("POS X=%.2f Y=%.2f Th=%.2f ", xPos, yPos, theta);
     Serial.printf("ENC L=%ld R=%ld ", left_encoder_value, right_encoder_value);
     Serial.printf("SPD L=%.2f mm/s R=%.2f mm/s\n", vlSpeedFiltered, vrSpeedFiltered);
     printTimer = millis();
   }
 }
 
 void processCommand(String command) {
   command.trim();
   if (command.startsWith("SET_PWM")) {
     int leftA_PWM = 0, leftB_PWM = 0, rightA_PWM =0, rightB_PWM = 0;
     if (parseSetPWM(command, &leftA_PWM, &leftB_PWM, &rightA_PWM, &rightB_PWM)) {
       setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
       Serial.print("OK: Set PWM");
     }
     return;
   } else if (command.startsWith("SET_POSE")) {
     float x, y, th;
     if (parseSetPose(command, &x, &y, &th)) {
       xPos = x;
       yPos = y;
       theta = th;
       Serial.println("OK: Pose set");
     }
   } else {
     Serial.println("ERROR: Unknown command");
   }
 }
 
 bool parseSetPose(const String& command, float* x, float* y, float* th) {
   int index1 = command.indexOf(' ');
   if (index1 == -1) return false;
   int index2 = command.indexOf(' ', index1 + 1);
   if (index2 == -1) return false;
   int index3 = command.indexOf(' ', index2 + 1);
   if (index3 == -1) return false;
   
   *x = command.substring(index1 + 1, index2).toFloat();
   *y = command.substring(index2 + 1, index3).toFloat();
   *th = command.substring(index3 + 1).toFloat();
   return true;
 }
 
 bool parseSetPWM(const String& command, int* leftA_PWM, int* leftB_PWM, int* rightA_PWM, int* rightB_PWM) {
   int index1 = command.indexOf(' ');
   if (index1 == -1) return false;
   int index2 = command.indexOf(' ', index1 + 1);
   if (index2 == -1) return false;
   int index3 = command.indexOf(' ', index2 + 1);
   if (index3 == -1) return false;
   int index4 = command.indexOf(' ', index3 + 1);
   if (index4 == -1) return false;
 
   *leftA_PWM = command.substring(index1 + 1, index2).toInt();
   *leftB_PWM = command.substring(index2 + 1, index3).toInt();
   *rightA_PWM = command.substring(index3 + 1, index4).toInt();
   *rightB_PWM = command.substring(index4 + 1).toInt();
   return true;
 }
 
 
 void setMotorsPWM(int leftA, int leftB, int rightA, int rightB) {
   leftA  = constrain(leftA, 0, 255);
   leftB  = constrain(leftB, 0, 255);
   rightA = constrain(rightA, 0, 255);
   rightB = constrain(rightB, 0, 255);
 
   analogWrite(LEFT_MOTOR_A,   leftA);
   analogWrite(LEFT_MOTOR_B,   leftB);
   analogWrite(RIGHT_MOTOR_A,  rightA);
   analogWrite(RIGHT_MOTOR_B,  rightB);
 }