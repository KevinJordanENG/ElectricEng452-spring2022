/* Capstone project for EE405 Control Systems I & EE 452 Embedded Systems
/* Many functions and theory used provided by Joop Brokking http://brokking.net/ymfc-al_main.html
/* By Kevin Jordan kevin.jordan@und.edu */

#include <Servo.h>
#include <Wire.h>
#include <Ultrasonic.h>

Servo ESC1, ESC2, ESC3, ESC4;
Ultrasonic dist_sensor(12, 13);
const int LED = 10;
int temperature, ground, max_altitude=8;
long loop_timer, loop_counter=0;
int prop_speed=1000, sys_phase=1;
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;
boolean set_gyro_angles;
float roll_level_adjust, pitch_level_adjust;
int esc_1_sig, esc_2_sig, esc_3_sig, esc_4_sig;
float pid_error_temp;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
/* PID setup/gain tuning */
float pid_p_gain_roll = 1.2;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.01;               //Gain setting for the roll I-controller
float pid_d_gain_roll = 0.5;               //Gain setting for the roll D-controller
int pid_max_roll = 400;
float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;
float pid_p_gain_yaw = 3;                //Gain setting for the yaw P-controller. //4.0
float pid_i_gain_yaw = 0.02;                //Gain setting for the yaw I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the yaw D-controller.
int pid_max_yaw = 400;

void setup() {
    // Phase 1 setup and calibration
    pinMode(10, OUTPUT);
    Serial.begin(115200);
    Wire.begin();
    setup_mpu_6050_registers();
    // Find avg gyro offset
    Serial.println("Calibrating...");
    for (int cal=0; cal<2000 ; cal++) {
        read_mpu_6050_data();
        gyro_x_cal += gyro_x;
        gyro_y_cal += gyro_y;
        gyro_z_cal += gyro_z;
        delay(3);
    }
    gyro_x_cal /= 2000;
    gyro_y_cal /= 2000;
    gyro_z_cal /= 2000;
    //setup the range of PWM signals to be sent to ESCs
    ESC1.attach(4, 1000, 2000);
    ESC2.attach(5, 1000, 2000);
    ESC3.attach(6, 1000, 2000);
    ESC4.attach(7, 1000, 2000);
    //Calibrate ESC's
    /*ESC1.write(2000);
    for (int i=0; i<3; i++) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    for (int i=0; i<1000; i++) {
        ESC1.write(2000-i);
        delay(1);
    }
    ESC1.write(1000);
    for (int i=0; i<3; i++) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    ESC2.write(2000);
    for (int i=0; i<3; i++) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    for (int i=0; i<1000; i++) {
        ESC2.write(2000-i);
        delay(1);
    }
    ESC2.write(1000);
    for (int i=0; i<3; i++) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    ESC3.write(2000);
    for (int i=0; i<3; i++) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    for (int i=0; i<1000; i++) {
        ESC3.write(2000-i);
        delay(1);
    }
    ESC3.write(1000);
    for (int i=0; i<3; i++) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    ESC4.write(2000);
    for (int i=0; i<3; i++) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    for (int i=0; i<1000; i++) {
        ESC4.write(2000-i);
        delay(1);
    }
    ESC4.write(1000);*/
    delay(3000);

    //get initial distance to ground at rest
    ground = dist_sensor.read(INC);
    Serial.println(ground);
    sys_phase = 2;
    loop_timer = micros();
}

void loop() {
    if (sys_phase == 2) {
    // takeoff, hover, land sequence
        digitalWrite(LED, HIGH);
        //Reset the PID controllers for a bumpless start.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        while (loop_counter != 4000) {
            if (loop_counter <= 500) {
                prop_speed = 1150;
            }
            else if ((loop_counter <= 1000) && (loop_counter > 500)) {
                prop_speed = 1300;
            }
            else if ((loop_counter <= 1750) && (loop_counter > 1000)) {
                prop_speed = 1350;
            }
            else if ((loop_counter <= 2250) && (loop_counter > 1750)) {
                prop_speed = 1400;
            }
            else if ((loop_counter <= 3000) && (loop_counter > 2250)) {
                prop_speed = 1350;
            }
            else if ((loop_counter <= 3500) && (loop_counter > 3000)) {
                prop_speed = 1300;
            }
            else if ((loop_counter < 4000) && (loop_counter > 3500)) {
                prop_speed = 1150;
            }

            // Get position data
            read_mpu_6050_data();
            acc_z *= -1;
            gyro_x -= gyro_x_cal;
            gyro_y -= gyro_y_cal;
            gyro_z = -1*(gyro_z - gyro_z_cal);

            // PID inputs
            gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_x / 65.5) * 0.3);
            gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_y / 65.5) * 0.3);
            gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z / 65.5) * 0.3);
        
            // Gyro angle calc (250Hz)
            angle_pitch += gyro_x * 0.0000611;
            angle_roll += gyro_y * 0.0000611;
        
            // couple pitch / roll axis  of gyro
            angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
            angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
        
            // Accel angle calc
            acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
            if(abs(acc_y) < acc_total_vector){ 
                angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
            }
            if(abs(acc_x) < acc_total_vector){
                angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
            }
        
            // !!!!spirit level FIXME FOR EACH NEW MPU6050 BOARD OR AFTER REMOUNTING!!!!!!!
            angle_pitch_acc -= 0.228;
            angle_roll_acc -= 8.203;
        
            // Smooth IMU sensor signal to process by combining gyro & accel readings
            if (set_gyro_angles) {
                angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
                angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
            }
            else {
                angle_pitch = angle_pitch_acc;
                angle_roll = angle_roll_acc;
                set_gyro_angles = true;
            }

            // Attitude angle corrections needed
            pitch_level_adjust = 0;//angle_pitch * 15;
            roll_level_adjust = 0;//angle_roll * 15;
        
            pid_pitch_setpoint = 0;
            pid_pitch_setpoint -= pitch_level_adjust;
            pid_pitch_setpoint /= 3.0;
            pid_roll_setpoint = 0;
            pid_roll_setpoint -= roll_level_adjust;
            pid_roll_setpoint /= 3.0;
            pid_yaw_setpoint = 0;

            calculate_pid();

            esc_1_sig = prop_speed - pid_output_pitch + pid_output_roll - pid_output_yaw;
            esc_2_sig = prop_speed + pid_output_pitch + pid_output_roll + pid_output_yaw;
            esc_3_sig = prop_speed + pid_output_pitch - pid_output_roll - pid_output_yaw;
            esc_4_sig = prop_speed - pid_output_pitch - pid_output_roll + pid_output_yaw;
            
            if (esc_1_sig < 1100) esc_1_sig = 1100;
            if (esc_2_sig < 1100) esc_2_sig = 1100;
            if (esc_3_sig < 1100) esc_3_sig = 1100;
            if (esc_4_sig < 1100) esc_4_sig = 1100;
            if (esc_1_sig > 2000) esc_1_sig = 2000;
            if (esc_2_sig > 2000) esc_2_sig = 2000;
            if (esc_3_sig > 2000) esc_3_sig = 2000;
            if (esc_4_sig > 2000) esc_4_sig = 2000;
            
            Serial.print("gX = ");
            Serial.print(gyro_x);
            Serial.print(" : gY = ");
            Serial.print(gyro_y);
            Serial.print(" : gZ = ");
            Serial.print(gyro_z);
            Serial.print(" : Ax = ");
            Serial.print(acc_x);
            Serial.print(" : Ay = ");
            Serial.print(acc_y);
            Serial.print(" : Az = ");
            Serial.print(acc_z);
            //Serial.print(" : ESC4 = ");
            //Serial.print(esc_4_sig);
            Serial.println();

            /*ESC1.write(esc_1_sig);
            ESC2.write(esc_2_sig);
            ESC3.write(esc_3_sig);
            ESC4.write(esc_4_sig);*/

            loop_counter++;
            while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
            loop_timer = micros();
        }
        sys_phase = 3;
    }
    else if (sys_phase == 3) {
        // ground period after flight
        ESC1.write(1000);
        ESC2.write(1000);
        ESC3.write(1000);
        ESC4.write(1000);
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
}

void read_mpu_6050_data(){
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    while (Wire.available() < 14);
    acc_x = Wire.read()<<8|Wire.read();
    acc_y = Wire.read()<<8|Wire.read();
    acc_z = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_x = Wire.read()<<8|Wire.read();
    gyro_y = Wire.read()<<8|Wire.read();
    gyro_z = Wire.read()<<8|Wire.read();
}

void setup_mpu_6050_registers(){
    //Activate MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    //Config accel (+/-8g)
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    //Config gyro (500d/s)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
}

void calculate_pid() {
    //Roll calculations
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
    else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
    else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

    pid_last_roll_d_error = pid_error_temp;

    //Pitch calculations
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
    else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
    else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

    pid_last_pitch_d_error = pid_error_temp;

    //Yaw calculations
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
    else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
    else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

    pid_last_yaw_d_error = pid_error_temp;
}
