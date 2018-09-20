#pragma once

void mpu6050_init(enum Rotation rotation);
void mpu6050_read(void);

void mpu6050_get_gyro(float *g);
float mpu6050_get_acc_x(void);
float mpu6050_get_acc_y(void);
float mpu6050_get_acc_z(void);
float mpu6050_get_gyro_x(void);
float mpu6050_get_gyro_y(void);
float mpu6050_get_gyro_z(void);
void mpu6050_set_accel(float *a);
void mpu6050_set_gyro(float *g);
void mpu6050_set_acc_x(float a);
void mpu6050_set_acc_y(float a);
void mpu6050_set_acc_z(float a);
void mpu6050_set_gyro_x(float g);
void mpu6050_set_gyro_y(float g);
void mpu6050_set_gyro_z(float g);
void mpu6050_set_gyro_offset_x(float offset);
void mpu6050_set_gyro_offset_y(float offset);
void mpu6050_set_gyro_offset_z(float offset);
bool mpu6050_update(void);
void mpu6050_clean_update(void);



