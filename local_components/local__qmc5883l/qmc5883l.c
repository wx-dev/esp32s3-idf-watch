#include "include/qmc5883l.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "QMC5883L";
int offsetX,offsetY,offsetZ;
#define CalThreshold 0

esp_err_t qmc5883L_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, QMC5883L_SENSOR_ADDR, &reg_addr, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t qmc5883L_register_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(I2C_MASTER_NUM, QMC5883L_SENSOR_ADDR, write_buf, sizeof(write_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
void qmc5883l_init(void) {
    uint8_t id = 0;

    while (id != 0xff)  // 确定ID号是否正确
    {
        qmc5883L_register_read(QMC5883L_CHIPID, &id, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "QMC5883L OK!");
    qmc5883L_register_write_byte(QMC5883L_CTRL2, 0x80); // 复位芯片 
    vTaskDelay(10 / portTICK_PERIOD_MS);
    qmc5883L_register_write_byte(QMC5883L_CTRL1, 0x05); //Continuous模式 50Hz 
    qmc5883L_register_write_byte(QMC5883L_CTRL2, 0x00);
    qmc5883L_register_write_byte(QMC5883L_FBR, 0x01);
}

void qmc5883l_read_xyz(t_sQMC5883L *p) {
    uint8_t status, data_ready = 0;
    int16_t mag_reg[3];

    qmc5883L_register_read(QMC5883L_STATUS, &status, 1); // 读状态寄存器 

    if (status & 0x01) {
        data_ready = 1;
    }
    if (data_ready == 1) {
        data_ready = 0;
        qmc5883L_register_read(QMC5883L_XOUT_L, (uint8_t *) mag_reg, 6);

        p->mag_x = mag_reg[0];
        p->mag_y = mag_reg[1];
        p->mag_z = mag_reg[2];
        //p->azimuth = (float)atan2(p->mag_y, p->mag_x) * 180.0 / 3.1415926 + 180.0;
    }
}
void calibrateMag()
{
    int x,y,z; //三轴数据
    int xMax, xMin, yMax, yMin, zMax, zMin;
    //初始化
    t_sQMC5883L p;
    qmc5883l_read_xyz(&p);
    xMax=xMin=x=p.mag_x;
    yMax=yMin=y=p.mag_y;
    zMax=zMin=z=p.mag_z;
    offsetX = offsetY = offsetZ = 0;
    for(int i=0;i<200;i++)
    {
        qmc5883l_read_xyz(&p);
        x=p.mag_x;
        y=p.mag_y;
        z=p.mag_z;
        // 计算最大值与最小值
        // 计算传感器绕X,Y,Z轴旋转时的磁场强度最大值和最小值
        if (x > xMax)
            xMax = x;
        if (x < xMin )
            xMin = x;
        if(y > yMax )
            yMax = y;
        if(y < yMin )
            yMin = y;
        if(z > zMax )
            zMax = z;
        if(z < zMin )
            zMin = z;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    //计算修正量
    if (abs(xMax - xMin) > CalThreshold)
        offsetX = (xMax + xMin) / 2;
    if (abs(yMax - yMin) > CalThreshold)
        offsetY = (yMax + yMin) / 2;
    if (abs(zMax - zMin) > CalThreshold)
        offsetZ = (zMax + zMin) / 2;
    ESP_LOGI(TAG, "offsetX = %d  offsetY = %d offsetZ = %d ", offsetX, offsetY, offsetZ);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
}

void qmc5883l_fetch_azimuth(t_sQMC5883L *p) {
    qmc5883l_read_xyz(p);
    float headingRadians = atan2((double)((p->mag_y)-offsetY),(double)((p->mag_x)-offsetX));
    // 保证数据在0-2*PI之间
    if (headingRadians < 0) {
        headingRadians += 2 * M_PI;
    }
    int headingDegrees = headingRadians * 180 / M_PI;
    headingDegrees += -5; //磁偏角
    //保证数据在0-360之间
    if (headingDegrees > 360) {
        headingDegrees -= 360;
    }
    p->azimuth = (float) headingDegrees;
}

