//
// Created by admin on 2023/11/27.
//
#include "Usermain.h"
#include "LegacyControlData.h"
#include "SensorFusionData.h"
#include "Matrix.h"
#include "Extension.h"
#include "MahonyAHRS.h"
#include "Legacy.h"

#ifndef CONTROL_FRAME_MAIN_SENSOR_H
#define CONTROL_FRAME_MAIN_SENSOR_H



// 以下 MS5837_30BA_ 名称是历史命名，不能据此确认实物型号；新路径查 FusionConfiguration。
#define B02_IIC_ADDRESS 0xEC // 7 位 0x76 左移一位，HAL 地址 0xEC

#define MS5837_30BA_ResetCommand     0x1E                // 复位命令，0x1E；02BA/30BA 使用同一命令值。
#define	MS5837_30BA_PROM_RD 	       0xA0                // PROM 首地址；依次读取 0xA0、A2、A4、A6、A8、AA、AC。
#define MS5837_30BA_ADC_RD           0x00                // ADC 读取命令 0x00。

#define MS5837_30BA_D1_OSR256					 0x40
#define MS5837_30BA_D1_OSR512					 0x42
#define MS5837_30BA_D1_OSR1024					 0x44
#define MS5837_30BA_D1_OSR2048					 0x46
#define MS5837_30BA_D1_OSR4096					 0x48
#define MS5837_30BA_D2_OSR256					 0x50
#define MS5837_30BA_D2_OSR512					 0x52
#define MS5837_30BA_D2_OSR1024					 0x54
#define MS5837_30BA_D2_OSR2048					 0x56
#define MS5837_30BA_D2_OSR4096					 0x58

#define PRESSURE_0_V30 1005.646
#define PRESSURE_1_V30 1004.711
#define PRESSURE_2_V30 1004.350
#define PRESSURE_3_V30 1005.058

#define PRESSURE_0_V31 1014.227
#define PRESSURE_1_V31 1003.288
#define PRESSURE_2_V31 1003.028
#define PRESSURE_3_V31 1003.799

#define PRESSURE_0_V32 1009.775
#define PRESSURE_1_V32 1010.957
#define PRESSURE_2_V32 1010.074
#define PRESSURE_3_V32 1009.850

#define PRESSURE_0_V33 1004.227
#define PRESSURE_1_V33 1003.288
#define PRESSURE_2_V33 1003.028
#define PRESSURE_3_V33 1003.799

#define PRESSURE_0_V40 1003.599
#define PRESSURE_1_V40 1004.981
#define PRESSURE_2_V40 1004.411
#define PRESSURE_3_V40 1003.102

typedef struct Sensor_Site{
    float x[SENSOR_NUM];
    float y[SENSOR_NUM];
    float z[SENSOR_NUM];
} Sensor_Site_t;

enum class PS_HANDLE_STATE {
        GET_TEMPERATURE = 0X00,
        GET_PRESSURE = 0X01,
        CALCULATE = 0X02
    };

class PressureSensor: public  Device{

    unsigned char MS5837_30BA_Crc4(int id);
    unsigned long MS5837_30BA_GetConversion(uint8_t command);
    uint8_t MS5837_30BA_PROM(int id);
    void MS5837_30BA_ReSet(void);
    float MS5837_30BA_GetData(int id);
    //signed int MS5837_30BA_GetTemp(int id);

    void Init_single(int id);
    void Handle_single(int id);
    void Handle_all();
#if LC_IMU_ASYNC_ENABLED
    void AcquireTimedPressure();
    uint32_t temperature_adc_[4] = {}, pressure_adc_[4] = {}, pressure_time_[4] = {};
    uint8_t acquisition_mask_ = 0;
    lower_controller::PressureArraySample physical_sample_ = {};
    uint32_t calibration_epoch_ = 1;
    bool legacy_frame_valid_ = false;
#endif
    void FilterMeasurement(const lower_controller::PressureMeasurement& sample);
    lower_controller::PressureMeasurement last_measurement_ = {};
    bool frame_bus_ok_ = true;      // 当前三步水压帧期间 I2C2 是否全部成功
    uint32_t bus_frame_drops_ = 0;  // 因 I2C2 出错被丢弃的整帧数
    uint32_t estimated_sequence_ = 0;
    uint32_t conversion_started_us_ = 0;
    lower_controller::LegacyPressureFeedback feedback_ = {};
    void Calibrate_single(int id);
    void Calibrate();
    void OutputData_single(int id);
    void OutputData();
    void Solve_plane_3(float* data,float* h,float* x,float* y,float* z);
    void Solve_plane(float* data,float* h,float* x,float* y,float* z,int num);
    void Update_plane();//计算水面方程

    void Delay_us(uint32_t us);

    //void get_angle(float q[4], float *yaw, float *pitch, float *roll);

    signed int dT,TEMP;
    uint32_t Cal_C[6][7];
    int32_t OFFi,SENSi,Ti;
    int64_t OFF2;
    int64_t SENS2;
    uint32_t TEMP2;
    int64_t OFF_,SENS;
    uint32_t D1_Pres,D2_Temp;
    unsigned char flag_ok[6];
    
    int32_t CurrentID;

    Sensor_Site_t site;

public:

    // 自动校准仅在上电、调度器启动前执行；成功后偏置保留在 RAM。
    bool CalibrateAtStartup();
    bool StartupCalibrationOk() const { return startup_calibration_ok_; }
    struct StartupCalibrationDiagnostics
    {
        // 0=未执行/恢复锁定，1=采集中，2=成功，3=失败，
        // 4=四路零点互相差太多（多半没在空气中标定，见 CalibrateAtStartup）
        uint32_t state, samples;
        uint32_t failed_channel; // 0..3；4 表示全局条件不满足
        float surface_pa[4], spread_pa[4];
        float channel_spread_pa;  // 四路零点的最大-最小，Pa；空气中应当接近 0
    } startup_calibration = {};

private:
    bool startup_calibration_ok_ = false;
public:

    void Init();
    void Handle();
    void Receive();

    // Handle remains the compatibility coordinator: acquire first, then estimate.
    // Handle 保留原调用位置；Acquire 只交付测量，Estimate 消费新帧并生成旧单位反馈。
    void Acquire();
#if LC_IMU_ASYNC_ENABLED
    const lower_controller::PressureArraySample& PhysicalMeasurement() const { return physical_sample_; }
    bool LegacyFrameValid() const { return legacy_frame_valid_; }
    void ResetTimedAcquisition() { ps_state = PS_HANDLE_STATE::GET_TEMPERATURE; acquisition_mask_ = 0; legacy_frame_valid_ = false; ++calibration_epoch_; }
#endif
    uint32_t BusFrameDrops() const { return bus_frame_drops_; }
    bool PromValid() const { for (int i = 0; i < SENSOR_NUM; ++i) if (!flag_ok[i]) return false; return true; }
    void Estimate(const lower_controller::PressureMeasurement& sample, bool outer_loop_due);
    const lower_controller::LegacyPressureFeedback& Feedback() const { return feedback_; }
    const lower_controller::PressureMeasurement& LastMeasurement() const { return last_measurement_; }

    float Temperature;
    signed int data_temp[6];
    float data_pressure_offset[6]; // 压强零偏值，在水上测量
    float data_pressure_raw[6]; // 水下压强原始值
    float data_pressure[6]; // 温补压强减零偏后的 legacy 数值，尚未做 rho*g 换算
    float data_plane[5]; // 旧平面拟合辅助量 [A,B,C,D,lambda]；法向量归一化约束 A²+B²+C²=1，当前 ESKF 不调用此路径
    float data_depth; // 旧控制协议按 cm 使用的 legacy 深度量；不保证为物理厘米
    float data_roll,data_pitch; // 旧水压差组合量，不是弧度制姿态角
    float data_level[3];
    //float quat[4];
    float pitch,roll; // 俯仰角、横滚角
    bool flag_calibrate; // 校准水压计零偏的标志
    bool flag_read_pres; // 读取水压计读数的标志

    //uint8_t flag_Busy;
    static PressureSensor pressure_sensor;
    PS_HANDLE_STATE ps_state = PS_HANDLE_STATE::GET_TEMPERATURE;

};

class Sonar: public Device{

    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data;
    int32_t data_offset;

public:

    void Init();
    void Handle();
    void Receive();
    static Sonar sonar;
};

#endif //CONTROL_FRAME_MAIN_SENSOR_H
