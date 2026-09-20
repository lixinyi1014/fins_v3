#include "Sensor.h"
#include "Kalman_Filter.h"
#include "LegacyEstimation.h"
#include "I2cBusAccess.h"
#include "ControllerRtosHooks.h"
#if LC_USE_FREERTOS
#include "PressureCompensation.h"
#endif

// OSR commands below retain the original conversion configuration.
// 下方转换命令保留原 OSR1024，集中配置必须与实际命令一致。
#if LC_PRESSURE_OSR != 1024U
#error "Update the pressure conversion commands and timing contract together."
#endif

extern I2C_HandleTypeDef hi2c2;

PressureSensor PressureSensor::pressure_sensor;

KalmanFilter Pressure_Kf[4] = {KalmanFilter(0.01, 0.05, 1.0, 0.0),
                               KalmanFilter(0.01, 0.05, 1.0, 0.0),
                               KalmanFilter(0.01, 0.05, 1.0, 0.0),
                               KalmanFilter(0.01, 0.05, 1.0, 0.0)};

void PressureSensor::Init()
{

    // Original inactive plane-solver coordinates. Active ESKF geometry is in FusionConfiguration.cpp.
    Sensor_Site_t _site = {
        .x = {-12, 12, 12, -12},
        .y = {-5.4, -5.4, 5.4, 5.4},
        .z = {0, 0, 0, 0}
        /*.x={0},
        .y={1},
        .z={2},*/
    };
    site = _site;

    CurrentID = 0;

    // flag_Busy = 0;

    bsp_flash_read(&flashData);

    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        TCA_SetChannel(i);
        HAL_Delay(5);
        Init_single(i);
    }

    data_plane[0] = 0;
    data_plane[1] = 0;
    data_plane[2] = 1;
    data_plane[3] = 0;
    data_plane[4] = 0;

    flag_calibrate = false;
    flag_read_pres = false;

    /*quat[0]=1.0f;
    quat[1]=0.0f;
    quat[2]=0.0f;
    quat[3]=0.0f;
*/
}

#if LC_USE_FREERTOS
bool PressureSensor::CalibrateAtStartup()
{
    // TODO(操作)：必须四个压力口在空气中、静止时上电。软件无法识别“在水下但压力稳定”。
    // 不写 Flash：每次正常上电重新测零；看门狗恢复启动由 main 禁止重新归零。
    startup_calibration = {};
    startup_calibration.state = 1;
    startup_calibration.failed_channel = 4;
    startup_calibration_ok_ = false;
    if (LcRuntime_IsRunning() || !PromValid())
    {
        for (unsigned i = 0; i < 4; ++i)
            if (!flag_ok[i]) { startup_calibration.failed_channel = i; break; }
        startup_calibration.state = 3;
        return false;
    }
    HAL_Delay(LC_STARTUP_PRESSURE_SETTLE_MS);
    LcBus_Begin(&hi2c2, LC_CALIBRATION_BUDGET_US);
    double pressure_sum[4] = {}, legacy_sum[4] = {};
    float minimum[4] = {1e9f,1e9f,1e9f,1e9f}, maximum[4] = {};
    for (unsigned sample = 0; sample < LC_STARTUP_PRESSURE_SAMPLES; ++sample)
    {
        for (unsigned i = 0; i < 4; ++i)
        {
            startup_calibration.failed_channel = i;
            TCA_SetChannel(i);
            const float legacy = MS5837_30BA_GetData(i); // 同一对 D1/D2 同时用于两条反馈路径。
            const auto physical = lower_controller::CompensateMs5837(
                lower_controller::GetFusionConfiguration().pressure_model, Cal_C[i], D1_Pres, D2_Temp);
            if (!LcBus_Ok(&hi2c2) || !physical.valid || !isfinite(legacy) ||
                physical.pressure_pa < 80000 || physical.pressure_pa > 120000)
            {
                startup_calibration.state = 3;
                return false; // 不提交部分通道的偏置，不沿用失效 ADC。
            }
            pressure_sum[i] += physical.pressure_pa;
            legacy_sum[i] += legacy;
            minimum[i] = fminf(minimum[i], physical.pressure_pa);
            maximum[i] = fmaxf(maximum[i], physical.pressure_pa);
            startup_calibration.spread_pa[i] = maximum[i] - minimum[i];
            if (startup_calibration.spread_pa[i] > LC_STARTUP_PRESSURE_SPREAD_PA)
            {
                startup_calibration.state = 3;
                return false;
            }
        }
        startup_calibration.samples = sample + 1;
    }
    float surface[4];
    for (unsigned i = 0; i < 4; ++i) surface[i] = float(pressure_sum[i]/LC_STARTUP_PRESSURE_SAMPLES);
    if (!lower_controller::SetStartupSurfacePressure(surface))
    {
        startup_calibration.state = 3;
        return false;
    }
    for (unsigned i = 0; i < 4; ++i)
    {
        startup_calibration.surface_pa[i] = surface[i];
        data_pressure_offset[i] = float(legacy_sum[i]/LC_STARTUP_PRESSURE_SAMPLES);
        data_pressure[i] = data_pressure_raw[i] = 0;
        Pressure_Kf[i] = KalmanFilter(0.01, 0.05, 1.0, 0.0);
    }
    last_measurement_ = {};
    feedback_ = {};
    estimated_sequence_ = 0;
    data_depth = data_roll = data_pitch = 0;
    ps_state = PS_HANDLE_STATE::GET_TEMPERATURE;
#if LC_IMU_ASYNC_ENABLED
    ResetTimedAcquisition();
#endif
    startup_calibration.failed_channel = 4;
    startup_calibration.state = 2;
    startup_calibration_ok_ = true;
    return true;
}
#endif

void PressureSensor::Handle()
{
    const PS_HANDLE_STATE previous = ps_state;
    Acquire();
    if (!LcBus_Ok(&hi2c2)) return; // 本次总线请求失败：不交付新估计或继续累计校准样本
    // CALCULATE is the NEXT driver step. The old outer loop uses the previous frame.
    // CALCULATE 表示下一步将读取压强；由协调入口保留旧外环相位。
    Estimate(last_measurement_, previous != ps_state && ps_state == PS_HANDLE_STATE::CALCULATE); // 4 路旧压力；true 表示旧外环相位
}

void PressureSensor::Acquire()
{
    // Preserve calibration behavior and one state-machine step per scheduler tick.
    // 保留校准分支和每个主节拍推进一步的三阶段采集流程。
    if (flag_calibrate) {
        Calibrate();
        flag_calibrate = false;
    } else {
#if LC_IMU_ASYNC_ENABLED
        AcquireTimedPressure();
#else
        Handle_all();
#endif
    }
}

void PressureSensor::Estimate(const lower_controller::PressureMeasurement& sample, bool outer_loop_due)
{
    // Consume each delivered frame once; repeated calls only refresh the loop phase.
    // 一帧只滤波一次；其余节拍仅刷新外环相位，本函数不读取外设或驱动状态。
    if (sample.sequence != estimated_sequence_) {
        FilterMeasurement(sample); // sample 为温补后、去偏滤波前的四路压力
        estimated_sequence_ = sample.sequence;
    }
    feedback_ = lower_controller::EstimateLegacyPressure(data_pressure,
        outer_loop_due, estimated_sequence_);
    data_depth = feedback_.depth_legacy; // 四路均值，旧深度刻度
    data_roll = feedback_.roll_difference_legacy; // p0+p1-p2-p3，旧压差
    data_pitch = feedback_.pitch_difference_legacy; // p0+p3-p1-p2，旧压差
}

void PressureSensor::FilterMeasurement(const lower_controller::PressureMeasurement& sample)
{
    for (int i = 0; i < SENSOR_NUM; ++i) {
        // A failed PROM must not introduce an uninitialized stack value into feedback.
        // PROM 无效通道保留上一值，避免把未初始化值传入滤波；此处不新增故障控制策略。
        if ((sample.prom_valid_mask & (1U << i)) == 0U) continue;
        data_pressure_raw[i] = sample.pressure_legacy[i];
        float pressure = data_pressure_raw[i] - data_pressure_offset[i];
        pressure = Pressure_Kf[i].update(pressure);
        data_pressure[i] = pressure;
    }
}

void PressureSensor::Receive()
{
    if (strncmp((char *)RxBuffer, "CA", 2) == 0)
    {
        flag_calibrate = true;
    }
    else if (strncmp((char *)RxBuffer, "VA", 2) == 0)
    {
        flag_read_pres = true;
    }
}

void PressureSensor::Delay_us(uint32_t us)
{
    LcTime_DelayUs(us);
}

unsigned char PressureSensor::MS5837_30BA_Crc4(int id)
{
    int cnt;
    int t;
    unsigned int n_rem = 0;
    unsigned char n_bit;
    unsigned char a = 0;
    unsigned char b = 0;
    unsigned short int n_prom[8];

    for (t = 0; t < 7; t++)
    {
        n_prom[t] = Cal_C[id][t];
    }
    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1)
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    a = (n_rem ^ 0x00);
    b = Cal_C[id][0] >> 12;
    if (a == b)
    {
        return 1;
    }
    else
        return 0;
}

void PressureSensor::MS5837_30BA_ReSet(void)
{
    uint8_t data[1];
    data[0] = MS5837_30BA_ResetCommand;
    LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, data, 1, 0xffff);
}

uint8_t PressureSensor::MS5837_30BA_PROM(int id)
{
    uint8_t memaddr[1];
    uint8_t data[2];
    int i;

    MS5837_30BA_ReSet(); // 复位 MS5837 后再读取 PROM。
    HAL_Delay(20);
    for (i = 0; i < 7; i++)
    {
        memaddr[0] = MS5837_30BA_PROM_RD + (i * 2);
        LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, memaddr, 1, 0xffff);
        LcBus_Receive(&hi2c2, B02_IIC_ADDRESS, data, 2, 0xffff);
        Cal_C[id][i] = (((uint16_t)data[0] << 8) | data[1]);
    }
    return !Cal_C[id][0];
}

unsigned long PressureSensor::MS5837_30BA_GetConversion(uint8_t command)
{
    unsigned long conversion = 0;
    uint8_t data = MS5837_30BA_ADC_RD;
    uint8_t temp[3];
    LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, &command, 1, 0xffff);
#if LC_USE_FREERTOS
    HAL_Delay(4); // OSR1024 至少 3 ms；额外 1 tick 覆盖 FreeRTOS tick 对齐的不确定性。
#else
    HAL_Delay(2); // 只供原始算法回归；生产 FreeRTOS 路径使用上方转换等待。
#endif
    // Delay_us(15000);
    LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, &data, 1, 0xffff);

    LcBus_Receive(&hi2c2, B02_IIC_ADDRESS, temp, 3, 0xffff);
    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;
}

/*signed int PressureSensor::MS5837_30BA_GetTemp(int id)
{
    if(flag_ok[id]) {
        D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR_8192);
        HAL_Delay(20);
        dT = D2_Temp - (((uint32_t) Cal_C[id][5]) * 256l);
        return dT;
    }
    else return -1;
}*/

float PressureSensor::MS5837_30BA_GetData(int id)
{

    if (flag_ok[id])
    {
        D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR1024);
        // HAL_Delay(1);
        // Delay_us(30000);
        D1_Pres = MS5837_30BA_GetConversion(MS5837_30BA_D1_OSR1024);
#if LC_USE_FREERTOS
        // 通信失败产生的 0/全 1 不能先进入旧温补平方项；先拒绝，再由校准/控制调用方处理。
        if (!LcBus_Ok(&hi2c2) || D1_Pres == 0 || D2_Temp == 0 ||
            D1_Pres >= 0xffffffU || D2_Temp >= 0xffffffU) return NAN;
#endif
        // HAL_Delay(3);
        // Delay_us(30000);
        dT = D2_Temp - (((uint32_t)Cal_C[id][5]) * 256l);
        SENS = (int64_t)Cal_C[id][1] * 65536l + ((int64_t)Cal_C[id][3] * dT) / 128l;
        OFF_ = (int64_t)Cal_C[id][2] * 131072l + ((int64_t)Cal_C[id][4] * dT) / 64l;

        TEMP = 2000l + (int64_t)(dT)*Cal_C[id][6] / 8388608LL;

        // 旧二阶温补表达式，仅用于 legacy 数值兼容；新 SI 补偿见 PressureCompensation.cpp。
        if (TEMP < 2000) // low temp
        {

            Ti = (11 * (int64_t)(dT) * (int64_t)(dT) / (34359738368LL));
            OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
        }
        else
        { // high temp
            Ti = 2LL * int64_t(dT) * int64_t(dT) / 137438953472LL;
            OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
            SENSi = 0;
        }
        OFF2 = OFF_ - OFFi;
        SENS2 = SENS - SENSi;
        /*if(D1_Pres==0||D2_Temp==0) return last_measure[id];
        else{
            float pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
            last_measure[id]=pressure;
            return pressure;

        }*/
        float pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
        return pressure;


    }
    else
        return -1;
}

void PressureSensor::Init_single(int id)
{
    data_pressure_offset[id] = 0;
    if (!flag_ok[id])
    {
        MS5837_30BA_PROM(id);
        flag_ok[id] = MS5837_30BA_Crc4(id);
    }
		// ------ TODO: Update the bias values here after receiving the calibration data.
    //------TODO:收到校准数据后修改此处零偏值
    /*
        for(int i=0;i<4;++i){
            data_pressure_offset[i] = flashData.pressure_offset[i]/1000.0;
        }
        */
    switch (Robot_Version)
    {
    case V30:
        data_pressure_offset[0] = PRESSURE_0_V30;
        data_pressure_offset[1] = PRESSURE_1_V30;
        data_pressure_offset[2] = PRESSURE_2_V30;
        data_pressure_offset[3] = PRESSURE_3_V30;
        break;
    case V31:
        data_pressure_offset[0] = PRESSURE_0_V31;
        data_pressure_offset[1] = PRESSURE_1_V31;
        data_pressure_offset[2] = PRESSURE_2_V31;
        data_pressure_offset[3] = PRESSURE_3_V31;
        break;
    case V32:
        data_pressure_offset[0] = PRESSURE_0_V32;
        data_pressure_offset[1] = PRESSURE_1_V32;
        data_pressure_offset[2] = PRESSURE_2_V32;
        data_pressure_offset[3] = PRESSURE_3_V32;
		break;
    case V33:
        data_pressure_offset[0] = PRESSURE_0_V33;
        data_pressure_offset[1] = PRESSURE_1_V33;
        data_pressure_offset[2] = PRESSURE_2_V33;
        data_pressure_offset[3] = PRESSURE_3_V33;
		break;
    case V40:
        data_pressure_offset[0] = PRESSURE_0_V40;
        data_pressure_offset[1] = PRESSURE_1_V40;
        data_pressure_offset[2] = PRESSURE_2_V40;
        data_pressure_offset[3] = PRESSURE_3_V40;
		break;
    }
}

void PressureSensor::Calibrate()
{
    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        TCA_SetChannel(i);
        // HAL_Delay(5);
        Calibrate_single(i);
        if (!LcBus_Ok(&hi2c2)) return; // 本次总线请求失败：不交付新估计或继续累计校准样本
    }
    // bsp_flash_write(&flashData);
}

void PressureSensor::Calibrate_single(int id)
{
    uint8_t TxBuffer[SENSOR_NUM * 9] = {0};
    data_pressure_offset[id] = 0;
    for (int j = 0; j < 100; ++j)
    {
        data_pressure_raw[id] = MS5837_30BA_GetData(id);
        if (!LcBus_Ok(&hi2c2)) return; // 本次总线请求失败：不交付新估计或继续累计校准样本
        data_pressure_offset[id] += data_pressure_raw[id] / 100;
    }
    int data_temp = (int)(data_pressure_offset[id] * 1000);
    // flashData.pressure_offset[id] = data_temp;
    int data_digit[7];
    for (int j = 0; j < 7; ++j)
    {
        data_digit[j] = data_temp % 10;
        data_temp /= 10;
    }
    TxBuffer[id * 9] = '0' + data_digit[6];
    TxBuffer[id * 9 + 1] = '0' + data_digit[5];
    TxBuffer[id * 9 + 2] = '0' + data_digit[4];
    TxBuffer[id * 9 + 3] = '0' + data_digit[3];
    TxBuffer[id * 9 + 4] = '.';
    TxBuffer[id * 9 + 5] = '0' + data_digit[2];
    TxBuffer[id * 9 + 6] = '0' + data_digit[1];
    TxBuffer[id * 9 + 7] = '0' + data_digit[0];
    if (id == 3)
        TxBuffer[id * 9 + 8] = '\n';
    else
        TxBuffer[id * 9 + 8] = ',';
    #if LC_USE_FREERTOS
    LcSerial_Write(TxBuffer, sizeof(TxBuffer)); // 发送队列复制数据
#else
    HAL_UART_Transmit(&huart6, TxBuffer, sizeof(TxBuffer), 0xffff);
#endif
}

void PressureSensor::OutputData()
{
    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        OutputData_single(i);
    }
}

void PressureSensor::OutputData_single(int id)
{
    uint8_t TxBuffer[10] = {0};
    bool IsPositive = true;
    int data_temp = (int)(data_pressure[id] * 1000);
    if (data_pressure[id] < 0)
    {
        IsPositive = false;
        data_temp = -data_temp;
    }
    int data_digit[7];
    for (int j = 0; j < 7; ++j)
    {
        data_digit[j] = data_temp % 10;
        data_temp /= 10;
    }
    if (IsPositive)
        TxBuffer[0] = ' ';
    else
        TxBuffer[0] = '-';
    TxBuffer[1] = '0' + data_digit[6];
    TxBuffer[2] = '0' + data_digit[5];
    TxBuffer[3] = '0' + data_digit[4];
    TxBuffer[4] = '0' + data_digit[3];
    TxBuffer[5] = '.';
    TxBuffer[6] = '0' + data_digit[2];
    TxBuffer[7] = '0' + data_digit[1];
    TxBuffer[8] = '0' + data_digit[0];
    if (id == 3)
        TxBuffer[9] = '\n';
    else
        TxBuffer[9] = ',';
    #if LC_USE_FREERTOS
    LcSerial_Write(TxBuffer, sizeof(TxBuffer)); // 发送队列复制数据
#else
    HAL_UART_Transmit(&huart6, TxBuffer, sizeof(TxBuffer), 0x00ff);
#endif
}

void PressureSensor::Handle_single(int id)
{

    data_pressure_raw[id] = MS5837_30BA_GetData(id);
    float temp = data_pressure_raw[id] - data_pressure_offset[id];
    temp = Pressure_Kf[id].update(temp); // First-order low-pass filtering. // 一阶低通滤波
    data_pressure[id] = temp;

    // if (temp < 500 && temp > -10)
    // {
    //     data_pressure[id] = temp;
    // }
    // if(data_pressure[id]>200||data_pressure[id]<-10) data_pressure[id]=last_measure[id]- data_pressure_offset[id];
}

// Following the approach of `Handle_single`, overall efficiency is improved by overlapping the wait time after the first I2C request.
// 仿照Handle_single的思路，通过重叠I2C第一次请求后的等待时间实现整体效率提高
void PressureSensor::Handle_all()
{
    static uint8_t data = MS5837_30BA_ADC_RD;
    static uint8_t command_tmp = MS5837_30BA_D2_OSR1024;
    static uint8_t command_pres = MS5837_30BA_D1_OSR1024;
    static uint8_t temp[3];
    static unsigned long conversion[8];    // conversion[0..3] 为四路 D2 温度 ADC，[4..7] 为四路 D1 压力 ADC；这里是数组元素，不是二进制位

#if LC_USE_FREERTOS
    if (ps_state != PS_HANDLE_STATE::GET_TEMPERATURE &&
        LcTime_NowUs() - conversion_started_us_ < LC_PRESSURE_CONVERSION_US) return; // 3000 us；晚释放时不提前读取 ADC
#endif
    switch (ps_state) {
        case PS_HANDLE_STATE::GET_TEMPERATURE:
            // Notify the sensor to prepare temperature data.
						// 告知传感器准备温度数据
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                TCA_SetChannel(i);
                LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, &command_tmp, 1, 1); // 地址 0xEC，命令 0x54；1 字节，HAL 超时参数 1 ms
            }
            if (!LcBus_Ok(&hi2c2)) { ps_state = PS_HANDLE_STATE::GET_TEMPERATURE; return; }
#if LC_USE_FREERTOS
            conversion_started_us_ = LcTime_NowUs(); // 最后一路转换命令发送完成时间；随后至少等待 3000 us
#endif
            ps_state = PS_HANDLE_STATE::GET_PRESSURE;
            break;

        case PS_HANDLE_STATE::GET_PRESSURE:
            // Acquire temperature information.
						// 收集温度信息
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                TCA_SetChannel(i);
                LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, &data, 1, 1); // 0xEC 器件写 ADC 读取命令 0x00；1 字节，HAL 超时参数 1 ms
                LcBus_Receive(&hi2c2, B02_IIC_ADDRESS, temp, 3, 1); // 从 0xEC 读取 3 字节 ADC 结果；HAL 超时参数 1 ms
                conversion[i] = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
            }

						// Notify the sensor to prepare pressure data.
            // 告知传感器准备压强数据
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                TCA_SetChannel(i);
                LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, &command_pres, 1, 1); // 地址 0xEC，命令 0x44；1 字节，HAL 超时参数 1 ms
            }
            if (!LcBus_Ok(&hi2c2)) { ps_state = PS_HANDLE_STATE::GET_TEMPERATURE; return; }
#if LC_USE_FREERTOS
            conversion_started_us_ = LcTime_NowUs(); // 最后一路转换命令发送完成时间；随后至少等待 3000 us
#endif
            ps_state = PS_HANDLE_STATE::CALCULATE;
            break;

        case PS_HANDLE_STATE::CALCULATE:
            // Retrieve the pressure readings.
						// 获取压强数值
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                TCA_SetChannel(i);
                LcBus_Transmit(&hi2c2, B02_IIC_ADDRESS, &data, 1, 1); // 0xEC 器件写 ADC 读取命令 0x00；1 字节，HAL 超时参数 1 ms
                LcBus_Receive(&hi2c2, B02_IIC_ADDRESS, temp, 3, 1); // 从 0xEC 读取 3 字节 ADC 结果；HAL 超时参数 1 ms
                conversion[i+4] = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
            }

						// Refer to `float PressureSensor::MS5837_30BA_GetData(int id)` to compute the four raw pressure values.
            // 参考float PressureSensor::MS5837_30BA_GetData(int id)计算四个原始压强
            if (!LcBus_Ok(&hi2c2)) { ps_state = PS_HANDLE_STATE::GET_TEMPERATURE; return; }
            lower_controller::PressureMeasurement sample = {};
            sample.sequence = last_measurement_.sequence + 1U;
            sample.read_completed_ms = HAL_GetTick();
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                sample.pressure_legacy[i] = data_pressure_raw[i];
                if (flag_ok[i])
                {
                    sample.prom_valid_mask |= (1U << i);
                    D2_Temp = conversion[i];
                    D1_Pres = conversion[i+4];
                    dT = D2_Temp - (((uint32_t)Cal_C[i][5]) * 256l);
                    SENS = (int64_t)Cal_C[i][1] * 65536l + ((int64_t)Cal_C[i][3] * dT) / 128l;
                    OFF_ = (int64_t)Cal_C[i][2] * 131072l + ((int64_t)Cal_C[i][4] * dT) / 64l;

                    TEMP = 2000l + (int64_t)(dT)*Cal_C[i][6] / 8388608LL;
                    if (TEMP < 2000) // low temp
                    {

                        Ti = (11 * (int64_t)(dT) * (int64_t)(dT) / (34359738368LL));
                        OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
                        SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
                    }
                    else
                    { // high temp
                        Ti = 2LL * int64_t(dT) * int64_t(dT) / 137438953472LL;
                        OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
                        SENSi = 0;
                    }
                    OFF2 = OFF_ - OFFi;
                    SENS2 = SENS - SENSi;
                    sample.pressure_legacy[i] = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
                }
                // else
                //     return -1;
            }

        // Deliver one completed array measurement; the estimator consumes it after Acquire.
        // 一轮采集只交付一次阵列测量；滤波由随后独立调用的 Estimate 执行。
        last_measurement_ = sample;
        ps_state = PS_HANDLE_STATE::GET_TEMPERATURE;
        break;
    }
}

void PressureSensor::Solve_plane_3(float *data, float *h, float *x, float *y, float *z)
{
    float pos_data[9];
    for (int i = 0; i < 3; ++i)
    {
        pos_data[i * 3] = x[i];
        pos_data[i * 3 + 1] = y[i];
        pos_data[i * 3 + 2] = z[i];
    }

    double a = 0, b = 0, c = 0, d = 0;
    Matrix pos(3, 3, pos_data);

    Matrix pos_inv;
    if (pos_inv.inv(pos))
    {

        float a1 = 0, a2 = 0, b1 = 0, b2 = 0, c1 = 0, c2 = 0, ax = 0, bx = 0, cx = 0;

        for (int i = 1; i < 4; ++i)
        {
            a1 -= pos_inv.mat[1][i];
            b1 -= pos_inv.mat[2][i];
            c1 -= pos_inv.mat[3][i];
            a2 -= pos_inv.mat[1][i] * h[i - 1];
            b2 -= pos_inv.mat[2][i] * h[i - 1];
            c2 -= pos_inv.mat[3][i] * h[i - 1];
        }

        ax = a1 * a1 + b1 * b1 + c1 * c1;
        bx = 2 * (a1 * a2 + b1 * b2 + c1 * c2);
        cx = a2 * a2 + b2 * b2 + c2 * c2 - 1;
        data[0] = (-bx - sqrt(bx * bx - 4 * ax * cx)) / (2 * ax);
        data[1] = a1 * d + a2;
        data[2] = b1 * d + b2;
        data[3] = c1 * d + c2;
    }
}

void PressureSensor::Solve_plane(float *data, float *h, float *x, float *y, float *z, int num)
{
    float xi = 0, yi = 0, zi = 0, hi = 0, xi2 = 0, yi2 = 0, zi2 = 0, xiyi = 0, xizi = 0, yizi = 0, xihi = 0, yihi = 0, zihi = 0;
    for (int i = 0; i < num; ++i)
    {
        xi += x[i];
        yi += y[i];
        zi += z[i];
        hi += h[i];
        xi2 += x[i] * x[i];
        yi2 += y[i] * y[i];
        zi2 += z[i] * z[i];
        xiyi += x[i] * y[i];
        xizi += x[i] * z[i];
        yizi += y[i] * z[i];
        xihi += x[i] * h[i];
        yihi += y[i] * h[i];
        zihi += z[i] * h[i];
    }
    float gradient[5] = {0};
    gradient[0] = xi2 * data[0] + xiyi * data[1] + xizi * data[2] + xi * data[3] + xihi + data[0] * data[4];
    gradient[1] = xiyi * data[0] + yi2 * data[1] + yizi * data[2] + yi * data[3] + yihi + data[1] * data[4];
    gradient[2] = xizi * data[0] + yizi * data[1] + zi2 * data[2] + zi * data[3] + zihi + data[2] * data[4];
    gradient[3] = xi * data[0] + yi * data[1] + zi * data[2] + num * data[3] + hi;
    gradient[4] = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] - 1;

    float jacobi_data[25] = {xi2 + data[4], xiyi, xizi, xi, data[0],
                             xiyi, yi2 + data[4], yizi, yi, data[1],
                             xizi, yizi, zi2 + data[4], zi, data[2],
                             xi, yi, zi, (float)num, 0,
                             data[0], data[1], data[2], 0, 0};
    Matrix jacobi(5, 5, jacobi_data);
    Matrix jacobi_inv;
    if (jacobi_inv.inv(jacobi))
    {
        float data_dx[5] = {0};
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                data_dx[i] += jacobi_inv.mat[i + 1][j + 1] * gradient[j];
            }
            data[i] -= 0.5 * data_dx[i];
        }
    }
}

void PressureSensor::Update_plane()
{
    Solve_plane(data_plane, data_pressure, site.x, site.y, site.z, SENSOR_NUM);

    for (int i = 0; i < 3; ++i)
    {
        data_level[i] = data_plane[i];
    }
}
/*
void PressureSensor::get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
*/

void Sonar::Init()
{
    data_offset = 0;
    for (int i = 0; i < 100; ++i)
    {
        Handle();
        HAL_Delay(20);
        if (RxBuffer[0] == 0xFF)
            data_offset += (RxBuffer[1] * 256 + RxBuffer[2]) / 100;
    }
}

void Sonar::Handle()
{
    // if(tim_id == 1){
    uint8_t rqst[1] = {' '};
    HAL_UART_Transmit_IT(&huart6, rqst, 1);
    //}
}

void Sonar::Receive()
{
    if (RxBuffer[0] == 0xFF)
    {
        data = RxBuffer[1] * 256 + RxBuffer[2] - data_offset;
    }
    else
        data = -1;

    // HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}
