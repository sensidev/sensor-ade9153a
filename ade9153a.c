/**
    Copyright (c) 2018, Analog Devices, Inc.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted (subject to the limitations in the disclaimer
    below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

    * Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

    NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
    BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
    CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
    BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
    TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
    LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ade9153a.h"
#include "ade9153a_regs.h"
#include "ade9153a_spi.h"

static uint16_t get_cmd_for(uint16_t address, SPIOperation_t operation);

/**
 * Example of how to configure ADE9153A analog frontend.
 */
void ade9153a_setup_example() {
    ade9153a_spi_write_16(REG_AI_PGAGAIN, ADE9153A_AI_PGAGAIN);
    ade9153a_spi_write_32(REG_CONFIG0, ADE9153A_CONFIG0);
    ade9153a_spi_write_16(REG_CONFIG1, ADE9153A_CONFIG1);
    ade9153a_spi_write_16(REG_CONFIG2, ADE9153A_CONFIG2);
    ade9153a_spi_write_16(REG_CONFIG3, ADE9153A_CONFIG3);
    ade9153a_spi_write_16(REG_ACCMODE, ADE9153A_ACCMODE);
    ade9153a_spi_write_32(REG_VLEVEL, ADE9153A_VLEVEL);
    ade9153a_spi_write_16(REG_ZX_CFG, ADE9153A_ZX_CFG);
    ade9153a_spi_write_32(REG_MASK, ADE9153A_MASK);
    ade9153a_spi_write_32(REG_ACT_NL_LVL, ADE9153A_ACT_NL_LVL);
    ade9153a_spi_write_32(REG_REACT_NL_LVL, ADE9153A_REACT_NL_LVL);
    ade9153a_spi_write_32(REG_APP_NL_LVL, ADE9153A_APP_NL_LVL);
    ade9153a_spi_write_16(REG_COMPMODE, ADE9153A_COMPMODE);
    ade9153a_spi_write_32(REG_VDIV_RSMALL, ADE9153A_VDIV_RSMALL);
    ade9153a_spi_write_16(REG_EP_CFG, ADE9153A_EP_CFG);
    ade9153a_spi_write_16(REG_EGY_TIME, ADE9153A_EGY_TIME); //Energy accumulation ON
    ade9153a_spi_write_16(REG_TEMP_CFG, ADE9153A_TEMP_CFG);
}

/**
 * Writes 16bit data to a 16 bit register.
 */
ADE9153AStatus_t ade9153a_spi_write_16(uint16_t address, uint16_t data) {
    uint16_t cmd = get_cmd_for(address, SPI_WRITE);
    uint8_t data_bytes[4] = {
            cmd >> 8u,
            cmd,
            data >> 8u,
            data
    };
    return ade9153a_spi_write(data_bytes, 4);
}

/**
 * Writes 32bit data to a 32 bit register.
 */
ADE9153AStatus_t ade9153a_spi_write_32(uint16_t address, uint32_t data) {
    uint16_t cmd = get_cmd_for(address, SPI_WRITE);
    uint8_t data_bytes[6] = {
            cmd >> 8u,
            cmd,
            data >> 24u,
            data >> 16u,
            data >> 8u,
            data,
    };
    return ade9153a_spi_write(data_bytes, 6);
}

/**
 * Reads 16bit data from register.
 */
uint16_t ade9153a_spi_read_16(uint16_t address) {
    uint16_t data;
    uint16_t cmd = get_cmd_for(address, SPI_READ);
    uint8_t data_buffer[2] = {};

    ade9153a_spi_read(cmd, data_buffer, 2);

    data = \
        (uint16_t) (data_buffer[0] << 8u) & \
        (uint16_t) (data_buffer[1]);

    return data;
}

/**
 * Reads 32bit data from register.
 */
uint32_t ade9153a_spi_read_32(uint16_t address) {
    uint16_t data;
    uint16_t cmd = get_cmd_for(address, SPI_READ);
    uint8_t data_buffer[4] = {};

    ade9153a_spi_read(cmd, data_buffer, 4);

    data = \
        (uint32_t) (data_buffer[0] << 24u) & \
        (uint32_t) (data_buffer[1] << 16u) & \
        (uint32_t) (data_buffer[2] << 8u) & \
        (uint32_t) (data_buffer[3]);

    return data;
}

/**
 * Reads the metrology data from the ADE9153A.
 * @param data
 */
void ade9153a_read_energy_regs(EnergyRegs_t *data) {
    int32_t raw_value;
    float value;

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AWATTHR_HI));
    data->ActiveEnergyReg = raw_value;
    value = (float) raw_value * CAL_ENERGY_CC / 1000;
    data->ActiveEnergyValue = value; // Energy in mWhr

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AFVARHR_HI));
    data->FundReactiveEnergyReg = raw_value;
    value = (float) raw_value * CAL_ENERGY_CC / 1000;
    data->FundReactiveEnergyValue = value; // Energy in mVARhr

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AVAHR_HI));
    data->ApparentEnergyReg = raw_value;
    value = (float) raw_value * CAL_ENERGY_CC / 1000;
    data->ApparentEnergyValue = value; // Energy in mVAhr
}

void ade9153a_read_power_regs(PowerRegs_t *data) {
    int32_t raw_value;
    float value;

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AWATT));
    data->ActivePowerReg = raw_value;
    value = (float) raw_value * CAL_POWER_CC / 1000;
    data->ActivePowerValue = value; // Power in mW

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AFVAR));
    data->FundReactivePowerReg = raw_value;
    value = (float) raw_value * CAL_POWER_CC / 1000;
    data->FundReactivePowerValue = value; // Power in mVAR

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AVA));
    data->ApparentPowerReg = raw_value;
    value = (float) raw_value * CAL_POWER_CC / 1000;
    data->ApparentPowerValue = value; // Power in mVA
}

void ade9153a_read_rms_regs(RMSRegs_t *Data) {
    uint32_t raw_value;
    float value;

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AIRMS));
    Data->CurrentRMSReg = raw_value;
    value = (float) raw_value * CAL_IRMS_CC / 1000; // RMS in mA
    Data->CurrentRMSValue = value;

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AVRMS));
    Data->VoltageRMSReg = raw_value;
    value = (float) raw_value * CAL_VRMS_CC / 1000; // RMS in mV
    Data->VoltageRMSValue = value;
}

void ade9153a_read_half_rms_regs(HalfRMSRegs_t *data) {
    uint32_t raw_value;
    float value;

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AIRMS_OC));
    data->HalfCurrentRMSReg = raw_value;
    value = (float) raw_value * CAL_IRMS_CC / 1000; // Half-RMS in mA
    data->HalfCurrentRMSValue = value;

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_AVRMS_OC));
    data->HalfVoltageRMSReg = raw_value;
    value = (float) raw_value * CAL_VRMS_CC / 1000; // Half-RMS in mV
    data->HalfVoltageRMSValue = value;
}

void ade9153a_read_pq_regs(PQRegs_t *data) {
    int32_t raw_value;
    float value;
    double angle_multiplier_constant;

    raw_value = (int32_t) (ade9153a_spi_read_32(REG_APF)); // Read PF register
    data->PowerFactorReg = raw_value;
    value = (float) raw_value / 134217728.0f; // Calculate PF. TODO: Where is this value coming from?
    data->PowerFactorValue = value;
    raw_value = (int32_t) (ade9153a_spi_read_32(REG_APERIOD)); // Read PERIOD register
    data->PeriodReg = raw_value;
    value = (float) (4000 * 65536) / (float) (raw_value + 1); // Calculate Frequency
    data->FrequencyValue = value;

    uint16_t frequency_raw_value = ade9153a_spi_read_16(REG_ACCMODE); // Read frequency setting register
    if ((frequency_raw_value & 0x0010u) > 0) {
        angle_multiplier_constant = 0.02109375; // multiplier constant for 60Hz system
    } else {
        angle_multiplier_constant = 0.017578125; // multiplier constant for 50Hz system
    }

    raw_value = (int16_t) (ade9153a_spi_read_16(REG_ANGL_AV_AI)); // Read ANGLE register
    data->AngleReg_AV_AI = raw_value;

    value = (float) (raw_value * angle_multiplier_constant); // Calculate Angle in degrees
    data->AngleValue_AV_AI = value;
}

void ade9153a_read_acal_regs(AcalRegs_t *data) {
    uint32_t raw_data;
    float value;

    raw_data = (int32_t) (ade9153a_spi_read_32(REG_MS_ACAL_AICC)); // Read AICC register
    data->AcalAICCReg = raw_data;
    value = (float) raw_data / (float) 2048; // Calculate Conversion Constant (CC)
    data->AICC = value;
    raw_data = (int32_t) (ade9153a_spi_read_32(REG_MS_ACAL_AICERT)); // Read AICERT register
    data->AcalAICERTReg = raw_data;

    raw_data = (int32_t) (ade9153a_spi_read_32(REG_MS_ACAL_AVCC)); // Read AVCC register
    data->AcalAVCCReg = raw_data;
    value = (float) raw_data / (float) 2048; // Calculate Conversion Constant (CC)
    data->AVCC = value;
    raw_data = (int32_t) (ade9153a_spi_read_32(REG_MS_ACAL_AVCERT)); // Read AICERT register
    data->AcalAVCERTReg = raw_data;
}


/**
 * Start auto-calibration on the respective channel.
 * @return true if started correctly, false otherwise.
 */
bool ade9153a_start_acal_ai_normal() {
    uint32_t ready_reg_value = 0;
    int waitTime = 0;

    while ((ready_reg_value & 0x00000001u) == 0) {
        ready_reg_value = ade9153a_spi_read_32(REG_MS_STATUS_CURRENT); // Read system ready bit
        if (waitTime > 11) {
            return false;
        }
        ade9153a_spi_delay_ms(100);
        waitTime++;
    }

    ade9153a_spi_write_32(REG_MS_ACAL_CFG, 0x00000013);
    return true;
}

bool ade9153a_start_acal_ai_turbo() {
    uint32_t ready_reg_value = 0;
    int waitTime = 0;

    while ((ready_reg_value & 0x00000001u) == 0) {
        ready_reg_value = ade9153a_spi_read_32(REG_MS_STATUS_CURRENT); // Read system ready bit
        if (waitTime > 15) {
            return false;
        }
        ade9153a_spi_delay_ms(100);
        waitTime++;
    }

    ade9153a_spi_write_32(REG_MS_ACAL_CFG, 0x00000017);
    return true;
}

bool ade9153a_start_acal_av() {
    uint32_t ready = 0;
    int waitTime = 0;

    while ((ready & 0x00000001u) == 0) {
        ready = ade9153a_spi_read_32(REG_MS_STATUS_CURRENT); // Read system ready bit
        if (waitTime > 15) {
            return false;
        }
        ade9153a_spi_delay_ms(100);
        waitTime++;
    }

    ade9153a_spi_write_32(REG_MS_ACAL_CFG, 0x00000043);
    return true;
}

void ade9153a_stop_acal() {
    ade9153a_spi_write_32(REG_MS_ACAL_CFG, 0x00000000);
}

bool ade9153a_apply_acal(float AICC, float AVCC) {
    ADE9153AStatus_t status;

    int32_t AIGAIN;
    int32_t AVGAIN;

    AIGAIN = (-(AICC / (CAL_IRMS_CC * 1000)) - 1) * 134217728.0f;
    AVGAIN = (AVCC / (CAL_VRMS_CC * 1000) - 1) * 134217728.0f;

    status = ade9153a_spi_write_32(REG_AIGAIN, AIGAIN);
    if (ADE9153A_OK != status) return false;

    status = ade9153a_spi_write_32(REG_AVGAIN, AVGAIN);
    if (ADE9153A_OK != status) return false;

    return true;
}

/**
 * Starts a new acquisition cycle.
 * Waits for constant time and returns register value and temperature in Degree Celsius
 * @param data
 */
void ade9153a_read_temperature(Temperature_t *data) {
    uint32_t trim;
    uint16_t gain;
    uint16_t offset;
    uint16_t temp_reg_value;
    float temp_value;

    ade9153a_spi_write_16(REG_TEMP_CFG, ADE9153A_TEMP_CFG); // Start temperature acquisition cycle
    ade9153a_spi_delay_ms(10); // delay of 2ms. Increase delay if TEMP_TIME is changed

    trim = ade9153a_spi_read_32(REG_TEMP_TRIM);
    gain = (trim & 0xFFFFu);  // Extract 16 LSB
    offset = ((trim >> 16u) & 0xFFFFu); // Extract 16 MSB
    temp_reg_value = ade9153a_spi_read_16(REG_TEMP_RSLT); // Read Temperature result register
    temp_value = ((float) offset / 32.00f) - ((float) temp_reg_value * (float) gain / (float) 131072);

    data->TemperatureReg = temp_reg_value;
    data->TemperatureVal = temp_value;
}

static uint16_t get_cmd_for(uint16_t address, SPIOperation_t operation) {
    return (((uint16_t) (address << 4u) & 0xFFF0u) + operation);
}