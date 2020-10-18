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

void SetupADE9153A() {
    SPI_Write_16(REG_AI_PGAGAIN, ADE9153A_AI_PGAGAIN);
    SPI_Write_32(REG_CONFIG0, ADE9153A_CONFIG0);
    SPI_Write_16(REG_CONFIG1, ADE9153A_CONFIG1);
    SPI_Write_16(REG_CONFIG2, ADE9153A_CONFIG2);
    SPI_Write_16(REG_CONFIG3, ADE9153A_CONFIG3);
    SPI_Write_16(REG_ACCMODE, ADE9153A_ACCMODE);
    SPI_Write_32(REG_VLEVEL, ADE9153A_VLEVEL);
    SPI_Write_16(REG_ZX_CFG, ADE9153A_ZX_CFG);
    SPI_Write_32(REG_MASK, ADE9153A_MASK);
    SPI_Write_32(REG_ACT_NL_LVL, ADE9153A_ACT_NL_LVL);
    SPI_Write_32(REG_REACT_NL_LVL, ADE9153A_REACT_NL_LVL);
    SPI_Write_32(REG_APP_NL_LVL, ADE9153A_APP_NL_LVL);
    SPI_Write_16(REG_COMPMODE, ADE9153A_COMPMODE);
    SPI_Write_32(REG_VDIV_RSMALL, ADE9153A_VDIV_RSMALL);
    SPI_Write_16(REG_EP_CFG, ADE9153A_EP_CFG);
    SPI_Write_16(REG_EGY_TIME, ADE9153A_EGY_TIME); //Energy accumulation ON
    SPI_Write_16(REG_TEMP_CFG, ADE9153A_TEMP_CFG);
}

static uint16_t get_cmd_for(uint16_t address, SPIOperation_t operation);

/**
 * Writes 16bit data to a 16 bit register.
 */
void SPI_Write_16(uint16_t address, uint16_t data) {
    uint16_t cmd = get_cmd_for(address, SPI_WRITE);
    uint8_t data_bytes[4] = {
            data >> 8u,
            data >> 0u
    };
    ade9153a_spi_write(cmd, data_bytes, 2);
}

/**
 * Writes 32bit data to a 32 bit register.
 */
void SPI_Write_32(uint16_t address, uint32_t data) {
    uint16_t cmd = get_cmd_for(address, SPI_WRITE);
    uint8_t data_bytes[6] = {
            data >> 24u,
            data >> 16u,
            data >> 8u,
            data >> 0u,
    };
    ade9153a_spi_write(cmd, data_bytes, 4);
}

/**
 * Reads 16bit data from register.
 */
uint16_t SPI_Read_16(uint16_t address) {
    uint16_t data;
    uint16_t cmd = get_cmd_for(address, SPI_READ);
    uint8_t data_buffer[2] = {};

    ade9153a_spi_read(cmd, data_buffer, 2);

    data = \
        (uint16_t) (data_buffer[0] << 8u) & \
        (uint16_t) (data_buffer[1] << 0u);

    return data;
}

/**
 * Reads 32bit data from register.
 */
uint32_t SPI_Read_32(uint16_t address) {
    uint16_t data;
    uint16_t cmd = get_cmd_for(address, SPI_READ);
    uint8_t data_buffer[4] = {};

    ade9153a_spi_read(cmd, data_buffer, 4);

    data = \
        (uint32_t) (data_buffer[0] << 24u) & \
        (uint32_t) (data_buffer[1] << 16u) & \
        (uint32_t) (data_buffer[2] << 8u) & \
        (uint32_t) (data_buffer[3] << 0u);

    return data;
}

/*
Description: Reads the metrology data from the ADE9153A
Input: Structure name
Output: Respective metrology data
*/

void ReadEnergyRegs(struct EnergyRegs *Data) {
    int32_t tempReg;
    float tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_AWATTHR_HI));
    Data->ActiveEnergyReg = tempReg;
    tempValue = (float) tempReg * CAL_ENERGY_CC / 1000;
    Data->ActiveEnergyValue = tempValue;                //Energy in mWhr

    tempReg = (int32_t) (SPI_Read_32(REG_AFVARHR_HI));
    Data->FundReactiveEnergyReg = tempReg;
    tempValue = (float) tempReg * CAL_ENERGY_CC / 1000;
    Data->FundReactiveEnergyValue = tempValue;            //Energy in mVARhr

    tempReg = (int32_t) (SPI_Read_32(REG_AVAHR_HI));
    Data->ApparentEnergyReg = tempReg;
    tempValue = (float) tempReg * CAL_ENERGY_CC / 1000;
    Data->ApparentEnergyValue = tempValue;                //Energy in mVAhr
}

void ReadPowerRegs(struct PowerRegs *Data) {
    int32_t tempReg;
    float tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_AWATT));
    Data->ActivePowerReg = tempReg;
    tempValue = (float) tempReg * CAL_POWER_CC / 1000;
    Data->ActivePowerValue = tempValue;                    //Power in mW

    tempReg = (int32_t) (SPI_Read_32(REG_AFVAR));
    Data->FundReactivePowerReg = tempReg;
    tempValue = (float) tempReg * CAL_POWER_CC / 1000;
    Data->FundReactivePowerValue = tempValue;            //Power in mVAR

    tempReg = (int32_t) (SPI_Read_32(REG_AVA));
    Data->ApparentPowerReg = tempReg;
    tempValue = (float) tempReg * CAL_POWER_CC / 1000;
    Data->ApparentPowerValue = tempValue;                //Power in mVA
}

void ReadRMSRegs(struct RMSRegs *Data) {
    uint32_t tempReg;
    float tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_AIRMS));
    Data->CurrentRMSReg = tempReg;
    tempValue = (float) tempReg * CAL_IRMS_CC / 1000;    //RMS in mA
    Data->CurrentRMSValue = tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_AVRMS));
    Data->VoltageRMSReg = tempReg;
    tempValue = (float) tempReg * CAL_VRMS_CC / 1000;    //RMS in mV
    Data->VoltageRMSValue = tempValue;
}

void ReadHalfRMSRegs(struct HalfRMSRegs *Data) {
    uint32_t tempReg;
    float tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_AIRMS_OC));
    Data->HalfCurrentRMSReg = tempReg;
    tempValue = (float) tempReg * CAL_IRMS_CC / 1000;    //Half-RMS in mA
    Data->HalfCurrentRMSValue = tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_AVRMS_OC));
    Data->HalfVoltageRMSReg = tempReg;
    tempValue = (float) tempReg * CAL_VRMS_CC / 1000;    //Half-RMS in mV
    Data->HalfVoltageRMSValue = tempValue;
}

void ReadPQRegs(struct PQRegs *Data) {
    int32_t tempReg;
    uint16_t temp;
    float mulConstant;
    float tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_APF)); //Read PF register
    Data->PowerFactorReg = tempReg;
    tempValue = (float) tempReg / (float) 134217728; //Calculate PF
    Data->PowerFactorValue = tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_APERIOD)); //Read PERIOD register
    Data->PeriodReg = tempReg;
    tempValue = (float) (4000 * 65536) / (float) (tempReg + 1); //Calculate Frequency
    Data->FrequencyValue = tempValue;

    temp = SPI_Read_16(REG_ACCMODE); //Read frequency setting register
    if ((temp & 0x0010) > 0) {
        mulConstant = 0.02109375;  //multiplier constant for 60Hz system
    } else {
        mulConstant = 0.017578125; //multiplier constant for 50Hz system
    }

    tempReg = (int16_t) (SPI_Read_16(REG_ANGL_AV_AI)); //Read ANGLE register
    Data->AngleReg_AV_AI = tempReg;
    tempValue = tempReg * mulConstant;    //Calculate Angle in degrees
    Data->AngleValue_AV_AI = tempValue;
}

void ReadAcalRegs(struct AcalRegs *Data) {
    uint32_t tempReg;
    float tempValue;

    tempReg = (int32_t) (SPI_Read_32(REG_MS_ACAL_AICC)); //Read AICC register
    Data->AcalAICCReg = tempReg;
    tempValue = (float) tempReg / (float) 2048; //Calculate Conversion Constant (CC)
    Data->AICC = tempValue;
    tempReg = (int32_t) (SPI_Read_32(REG_MS_ACAL_AICERT)); //Read AICERT register
    Data->AcalAICERTReg = tempReg;

    tempReg = (int32_t) (SPI_Read_32(REG_MS_ACAL_AVCC)); //Read AVCC register
    Data->AcalAVCCReg = tempReg;
    tempValue = (float) tempReg / (float) 2048; //Calculate Conversion Constant (CC)
    Data->AVCC = tempValue;
    tempReg = (int32_t) (SPI_Read_32(REG_MS_ACAL_AVCERT)); //Read AICERT register
    Data->AcalAVCERTReg = tempReg;
}

/*
Description: Start autocalibration on the respective channel
Input: -
Output: Did it start correctly?
*/

bool StartAcal_AINormal(void) {
    uint32_t ready = 0;
    int waitTime = 0;

    ready = SPI_Read_32(REG_MS_STATUS_CURRENT);                //Read system ready bit

    while ((ready & 0x00000001) == 0) {
        if (waitTime > 11) {
            return false;
        }
        ade9153a_spi_delay_ms(100);
        waitTime++;
    }

    SPI_Write_32(REG_MS_ACAL_CFG, 0x00000013);
    return true;
}

bool StartAcal_AITurbo(void) {
    uint32_t ready = 0;
    int waitTime = 0;

    while ((ready & 0x00000001) == 0) {
        ready = SPI_Read_32(REG_MS_STATUS_CURRENT);        //Read system ready bit
        if (waitTime > 15) {
            return false;
        }
        ade9153a_spi_delay_ms(100);
        waitTime++;
    }

    SPI_Write_32(REG_MS_ACAL_CFG, 0x00000017);
    return true;
}

bool StartAcal_AV(void) {
    uint32_t ready = 0;
    int waitTime = 0;

    while ((ready & 0x00000001) == 0) {
        ready = SPI_Read_32(REG_MS_STATUS_CURRENT);        //Read system ready bit
        if (waitTime > 15) {
            return false;
        }
        ade9153a_spi_delay_ms(100);
        waitTime++;
    }

    SPI_Write_32(REG_MS_ACAL_CFG, 0x00000043);
    return true;
}

void StopAcal(void) {
    SPI_Write_32(REG_MS_ACAL_CFG, 0x00000000);
}

bool ApplyAcal(float AICC, float AVCC) {
    int32_t AIGAIN;
    int32_t AVGAIN;

    AIGAIN = (AICC / (CAL_IRMS_CC * 1000) - 1) * 134217728;
    AVGAIN = (AVCC / (CAL_VRMS_CC * 1000) - 1) * 134217728;

    SPI_Write_32(REG_AIGAIN, AIGAIN);
    SPI_Write_32(REG_AVGAIN, AVGAIN);
}

/*
Description: Starts a new acquisition cycle. Waits for constant time and returns register value and temperature in Degree Celsius
Input:	Structure name
Output: Register reading and temperature value in Degree Celsius
*/
void ReadTemperature(Temperature_t *data) {
    uint32_t trim;
    uint16_t gain;
    uint16_t offset;
    uint16_t tempReg;
    float tempValue;

    SPI_Write_16(REG_TEMP_CFG, ADE9153A_TEMP_CFG);//Start temperature acquisition cycle
    ade9153a_spi_delay_ms(10); //delay of 2ms. Increase delay if TEMP_TIME is changed

    trim = SPI_Read_32(REG_TEMP_TRIM);
    gain = (trim & 0xFFFF);  //Extract 16 LSB
    offset = ((trim >> 16) & 0xFFFF); //Extract 16 MSB
    tempReg = SPI_Read_16(REG_TEMP_RSLT);    //Read Temperature result register
    tempValue = ((float) offset / 32.00) - ((float) tempReg * (float) gain / (float) 131072);

    data->TemperatureReg = tempReg;
    data->TemperatureVal = tempValue;
}

static uint16_t get_cmd_for(uint16_t address, SPIOperation_t operation) {
    return (((uint16_t) (address << 4u) & 0xFFF0u) + operation);
}