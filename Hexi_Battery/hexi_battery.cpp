/** Battery Driver for Hexiwear 
 *  This file contains battery driver functionality for check battery level and status
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NXP, nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * visit: http://www.mikroe.com and http://www.nxp.com
 *
 * get support at: http://www.mikroe.com/forum and https://community.nxp.com
 *
 * Project HEXIWEAR, 2015
 */

#include "hexi_battery.h"
#include "mbed.h"

#define BITBAND_ACCESS32(Reg,Bit) (*((uint32_t volatile*)(0x42000000u + (32u*((uintptr_t)(Reg) - (uintptr_t)0x40000000u)) + (4u*((uintptr_t)(Bit))))))
#define ADC_SC1_REG(base,index)                  ((base)->SC1[index])
#define ADC_WR_SC1(base, index, value) (ADC_SC1_REG(base, index) = (value))
#define ADC_BRD_SC1_COCO(base, index) (BITBAND_ACCESS32(&ADC_SC1_REG(base, index), ADC_SC1_COCO_SHIFT))
#define ADC_RD_R_D(base, index) ((ADC_R_REG(base, index) & ADC_R_D_MASK) >> ADC_R_D_SHIFT)
#define ADC_BRD_R_D(base, index) (ADC_RD_R_D(base, index))
#define ADC_R_REG(base,index)                    ((base)->R[index])
#define SIM_BWR_SCGC_BIT(base, index, value) (BITBAND_ACCESS32(&SIM_SCGC_BIT_REG((base), (index)), SIM_SCGC_BIT_SHIFT(index)) = (uint32_t)(value))
#define SIM_SCGC_BIT_SHIFT(index)            ((uint32_t)(index) & ((1U << 5) - 1U))
#define SIM_SCGC_BIT_REG(base, index)        (*((volatile uint32_t *)&SIM_SCGC1_REG(base) + (((uint32_t)(index) >> 5) - 0U)))
#define SIM_SCGC1_REG(base)                      ((base)->SCGC1)

#define ADC_RD_CFG1(base)        (ADC_CFG1_REG(base))
#define ADC_WR_CFG1(base, value) (ADC_CFG1_REG(base) = (value))

#define ADC_CFG1_REG(base)                       ((base)->CFG1)
#define ADC_WR_CFG2(base, value) (ADC_CFG2_REG(base) = (value))
#define ADC_CFG2_REG(base)                       ((base)->CFG2)

#define ADC_WR_CV1(base, value)  (ADC_CV1_REG(base) = (value))
#define ADC_CV1_REG(base)                        ((base)->CV1)
#define ADC_WR_CV2(base, value)  (ADC_CV2_REG(base) = (value))
#define ADC_CV2_REG(base)                        ((base)->CV2)

#define ADC_WR_SC2(base, value)  (ADC_SC2_REG(base) = (value))
#define ADC_SC2_REG(base)                        ((base)->SC2)

#define ADC_WR_SC3(base, value)  (ADC_SC3_REG(base) = (value))
#define ADC_SC3_REG(base)                        ((base)->SC3)

#define ADC_RD_SC2(base)         (ADC_SC2_REG(base))
#define ADC_RD_CFG2(base)        (ADC_CFG2_REG(base))
#define ADC_CFG2_REG(base)                       ((base)->CFG2)

#define ADC_RD_SC3(base)         (ADC_SC3_REG(base))
#define ADC_SC3_REG(base)                        ((base)->SC3)

#define ADC_INSTANCE_COUNT (2U) /*!< Number of instances of the ADC module. */
const IRQn_Type g_adcIrqId[ADC_INSTANCE_COUNT] = ADC_IRQS;

typedef enum _adc16_chn {
    kAdc16Chn0  = 0U,  /*!< AD0. */
    kAdc16Chn1  = 1U,  /*!< AD1. */
    kAdc16Chn2  = 2U,  /*!< AD2. */
    kAdc16Chn3  = 3U,  /*!< AD3. */
    kAdc16Chn4  = 4U,  /*!< AD4. */
    kAdc16Chn5  = 5U,  /*!< AD5. */
    kAdc16Chn6  = 6U,  /*!< AD6. */
    kAdc16Chn7  = 7U,  /*!< AD6. */
    kAdc16Chn8  = 8U,  /*!< AD8.  */
    kAdc16Chn9  = 9U,  /*!< AD9.  */
    kAdc16Chn10 = 10U, /*!< AD10. */
    kAdc16Chn11 = 11U, /*!< AD11. */
    kAdc16Chn12 = 12U, /*!< AD12. */
    kAdc16Chn13 = 13U, /*!< AD13. */
    kAdc16Chn14 = 14U, /*!< AD14. */
    kAdc16Chn15 = 15U, /*!< AD15. */
    kAdc16Chn16 = 16U, /*!< AD16. */
    kAdc16Chn17 = 17U, /*!< AD17. */
    kAdc16Chn18 = 18U, /*!< AD18. */
    kAdc16Chn19 = 19U, /*!< AD19. */
    kAdc16Chn20 = 20U, /*!< AD20. */
    kAdc16Chn21 = 21U, /*!< AD21. */
    kAdc16Chn22 = 22U, /*!< AD22. */
    kAdc16Chn23 = 23U, /*!< AD23. */
    kAdc16Chn24 = 24U, /*!< AD24. */
    kAdc16Chn25 = 25U, /*!< AD25. */
    kAdc16Chn26 = 26U, /*!< AD26. */
    kAdc16Chn27 = 27U, /*!< AD27. */
    kAdc16Chn28 = 28U, /*!< AD28. */
    kAdc16Chn29 = 29U, /*!< AD29. */
    kAdc16Chn30 = 30U, /*!< AD30. */
    kAdc16Chn31 = 31U,  /*!< AD31. */

    kAdc16Chn0d = kAdc16Chn0,  /*!< DAD0. */
    kAdc16Chn1d = kAdc16Chn1,  /*!< DAD1. */
    kAdc16Chn2d = kAdc16Chn2,  /*!< DAD2. */
    kAdc16Chn3d = kAdc16Chn3,  /*!< DAD3. */
    kAdc16Chn4a = kAdc16Chn4,  /*!< AD4a. */
    kAdc16Chn5a = kAdc16Chn5,  /*!< AD5a. */
    kAdc16Chn6a = kAdc16Chn6,  /*!< AD6a. */
    kAdc16Chn7a = kAdc16Chn7,  /*!< AD7a. */
    kAdc16Chn4b = kAdc16Chn4,  /*!< AD4b. */
    kAdc16Chn5b = kAdc16Chn5,  /*!< AD5b. */
    kAdc16Chn6b = kAdc16Chn6,  /*!< AD6b. */
    kAdc16Chn7b = kAdc16Chn7   /*!< AD7b. */

} adc16_chn_t;

typedef enum _adc16_status {
    kStatus_ADC16_Success         = 0U, /*!< Success. */
    kStatus_ADC16_InvalidArgument = 1U, /*!< Invalid argument existed. */
    kStatus_ADC16_Failed          = 2U  /*!< Execution failed. */
} adc16_status_t;

typedef struct Adc16ChnConfig {
    adc16_chn_t chnIdx;          /*!< Select the sample channel index. */
    bool convCompletedIntEnable; /*!< Enable the conversion complete interrupt. */
#if FSL_FEATURE_ADC16_HAS_DIFF_MODE
    bool diffConvEnable;         /*!< Enable the differential conversion. */
#endif  /** FSL_FEATURE_ADC16_HAS_DIFF_MODE */
} adc16_chn_config_t;

extern const adc16_chn_config_t BATTERY_ADC_ChnConfig;

const adc16_chn_config_t BATTERY_ADC_ChnConfig = {
    .chnIdx = kAdc16Chn16,
    .convCompletedIntEnable = false,
    .diffConvEnable = false
};

ADC_Type * const g_adcBase[] = ADC_BASE_PTRS;


void ADC16_HAL_ConfigChn(ADC_Type * base, uint32_t chnGroup, const adc16_chn_config_t *configPtr)
{
    uint16_t tmp = 0U;

    /** Interrupt enable. */
    if (configPtr->convCompletedIntEnable) {
        tmp |= ADC_SC1_AIEN_MASK;
    }

    /** Differential mode enable. */
#if FSL_FEATURE_ADC16_HAS_DIFF_MODE
    if (configPtr->diffConvEnable) {
        tmp |= ADC_SC1_DIFF_MASK;
    }
#endif  /** FSL_FEATURE_ADC16_HAS_DIFF_MODE */

    /** Input channel select. */
    tmp |= ADC_SC1_ADCH((uint32_t)(configPtr->chnIdx));

    ADC_WR_SC1(base, chnGroup, tmp);
}

adc16_status_t ADC16_DRV_ConfigConvChn(uint32_t instance,
                                       uint32_t chnGroup, const adc16_chn_config_t *configPtr)
{
    ADC_Type * base = g_adcBase[instance];

    if (!configPtr) {
        return kStatus_ADC16_InvalidArgument;
    }

    ADC16_HAL_ConfigChn(base, chnGroup, configPtr);

    return kStatus_ADC16_Success;
}

static inline bool ADC16_HAL_GetChnConvCompletedFlag(ADC_Type * base, uint32_t chnGroup)
{
    return (1U == ADC_BRD_SC1_COCO(base, chnGroup) );
}

void ADC16_DRV_WaitConvDone(uint32_t instance, uint32_t chnGroup)
{
    ADC_Type * base = g_adcBase[instance];

    while ( !ADC16_HAL_GetChnConvCompletedFlag(base, chnGroup) )
        {}
}

static inline uint16_t ADC16_HAL_GetChnConvValue(ADC_Type * base, uint32_t chnGroup )
{
    return (uint16_t)(ADC_BRD_R_D(base, chnGroup) );
}

uint16_t ADC16_DRV_GetConvValueRAW(uint32_t instance, uint32_t chnGroup)
{

    ADC_Type * base = g_adcBase[instance];

    return ADC16_HAL_GetChnConvValue(base, chnGroup);
}

int16_t ADC16_DRV_GetConvValueSigned(uint32_t instance, uint32_t chnGroup)
{
    return (int16_t)ADC16_DRV_GetConvValueRAW(instance, chnGroup);
}

void ADC16_DRV_PauseConv(uint32_t instance, uint32_t chnGroup)
{
    adc16_chn_config_t configStruct;

    configStruct.chnIdx = kAdc16Chn31;
    configStruct.convCompletedIntEnable = false;
#if FSL_FEATURE_ADC16_HAS_DIFF_MODE
    configStruct.diffConvEnable = false;
#endif
    ADC16_DRV_ConfigConvChn(instance, chnGroup, &configStruct);
}

typedef enum _adc16_clk_divider {
    kAdc16ClkDividerOf1 = 0U, /*!< For divider 1 from the input clock to ADC16. @internal gui name="1" */
    kAdc16ClkDividerOf2 = 1U, /*!< For divider 2 from the input clock to ADC16. @internal gui name="2" */
    kAdc16ClkDividerOf4 = 2U, /*!< For divider 4 from the input clock to ADC16. @internal gui name="4" */
    kAdc16ClkDividerOf8 = 3U  /*!< For divider 8 from the input clock to ADC16. @internal gui name="8" */
} adc16_clk_divider_t;

typedef enum _adc16_resolution {
    kAdc16ResolutionBitOf8or9 = 0U,
    /*!< 8-bit for single end sample, or 9-bit for differential sample. @internal gui name="" */
    kAdc16ResolutionBitOfSingleEndAs8 = kAdc16ResolutionBitOf8or9, /*!< 8-bit for single end sample. @internal gui name="8 bit in single mode" */
    kAdc16ResolutionBitOfDiffModeAs9 = kAdc16ResolutionBitOf8or9, /*!< 9-bit for differential sample. @internal gui name="9 bit in differential mode" */

    kAdc16ResolutionBitOf12or13 = 1U,
    /*!< 12-bit for single end sample, or 13-bit for differential sample. @internal gui name="" */
    kAdc16ResolutionBitOfSingleEndAs12 = kAdc16ResolutionBitOf12or13, /*!< 12-bit for single end sample. @internal gui name="12 bit in single mode" */
    kAdc16ResolutionBitOfDiffModeAs13 = kAdc16ResolutionBitOf12or13, /*!< 13-bit for differential sample. @internal gui name="13 bit in differential mode" */

    kAdc16ResolutionBitOf10or11 = 2U,
    /*!< 10-bit for single end sample, or 11-bit for differential sample. @internal gui name="" */
    kAdc16ResolutionBitOfSingleEndAs10 = kAdc16ResolutionBitOf10or11, /*!< 10-bit for single end sample. @internal gui name="10 bit in single mode" */
    kAdc16ResolutionBitOfDiffModeAs11 = kAdc16ResolutionBitOf10or11 /*!< 11-bit for differential sample. @internal gui name="11 bit in differential mode" */
#if (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    , kAdc16ResolutionBitOf16 = 3U,
    /*!< 16-bit for both single end sample and differential sample. @internal gui name="16-bit" */
    kAdc16ResolutionBitOfSingleEndAs16 = kAdc16ResolutionBitOf16, /*!< 16-bit for single end sample. @internal gui name="" */
    kAdc16ResolutionBitOfDiffModeAs16 = kAdc16ResolutionBitOf16 /*!< 16-bit for differential sample. @internal gui name="" */

#endif  /** FSL_FEATURE_ADC16_MAX_RESOLUTION */
} adc16_resolution_t;

typedef enum _adc16_long_sample_cycle {
    kAdc16LongSampleCycleOf24 = 0U, /*!< 20 extra ADCK cycles, 24 ADCK cycles total. */
    kAdc16LongSampleCycleOf16 = 1U, /*!< 12 extra ADCK cycles, 16 ADCK cycles total. */
    kAdc16LongSampleCycleOf10 = 2U, /*!< 6 extra ADCK cycles, 10 ADCK cycles total. */
    kAdc16LongSampleCycleOf4  = 3U  /*!< 2 extra ADCK cycles, 6 ADCK cycles total. */
} adc16_long_sample_cycle_t;

typedef enum _adc16_clk_src_mode {
    kAdc16ClkSrcOfBusClk  = 0U, /*!< For input as bus clock. @internal gui name="Bus clock" */
    kAdc16ClkSrcOfAltClk2 = 1U, /*!< For input as alternate clock 2 (AltClk2). @internal gui name="Alternate clock 2" */
    kAdc16ClkSrcOfAltClk  = 2U, /*!< For input as alternate clock (ALTCLK). @internal gui name="Alternate clock 1" */
    kAdc16ClkSrcOfAsynClk = 3U  /*!< For input as asynchronous clock (ADACK). @internal gui name="Asynchronous clock" */
} adc16_clk_src_mode_t;

typedef enum _adc16_ref_volt_src {
    kAdc16RefVoltSrcOfVref = 0U, /*!< For external pins pair of VrefH and VrefL. @internal gui name="Vref pair" */
    kAdc16RefVoltSrcOfValt = 1U  /*!< For alternate reference pair of ValtH and ValtL. @internal gui name="Valt pair" */
} adc16_ref_volt_src_t;

typedef struct Adc16ConverterConfig {
    bool                    lowPowerEnable; /*!< Enable low power. @internal gui name="Low power mode" id="LowPowerMode" */
    adc16_clk_divider_t     clkDividerMode; /*!< Select the divider of input clock source. @internal gui name="Clock divider" id="ClockDivider" */
    bool                    longSampleTimeEnable; /*!< Enable the long sample time. @internal gui name="Long sample time" id="LongSampleTime" */
    adc16_resolution_t      resolution; /*!< Select the sample resolution mode. @internal gui name="Resolution" id="Resolution" */
    adc16_clk_src_mode_t    clkSrc; /*!< Select the input clock source to converter. @internal gui name="Clock source" id="ClockSource" */
    bool                    asyncClkEnable; /*!< Enable the asynchronous clock inside the ADC. @internal gui name="Internal async. clock" id="InternalAsyncClock" */
    bool                    highSpeedEnable; /*!< Enable the high speed mode. @internal gui name="High speed mode" id="HighSpeed" */
    adc16_long_sample_cycle_t longSampleCycleMode; /*!< Select the long sample mode. @internal gui name="Long sample mode" id="LongSampleMode" */
    bool                    hwTriggerEnable; /*!< Enable hardware trigger function. @internal gui name="Hardware trigger" id="HwTrigger" */
    adc16_ref_volt_src_t    refVoltSrc; /*!< Select the reference voltage source. @internal gui name="Voltage reference" id="ReferenceVoltage" */
    bool                    continuousConvEnable; /*!< Enable continuous conversion mode. @internal gui name="Continuous mode" id="ContinuousMode" */
#if FSL_FEATURE_ADC16_HAS_DMA
    bool                    dmaEnable; /*!< Enable the DMA for ADC converter. @internal gui name="DMA mode" id="DMASupport" */
#endif  /** FSL_FEATURE_ADC16_HAS_DMA */
} adc16_converter_config_t;

const adc16_converter_config_t BATTERY_ADC_InitConfig = {
    .lowPowerEnable = false,
    .clkDividerMode = kAdc16ClkDividerOf1,
    .longSampleTimeEnable = false,
    .resolution = kAdc16ResolutionBitOf16,
    .clkSrc = kAdc16ClkSrcOfBusClk,
    .asyncClkEnable = false,
    .highSpeedEnable = true,
    .longSampleCycleMode = kAdc16LongSampleCycleOf4,
    .hwTriggerEnable = false,
    .refVoltSrc = kAdc16RefVoltSrcOfVref,
    .continuousConvEnable = false,
    .dmaEnable = false,
};

#define FSL_SIM_SCGC_BIT(SCGCx, n) (((SCGCx-1U)<<5U) + n)

typedef enum _sim_clock_gate_name {
    kSimClockGateI2c2      = FSL_SIM_SCGC_BIT(1U, 6U),
    kSimClockGateUart4     = FSL_SIM_SCGC_BIT(1U, 10U),
    kSimClockGateUart5     = FSL_SIM_SCGC_BIT(1U, 11U),
    kSimClockGateEnet0     = FSL_SIM_SCGC_BIT(2U, 0U),
    kSimClockGateDac0      = FSL_SIM_SCGC_BIT(2U, 12U),
    kSimClockGateDac1      = FSL_SIM_SCGC_BIT(2U, 13U),
    kSimClockGateSpi2      = FSL_SIM_SCGC_BIT(3U, 12U),
    kSimClockGateSdhc0     = FSL_SIM_SCGC_BIT(3U, 17U),
    kSimClockGateFtm3      = FSL_SIM_SCGC_BIT(3U, 25U),
    kSimClockGateAdc1      = FSL_SIM_SCGC_BIT(3U, 27U),
    kSimClockGateEwm0      = FSL_SIM_SCGC_BIT(4U, 1U),
    kSimClockGateCmt0      = FSL_SIM_SCGC_BIT(4U, 2U),
    kSimClockGateI2c0      = FSL_SIM_SCGC_BIT(4U, 6U),
    kSimClockGateI2c1      = FSL_SIM_SCGC_BIT(4U, 7U),
    kSimClockGateUart0     = FSL_SIM_SCGC_BIT(4U, 10U),
    kSimClockGateUart1     = FSL_SIM_SCGC_BIT(4U, 11U),
    kSimClockGateUart2     = FSL_SIM_SCGC_BIT(4U, 12U),
    kSimClockGateUart3     = FSL_SIM_SCGC_BIT(4U, 13U),
    kSimClockGateUsbfs0    = FSL_SIM_SCGC_BIT(4U, 18U),
    kSimClockGateCmp       = FSL_SIM_SCGC_BIT(4U, 19U),
    kSimClockGateVref0     = FSL_SIM_SCGC_BIT(4U, 20U),
    kSimClockGateLptmr0    = FSL_SIM_SCGC_BIT(5U, 0U),
    kSimClockGatePortA     = FSL_SIM_SCGC_BIT(5U, 9U),
    kSimClockGatePortB     = FSL_SIM_SCGC_BIT(5U, 10U),
    kSimClockGatePortC     = FSL_SIM_SCGC_BIT(5U, 11U),
    kSimClockGatePortD     = FSL_SIM_SCGC_BIT(5U, 12U),
    kSimClockGatePortE     = FSL_SIM_SCGC_BIT(5U, 13U),
    kSimClockGateFtf0      = FSL_SIM_SCGC_BIT(6U, 0U),
    kSimClockGateDmamux0   = FSL_SIM_SCGC_BIT(6U, 1U),
    kSimClockGateFlexcan0  = FSL_SIM_SCGC_BIT(6U, 4U),
    kSimClockGateRnga0     = FSL_SIM_SCGC_BIT(6U, 9U),
    kSimClockGateSpi0      = FSL_SIM_SCGC_BIT(6U, 12U),
    kSimClockGateSpi1      = FSL_SIM_SCGC_BIT(6U, 13U),
    kSimClockGateSai0      = FSL_SIM_SCGC_BIT(6U, 15U),
    kSimClockGateCrc0      = FSL_SIM_SCGC_BIT(6U, 18U),
    kSimClockGateUsbdcd0   = FSL_SIM_SCGC_BIT(6U, 21U),
    kSimClockGatePdb0      = FSL_SIM_SCGC_BIT(6U, 22U),
    kSimClockGatePit0      = FSL_SIM_SCGC_BIT(6U, 23U),
    kSimClockGateFtm0      = FSL_SIM_SCGC_BIT(6U, 24U),
    kSimClockGateFtm1      = FSL_SIM_SCGC_BIT(6U, 25U),
    kSimClockGateFtm2      = FSL_SIM_SCGC_BIT(6U, 26U),
    kSimClockGateAdc0      = FSL_SIM_SCGC_BIT(6U, 27U),
    kSimClockGateRtc0      = FSL_SIM_SCGC_BIT(6U, 29U),
    kSimClockGateFlexbus0  = FSL_SIM_SCGC_BIT(7U, 0U),
    kSimClockGateDma0      = FSL_SIM_SCGC_BIT(7U, 1U),
    kSimClockGateMpu0      = FSL_SIM_SCGC_BIT(7U, 2U),
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_clock_gate_name_k64f12_t;
#else
} sim_clock_gate_name_t;
#endif

static const sim_clock_gate_name_t adcGateTable[] = {
    kSimClockGateAdc0,
    kSimClockGateAdc1
};

static inline void SIM_HAL_EnableClock(SIM_Type * base, sim_clock_gate_name_t name)
{
    SIM_BWR_SCGC_BIT(base, name, 1U);
}

void CLOCK_SYS_EnableAdcClock(uint32_t instance)
{
    SIM_HAL_EnableClock(SIM, adcGateTable[instance]);
}

void ADC16_HAL_Init(ADC_Type * base)
{
    ADC_WR_CFG1(base, 0U);
    ADC_WR_CFG2(base, 0U);
    ADC_WR_CV1(base, 0U);
    ADC_WR_CV2(base, 0U);
    ADC_WR_SC2(base, 0U);
    ADC_WR_SC3(base, 0U);
#if FSL_FEATURE_ADC16_HAS_PGA
    ADC_WR_PGA(base, 0U);
#endif  /** FSL_FEATURE_ADC16_HAS_PGA */
}

void ADC16_HAL_ConfigConverter(ADC_Type * base, const adc16_converter_config_t *configPtr)
{
    uint16_t cfg1, cfg2, sc2, sc3;

    cfg1 = ADC_RD_CFG1(base);
    cfg1 &= ~(  ADC_CFG1_ADLPC_MASK
                | ADC_CFG1_ADIV_MASK
                | ADC_CFG1_ADLSMP_MASK
                | ADC_CFG1_MODE_MASK
                | ADC_CFG1_ADICLK_MASK );

    /** Low power mode. */
    if (configPtr->lowPowerEnable) {
        cfg1 |= ADC_CFG1_ADLPC_MASK;
    }
    /** Clock divider. */
    cfg1 |= ADC_CFG1_ADIV(configPtr->clkDividerMode);
    /** Long sample time. */
    if (configPtr->longSampleTimeEnable) {
        cfg1 |= ADC_CFG1_ADLSMP_MASK;
    }
    /** Sample resolution mode. */
    cfg1 |= ADC_CFG1_MODE(configPtr->resolution);
    /** Clock source input. */
    cfg1 |= ADC_CFG1_ADICLK(configPtr->clkSrc);

    cfg2 = ADC_RD_CFG2(base);
    cfg2 &= ~( ADC_CFG2_ADACKEN_MASK
               | ADC_CFG2_ADHSC_MASK
               | ADC_CFG2_ADLSTS_MASK );
    /** Asynchronous clock output enable. */
    if (configPtr->asyncClkEnable) {
        cfg2 |= ADC_CFG2_ADACKEN_MASK;
    }
    /** High speed configuration. */
    if (configPtr->highSpeedEnable) {
        cfg2 |= ADC_CFG2_ADHSC_MASK;
    }
    /** Long sample time select. */
    cfg2 |= ADC_CFG2_ADLSTS(configPtr->longSampleCycleMode);

    sc2 = ADC_RD_SC2(base);
    sc2 &= ~( ADC_SC2_ADTRG_MASK
              | ADC_SC2_REFSEL_MASK
#if FSL_FEATURE_ADC16_HAS_DMA
              | ADC_SC2_DMAEN_MASK
#endif  /** FSL_FEATURE_ADC16_HAS_DMA */
            );
    /** Conversion trigger select. */
    if (configPtr->hwTriggerEnable) {
        sc2 |= ADC_SC2_ADTRG_MASK;
    }
    /** Voltage reference selection. */
    sc2 |= ADC_SC2_REFSEL(configPtr->refVoltSrc);
#if FSL_FEATURE_ADC16_HAS_DMA
    /** DMA. */
    if (configPtr->dmaEnable) {
        sc2 |= ADC_SC2_DMAEN_MASK;
    }
#endif  /** FSL_FEATURE_ADC16_HAS_DMA */

    sc3 = ADC_RD_SC3(base);
    sc3 &= ~( ADC_SC3_ADCO_MASK
              | ADC_SC3_CALF_MASK );
    /** Continuous conversion enable. */
    if (configPtr->continuousConvEnable) {
        sc3 |= ADC_SC3_ADCO_MASK;
    }

    ADC_WR_CFG1(base, cfg1);
    ADC_WR_CFG2(base, cfg2);
    ADC_WR_SC2(base, sc2);
    ADC_WR_SC3(base, sc3);
}

static inline void INT_SYS_EnableIRQ(IRQn_Type irqNumber)
{
    /** call core API to enable the IRQ*/
    NVIC_EnableIRQ(irqNumber);
}

adc16_status_t ADC16_DRV_Init(uint32_t instance, const adc16_converter_config_t *userConfigPtr)
{
    ADC_Type * base = g_adcBase[instance];

    if (!userConfigPtr) {
        return kStatus_ADC16_InvalidArgument;
    }
    /** Enable clock for ADC. */
    CLOCK_SYS_EnableAdcClock(instance);

    /** Reset all the register to a known state. */
    ADC16_HAL_Init(base);
    ADC16_HAL_ConfigConverter(base, userConfigPtr);

    /** Enable ADC interrupt in NVIC level.*/
    INT_SYS_EnableIRQ(g_adcIrqId[instance] );

    return kStatus_ADC16_Success;
}

static uint8_t bat_convert_data(uint16_t input)
{
    uint8_t output = 0;

    uint16_t bat_mvolts = (uint16_t)( ( (float)input * ( 3.3 / 65535.0 ) ) * 1000 );

    if ( bat_mvolts > 2670 ) {
        output = 100;
    }

    else if ( bat_mvolts > 2500 ) {
        output = (uint8_t)( 50 + 50.0 * ( ( bat_mvolts - 2500 ) / 170.0 ) );
    } else if ( bat_mvolts > 2430 ) {
        output = (uint8_t)( 30 + 20.0 * ( ( bat_mvolts - 2430 ) / 70.0 ) );
    } else if ( bat_mvolts > 2370 ) {
        output = (uint8_t)( 10 + 20.0 * ( ( bat_mvolts - 2370 ) / 60.0 ) );
    } else {
        output = 0;
    }
    return output;

}

HexiwearBattery::HexiwearBattery()
{
    batCharging = new DigitalIn(PTC12);
    batSensSwitch = new DigitalOut(PTC14);
    ADC16_DRV_Init(0, &BATTERY_ADC_InitConfig);
    ADC16_DRV_ConfigConvChn(0, 0U, &BATTERY_ADC_ChnConfig);
}

HexiwearBattery::~HexiwearBattery()
{
    delete batSensSwitch;
    delete batCharging;
}

void HexiwearBattery::sensorOn()
{
    *batSensSwitch = 0;
};


void HexiwearBattery::sensorOff()
{
    *batSensSwitch = 1;
};

bool HexiwearBattery::isBatteryCharging()
{
    return *batCharging == 0;
}

uint8_t HexiwearBattery::readLevelPercent()
{
    ADC16_DRV_ConfigConvChn( 0, 0, &BATTERY_ADC_ChnConfig);
    ADC16_DRV_WaitConvDone ( 0, 0 );
    int16_t result = ADC16_DRV_GetConvValueSigned( 0, 0 );
    ADC16_DRV_PauseConv(0, 0 );
    return bat_convert_data(result);
}

