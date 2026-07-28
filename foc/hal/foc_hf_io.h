/*******************************************************************************
 * @file    foc_hf_io.h
 * @brief   FOC 高频 Fast Path 移植协议（版本化接口）
 ******************************************************************************/

#ifndef __FOC_HF_IO_H__
#define __FOC_HF_IO_H__

#include <stdint.h>
#include "foc_hal_types.h"
#include "foc_modulation.h"

#define FOC_HF_IO_ABI_VERSION 1U

typedef struct {
    uint16_t wAbiVersion;   /**< ABI 版本，须等于 FOC_HF_IO_ABI_VERSION */
    void *pContext;         /**< 平台上下文 */
    foc_result_t (*fnSampleCurrent)(void *pContext,
                                    phase_current_handle_t *ptCurrent);
                                /**< 高频电流采样 */
    foc_result_t (*fnCommitDuty)(void *pContext,
                                 const foc_duty_abc_t *ptDuty);
                                /**< 提交三相占空比 */
    void (*fnEmergencyStop)(void *pContext);    /**< 紧急停止 */
} foc_hf_io_if_t;

#endif /* __FOC_HF_IO_H__ */
