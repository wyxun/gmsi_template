/*******************************************************************************
 * @file    foc_controller.h
 * @brief   Runtime-selectable scalar controller interface
 ******************************************************************************/

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include <stdbool.h>

#include "foc_pid.h"

typedef struct {
    void *pContext;                                     /**< 控制器上下文 */
    void (*fnReset)(void *pContext);                    /**< 复位函数 */
    foc_scalar_t (*fnStep)(void *pContext,
                           foc_scalar_t qReference,
                           foc_scalar_t qFeedback);     /**< 步进函数 */
    void (*fnTrack)(void *pContext,
                    foc_scalar_t qOutput,
                    foc_scalar_t qReference,
                    foc_scalar_t qFeedback);            /**< 跟踪函数 */
} foc_controller_if_t;

/**
 * @brief  用 PID 实例构造控制器接口
 * @param  ptPid  PID 实例指针
 * @return        控制器接口（fnReset/fnStep/fnTrack）
 */
foc_controller_if_t foc_controller_FromPid(foc_pid_t *ptPid);
/**
 * @brief  检查控制器接口是否有效（所有函数指针非空）
 * @param  ptController  控制器接口指针
 * @return               true=有效, false=无效
 */
bool foc_controller_IsValid(const foc_controller_if_t *ptController);
/**
 * @brief  检查控制器是否支持 Track 操作
 * @param  ptController  控制器接口指针
 * @return               true=支持 Track, false=不支持
 */
bool foc_controller_CanTrack(const foc_controller_if_t *ptController);
/**
 * @brief  复位控制器状态
 * @param  ptController  控制器接口指针
 */
void foc_controller_Reset(const foc_controller_if_t *ptController);
/**
 * @brief  无条件跟踪：用给定输出、参考和反馈更新控制器积分器状态
 * @param  ptController  控制器接口指针
 * @param  qOutput       当前输出
 * @param  qReference    参考值
 * @param  qFeedback     反馈值
 */
void foc_controller_Track(const foc_controller_if_t *ptController,
                          foc_scalar_t qOutput,
                          foc_scalar_t qReference,
                          foc_scalar_t qFeedback);
/**
 * @brief  执行一步控制器运算
 * @param  ptController  控制器接口指针
 * @param  qReference    参考值
 * @param  qFeedback     反馈值
 * @return               控制输出
 */
foc_scalar_t foc_controller_Step(const foc_controller_if_t *ptController,
                                 foc_scalar_t qReference,
                                 foc_scalar_t qFeedback);

#endif /* FOC_CONTROLLER_H */
