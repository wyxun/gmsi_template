# AT32F413 Motor EVB V1 Port Design

## Summary

Port the AT32F413 Motor EVB V1 (Artery) to the MODUS template as a new FOC-capable
target. Reference the official demo's hardware init (AT32F413_MC_Library_Project_V2.1.4,
`AT_MOTOR_EVB_V1` pin mapping from `mc_hwio_v1.h`) but use the project's own FOC
framework. Add the AT32F413 firmware library as a git submodule from Gitee.

## Target Hardware

- **Chip**: AT32F413RCT7 (Cortex-M4F, 256KB Flash, 64KB RAM, LQFP64)
- **Board**: Artery Motor EVB V1 (`AT_MOTOR_EVB_V1`)
- **FPU**: hard (`-mfloat-abi=hard -mfpu=fpv4-sp-d16`)
- **Clock**: HICK 48MHz, PLL ×50 → 200MHz SYSCLK, AHB=200M, APB1/2=100M

## Git Submodule

```
path = vendor/cortex-m/AT32F413_Firmware_Library
url = https://gitee.com/arterytek/AT32F413_Firmware_Library.git
ignore = untracked
```

Same pattern as the existing `vendor/cortex-m/AT32F421_Firmware_Library`.

## Files to Create

### target/at32f413/

| File | Purpose |
|---|---|
| `target.mk` | CPU flags (FPU on), driver selection (TMR+ADC on), FOC enabled, linker/startup paths |
| `AT32F413xC_FLASH.ld` | Linker script (256KB/64KB), derived from vendor library template |
| `at32f413_it.c` | ISR handlers: SysTick (perfc+modus+timer), USART1, TMR1 OVF+BRK, ADC1_2, fault stubs |
| `at32f413_conf.h` | AT32F413 peripheral config (standard-periph-library style) |
| `perfc_port_user.c/h` | perf_counter timer port (SysTick 1ms, same as AT32F421) |
| `openocd.cfg` | OpenOCD config for AT32F413 |

### peripheral/at32f413/

| File | Purpose |
|---|---|
| `mdi_hw.h` | Hardware resource pool (LEDs, PWM×3, ADC×5, comparator, stream) |
| `port_mdi.h` | Internal types (USART priv, PWM priv) |
| `port_mdi.c` | MDI adapter instances + HW global, USART ringbuf logic (reused from AT32F421) |
| `port_sys.c` | `peripheral_Init()` entry: clock → LED → PWM → ADC → USART → SysTick |
| `halpwm.c/h` | TMR1 3-phase complementary PWM, dead-time, brake, CH4 ADC trigger |
| `haladc.c/h` | ADC1 ordinary (DMA, 4 channels) + preempt (3-shunt current + voltage monitor) |
| `halusart.c/h` | USART1 PB6/7 remap, 115200, RDBF+TDC interrupt, ringbuf backend |
| `halled.c/h` | Status LEDs init (PB9 error, PC13/14/15 status) |
| `foc_hal_mdi_adapter.c` | FOC HAL ↔ MDI bridge (pwm ops + adc ops + register) |

## Pin Mapping (AT_MOTOR_EVB_V1)

### PWM — TMR1
| Signal | Pin | TMR1 Channel |
|---|---|---|
| U high-side | PA8 | CH1 |
| V high-side | PA9 | CH2 |
| W high-side | PA10 | CH3 |
| U low-side | PB13 | CH1N |
| V low-side | PB14 | CH2N |
| W low-side | PB15 | CH3N |
| Brake input | PB12 | BKIN |
| ADC trigger out | PA11 | CH4 |

### ADC — ADC1
| Signal | Pin | Channel | Type |
|---|---|---|---|
| Phase A current | PA0 | CH0 | Preempt |
| Phase B current | PA1 | CH1 | Preempt |
| Phase C current | PA2 | CH2 | Preempt |
| Bus current | PA3 | CH3 | Preempt |
| Bus voltage | PA7 | CH7 | Ordinary (DMA) |
| MOS temperature | PB1 | CH9 | Ordinary (DMA) |
| Potentiometer | PC0 | CH10 | Ordinary (DMA) |
| IBUS average | PA3 | CH3 | Ordinary (DMA) |

### Communication — USART1
| Signal | Pin | Notes |
|---|---|---|
| TX | PB6 | Remap USART1_GMUX_0001 |
| RX | PB7 | Remap USART1_GMUX_0001 |

### Status LEDs
| LED | Pin | Active |
|---|---|---|
| Error | PB9 | Low |
| Status1 | PC13 | Low |
| Status2 | PC14 | Low |
| Status3 | PC15 | Low |

## peripheral_Init() Flow

```
nvic_priority_group_config(NVIC_PRIORITY_GROUP_4)
SystemClock_Config()        // HICK→PLL×50→200MHz
halled_Init()               // PB9, PC13/14/15 LEDs off
halpwm_Init()               // TMR1 complementary PWM, dead-time, CH4 ADC trigger
haladc_Init()               // ADC1 ordinary(DMA) + preempt(3-shunt), voltage monitor
halusart_Init()             // USART1 PB6/7, 115200, RDBF+TDC interrupt
SysTick_Config(200000)      // 1ms tick from 200MHz
```

## FOC Integration

The `foc_hal_mdi_adapter.c` follows the STM32G431 pattern:
- `foc_pwm_ops_t` → MDI PWM objects (HW.ptMotorU/V/W) via `tmr_channel_value_set`
- `foc_adc_ops_t` → MDI ADC objects, `adc_preempt_conversion_data_get` for raw samples
- Shared `foc_hal_current_reconstruct()` for math (chip-agnostic)
- 3-shunt topology (matches `FOC_DEFAULT_SENSING_TOPOLOGY` in `foc_config.h`)
- ADC preempt triggered by TMR1 CH4 hardware (no software start needed in fnStartConversion)

## ISR Wiring (at32f413_it.c)

| IRQ | Handler | Action |
|---|---|---|
| SysTick | `SysTick_Handler` | perfc tick → modus_Clock → usart timer 1ms |
| TMR1_OVF_TMR10 | `TMR1_OVF_TMR10_IRQHandler` | Clear OVF flag → FOC control loop (future) |
| TMR1_BRK_TMR9 | `TMR1_BRK_TMR9_IRQHandler` | Clear BRK flag → emergency stop |
| ADC1_2 | `ADC1_2_IRQHandler` | Clear preempt end flag → current reconstruction |
| USART1 | `USART1_IRQHandler` | `at32_usart_irq_handler` |

## target.mk Key Settings

- `CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16`
- `C_DEFS += -DAT32F413RCT7 -DUSE_STDPERIPH_DRIVER -DFOC_SUPPORT=1 -DAT_MOTOR_EVB_V1`
- `FOC_SOURCES` enabled (same as STM32G431)
- Driver selection: CRM, GPIO, MISC, FLASH, USART, TMR, ADC, DMA, DEBUG on; rest off

## Not in Scope (Future)

- Hall sensor support (TMR3, PB4/PB5/PB0)
- Encoder support (TMR5, PF4/PF5, TMR3 capture)
- Brake PWM (TMR10, PB8)
- Button/EXINT (PA12 user button, mode switches)
- FOC angle init / motor parameter ID (runtime features, not port bring-up)
- Internal VREF calibration ratio
