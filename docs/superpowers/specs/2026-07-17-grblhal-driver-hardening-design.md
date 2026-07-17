# grblHAL AT32F407 Driver Hardening Design

## Goal

Harden the AT32F407 grblHAL adaptation so step generation, USB CDC and UART
streaming, cancellation, configuration changes, spindle enable, and fatal
shutdown obey the grblHAL contracts without advertising unavailable hardware.

## Scope Decisions

- Fix review items 1-3, 6, 8-18, 23-24, and 27.
- Keep limit and control capabilities disabled until final physical inputs are
  selected. Limit capability and callbacks must remain internally consistent.
- Keep coolant and probe behavior unchanged for now.
- Record true stream suspension, blocking delays, and connection detection as
  known follow-up work instead of claiming support.
- Implement a unidirectional DC spindle: PA6 enables the motor for M3 and
  disables it for M5. PA5 remains at its safe inactive level. M4/reverse and
  PWM speed control are not advertised.
- AT32F407 currently selects USB CDC as its primary grblHAL stream. UART is
  hardened as a supported alternative because it shares the same stream HAL.

## Architecture

### Stepper timing

TMR5 remains the main step interrupt source and is configured for its documented
32-bit extension. Its clock remains 120 MHz. DWT supplies short pulse and
direction setup delays, but conversions use `SystemCoreClock`, because CYCCNT
runs at the 240 MHz CPU clock.

The driver caches the configured step pulse width and direction setup delay in
CPU cycles. `settings_changed()` refreshes these values and the supported
inversion masks. `pulse_start()` applies direction outputs, waits for direction
setup only when direction changes, asserts the configured step signals, waits
the configured pulse width, and clears them. `go_idle(clear_signals)` clears
STEP/DIR when requested. The board has a shared active-low enable, so any
enabled axis enables the shared output; zero enabled axes disables it. Per-axis
disable and motor-hold behavior are not advertised.

### UART DMA streaming

The DMA circular staging buffer and the software RX ring buffer use separate
arrays. RX extraction has a single critical section protecting the DMA cursor
and ring writes from main/USART/DMA re-entry. Half-transfer, full-transfer, and
IDLE processing preserve bounded latency. Overflow is counted rather than
silently corrupting memory.

TX writes retry until all bytes are queued. While waiting, they call
`hal.stream_blocking_callback()` so realtime protocol work continues. An abort
from that callback terminates the write rather than deadlocking. RX count/free
reflect the software queue after polling.

### USB CDC streaming

The USB RX queue remains distinct from the vendor endpoint buffer. Each poll
drains complete endpoint packets, handles realtime characters before normal
queuing, and counts overflow. RX count and free-space report the local queue.

USB TX sends endpoint-sized chunks and retries endpoint-busy responses through
`hal.stream_blocking_callback()`. It never reports success for unsent bytes.
Retry is bounded by disconnect/abort state rather than an arbitrary spin count.
The stream connection flag uses the USB core configured/connected state when
the vendor stack exposes a reliable state; otherwise the current always-on
behavior is documented as the item-26 follow-up.

`cancel_read_buffer()` flushes pending normal input and enqueues ASCII CAN
through the active realtime handler. `suspend_read` is left NULL until a real
save/restore implementation exists; this is the archived item-7 limitation.

### Driver lifecycle and capabilities

`driver_init()` rejects a mismatched `HAL_VERSION`. `driver_setup()` is
idempotent: timers, DWT, plugin hooks, and MODUS hooks are installed once.
Repeated setup updates settings without chaining callbacks onto themselves.

Limits remain capability zero and do not read provisional pins. Control,
coolant, and probe remain unadvertised or unchanged according to current board
scope. Unsupported step pulse delay, axis enable, direction, and spindle
features are not claimed.

### Spindle and fatal shutdown

The spindle HAL registers one on/off spindle instance. Its set-state handler
maps spindle-on to PA6 active and spindle-off to PA6 inactive, reports the same
cached state, and does not advertise PWM or direction. Initialization and every
fatal path force PA6 off and PA5 to the safe default.

A shared emergency shutdown helper disables the TMR5 interrupt/counter,
disables the shared stepper output, clears all STEP pins, and disables the
spindle. `_exit()` and `__assert_func()` invoke it before entering their terminal
loop. Existing Cortex fault handlers should invoke the same helper where the
linkage permits without replacing their diagnostic dump.

## Error Handling and Observability

- UART and USB RX overflow counters retain evidence of dropped input.
- Stream TX aborts only when the grblHAL blocking callback requests abort or
  transport state proves disconnected.
- HAL version mismatch makes driver initialization fail explicitly.
- Fatal shutdown places motion outputs in safe states before halting.
- Item 7 (true stream suspension), item 25 (blocking delay design), and item 26
  (transport connection semantics) are recorded in the driver adaptation notes.

## Testing

Host-side contract tests isolate pure calculations and queue behavior from AT32
register access. They cover:

- distinct DMA staging and RX queue storage;
- non-reentrant RX extraction and overflow accounting;
- RX count/free and cancel-to-CAN behavior for UART and USB;
- complete TX with endpoint/queue busy retries and callback abort;
- 32-bit TMR5 periods;
- CPU-clock pulse and direction-delay conversions;
- settings refresh and signal inversion;
- idempotent setup and HAL version rejection;
- shared stepper enable and idle clearing;
- M3/M5 spindle state without M4/PWM capability;
- emergency shutdown outputs.

The final verification is a clean AT32F407 debug build, followed by size output.
Hardware verification should then stream long G-code over USB CDC while status
reports are active and observe STEP/DIR/PA6 timing with a logic analyzer.

