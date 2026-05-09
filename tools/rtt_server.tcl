adapter speed 2000
tcl_port 0
init
reset run
sleep 500
stm32g4x.cpu rtt setup 0x20000010 0xa8 "SEGGER RTT"
stm32g4x.cpu rtt start
rtt server start 9090 0
rtt server start 9091 1
