<#
.SYNOPSIS
  MODUS MCU debug helper — sends commands to OpenOCD via telnet (port 4444).

.DESCRIPTION
  Wraps common OpenOCD debug operations for interactive or AI-driven debugging.
  All commands connect to localhost:4444 (OpenOCD telnet port).

  Targets: STM32G431 and AT32F421 (both use the same OpenOCD telnet interface).

.PARAMETER Action
  The action to perform. Available actions:

    halt       — Halt (pause) the CPU. Use before reading registers/variables.
    resume     — Resume CPU execution.
    reset      — Reset the MCU and halt at the reset vector.
    regs       — Dump all core registers (PC, SP, LR, R0-R12, xPSR, etc.).
    reg <name> — Read a single register by name (e.g. "sp", "pc", "lr").
    peek <addr>        — Read uint32 at hex address (e.g. "0x20000000").
    mdw <addr> [count] — Memory dump words, default 16.
    mdh <addr> [count] — Memory dump half-words.
    mdb <addr> [count] — Memory dump bytes.
    stack [count]      — Dump the stack area around current SP.
    disasm <addr> [count] — Disassemble instructions at address.
    var <elf> <name>   — Read a global/static variable value by name.
                          Requires llvm-nm in PATH to look up the symbol address,
                          then reads it via peek.

.PARAMETER Extra
  Additional arguments depending on the action (address, count, etc.).

.EXAMPLE
  .\dev_debug.ps1 halt
  .\dev_debug.ps1 regs
  .\dev_debug.ps1 peek 0x20000000
  .\dev_debug.ps1 mdw 0x20000000 32
  .\dev_debug.ps1 stack 24
  .\dev_debug.ps1 resume
  .\dev_debug.ps1 var build/template.elf g_wTickCounter

.NOTES
  Requires OpenOCD running with telnet on port 4444 (started by make.bat rtt).
  For the 'stack' and 'var' actions, llvm-nm must be in PATH or LLVM_PATH env var.
#>

param(
    [Parameter(Position=0, Mandatory=$true)]
    [string]$Action,

    [Parameter(Position=1, ValueFromRemainingArguments=$true)]
    [string[]]$Extra
)

$OCD_HOST = "localhost"
$OCD_PORT = 4444
$TIMEOUT_MS = 5000
$LLVM_NM = "llvm-nm.exe"

# Try to locate llvm-nm
if ($env:LLVM_PATH) {
    $LLVM_NM = Join-Path $env:LLVM_PATH "llvm-nm.exe"
}

function Send-Ocd {
    param([string]$Cmd)
    try {
        $client = New-Object System.Net.Sockets.TcpClient
        if (-not $client.ConnectAsync($OCD_HOST, $OCD_PORT).Wait($TIMEOUT_MS)) {
            Write-Error "Connection to OpenOCD timed out. Is OpenOCD running? (try: make.bat rtt)"
            return ""
        }
        $stream = $client.GetStream()
        $writer = New-Object System.IO.StreamWriter($stream)
        $reader = New-Object System.IO.StreamReader($stream)
        
        # Flush the initial OpenOCD greeting message
        Start-Sleep -Milliseconds 200
        while ($stream.DataAvailable) {
            [void]$reader.ReadLine()
        }
        
        $writer.WriteLine($Cmd)
        $writer.Flush()
        
        # Wait for data to arrive (up to 500ms)
        $attempts = 0
        while (-not $stream.DataAvailable -and $attempts -lt 10) {
            Start-Sleep -Milliseconds 50
            $attempts++
        }
        
        $result = ""
        while ($stream.DataAvailable) {
            $result += $reader.ReadLine() + "`r`n"
        }
        $client.Close()
        return $result.Trim()
    }
    catch {
        Write-Error "OpenOCD telnet error: $_"
        return ""
    }
}

function Get-SP {
    # Read SP via OpenOCD 'reg sp' and parse the hex value
    $out = Send-Ocd "reg sp"
    if ($out -match 'sp[^:]*:\s*0x([0-9A-Fa-f]+)') {
        return [uint32]("0x" + $Matches[1])
    }
    Write-Error "Could not read SP register."
    return 0
}

switch ($Action) {
    "halt" {
        Write-Output "Halting CPU..."
        Send-Ocd "halt"
        Write-Output "CPU halted."
    }

    "resume" {
        Write-Output "Resuming CPU..."
        Send-Ocd "resume"
        Write-Output "CPU resumed."
    }

    "reset" {
        Write-Output "Resetting MCU (halt at reset)..."
        Send-Ocd "reset halt"
        Write-Output "MCU reset."
    }

    "regs" {
        Write-Output "Reading core registers..."
        $out = Send-Ocd "reg"
        Write-Output $out
    }

    "reg" {
        $name = if ($Extra.Count -ge 1) { $Extra[0] } else { "sp" }
        $out = Send-Ocd "reg $name"
        Write-Output $out
    }

    "peek" {
        $addr = if ($Extra.Count -ge 1) { $Extra[0] } else { "0x20000000" }
        $out = Send-Ocd "mrw $addr"
        Write-Output $out
    }

    "mdw" {
        $addr = if ($Extra.Count -ge 1) { $Extra[0] } else { "0x20000000" }
        $cnt  = if ($Extra.Count -ge 2) { $Extra[1] } else { "16" }
        $out = Send-Ocd "mdw $addr $cnt"
        Write-Output $out
    }

    "mdh" {
        $addr = if ($Extra.Count -ge 1) { $Extra[0] } else { "0x20000000" }
        $cnt  = if ($Extra.Count -ge 2) { $Extra[1] } else { "16" }
        $out = Send-Ocd "mdh $addr $cnt"
        Write-Output $out
    }

    "mdb" {
        $addr = if ($Extra.Count -ge 1) { $Extra[0] } else { "0x20000000" }
        $cnt  = if ($Extra.Count -ge 2) { $Extra[1] } else { "32" }
        $out = Send-Ocd "mdb $addr $cnt"
        Write-Output $out
    }

    "stack" {
        $cnt = if ($Extra.Count -ge 1) { [int]$Extra[0] } else { 16 }
        $sp = Get-SP
        if ($sp -eq 0) { break }
        $addr = "0x{0:X8}" -f $sp
        Write-Output "SP = $addr, dumping $cnt words:"
        $out = Send-Ocd "mdw $addr $cnt"
        Write-Output $out
    }

    "disasm" {
        $addr = if ($Extra.Count -ge 1) { $Extra[0] } else { "0x08000000" }
        $cnt  = if ($Extra.Count -ge 2) { $Extra[1] } else { "10" }
        $out = Send-Ocd "arm disassemble $addr $cnt"
        Write-Output $out
    }

    "var" {
        # Read a global/static variable by name using ELF symbol lookup
        if ($Extra.Count -lt 2) {
            Write-Error "Usage: dev_debug.ps1 var <elf_path> <var_name>"
            Write-Output "Example: dev_debug.ps1 var build/template.elf g_wTickCounter"
            break
        }
        $elfPath = $Extra[0]
        $varName = $Extra[1]
        if (-not (Test-Path $elfPath)) {
            Write-Error "ELF file not found: $elfPath"
            break
        }
        # Use llvm-nm to find the symbol address
        $nmOut = & $LLVM_NM $elfPath 2>&1 | Select-String -Pattern "\b$varName$"
        if (-not $nmOut) {
            Write-Error "Symbol '$varName' not found in $elfPath"
            break
        }
        $symAddr = ($nmOut -split '\s+')[0]
        Write-Output "Symbol '$varName' at address 0x$symAddr"
        $out = Send-Ocd "mrw 0x$symAddr"
        Write-Output "Value: $out"
    }

    default {
        Write-Output @"
MODUS dev_debug.ps1 — MCU debug helper
=======================================
  halt              Halt the CPU
  resume            Resume execution
  reset             Reset MCU and halt
  regs              Dump all core registers
  reg <name>        Read single register (sp, pc, lr, ...)
  peek <addr>       Read uint32 at address
  mdw <addr> [n]    Dump n words (default 16)
  mdh <addr> [n]    Dump n half-words
  mdb <addr> [n]    Dump n bytes
  stack [n]         Dump stack around SP (default 16 words)
  disasm <addr> [n] Disassemble n instructions
  var <elf> <name>  Read global/static variable value
"@
    }
}
