# Serial RX test v2: continuous read
param(
    [string]$Port = "COM4",
    [int]$Baud = 115200,
    [string]$Send = "",
    [int]$DurationMs = 4000
)

$p = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$p.ReadTimeout = 500
$p.Encoding = [System.Text.Encoding]::ASCII

try {
    $p.Open()
    Write-Host "[OK] Opened $Port @ $Baud"

    # Flush
    Start-Sleep -Milliseconds 200
    if ($p.BytesToRead -gt 0) { [void]$p.ReadExisting() }

    # Send if specified
    if ($Send -ne "") {
        $bytes = [System.Text.Encoding]::ASCII.GetBytes("$Send`r`n")
        $p.Write($bytes, 0, $bytes.Length)
        Write-Host "[TX] Sent: $Send"
    }

    # Continuously read for DurationMs
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $buf = New-Object byte[] 256
    while ($sw.ElapsedMilliseconds -lt $DurationMs) {
        if ($p.BytesToRead -gt 0) {
            $n = $p.Read($buf, 0, [Math]::Min($p.BytesToRead, 256))
            $text = [System.Text.Encoding]::ASCII.GetString($buf, 0, $n)
            # Show hex + ascii
            $hex = ($buf[0..($n-1)] | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
            Write-Host "[RX $n bytes] HEX: $hex"
            Write-Host "[RX ASCII ] $text"
        }
        Start-Sleep -Milliseconds 50
    }
} catch {
    Write-Host "[ERROR] $_"
} finally {
    if ($p.IsOpen) { $p.Close() }
    Write-Host "[OK] Closed $Port"
}
