Add-Type -TypeDefinition @"
using System;
using System.IO.Ports;
using System.Threading;
public class ReadHex {
    public static void Run() {
        var port = new SerialPort("COM4", 115200, Parity.None, 8, StopBits.One);
        port.ReadTimeout = 2000;
        try {
            port.Open();
            // 持续读取直到超时
            byte[] buf = new byte[8192];
            int count = 0;
            try {
                while (count < buf.Length) { buf[count++] = (byte)port.ReadByte(); }
            } catch (TimeoutException) { }
            if (count > 0) {
                Console.Write("Rx " + count + " bytes: ");
                for (int i = 0; i < count; i++) {
                    if (i > 0 && (i % 16) == 0) Console.WriteLine();
                    Console.Write(buf[i].ToString("X2") + " ");
                }
                Console.WriteLine();
                // 也尝试打印ASCII
                Console.Write("ASCII: ");
                for (int i = 0; i < count; i++) {
                    byte b = buf[i];
                    Console.Write(b >= 32 && b < 127 ? (char)b : '.');
                }
                Console.WriteLine();
            } else {
                Console.WriteLine("No data received");
            }
        } finally { port.Close(); }
    }
}
"@
[ReadHex]::Run()
