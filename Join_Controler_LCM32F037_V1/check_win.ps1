Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
using System.Text;

public class Win32 {
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll")]
    public static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);
    [DllImport("user32.dll")]
    public static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);
    [DllImport("user32.dll")]
    public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);
    [DllImport("user32.dll")]
    public static extern bool IsWindowVisible(IntPtr hWnd);
}
"@

[Win32]::EnumWindows({
    param($hwnd, $lparam)
    $pid = 0
    [Win32]::GetWindowThreadProcessId($hwnd, [ref]$pid)
    $proc = Get-Process -Id $pid -ErrorAction SilentlyContinue
    if ($proc -and ($proc.Name -like "*UV4*" -or $proc.Name -like "*JLink*")) {
        $sb = New-Object System.Text.StringBuilder 256
        [Win32]::GetWindowText($hwnd, $sb, 256)
        $vis = [Win32]::IsWindowVisible($hwnd)
        Write-Host "$($proc.Name) ($pid) [Visible=$vis]: $($sb.ToString())"
    }
    return $true
}, 0)
