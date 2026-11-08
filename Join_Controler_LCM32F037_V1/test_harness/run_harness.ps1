<#
.SYNOPSIS
    Unified Build + PC Simulation (SIL) + Hardware-in-the-Loop (HIL) Test Harness Runner
    for ROBOT-LM Join_Controler_LCM32F037_V1.

.PARAMETER Mode
    "all" (default): Run Keil Build + PC SIL Simulation + HIL Hardware Test (if COM port present)
    "sim"          : Run Keil Build + PC SIL Simulation only (no hardware needed)
    "hil"          : Run HIL Hardware Test only on -Port

.PARAMETER Port
    Serial COM port for HIL hardware test (default: "COM26")

.PARAMETER Flash
    Switch to flash the compiled firmware via Keil UV4 + J-Link before running HIL test
#>
param(
    [ValidateSet("all", "sim", "hil")]
    [string]$Mode = "all",
    [string]$Port = "COM26",
    [switch]$Flash
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

$Uv4Path = "D:\Program Files\Kei_v5\UV4\UV4.exe"
$HostGccDir = "C:\Andestech\AndeSight_RDS_v323\cygwin\bin"
$PythonExe = "C:\Users\DungTranBK\.platformio\penv\Scripts\python.exe"
if (-not (Test-Path $PythonExe)) {
    $PythonExe = "python"
}

Write-Host "==========================================================================" -ForegroundColor Cyan
Write-Host "  ROBOT-LM JOIN CONTROLLER - UNIFIED TEST HARNESS PIPELINE ($Mode)" -ForegroundColor Cyan
Write-Host "==========================================================================" -ForegroundColor Cyan

# --------------------------------------------------------------------------
# STEP 1: Firmware Build Verification (Keil MDK-ARM V6.24)
# --------------------------------------------------------------------------
if ($Mode -ne "hil") {
    Write-Host "`n[STEP 1/3] Building Firmware with Keil UV4 (ARMCLANG V6.24)..." -ForegroundColor Yellow
    Stop-Process -Name "UV4" -Force -ErrorAction SilentlyContinue
    Start-Process -FilePath $Uv4Path -ArgumentList "-b `"Join_Controler_LCM32F037_V1.uvprojx`" -j0 -o `"build_log.txt`"" -Wait
    $BuildLog = Get-Content "build_log.txt"
    $SummaryLine = $BuildLog | Select-String "Error\(s\), .*Warning\(s\)"
    Write-Host "  $SummaryLine"
    if (-not ($BuildLog -match "0 Error\(s\), 0 Warning\(s\)")) {
        Write-Host "[FAIL] Firmware build failed or produced warnings!" -ForegroundColor Red
        exit 1
    }
    Write-Host "  -> Keil Firmware Build: PASS" -ForegroundColor Green
}

# --------------------------------------------------------------------------
# STEP 2: PC Host Software-in-the-Loop (SIL) Simulation & Unit Test Harness
# --------------------------------------------------------------------------
if ($Mode -eq "all" -or $Mode -eq "sim") {
    Write-Host "`n[STEP 2/3] Compiling & Running PC SIL Simulation Harness..." -ForegroundColor Yellow
    $env:PATH = "$HostGccDir;" + $env:PATH
    & gcc.exe -std=c99 -O2 -Wall -IInclude -DMOTOR_JOINT_TYPE=2 `
        test_harness/sim/sim_harness.c `
        Source/mid/motor_ctrl.c `
        Source/mid/motor_monitor.c `
        Source/mid/protocol.c `
        Source/mid/foc_math.c `
        Source/mid/trig_table.c `
        Source/mid/control_table.c `
        Source/bsp/bsp_mt9102.c `
        -lm -o test_harness/sim/sim_harness.exe

    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FAIL] Host GCC compilation of sim_harness failed!" -ForegroundColor Red
        exit 1
    }

    & "$RepoRoot\test_harness\sim\sim_harness.exe"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FAIL] PC SIL Simulation Test Harness reported failures!" -ForegroundColor Red
        exit 1
    }
}

# --------------------------------------------------------------------------
# STEP 3: Optional J-Link Flash & Hardware-in-the-Loop (HIL) Test Suite
# --------------------------------------------------------------------------
if ($Mode -eq "all" -or $Mode -eq "hil") {
    $AvailablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($AvailablePorts -notcontains $Port) {
        if ($Mode -eq "hil") {
            Write-Host "`n[ERROR] Serial port $Port is not connected (Available: $($AvailablePorts -join ', '))" -ForegroundColor Red
            exit 1
        } else {
            Write-Host "`n[STEP 3/3] Hardware port $Port not detected -> Skipping HIL Hardware Suite (PC SIL Passed)." -ForegroundColor DarkYellow
            exit 0
        }
    }

    if ($Flash) {
        Write-Host "`n[STEP 3a] Flashing Firmware to Board via Keil UV4 + J-Link..." -ForegroundColor Yellow
        Stop-Process -Name "UV4" -Force -ErrorAction SilentlyContinue
        Start-Process -FilePath $Uv4Path -ArgumentList "-f `"Join_Controler_LCM32F037_V1.uvprojx`" -j0 -o `"flash_log.txt`"" -Wait
        Get-Content "flash_log.txt" | Select-Object -Last 6
        Start-Sleep -Seconds 1
    }

    Write-Host "`n[STEP 3/3] Running HIL Hardware Test Suite on $Port..." -ForegroundColor Yellow
    & $PythonExe "test_harness/hil/hil_harness.py" --port $Port
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FAIL] HIL Hardware Test Suite reported failures!" -ForegroundColor Red
        exit 1
    }
}

Write-Host "`n[ALL HARNESS CHECKS PASSED]" -ForegroundColor Green
exit 0
