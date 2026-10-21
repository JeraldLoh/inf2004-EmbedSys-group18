param([string]$RepoRoot = (Split-Path -Parent $PSScriptRoot))
$ErrorActionPreference = 'Stop'
$taskRoot = [IO.Path]::GetFullPath($RepoRoot)
$taskRequired = @(
    'app_program/app_main.c', 'app_program/app.h',
    'app_program/vehicle_controller/vehicle_controller.c',
    'app_program/vehicle_controller/safety.c',
    'app_program/subsystem2/motion_math.c',
    'app_program/subsystem2/motion_math.h',
    'app_program/subsystem3/barcode_decoder.c',
    'app_program/subsystem3/barcode_decoder.h',
    'app_program/subsystem4/lsm303_decode.c',
    'app_program/subsystem4/lsm303_decode.h',
    'app_program/contracts/control_messages.h',
    'app_program/contracts/system_types.h',
    'app_program/project_config/pinmap.h',
    'build_make/Makefile', 'build_make/mtkernel_3/app_program/subdir.mk',
    'config/config.h', 'TEAM.md', 'testing/host/CMakeLists.txt'
)
foreach ($taskPath in $taskRequired) {
    if (-not (Test-Path -LiteralPath (Join-Path $taskRoot $taskPath))) {
        throw "Missing required path: $taskPath"
    }
}
foreach ($taskNumber in 1..5) {
    foreach ($taskSuffix in @('README.md', "subsystem$taskNumber.c",
        "subsystem$taskNumber.h", "subsystem${taskNumber}_test.c")) {
        $taskPath = Join-Path $taskRoot "app_program/subsystem$taskNumber/$taskSuffix"
        if (-not (Test-Path -LiteralPath $taskPath)) { throw "Missing buddy path: $taskPath" }
    }
}
$taskApp = Join-Path $taskRoot 'app_program'
$taskAllC = @(Get-ChildItem -LiteralPath $taskApp -Recurse -File -Filter '*.c')
$taskDiscovered = @((Get-ChildItem -LiteralPath $taskApp -File -Filter '*.c').FullName)
foreach ($taskFolder in Get-ChildItem -LiteralPath $taskApp -Directory) {
    $taskDiscovered += @((Get-ChildItem -LiteralPath $taskFolder.FullName -File -Filter '*.c').FullName)
}
foreach ($taskFile in $taskAllC) {
    if ($taskFile.FullName -notin $taskDiscovered) {
        throw "Target C source is deeper than the inspected Make wildcard: $($taskFile.FullName)"
    }
    $taskContent = Get-Content -LiteralPath $taskFile.FullName -Raw
    if ($taskContent -match '\bmain\s*\(') { throw "Unexpected second main in target tree: $($taskFile.FullName)" }
    if ($taskFile.Name -in @('task_led.c','task_producer.c','task_consumer.c','task_monitor.c')) {
        throw "Course demo left active: $($taskFile.FullName)"
    }
}
foreach ($taskFolder in @('app_program','testing/host')) {
    foreach ($taskFile in Get-ChildItem -LiteralPath (Join-Path $taskRoot $taskFolder) -Recurse -File) {
        if ($taskFile.Extension -notin @('.c','.h')) { continue }
        foreach ($taskLine in Get-Content -LiteralPath $taskFile.FullName) {
            if ($taskLine -match '^\s*#include\s+"(\.[^"]+)"') {
                $taskInclude = [IO.Path]::GetFullPath((Join-Path $taskFile.DirectoryName $Matches[1]))
                if (-not (Test-Path -LiteralPath $taskInclude)) {
                    throw "Broken relative include in $($taskFile.FullName): $taskLine"
                }
            }
        }
    }
}
$taskForbiddenRuntimePatterns = @(
    '\bint\s+main\s*\(',
    '\bstdio_init_all\s*\(',
    '\bsleep_ms\s*\(',
    '\bcyw43_[A-Za-z0-9_]*\s*\(',
    '\bmqtt_[A-Za-z0-9_]*\s*\('
)
foreach ($taskFile in $taskAllC) {
    $taskContent = Get-Content -LiteralPath $taskFile.FullName -Raw
    foreach ($taskPattern in $taskForbiddenRuntimePatterns) {
        if ($taskContent -match $taskPattern) {
            throw "Standalone/owner-unsafe API in target source $($taskFile.FullName): $taskPattern"
        }
    }
}
if (@(Get-ChildItem -LiteralPath (Join-Path $taskRoot 'firmware') -Recurse -File -Filter '*.c').Count -ne 0) {
    throw 'Duplicate C implementation remains under old firmware root.'
}
Write-Output "PASS: $($taskAllC.Count) target C files fit the one-level Make discovery rule."
Write-Output 'PASS: subsystem paths, relative includes and old-root redirect verified.'
Write-Output 'PASS: no standalone main/startup/delay or raw network call entered target application code.'
Write-Output 'Static check only: no Make, firmware build, flash or hardware operation performed.'
