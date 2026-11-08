# Builds and runs the host unit tests for common/transfer_core with MinGW gcc.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$gcc = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $gcc) { $gcc = Get-Item "$env:USERPROFILE\scoop\apps\gcc\current\bin\gcc.exe" }
$out = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $out | Out-Null
$core = Join-Path $root 'common\transfer_core'
& $gcc -std=c99 -O1 -g -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion `
    -I "$core\include" `
    "$core\src\proto.c" "$core\src\plant_model.c" "$core\src\controller_fsm.c" "$core\src\blackbox.c" `
    (Join-Path $PSScriptRoot 'test_core.c') -lm -o (Join-Path $out 'test_core.exe')
if ($LASTEXITCODE -ne 0) { Write-Host 'BUILD FAILED'; exit 1 }
& (Join-Path $out 'test_core.exe')
exit $LASTEXITCODE
