# Runs idf.py from ESP-IDF v6.0 in a project directory with a per-target build dir.
# Usage: .\tools\idf.ps1 -Project <dir> -Target esp32s3|esp32c3 [-Port COMx] <idf.py args...>
# Example: .\tools\idf.ps1 -Project firmware\controller -Target esp32s3 build
[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory)] [string] $Project,
    [Parameter(Mandatory)] [ValidateSet('esp32s3', 'esp32c3')] [string] $Target,
    [string] $Port,
    [Parameter(ValueFromRemainingArguments)] [string[]] $IdfArgs
)
$env:IDF_PATH = 'C:\esp\release-v6.0\esp-idf'
# export.ps1 prints progress on stderr; do not treat that as a failure.
. "$env:IDF_PATH\export.ps1" 2>$null | Out-Null
if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) { throw 'ESP-IDF v6.0 export failed: idf.py not on PATH' }

$proj = Resolve-Path $Project
$build = Join-Path $proj "build_$Target"
$sdk = Join-Path $proj "sdkconfig.$Target"
$common = @('-C', $proj, '-B', $build, "-DSDKCONFIG=$sdk", "-DIDF_TARGET=$Target")
if ($Port) { $common += @('-p', $Port) }
idf.py @common @IdfArgs
exit $LASTEXITCODE
