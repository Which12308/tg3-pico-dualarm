param([switch]$Smoke)
$ErrorActionPreference = 'Stop'
$TaskRoot = Split-Path -Parent $PSScriptRoot
$TaskVenv = Join-Path $TaskRoot '.desktop\venv'
$TaskPython = Join-Path $TaskVenv 'Scripts\python.exe'
try {
    if (-not (Test-Path -LiteralPath $TaskPython)) {
        $TaskBasePython = Get-Command python -ErrorAction SilentlyContinue
        if (-not $TaskBasePython) { throw '需要先安装 Python 3.12（含 Tcl/Tk），再双击启动。' }
        & $TaskBasePython.Source -c "import sys; assert sys.version_info[:2] == (3,12), 'Python_3_12_required'"
        if ($LASTEXITCODE -ne 0) { throw '未找到 Python 3.12。' }
        & $TaskBasePython.Source -m venv $TaskVenv
        if ($LASTEXITCODE -ne 0) { throw '无法创建电脑端独立环境。' }
    }
    & $TaskPython -s -c "import sys; assert sys.version_info[:2] == (3,12), 'Python_3_12_required'"
    if ($LASTEXITCODE -ne 0) { throw '项目独立环境 Python 版本不兼容。' }
    $TaskRequirements = Join-Path $PSScriptRoot 'requirements.txt'
    $TaskReceipt = Join-Path $TaskVenv 'requirements.sha256'
    $TaskDigest = (Get-FileHash -LiteralPath $TaskRequirements -Algorithm SHA256).Hash
    if (-not (Test-Path -LiteralPath $TaskReceipt) -or (Get-Content -LiteralPath $TaskReceipt -Raw).Trim() -ne $TaskDigest) {
        & $TaskPython -s -m pip --isolated install --no-user -r $TaskRequirements
        if ($LASTEXITCODE -ne 0) { throw '电脑端依赖安装失败，请查看上述具体错误。' }
        Set-Content -LiteralPath $TaskReceipt -Value $TaskDigest
    }
    if ($Smoke) {
        & $TaskPython -s -X utf8 (Join-Path $TaskRoot 'scripts\desktop_smoke.py') --require-desktop-dependencies
    } else {
        & $TaskPython -s (Join-Path $PSScriptRoot 'start.py')
    }
    if ($LASTEXITCODE -ne 0) { throw '启动器未正常完成。' }
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    Read-Host '按回车关闭'
    exit 2
}
