# Syntax only. This is deliberately NOT a Unity/IL2CPP/Android build check.
$ErrorActionPreference = 'Stop'
Add-Type -TypeDefinition 'public class TG3SyntaxWarmup {}'
$TaskRoot = Split-Path -Parent $PSScriptRoot
[void][IO.Directory]::CreateDirectory((Join-Path $TaskRoot 'artifacts'))
$TaskResults = @()
foreach ($TaskFile in (Get-ChildItem -LiteralPath (Join-Path $TaskRoot 'headset\Assets') -Recurse -Filter '*.cs')) {
    $TaskSyntax = [Microsoft.CodeAnalysis.CSharp.CSharpSyntaxTree]::ParseText([IO.File]::ReadAllText($TaskFile.FullName))
    $TaskErrors = @($TaskSyntax.GetDiagnostics() | Where-Object { $_.Severity.ToString() -eq 'Error' })
    $TaskResults += @{ file=$TaskFile.Name; syntax_errors=@($TaskErrors | ForEach-Object { $_.ToString() }) }
}
$TaskReport = @{ syntax_only=$true; unity_build_executed=$false; apk_built=$false; files=$TaskResults }
$TaskReport | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $TaskRoot 'artifacts\headset_syntax_result.json') -Encoding utf8
if (@($TaskResults | Where-Object { $_.syntax_errors.Count -gt 0 }).Count -gt 0) { exit 2 }
$TaskReport | ConvertTo-Json -Depth 6
