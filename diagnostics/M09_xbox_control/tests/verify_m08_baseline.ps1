$ErrorActionPreference = 'Stop'
$baselineRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\M08_go_to_pose')).Path
$snapshot = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'm08_baseline_hashes.json') -Raw | ConvertFrom-Json
$files = @(Get-ChildItem -LiteralPath $baselineRoot -File -Recurse | Where-Object { $_.FullName -notmatch '\\\.pio\\' })
if ($files.Count -ne $snapshot.Count) { throw 'M08 file count changed since the pre-M09 snapshot' }
foreach ($entry in $snapshot) {
    $baselineFile = Join-Path $baselineRoot $entry.Path
    if ((Get-FileHash -LiteralPath $baselineFile -Algorithm SHA256).Hash -ne $entry.Hash) {
        throw "M08 baseline changed: $($entry.Path)"
    }
}
Write-Output "PASS: all $($snapshot.Count) M08 files are byte-for-byte unchanged (excluding .pio build artifacts)."
