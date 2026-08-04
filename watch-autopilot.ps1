#Requires -Version 5.1
<#
.SYNOPSIS
  Record the autopilot's live status from remote-dispatch, continuously.

.DESCRIPTION
  Polls /autopilotStatus and appends a line whenever anything changes, so a run
  can be read AFTER it happened instead of having to be watched as it happens.
  Every run's notes are captured too, including the ones a finished run drops.

  Writes NDJSON (one full status object per change) plus a readable log beside
  it. Leave it running in a terminal; it costs nothing while nothing is moving.

.EXAMPLE
  .\watch-autopilot.ps1
  .\watch-autopilot.ps1 -IntervalMs 250 -Out runs\c3l
#>
[CmdletBinding()]
param(
  [string]$Server = 'http://127.0.0.1:7245',
  [string]$User = 'Andrei',
  [string]$Password = '',
  [int]$IntervalMs = 500,
  [string]$Out = 'autopilot-watch'
)

$ErrorActionPreference = 'Stop'
$headers = @{ Authorization = 'Basic ' + [Convert]::ToBase64String(
    [Text.Encoding]::ASCII.GetBytes("${User}:${Password}")) }

$jsonPath = "$Out.ndjson"
$logPath = "$Out.log"
foreach ($p in @($jsonPath, $logPath)) {
  $dir = Split-Path $p -Parent
  if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
}

Write-Host "watching $Server/autopilotStatus every ${IntervalMs}ms" -ForegroundColor Cyan
Write-Host "  $jsonPath"
Write-Host "  $logPath"
Write-Host 'Ctrl+C to stop.'

# Per-run memory of what was last written, so an unchanged status is silent and
# a run that is simply sitting still does not fill the file.
$seen = @{}

while ($true) {
  try {
    $status = Invoke-RestMethod "$Server/autopilotStatus" -Headers $headers -TimeoutSec 8
  }
  catch {
    Start-Sleep -Milliseconds ([Math]::Max(1000, $IntervalMs))
    continue
  }

  foreach ($prop in $status.PSObject.Properties) {
    $run = $prop.Value
    $stamp = (Get-Date).ToString('HH:mm:ss.fff')

    # The signature deliberately includes detail: it carries the drive loop's
    # own running commentary, which is where the reason for a stall shows up.
    $sig = '{0}|{1}|{2}|{3:F1}|{4:F1}' -f $run.state, $run.detail, $run.track, $run.span, $run.kmh
    if ($seen[$prop.Name] -eq $sig) { continue }
    $seen[$prop.Name] = $sig

    ($run | ConvertTo-Json -Depth 8 -Compress) | Add-Content $jsonPath
    $line = '{0} {1,-6} {2,-14} {3,7:F1}m {4,5:F1}km/h thr{5,4:F2} brk{6,4:F2} rev{7,3:F0} | {8} | {9}' -f `
      $stamp, $run.loco, $run.track, $run.span, $run.kmh,
      $run.throttle, $run.brake, $run.reverser, $run.state, $run.detail
    $line | Tee-Object -FilePath $logPath -Append | Write-Host

    # A terminal state closes the story: dump the run's whole reasoning, which
    # is the part that disappears once the run is cleared.
    if ($run.state -in @('arrived', 'coupled', 'error')) {
      $tail = @("$stamp --- $($run.loco) $($run.state.ToUpper()): $($run.detail)")
      foreach ($n in $run.notes) {
        $repeat = if ($n.count -gt 1) { "  [x$($n.count)]" } else { '' }
        $tail += ('           {0,9:F1}s {1}{2}' -f $n.at, $n.text, $repeat)
      }
      $tail | Tee-Object -FilePath $logPath -Append | Write-Host -ForegroundColor Yellow
      # Deliberately NOT forgetting the run here. A finished run keeps being
      # reported by the server for as long as it is listed, so clearing the
      # signature made every poll re-print the whole closing block.
      $seen[$prop.Name] = $sig
    }
  }

  Start-Sleep -Milliseconds $IntervalMs
}
