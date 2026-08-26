#Requires -Version 5.1
<#
.SYNOPSIS
  Builds the mod, packages it, and copies it into the game's Mods folder.

.DESCRIPTION
  pack.ps1 does the build and the zip; this adds the step it deliberately
  leaves out — installing the result over the running game's copy. The files
  come from pack.ps1's own staging directory, so what gets deployed is exactly
  what the zip ships: Autopilot.dll and info.json, nothing else.

  The install directory is read from Directory.Build.targets, which is
  gitignored and already points at this machine's game install, so there is
  only one place to correct when the game moves.

  A running game does not have to be closed. UnityModManager shadow-copies the
  assembly, so the file is not locked; press Shift+F10 in game to reload. Note
  that a reload drops every autopilot run in progress — the run table lives in
  the assembly being replaced — so trains under autopilot need dispatching
  again afterwards.

.PARAMETER Configuration
  Build configuration to package and deploy. Default: Release.

.PARAMETER NoBuild
  Package and deploy the existing build output instead of rebuilding first.

.PARAMETER OutDir
  Where pack.ps1 writes the zip. Default: the repository root.

.PARAMETER DvInstallDir
  Game install to deploy into. Default: the DvInstallDir from
  Directory.Build.targets.

.EXAMPLE
  & '.\pack&deploy.ps1'
  & '.\pack&deploy.ps1' -NoBuild
  & '.\pack&deploy.ps1' -Configuration Debug
#>
[CmdletBinding()]
param(
  [string]$Configuration = 'Release',
  [switch]$NoBuild,
  [string]$OutDir = $PSScriptRoot,
  [string]$DvInstallDir
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$modName = 'Autopilot'

# Where the game is. Asked of the same file MSBuild resolves its references
# from, rather than a second copy of the path that can drift out of step.
if (-not $DvInstallDir) {
  $targets = Join-Path $root 'Directory.Build.targets'
  if (-not (Test-Path $targets)) {
    throw "No Directory.Build.targets at $targets, and no -DvInstallDir given."
  }
  $node = ([xml](Get-Content $targets -Raw)).SelectSingleNode('//DvInstallDir')
  if (-not $node) { throw "Directory.Build.targets has no <DvInstallDir>." }
  $DvInstallDir = $node.InnerText.Trim()
}
if (-not (Test-Path $DvInstallDir)) {
  throw "Game install not found: $DvInstallDir"
}

# Build and zip. Everything about versions, TFM subfolders and what belongs in
# the package is pack.ps1's business; this script only installs its output.
$pack = Join-Path $root 'pack.ps1'
if (-not (Test-Path $pack)) { throw "pack.ps1 not found at $pack" }
$packArgs = @{ Configuration = $Configuration; OutDir = $OutDir }
if ($NoBuild) { $packArgs.NoBuild = $true }
& $pack @packArgs
if ($LASTEXITCODE -ne 0 -and $null -ne $LASTEXITCODE) {
  throw "pack.ps1 failed ($LASTEXITCODE)"
}

# pack.ps1 stages exactly the two shipped files here.
$staging = Join-Path $root "obj\pack\$modName"
if (-not (Test-Path $staging)) { throw "pack.ps1 left no staging directory at $staging" }

$dest = Join-Path $DvInstallDir "Mods\$modName"
if (-not (Test-Path $dest)) {
  Write-Host "Creating $dest" -ForegroundColor Cyan
  New-Item -ItemType Directory -Path $dest -Force | Out-Null
}

$running = @(Get-Process -Name 'DerailValley' -ErrorAction SilentlyContinue).Count -gt 0

Write-Host ""
Write-Host "Deploying to $dest..." -ForegroundColor Cyan
foreach ($file in Get-ChildItem $staging -File) {
  try {
    Copy-Item $file.FullName (Join-Path $dest $file.Name) -Force
  }
  catch {
    throw "Could not write $($file.Name) to $dest — $($_.Exception.Message)" +
      $(if ($running) { " The game is running; if it has the file locked, close it and run this again." } else { "" })
  }
  Write-Host "  $($file.Name) ($($file.Length) bytes)"
}

Write-Host ""
Write-Host "Deployed $modName ($Configuration)" -ForegroundColor Green
if ($running) {
  Write-Host "  Derail Valley is running - press Shift+F10 in game to reload." -ForegroundColor Yellow
  Write-Host "  The reload drops every autopilot run in progress; dispatch them again."
}
