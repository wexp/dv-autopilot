#Requires -Version 5.1
<#
.SYNOPSIS
  Builds the mod and packages it into a UnityModManager-ready zip.

.DESCRIPTION
  Produces Autopilot_v<version>.zip whose root contains a single folder,
  Autopilot/, holding Autopilot.dll and info.json — the layout UMM expects
  when installing from a zip.

.PARAMETER Configuration
  Build configuration to package. Default: Release.

.PARAMETER NoBuild
  Package the existing build output instead of rebuilding first.

.PARAMETER OutDir
  Where to write the zip. Default: the repository root.

.EXAMPLE
  .\pack.ps1
  .\pack.ps1 -NoBuild
  .\pack.ps1 -Configuration Debug -OutDir .\dist
#>
[CmdletBinding()]
param(
  [string]$Configuration = 'Release',
  [switch]$NoBuild,
  [string]$OutDir = $PSScriptRoot
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$modName = 'Autopilot'

# Version comes from info.json so the zip name and the shipped manifest agree.
$infoPath = Join-Path $root 'info.json'
if (-not (Test-Path $infoPath)) { throw "info.json not found at $infoPath" }
$version = (Get-Content $infoPath -Raw | ConvertFrom-Json).Version
if (-not $version) { throw 'Could not read Version from info.json' }

if (-not $NoBuild) {
  Write-Host "Building $Configuration..." -ForegroundColor Cyan
  & dotnet build -c $Configuration | Write-Host
  if ($LASTEXITCODE -ne 0) { throw "dotnet build failed ($LASTEXITCODE)" }
}

# Find the freshest built DLL for this configuration, whatever TFM subfolder it
# lands in (net48, ...).
$binDir = Join-Path $root "bin\$Configuration"
if (-not (Test-Path $binDir)) { throw "No build output at $binDir. Build first, or drop -NoBuild." }
$dll = Get-ChildItem $binDir -Recurse -Filter "$modName.dll" |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1
if (-not $dll) { throw "No $modName.dll under $binDir." }

# Stage Autopilot/ with exactly the two files the mod ships.
$staging = Join-Path $root "obj\pack\$modName"
if (Test-Path (Split-Path $staging)) { Remove-Item (Split-Path $staging) -Recurse -Force }
New-Item -ItemType Directory -Path $staging -Force | Out-Null
Copy-Item $dll.FullName (Join-Path $staging "$modName.dll")
Copy-Item $infoPath (Join-Path $staging 'info.json')

# Zip so the archive root holds the Autopilot/ folder.
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }
$zipPath = Join-Path (Resolve-Path $OutDir) "${modName}_v$version.zip"
if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
Compress-Archive -Path $staging -DestinationPath $zipPath -CompressionLevel Optimal

Write-Host ""
Write-Host "Packaged $modName v$version" -ForegroundColor Green
Write-Host "  dll: $($dll.FullName)"
Write-Host "  zip: $zipPath"
Write-Host ""
Write-Host "Release: git tag v$version && git push origin v$version, then attach the zip:"
Write-Host "  gh release create v$version `"$zipPath`" --title `"v$version`""
