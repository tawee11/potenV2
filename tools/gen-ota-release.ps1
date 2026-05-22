param(
  [string]$EnvName = "esp32s3_n16r16v",
  [string]$OutDir = "tools\GitReleaseOTA",
  [string]$Repo = "",
  [string]$FirmwareVersion = "",
  [string]$WebVersion = ""
)

$ErrorActionPreference = "Stop"

$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$mainPath = Join-Path $root "src\main.cpp"
$buildDir = Join-Path $root ".pio\build\$EnvName"
$outPath = Join-Path $root $OutDir

function Read-CppConst {
  param([string]$Text, [string]$Name)
  $pattern = 'static\s+const\s+char\s*\*\s*' + [regex]::Escape($Name) + '\s*=\s*"([^"]+)"\s*;'
  $match = [regex]::Match($Text, $pattern)
  if (-not $match.Success) {
    throw "Cannot find $Name in $mainPath"
  }
  return $match.Groups[1].Value
}

function Read-OriginRepo {
  $remote = (& git -C $root remote get-url origin 2>$null).Trim()
  if (-not $remote) {
    throw "Cannot detect git remote origin. Pass -Repo owner/name."
  }
  if ($remote -match 'github\.com[:/](?<owner>[^/]+)/(?<repo>[^/.]+)(\.git)?$') {
    return "$($Matches.owner)/$($Matches.repo)"
  }
  throw "Cannot parse GitHub repo from origin '$remote'. Pass -Repo owner/name."
}

$mainText = Get-Content -Raw -Path $mainPath
if (-not $FirmwareVersion) { $FirmwareVersion = Read-CppConst $mainText "FW_VERSION" }
if (-not $WebVersion) { $WebVersion = Read-CppConst $mainText "WEB_VERSION" }
if (-not $Repo) { $Repo = Read-OriginRepo }

New-Item -ItemType Directory -Force -Path $outPath | Out-Null

$firmwareSrc = Join-Path $buildDir "firmware.bin"
$littlefsSrc = Join-Path $buildDir "littlefs.bin"
$firmwareOut = Join-Path $outPath "firmware.bin"
$littlefsOut = Join-Path $outPath "littlefs.bin"

if (Test-Path $firmwareSrc) {
  Copy-Item -Force -Path $firmwareSrc -Destination $firmwareOut
  Write-Host "Copied firmware.bin"
} else {
  Write-Host "Skipped firmware.bin: not found at $firmwareSrc"
}

if (Test-Path $littlefsSrc) {
  Copy-Item -Force -Path $littlefsSrc -Destination $littlefsOut
  Write-Host "Copied littlefs.bin"
} else {
  Write-Host "Skipped littlefs.bin: not found at $littlefsSrc"
}

$versionJson = [ordered]@{
  firmware_version = $FirmwareVersion
  web_version = $WebVersion
  firmware = "https://github.com/$Repo/releases/latest/download/firmware.bin"
  filesystem = "https://github.com/$Repo/releases/latest/download/littlefs.bin"
}

$jsonPath = Join-Path $outPath "version.json"
$versionJson | ConvertTo-Json -Depth 3 | Set-Content -Encoding UTF8 -Path $jsonPath

$tag = "v$FirmwareVersion"
Write-Host "Generated $jsonPath"
Write-Host ""
Write-Host "Create new release command:"
Write-Host "gh release create $tag `"$OutDir\firmware.bin`" `"$OutDir\littlefs.bin`" `"$OutDir\version.json`" --repo `"$Repo`" --title `"$tag`" --notes `"Poten V2 OTA release`""
Write-Host ""
Write-Host "Upload to existing release command:"
Write-Host "gh release upload $tag `"$OutDir\firmware.bin`" `"$OutDir\littlefs.bin`" `"$OutDir\version.json`" --repo `"$Repo`" --clobber"
