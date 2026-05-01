$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $scriptDir

New-Item -ItemType Directory -Force -Path "output" | Out-Null

$imageName = "civil8025-report-latex"
$pdfName = "CIVIL8025_Project4_Report_TianTan.pdf"

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    throw "Docker is not available in PATH. Install Docker Desktop, restart PowerShell, and run this script again."
}

docker build -t $imageName .
docker run --rm -v "${scriptDir}:/work" $imageName

$builtPdf = Join-Path $scriptDir "output\main.pdf"
$targetPdf = Join-Path $scriptDir "output\$pdfName"

if (-not (Test-Path -LiteralPath $builtPdf)) {
    throw "Expected PDF was not produced: $builtPdf"
}

Move-Item -LiteralPath $builtPdf -Destination $targetPdf -Force
Write-Host "Report written to $targetPdf"
