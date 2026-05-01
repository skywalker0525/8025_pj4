$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $scriptDir

$env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")

if (-not (Get-Command pdflatex -ErrorAction SilentlyContinue)) {
    throw "pdflatex was not found. Install MiKTeX and open a new PowerShell session."
}

if (-not (Get-Command bibtex -ErrorAction SilentlyContinue)) {
    throw "bibtex was not found. Install MiKTeX and open a new PowerShell session."
}

New-Item -ItemType Directory -Force -Path "output" | Out-Null

pdflatex -interaction=nonstopmode -halt-on-error -output-directory=output main.tex
bibtex output/main
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=output main.tex
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=output main.tex

$builtPdf = Join-Path $scriptDir "output\main.pdf"
$targetPdf = Join-Path $scriptDir "output\CIVIL8025_Project4_Report_TianTan.pdf"

if (-not (Test-Path -LiteralPath $builtPdf)) {
    throw "Expected PDF was not produced: $builtPdf"
}

Move-Item -LiteralPath $builtPdf -Destination $targetPdf -Force
Write-Host "Report written to $targetPdf"
