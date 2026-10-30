$ErrorActionPreference = "Stop"
$VigilDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$UserPath = [Environment]::GetEnvironmentVariable("Path", "User")
$Entries = @($UserPath -split ";" | Where-Object { $_ })

if ($Entries -notcontains $VigilDirectory) {
    $NewPath = (@($Entries) + $VigilDirectory) -join ";"
    [Environment]::SetEnvironmentVariable("Path", $NewPath, "User")
}

if (($env:Path -split ";") -notcontains $VigilDirectory) {
    $env:Path = "$env:Path;$VigilDirectory"
}

Write-Host "VIGIL launcher installed. Open a new terminal and type: vigil --docker"
