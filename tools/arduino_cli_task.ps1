param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('verify', 'upload', 'list-boards', 'monitor')]
    [string]$Action,

    [Parameter(Mandatory = $true)]
    [string]$CliPath,

    [Parameter(Mandatory = $true)]
    [string]$Fqbn,

    [Parameter(Mandatory = $true)]
    [string]$SketchPath,

    [string]$Port = '',

    [string]$BaudRate = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')

if (-not (Test-Path $CliPath)) {
    throw "arduino-cli was not found at '$CliPath'. Update the wro2026.arduino.cliPath setting."
}

function Resolve-SketchDir {
    return Resolve-Path (Join-Path $repoRoot $SketchPath)
}

function Invoke-ArduinoCli {
    param(
        [Parameter(ValueFromRemainingArguments = $true)]
        [string[]]$Arguments
    )

    & $CliPath @Arguments
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

function Resolve-SerialPort {
    param(
        [string]$ConfiguredPort
    )

    if (-not [string]::IsNullOrWhiteSpace($ConfiguredPort)) {
        return $ConfiguredPort
    }

    $boardListJson = & $CliPath board list --json --fqbn $Fqbn
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    $detectedPorts = @(
        ($boardListJson | ConvertFrom-Json).detected_ports |
            Where-Object { $_.port.address } |
            ForEach-Object { $_.port.address }
    )
    $detectedPorts = @($detectedPorts | Sort-Object -Unique)

    if ($detectedPorts.Count -eq 1) {
        return $detectedPorts[0]
    }

    if ($detectedPorts.Count -eq 0) {
        throw 'No Arduino board detected. Connect the board or set wro2026.arduino.port in workspace settings.'
    }

    throw ("Multiple serial devices detected: {0}. Set wro2026.arduino.port in workspace settings." -f ($detectedPorts -join ', '))
}

function Resolve-BaudRate {
    param(
        [string]$ConfiguredBaudRate
    )

    if (-not [string]::IsNullOrWhiteSpace($ConfiguredBaudRate)) {
        return $ConfiguredBaudRate
    }

    throw 'No baud rate configured. Set wro2026.arduino.baudRate in workspace settings.'
}

switch ($Action) {
    'verify' {
        $sketchDir = Resolve-SketchDir
        Invoke-ArduinoCli compile --fqbn $Fqbn $sketchDir
    }
    'upload' {
        $sketchDir = Resolve-SketchDir
        $uploadPort = Resolve-SerialPort -ConfiguredPort $Port
        Write-Host "Uploading to $uploadPort"
        Invoke-ArduinoCli upload --fqbn $Fqbn --port $uploadPort $sketchDir
    }
    'list-boards' {
        Invoke-ArduinoCli board list
    }
    'monitor' {
        $monitorPort = Resolve-SerialPort -ConfiguredPort $Port
        $monitorBaudRate = Resolve-BaudRate -ConfiguredBaudRate $BaudRate
        Write-Host "Monitoring $monitorPort at $monitorBaudRate baud"
        Invoke-ArduinoCli monitor --port $monitorPort --config "baudrate=$monitorBaudRate"
    }
}
