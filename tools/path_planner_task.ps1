param(
    [Parameter(Mandatory = $true)]
    [string]$PythonPath,

    [Parameter(Mandatory = $true)]
    [string]$ScriptPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if (-not (Test-Path $PythonPath)) {
    throw "Path planner Python was not found at '$PythonPath'. Create the workspace venv or update wro2026.pathPlanner.pythonPath."
}

if (-not (Test-Path $ScriptPath)) {
    throw "Path planner script was not found at '$ScriptPath'. Update wro2026.pathPlanner.scriptPath."
}

& $PythonPath $ScriptPath
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}