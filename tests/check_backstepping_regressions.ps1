$ErrorActionPreference = 'Stop'

function Assert-PatternExists {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $match = Select-String -Path $Path -Pattern $Pattern
    if (-not $match) {
        throw $Message
    }
}

function Assert-PatternNotCommented {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $lines = Get-Content -Path $Path
    foreach ($line in $lines) {
        if ($line -match $Pattern -and $line -notmatch '^\s*%') {
            return
        }
    }

    throw $Message
}

$root = Split-Path -Parent $PSScriptRoot

$monitor = Join-Path $root 'func_SystemModelMonitorBlock.m'
$rollCmd = Join-Path $root 'func_BacksteppingRollCommand.m'
$adapt = Join-Path $root 'func_AdaptiveParameterEstimation.m'

Assert-PatternNotCommented `
    -Path $monitor `
    -Pattern 'omega_d_dot_max\s*=\s*5(\.0)?\s*;' `
    -Message 'omega_d_dot_max must be assigned on an executable line.'

Assert-PatternNotCommented `
    -Path $rollCmd `
    -Pattern 'phi_c\s*=\s*max\(-phi_max,\s*min\(phi_max,\s*phi_c\)\)\s*;' `
    -Message 'phi_c saturation must execute on a non-comment line.'

Assert-PatternNotCommented `
    -Path $adapt `
    -Pattern 'lambda_hat\s*=\s*max\(lambda_hat_min,\s*min\(lambda_hat_max,\s*lambda_hat\)\)\s*;' `
    -Message 'lambda_hat saturation must execute on a non-comment line.'

Assert-PatternExists `
    -Path $rollCmd `
    -Pattern 'V_chi\s*<=\s*1e-6' `
    -Message 'Backstepping roll command must guard against zero airspeed.'

Assert-PatternExists `
    -Path $rollCmd `
    -Pattern 'phi_safe\s*=' `
    -Message 'Backstepping roll command must clamp phi before tan/sec calculations.'

Write-Host 'Backstepping regression checks passed.'
