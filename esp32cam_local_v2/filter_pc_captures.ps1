param(
    [string]$InputDir = "D:\HOCTAP\ARDUINO\esp32cam_local\pc_captures",
    [string]$OutputDir = "D:\HOCTAP\ARDUINO\esp32cam_local\pc_captures_filtered",
    [ValidateSet("enhance", "edge")]
    [string]$Mode = "enhance",
    [switch]$LatestOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

Add-Type -AssemblyName System.Drawing

function Clamp-Byte {
    param([double]$Value)

    if ($Value -lt 0) { return 0 }
    if ($Value -gt 255) { return 255 }
    return [int][Math]::Round($Value)
}

function New-Matrix {
    param(
        [int]$Width,
        [int]$Height
    )

    $rows = New-Object 'double[][]' $Height
    for ($y = 0; $y -lt $Height; $y++) {
        $rows[$y] = New-Object 'double[]' $Width
    }
    return $rows
}

function Convert-ToGrayMatrix {
    param([System.Drawing.Bitmap]$Bitmap)

    $gray = New-Matrix -Width $Bitmap.Width -Height $Bitmap.Height
    for ($y = 0; $y -lt $Bitmap.Height; $y++) {
        for ($x = 0; $x -lt $Bitmap.Width; $x++) {
            $pixel = $Bitmap.GetPixel($x, $y)
            $gray[$y][$x] = (0.299 * $pixel.R) + (0.587 * $pixel.G) + (0.114 * $pixel.B)
        }
    }
    return $gray
}

function Blur-Matrix {
    param([double[][]]$Source)

    $height = $Source.Length
    $width = $Source[0].Length
    $blurred = New-Matrix -Width $width -Height $height

    for ($y = 0; $y -lt $height; $y++) {
        for ($x = 0; $x -lt $width; $x++) {
            $sum = 0.0
            $count = 0
            for ($dy = -1; $dy -le 1; $dy++) {
                $yy = [Math]::Min([Math]::Max($y + $dy, 0), $height - 1)
                for ($dx = -1; $dx -le 1; $dx++) {
                    $xx = [Math]::Min([Math]::Max($x + $dx, 0), $width - 1)
                    $sum += $Source[$yy][$xx]
                    $count++
                }
            }
            $blurred[$y][$x] = $sum / $count
        }
    }

    return $blurred
}

function Stretch-Contrast {
    param([double[][]]$Source)

    $height = $Source.Length
    $width = $Source[0].Length
    $min = [double]::PositiveInfinity
    $max = [double]::NegativeInfinity

    for ($y = 0; $y -lt $height; $y++) {
        for ($x = 0; $x -lt $width; $x++) {
            $value = $Source[$y][$x]
            if ($value -lt $min) { $min = $value }
            if ($value -gt $max) { $max = $value }
        }
    }

    $range = $max - $min
    $result = New-Matrix -Width $width -Height $height

    for ($y = 0; $y -lt $height; $y++) {
        for ($x = 0; $x -lt $width; $x++) {
            $normalized = if ($range -gt 0.001) {
                (($Source[$y][$x] - $min) * 255.0) / $range
            } else {
                $Source[$y][$x]
            }

            $result[$y][$x] = Clamp-Byte ((($normalized - 128.0) * 1.20) + 128.0)
        }
    }

    return $result
}

function Sobel-Edges {
    param([double[][]]$Source)

    $gxKernel = @(
        @(-1, 0, 1),
        @(-2, 0, 2),
        @(-1, 0, 1)
    )
    $gyKernel = @(
        @(-1, -2, -1),
        @(0, 0, 0),
        @(1, 2, 1)
    )

    $height = $Source.Length
    $width = $Source[0].Length
    $magnitudes = New-Matrix -Width $width -Height $height
    $maxMagnitude = 1.0

    for ($y = 0; $y -lt $height; $y++) {
        for ($x = 0; $x -lt $width; $x++) {
            $gx = 0.0
            $gy = 0.0
            for ($ky = -1; $ky -le 1; $ky++) {
                $yy = [Math]::Min([Math]::Max($y + $ky, 0), $height - 1)
                for ($kx = -1; $kx -le 1; $kx++) {
                    $xx = [Math]::Min([Math]::Max($x + $kx, 0), $width - 1)
                    $sample = $Source[$yy][$xx]
                    $gx += $sample * $gxKernel[$ky + 1][$kx + 1]
                    $gy += $sample * $gyKernel[$ky + 1][$kx + 1]
                }
            }

            $magnitude = [Math]::Sqrt(($gx * $gx) + ($gy * $gy))
            if ($magnitude -gt $maxMagnitude) { $maxMagnitude = $magnitude }
            $magnitudes[$y][$x] = $magnitude
        }
    }

    $edges = New-Matrix -Width $width -Height $height
    for ($y = 0; $y -lt $height; $y++) {
        for ($x = 0; $x -lt $width; $x++) {
            $edges[$y][$x] = Clamp-Byte (($magnitudes[$y][$x] * 255.0) / $maxMagnitude)
        }
    }

    return $edges
}

function Save-GrayJpeg {
    param(
        [double[][]]$Matrix,
        [string]$Path
    )

    $height = $Matrix.Length
    $width = $Matrix[0].Length
    $bitmap = New-Object System.Drawing.Bitmap($width, $height)

    try {
        for ($y = 0; $y -lt $height; $y++) {
            for ($x = 0; $x -lt $width; $x++) {
                $v = Clamp-Byte $Matrix[$y][$x]
                $bitmap.SetPixel($x, $y, [System.Drawing.Color]::FromArgb($v, $v, $v))
            }
        }
        $bitmap.Save($Path, [System.Drawing.Imaging.ImageFormat]::Jpeg)
    } finally {
        $bitmap.Dispose()
    }
}

if (-not (Test-Path -LiteralPath $InputDir)) {
    throw "InputDir not found: $InputDir"
}

New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null

$files = Get-ChildItem -LiteralPath $InputDir -File -Filter "*.jpg" | Sort-Object LastWriteTime
if ($LatestOnly) {
    $files = $files | Select-Object -Last 1
}

if (-not $files) {
    Write-Host "No JPG files found in $InputDir"
    exit 0
}

foreach ($file in $files) {
    Write-Host "Filtering $($file.Name) -> $Mode"
    $bitmap = [System.Drawing.Bitmap]::FromFile($file.FullName)
    try {
        $gray = Convert-ToGrayMatrix -Bitmap $bitmap
        $blurred = Blur-Matrix -Source $gray
        $filtered = if ($Mode -eq "edge") {
            Sobel-Edges -Source $blurred
        } else {
            Stretch-Contrast -Source $blurred
        }

        $outputPath = Join-Path $OutputDir $file.Name
        Save-GrayJpeg -Matrix $filtered -Path $outputPath
    } finally {
        $bitmap.Dispose()
    }
}

Write-Host "Done. Output: $OutputDir"
