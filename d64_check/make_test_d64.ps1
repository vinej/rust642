# Build a minimal 174848-byte D64 image containing one PRG file in the
# directory, mirrored from make_d64.rs. Output is suitable for exercising the
# KERNAL LOAD trap added to rust64.

param(
    [Parameter(Mandatory=$true)][string]$Prg,
    [Parameter(Mandatory=$true)][string]$DiskName,
    [Parameter(Mandatory=$true)][string]$Out
)

$ErrorActionPreference = "Stop"

$SECTOR = 256
$DIR_TRACK = 18

# Sectors-before-track (1-indexed; entry [0] unused).
$SECTORS_BEFORE = @(
    0,
    0,21,42,63,84,105,126,147,168,
    189,210,231,252,273,294,315,336,
    357,
    376,395,414,433,452,471,
    490,508,526,544,562,580,
    598,615,632,649,666
)
$SECTORS_PER_TRACK = @(
    0,
    21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
    19,19,19,19,19,19,19,
    18,18,18,18,18,18,
    17,17,17,17,17
)

function Sector-Off([int]$track, [int]$sector) {
    return ($SECTORS_BEFORE[$track] + $sector) * $SECTOR
}

function To-Petscii([string]$s) {
    $bytes = [System.Text.Encoding]::ASCII.GetBytes($s)
    for ($i = 0; $i -lt $bytes.Length; $i++) {
        $b = $bytes[$i]
        if ($b -ge 0x61 -and $b -le 0x7A) { $bytes[$i] = $b - 0x20 }
    }
    return $bytes
}

if (-not (Test-Path $Prg)) { throw "PRG not found: $Prg" }
$prgBytes = [System.IO.File]::ReadAllBytes($Prg)
Write-Output ("PRG: {0} bytes, load=0x{1:X2}{2:X2}" -f $prgBytes.Length, $prgBytes[1], $prgBytes[0])

$image = New-Object byte[] 174848

$payloadPerSector = 254
$numSectors = [Math]::Ceiling($prgBytes.Length / $payloadPerSector)

# Allocate sectors starting at track 17 sector 0, skipping the directory track 18.
$chain = New-Object System.Collections.Generic.List[object]
$t = 17; $s = 0
for ($i = 0; $i -lt $numSectors; $i++) {
    $chain.Add(@($t, $s))
    do {
        $s += 1
        if ($s -ge $SECTORS_PER_TRACK[$t]) {
            $s = 0
            $t += 1
            if ($t -eq $DIR_TRACK) { $t += 1 }
            if ($t -ge $SECTORS_PER_TRACK.Length) { throw "ran off the disk" }
        }
    } while ($t -eq $DIR_TRACK)
}

for ($i = 0; $i -lt $chain.Count; $i++) {
    $trk = $chain[$i][0]; $sec = $chain[$i][1]
    $off = Sector-Off $trk $sec
    $chunkStart = $i * $payloadPerSector
    $chunkEnd = [Math]::Min($chunkStart + $payloadPerSector, $prgBytes.Length)
    $chunkLen = $chunkEnd - $chunkStart
    if ($i + 1 -lt $chain.Count) {
        $image[$off]     = [byte]$chain[$i+1][0]
        $image[$off + 1] = [byte]$chain[$i+1][1]
    } else {
        $image[$off]     = 0
        $image[$off + 1] = [byte](1 + $chunkLen)
    }
    [Array]::Copy($prgBytes, $chunkStart, $image, $off + 2, $chunkLen)
}

# BAM at t18 s0
$bamOff = Sector-Off 18 0
$image[$bamOff]     = [byte]$DIR_TRACK
$image[$bamOff + 1] = 1
$image[$bamOff + 2] = [byte][char]'A'
$image[$bamOff + 3] = 0
for ($tr = 1; $tr -le 35; $tr++) {
    $off = $bamOff + 4 + ($tr - 1) * 4
    $image[$off]     = [byte]$SECTORS_PER_TRACK[$tr]
    $image[$off + 1] = 0xFF
    $image[$off + 2] = 0xFF
    $image[$off + 3] = 0xFF
}
for ($i = 0; $i -lt 27; $i++) { $image[$bamOff + 0x90 + $i] = 0xA0 }
$disk = To-Petscii $DiskName
$nLen = [Math]::Min($disk.Length, 16)
[Array]::Copy($disk, 0, $image, $bamOff + 0x90, $nLen)
$image[$bamOff + 0xA2] = [byte][char]'0'
$image[$bamOff + 0xA3] = [byte][char]'1'
$image[$bamOff + 0xA4] = 0xA0
$image[$bamOff + 0xA5] = [byte][char]'2'
$image[$bamOff + 0xA6] = [byte][char]'A'
$image[$bamOff + 0xA7] = 0xA0
$image[$bamOff + 0xA8] = 0xA0

# Directory at t18 s1
$dirOff = Sector-Off 18 1
$image[$dirOff]     = 0
$image[$dirOff + 1] = 0xFF
$entryOff = $dirOff + 2
$image[$entryOff]     = 0x82                          # closed PRG
$image[$entryOff + 1] = [byte]$chain[0][0]            # first track
$image[$entryOff + 2] = [byte]$chain[0][1]            # first sector
$leaf = [System.IO.Path]::GetFileNameWithoutExtension($Prg)
$fileName = To-Petscii ($leaf.ToUpper())
for ($i = 0; $i -lt 16; $i++) {
    if ($i -lt $fileName.Length) { $image[$entryOff + 3 + $i] = $fileName[$i] }
    else { $image[$entryOff + 3 + $i] = 0xA0 }
}
$image[$entryOff + 28] = [byte]($chain.Count -band 0xFF)
$image[$entryOff + 29] = [byte](($chain.Count -shr 8) -band 0xFF)

[System.IO.File]::WriteAllBytes($Out, $image)
Write-Output ("wrote {0} -> {1} ({2} sectors used)" -f $image.Length, $Out, $chain.Count)
