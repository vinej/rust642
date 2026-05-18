# rust64 launcher — pick a .d64, see its programs, click to play.
#
# Flow:
#   1. "Open D64..." -> file dialog -> we call d64_check list to populate the
#      ListBox.
#   2. Click a PRG -> "Run" extracts it to a temp .prg via d64_check extract
#      and launches rust64.exe with the 'run' flag so BASIC auto-RUNs it.
#
# Assumes the layout produced by the repo:
#   c:\jyv\rust\rust64\target\release\rust64.exe
#   c:\jyv\rust\d64_check\target\release\d64_check.exe
# Override by setting $env:RUST64_EXE / $env:D64_CHECK_EXE before launching.

$ErrorActionPreference = "Stop"

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# --- resolve binary locations -----------------------------------------------
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$rust64    = if ($env:RUST64_EXE)     { $env:RUST64_EXE }     else { Join-Path $scriptDir "rust64\target\release\rust64.exe" }
$d64check  = if ($env:D64_CHECK_EXE)  { $env:D64_CHECK_EXE }  else { Join-Path $scriptDir "d64_check\target\release\d64_check.exe" }

foreach ($p in @($rust64, $d64check)) {
    if (-not (Test-Path $p)) {
        [System.Windows.Forms.MessageBox]::Show(
            "Required binary not found:`n$p`n`nBuild rust64 and d64_check first (cargo build --release).",
            "rust64 launcher", "OK", "Error") | Out-Null
        exit 1
    }
}

# --- state ------------------------------------------------------------------
$script:currentD64 = ""
$script:entries    = @()   # parallel to $listBox.Items, holds Name/Size objects

# --- form -------------------------------------------------------------------
$form              = New-Object Windows.Forms.Form
$form.Text         = "rust64 launcher"
$form.StartPosition = "CenterScreen"
$form.Size         = New-Object Drawing.Size(520, 480)
$form.MinimumSize  = New-Object Drawing.Size(420, 320)

$openBtn           = New-Object Windows.Forms.Button
$openBtn.Text      = "Open D64..."
$openBtn.Location  = New-Object Drawing.Point(12, 12)
$openBtn.Size      = New-Object Drawing.Size(110, 28)
$openBtn.Anchor    = "Top, Left"
$form.Controls.Add($openBtn)

$pathLabel         = New-Object Windows.Forms.Label
$pathLabel.Text    = "(no disk loaded)"
$pathLabel.Location = New-Object Drawing.Point(132, 18)
$pathLabel.Size    = New-Object Drawing.Size(370, 20)
$pathLabel.Anchor  = "Top, Left, Right"
$pathLabel.AutoEllipsis = $true
$form.Controls.Add($pathLabel)

$listBox           = New-Object Windows.Forms.ListBox
$listBox.Location  = New-Object Drawing.Point(12, 50)
$listBox.Size      = New-Object Drawing.Size(490, 320)
$listBox.Anchor    = "Top, Left, Right, Bottom"
$listBox.Font      = New-Object Drawing.Font("Consolas", 10)
$listBox.IntegralHeight = $false
$form.Controls.Add($listBox)

$runBtn            = New-Object Windows.Forms.Button
$runBtn.Text       = "Run with rust64"
$runBtn.Location   = New-Object Drawing.Point(12, 380)
$runBtn.Size       = New-Object Drawing.Size(140, 32)
$runBtn.Anchor     = "Bottom, Left"
$runBtn.Enabled    = $false
$form.Controls.Add($runBtn)

# scale dropdown — rust64's bareword args are "x2" / "x4"; nothing for 1x
$scaleLabel        = New-Object Windows.Forms.Label
$scaleLabel.Text   = "Scale:"
$scaleLabel.Location = New-Object Drawing.Point(160, 388)
$scaleLabel.Size   = New-Object Drawing.Size(45, 20)
$scaleLabel.Anchor = "Bottom, Left"
$form.Controls.Add($scaleLabel)

$scaleBox          = New-Object Windows.Forms.ComboBox
$scaleBox.Location = New-Object Drawing.Point(205, 384)
$scaleBox.Size     = New-Object Drawing.Size(75, 24)
$scaleBox.Anchor   = "Bottom, Left"
$scaleBox.DropDownStyle = "DropDownList"
[void]$scaleBox.Items.AddRange(@("1x", "2x", "4x"))
$scaleBox.SelectedItem = "2x"
$form.Controls.Add($scaleBox)

$statusLabel       = New-Object Windows.Forms.Label
$statusLabel.Text  = "Open a D64 to begin."
$statusLabel.Location = New-Object Drawing.Point(290, 388)
$statusLabel.Size  = New-Object Drawing.Size(212, 20)
$statusLabel.Anchor = "Bottom, Left, Right"
$statusLabel.AutoEllipsis = $true
$form.Controls.Add($statusLabel)

# --- helpers ----------------------------------------------------------------
function Set-Status([string]$msg) { $statusLabel.Text = $msg }

function Update-RunButton {
    $runBtn.Enabled = ($script:currentD64 -ne "") -and ($listBox.SelectedIndex -ge 0)
}

function Get-SelectedName {
    if ($listBox.SelectedIndex -lt 0) { return $null }
    return $script:entries[$listBox.SelectedIndex].Name
}

function Load-D64([string]$path) {
    $listBox.Items.Clear()
    $script:entries    = @()
    $script:currentD64 = ""
    Set-Status "Reading directory..."

    # d64_check list outputs one TAB-separated "SIZE<TAB>NAME" per closed PRG.
    $output = & $d64check list $path 2>&1
    if ($LASTEXITCODE -ne 0) {
        [System.Windows.Forms.MessageBox]::Show(
            "Failed to read D64:`n$output", "rust64 launcher", "OK", "Error") | Out-Null
        Set-Status "Open a D64 to begin."
        return
    }

    foreach ($line in $output) {
        $parts = ([string]$line) -split "`t", 2
        if ($parts.Count -lt 2) { continue }
        $size = [int]$parts[0]
        $name = $parts[1].TrimEnd()
        $script:entries += [PSCustomObject]@{ Size = $size; Name = $name }
        $listBox.Items.Add(("{0,4}  {1}" -f $size, $name)) | Out-Null
    }

    $script:currentD64 = $path
    $pathLabel.Text = $path

    if ($listBox.Items.Count -eq 0) {
        Set-Status "No PRG files found on this disk."
    } else {
        $listBox.SelectedIndex = 0
        Set-Status ("{0} program(s). Double-click or press Run." -f $listBox.Items.Count)
    }
    Update-RunButton
}

function Run-Selected {
    $name = Get-SelectedName
    if (-not $name) { return }

    $tmp = Join-Path $env:TEMP ("rust64_launcher_{0}.prg" -f ([guid]::NewGuid().ToString("N").Substring(0, 8)))
    Set-Status ("Extracting '{0}'..." -f $name)
    $exOut = & $d64check extract $script:currentD64 $name $tmp 2>&1
    if ($LASTEXITCODE -ne 0) {
        [System.Windows.Forms.MessageBox]::Show(
            "Extract failed:`n$exOut", "rust64 launcher", "OK", "Error") | Out-Null
        Set-Status "Idle."
        return
    }

    Set-Status ("Launching rust64 with '{0}'..." -f $name)
    # rust64 reads rom/basic.rom etc. relative to CWD, so we need to launch
    # from the crate root (the parent of target\release\). For the standard
    # cargo layout that's three Split-Path -Parent hops up from the exe.
    $rust64Dir = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $rust64))

    $args = @($tmp, "run")
    switch ($scaleBox.SelectedItem) {
        "2x" { $args += "x2" }
        "4x" { $args += "x4" }
        # "1x" -> no scale arg (rust64's default)
    }
    Start-Process -FilePath $rust64 -ArgumentList $args -WorkingDirectory $rust64Dir | Out-Null
    Set-Status ("Running '{0}' @ {1}. (Close the emulator window when done.)" -f $name, $scaleBox.SelectedItem)
}

# --- events -----------------------------------------------------------------
$openBtn.Add_Click({
    $dlg = New-Object Windows.Forms.OpenFileDialog
    $dlg.Filter = "Commodore 64 disk images (*.d64)|*.d64|All files (*.*)|*.*"
    $dlg.Title  = "Choose a D64 image"
    if ($script:currentD64) {
        $dlg.InitialDirectory = Split-Path -Parent $script:currentD64
    } else {
        $dlg.InitialDirectory = $scriptDir
    }
    if ($dlg.ShowDialog() -eq "OK") { Load-D64 $dlg.FileName }
})

$listBox.Add_SelectedIndexChanged({ Update-RunButton })
$listBox.Add_DoubleClick({ if ($runBtn.Enabled) { Run-Selected } })
$runBtn.Add_Click({ Run-Selected })

# --- run --------------------------------------------------------------------
[Windows.Forms.Application]::EnableVisualStyles()
$form.Add_Shown({ $form.Activate() })
[Windows.Forms.Application]::Run($form)
