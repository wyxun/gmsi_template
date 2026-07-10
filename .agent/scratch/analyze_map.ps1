$content = Get-Content -Path "build\template.map"
$sizes = @{}

foreach ($line in $content) {
    if ($line -match "^\s*([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+\d+\s+(build/|third_party/|vendor/|D:\\)?([^ \(\):]+\.[oa](?:bj)?)(?:\(|:|\s|$)") {
        $vmaHex = $Matches[1]
        $sizeHex = $Matches[2]
        $objName = $Matches[4]

        try {
            $vma = [Convert]::ToInt64($vmaHex, 16)
            if ($vma -ge 134217728) {
                $size = [Convert]::ToInt32($sizeHex, 16)
                if ($size -gt 0) {
                    $sizes[$objName] += $size
                }
            }
        } catch {}
    }
}

$sorted = $sizes.GetEnumerator() | Sort-Object Value -Descending
Write-Host "Top object files by size in FLASH:"
Write-Host "=================================="
$sorted | Select-Object -First 40 | ForEach-Object {
    "{0,-60} : {1,8} bytes" -f $_.Key, $_.Value
}
