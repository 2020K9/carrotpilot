# Carrot Jetson 큰 설치파일 받기 (GitHub 1.9GB 조각 -> 하나로 합치고 검사)
# Downloads the 1.9 GB GitHub release parts, joins them into one file and checks
# every part, the signed manifest (pinned sha256) and the whole-file sha256.
#
# 사용법 / usage (PowerShell):
#   powershell -ExecutionPolicy Bypass -File get-carrot-jetson.ps1                 # v0.4.0 설치파일 ZIP
#   powershell -ExecutionPolicy Bypass -File get-carrot-jetson.ps1 -Item v032-zip  # v0.3.2 설치파일 ZIP
#   powershell -ExecutionPolicy Bypass -File get-carrot-jetson.ps1 -Item v040-img  # v0.4.0 이미지 .img.zst
param(
  [ValidateSet('v040-zip', 'v040-img', 'v032-zip')][string]$Item = 'v040-zip',
  [string]$OutDir = (Get-Location).Path
)
$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$Base = 'https://github.com/2020K9/carrotpilot/releases/download/'
# manifest = <name>.parts.json, signed with the 2020K9 release key (release_parts.py verifies the signature);
# its sha256 is pinned here so this script needs no extra crypto.
$Known = @{
  'v040-zip' = @{ Tag = 'k9-jetson-v0.4.0-boot-preview'; Name = 'carrot-jetson-windows.zip'; Size = 10098321699
                  Sha = 'b531393405eab54df134d51a3cad00f74430aca47e4068b855e92e2c58cc6be4'; Manifest = 'd3a05b672a8426291e86c3cc1dda62ad1d2bba4dddc49693f03e47b7a17591e8' }
  'v040-img' = @{ Tag = 'k9-jetson-v0.4.0-boot-preview'; Name = 'carrot-jetson.img.zst'; Size = 10073623634
                  Sha = 'b277f3c6318d5b3210eee6e6aa4e74e492f73ffe2f9a669e108822a7ceb6c614'; Manifest = '9a07efa8d3e50ac6d378949f889314ba5a0c908142586642890e3f024b5b2049' }
  'v032-zip' = @{ Tag = 'k9-jetson-v0.3.2-windows-preview'; Name = 'carrot-jetson-windows.zip'; Size = 8274619308
                  Sha = 'e11a0a1da0c7fc78ace17b527d0242c98d68a752888c072c6f18ed1151e506da'; Manifest = '2b48a20fdb05e4ce4b59965e7beb417c324e91df67194df90724e3e455f906a0' }
}
$k = $Known[$Item]
function Hex($bytes) { -join ($bytes | ForEach-Object { $_.ToString('x2') }) }

$manifestUrl = $Base + $k.Tag + '/' + $k.Name + '.parts.json'
Write-Host "목록 받는 중 / manifest: $manifestUrl"
$raw = (New-Object Net.WebClient).DownloadData($manifestUrl)
$sha = [Security.Cryptography.SHA256]::Create()
if ((Hex $sha.ComputeHash($raw)) -ne $k.Manifest) { throw '조각 목록(manifest) 검사 실패 / manifest sha256 mismatch' }
$m = [Text.Encoding]::UTF8.GetString($raw) | ConvertFrom-Json
if ($m.sha256 -ne $k.Sha -or [int64]$m.size -ne $k.Size) { throw '목록 내용이 예상과 다릅니다 / manifest content mismatch' }

$out = Join-Path $OutDir $k.Name
$tmp = $out + '.partial'
$free = (Get-PSDrive -Name (Split-Path -Qualifier $out).TrimEnd(':')).Free
if ($free -lt $k.Size + 1GB) { throw ("여유 공간 부족 / not enough free space: {0:N1} GB 필요" -f (($k.Size + 1GB) / 1GB)) }
$whole = [Security.Cryptography.SHA256]::Create()
$fs = [IO.File]::Open($tmp, 'Create', 'ReadWrite')
try {
  $buf = New-Object byte[] (1MB)
  foreach ($p in $m.parts) {
    $ok = $false
    for ($try = 1; $try -le 4 -and -not $ok; $try++) {
      $fs.SetLength([int64]$p.offset); $fs.Position = [int64]$p.offset
      $ph = [Security.Cryptography.SHA256]::Create(); $n = [int64]0
      try {
        $req = [Net.HttpWebRequest]::Create([string]$p.url); $req.UserAgent = 'carrot-release-parts'; $req.Timeout = 60000
        $resp = $req.GetResponse(); $s = $resp.GetResponseStream()
        while (($r = $s.Read($buf, 0, $buf.Length)) -gt 0) {
          $fs.Write($buf, 0, $r); [void]$ph.TransformBlock($buf, 0, $r, $null, 0); $n += $r
          Write-Progress -Activity $k.Name -Status ("{0} {1:N2} GB" -f $p.name, (([int64]$p.offset + $n) / 1e9)) -PercentComplete ((([int64]$p.offset + $n) * 100) / $k.Size)
        }
        $resp.Close()
        [void]$ph.TransformFinalBlock($buf, 0, 0)
        if ($n -ne [int64]$p.size -or (Hex $ph.Hash) -ne $p.sha256) { throw ('조각 검사 실패 / part mismatch ' + $p.name) }
        $ok = $true
      } catch { Write-Warning "$($p.name) 다시 받기 / retry ($try): $_"; Start-Sleep -Seconds (5 * $try) }
    }
    if (-not $ok) { throw ('조각을 받지 못했습니다 / failed ' + $p.name) }
    Write-Host ("조각 확인 OK: {0}" -f $p.name)
  }
  $fs.Position = 0
  $hash = Hex $whole.ComputeHash($fs)
} finally { $fs.Close() }
if ((Get-Item $tmp).Length -ne $k.Size -or $hash -ne $k.Sha) { Remove-Item $tmp; throw '합친 파일 검사 실패 / joined file sha256 mismatch' }
Move-Item -Force $tmp $out
Write-Host "완료 / done: $out"
Write-Host "SHA256 $hash (확인됨 / verified)"
