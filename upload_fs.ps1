# =============================================================================
#  DendyGO: сборка образа LittleFS из папки data и заливка его в ESP32-S3
# =============================================================================
#  Запуск (из папки скетча):
#     powershell -ExecutionPolicy Bypass -File .\upload_fs.ps1
#     powershell -ExecutionPolicy Bypass -File .\upload_fs.ps1 -Port COM4
#     powershell -ExecutionPolicy Bypass -File .\upload_fs.ps1 -NoFlash   # только собрать
#
#  Скрипт сам:
#    * находит mklittlefs.exe и esptool.exe в установленном ядре esp32;
#    * ПРОВЕРЯЕТ имена файлов: mklittlefs не умеет имена длиннее 32 символов
#      (на таком файле он обрывается и в LittleFS не попадает ничего);
#    * берёт адрес и размер раздела spiffs из partitions.csv;
#    * собирает littlefs.bin и (если не указан -NoFlash) заливает его в плату.
# =============================================================================
param(
  [string]$Port = '',
  [switch]$NoFlash
)

$ErrorActionPreference = 'Stop'
$proj = $PSScriptRoot
$data = Join-Path $proj 'data'
$part = Join-Path $proj 'partitions.csv'
$img  = Join-Path $proj 'littlefs.bin'

function Info($s) { Write-Host $s }
function Fail($s) { Write-Host $s -ForegroundColor Red; exit 1 }
function Ok($s)   { Write-Host $s -ForegroundColor Green }

if (-not (Test-Path $data)) { Fail "Нет папки data: $data" }
if (-not (Test-Path $part)) { Fail "Нет partitions.csv: $part" }

# --- 1. Инструменты ядра esp32 ------------------------------------------------
$pkg = Join-Path $env:LOCALAPPDATA 'Arduino15\packages\esp32\tools'
$mk  = Get-ChildItem $pkg -Recurse -Filter 'mklittlefs.exe' -ErrorAction SilentlyContinue |
       Sort-Object FullName | Select-Object -Last 1 -ExpandProperty FullName
$esp = Get-ChildItem $pkg -Recurse -Filter 'esptool.exe' -ErrorAction SilentlyContinue |
       Sort-Object FullName | Select-Object -Last 1 -ExpandProperty FullName
if (-not $mk)  { Fail "mklittlefs.exe не найден в $pkg (установлено ли ядро esp32?)" }
if (-not $esp) { Fail "esptool.exe не найден в $pkg" }
Info "mklittlefs : $mk"
Info "esptool    : $esp"

# --- 2. Проверка длин имён (лимит mklittlefs — 32 символа) --------------------
$bad = @()
Get-ChildItem $data -Recurse -File | ForEach-Object {
  $rel = $_.FullName.Substring($data.Length + 1)
  if ($_.Name.Length -gt 32) { $bad += ("{0}  ({1} симв.)" -f $rel, $_.Name.Length) }
}
if ($bad.Count -gt 0) {
  Write-Host ''
  Fail ("Длинные имена (>32 символов) — mklittlefs на них падает:`n  " +
        ($bad -join "`n  ") + "`n`nПереименуйте файлы короче и запустите снова.")
}
$roms = Get-ChildItem (Join-Path $data 'roms') -Filter '*.nes' -ErrorAction SilentlyContinue
Info ("рома(ов) в data\roms: " + @($roms).Count)

# --- 3. Раздел spiffs из partitions.csv ---------------------------------------
$p = $null
Get-Content $part | ForEach-Object {
  $c = ($_ -split ',') | ForEach-Object { $_.Trim() }
  if ($c.Count -ge 6 -and $c[2] -eq 'spiffs') {
    $p = [pscustomobject]@{ Name = $c[0]; Offset = $c[3]; Size = $c[4] }
  }
}
if (-not $p) { Fail "В partitions.csv нет раздела со subtype = spiffs" }
$size   = [Convert]::ToInt32($p.Size,   16)
$offset = [Convert]::ToInt32($p.Offset, 16)
Info ("раздел '{0}': адрес 0x{1:X6}, размер {2} байт ({3} КБ)" -f `
      $p.Name, $offset, $size, [int]($size / 1024))

# --- 4. Образ LittleFS ---------------------------------------------------------
if (Test-Path $img) { Remove-Item $img -Force }
$out = & $mk -c $data -p 256 -b 4096 -s $size $img 2>&1
Info ($out -join "`n")
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $img)) {
  Fail "mklittlefs завершился с ошибкой (код $LASTEXITCODE). Смотрите сообщения выше."
}
Ok ("образ готов: {0} ({1} байт)" -f $img, (Get-Item $img).Length)
Info "содержимое образа:"
Info ((& $mk -l -p 256 -b 4096 -s $size $img 2>&1) -join "`n")

if ($NoFlash) { Ok 'Ключ -NoFlash: образ собран, но не залит.'; exit 0 }

# --- 5. Заливка ----------------------------------------------------------------
if (-not $Port) {
  $ports = [System.IO.Ports.SerialPort]::GetPortNames()
  if ($ports.Count -eq 1) { $Port = $ports[0] }
  elseif ($ports.Count -eq 0) { Fail "COM-порты не найдены. Подключите плату или задайте -Port COMx" }
  else { Fail ("Найдено несколько портов: " + ($ports -join ', ') + ". Задайте -Port COMx") }
}
Info "закрываю порт в IDE? Заливаю через $Port ..."
$out = & $esp --chip esp32s3 --port $Port --baud 921600 --before default-reset `
              --after hard-reset write-flash $p.Offset $img 2>&1
Info ($out -join "`n")
if ($LASTEXITCODE -ne 0) { Fail "esptool завершился с ошибкой (код $LASTEXITCODE)" }
Ok "Готово: образ LittleFS залит в $Port. Нажмите RST — в меню появится список ромов."
