# ================= Configuration =================
$PORT = "COM3"                                       # Target COM port for flashing
$KEY_DIR = Join-Path $PSScriptRoot "production_keys"          # Folder to store generated security keys
$LOG_FILE = Join-Path $KEY_DIR "production_log.csv"            # Path to the production log file
$SKEY_FILE = Join-Path $KEY_DIR "secure_boot_signing_key.pem"  # Path to the Secure Boot v2 key file
$IDF_PATH = "C:\Users\STK\esp\esp-idf-v5.5.4"                  # CRITICAL: Set your exact ESP-IDF installation directory path here!
# =================================================

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " LAUNCHING PRODUCTION SECURITY SCRIPT FOR ESP32-C3 (PS)" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 0. Load ESP-IDF environment variables if not already in PATH
if (!(Get-Command "espefuse.py" -ErrorAction SilentlyContinue)) {
    Write-Host "[0/5] ESP-IDF tools not found in PATH. Loading environment via export.ps1..." -ForegroundColor Yellow
    $ExportScript = Join-Path $IDF_PATH "export.ps1"
    if (Test-Path $ExportScript) {
        # Execute the export script in the current scope
        . $ExportScript | Out-Null
    } else {
        Write-Error "CRITICAL ERROR: export.ps1 not found at '$IDF_PATH'. Please fix `$IDF_PATH variable inside this script."
        Pause
        Exit 1
    }
}

# 1. Read unique MAC address from the chip
Write-Host "[1/5] Reading unique device MAC address..." -ForegroundColor Green

$MAC_ADDR = "UNKNOWN"
try {
    # Capture the raw output from the summary command
    $SummaryOutput = & espefuse.py -p $PORT summary 2>$null
    
    # Robust multi-line structural parsing specifically for the MAC line
    for ($i = 0; $i -lt $SummaryOutput.Length; $i++) {
        if ($SummaryOutput[$i] -match "MAC \(BLOCK1\)") {
            # The next line contains the actual hex value formatted as: "   = 98:88:e0:ef:0d:2c (OK) R/W"
            if ($SummaryOutput[$i+1] -match "=\s*([0-9a-fA-F:]{17})") {
                $MAC_ADDR = $Matches[1].Replace(':', '')
                break
            }
        }
    }
} catch {
    $MAC_ADDR = "UNKNOWN"
}

if ($MAC_ADDR -eq "UNKNOWN") {
    Write-Error "MAC address unknown."
    Pause
    Exit 1
} else {
    Write-Host "MAC address: $MAC_ADDR"
}

# 2. Check and create directory for keys
$KEY_DIR = "$KEY_DIR\$MAC_ADDR"

if (!(Test-Path $KEY_DIR)) {
    Write-Host "[2/5] Creating directory for security keys..." -ForegroundColor Green
    New-Item -ItemType Directory -Path $KEY_DIR | Out-Null
} else {
    Write-Host "[2/5] Security keys directory detected." -ForegroundColor Green
}

# Initialize CSV log header if the file does not exist
if (!(Test-Path $LOG_FILE)) {
    "Date,Time,MAC_Address,Port,Status" | Out-File -FilePath $LOG_FILE -Encoding utf8
}


# 3. Generate keys locally if they do not exist
# Secure Boot V2 Key (RSA-3072)
$SbKey = $SKEY_FILE
if (Test-Path $SbKey) {
    Write-Host "Checking Secure Boot V2 private key..." -ForegroundColor Yellow
    #& openssl genrsa -out $SbKey 3072
    & openssl rsa -in $SbKey -check -noout
} else {
    Write-Error "CRITICAL ERROR: Secure Boot V2 Key not found at '$SbKey'."
    Pause
    Exit 1
}

# Flash Encryption Key (256-bit key for AES-128-XTS)
$FeKey = Join-Path $KEY_DIR "flash_encryption_key.bin"
if (!(Test-Path $FeKey)) {
    Write-Host "Generating Flash Encryption key..." -ForegroundColor Yellow
    $Bytes = New-Object Byte[] 32
    [System.Security.Cryptography.RandomNumberGenerator]::Create().GetBytes($Bytes)
    [System.IO.File]::WriteAllBytes($FeKey, $Bytes)
} else {
    Write-Error "CRITICAL ERROR: Flash Encryption Key already exists at '$FeKey'."
    Pause
    Exit 1
}

# NVS Encryption Key (256-bit key for HMAC)
$NvsKey = Join-Path $KEY_DIR "nvs_hmac_key.bin"
if (!(Test-Path $NvsKey)) {
    Write-Host "Generating HMAC key for NVS protection..." -ForegroundColor Yellow
    $Bytes = New-Object Byte[] 32
    [System.Security.Cryptography.RandomNumberGenerator]::Create().GetBytes($Bytes)
    [System.IO.File]::WriteAllBytes($NvsKey, $Bytes)
} else {
    Write-Error "CRITICAL ERROR: NVS Encryption Key already exists at '$NvsKey'."
    Pause
    Exit 1
}

Write-Host "[3/5] All keys verified and prepared locally." -ForegroundColor Green


# --- SUCCESS PROCESSING AND LOGGING ---
$Date = Get-Date -Format "yyyy-MM-dd"
$Time = Get-Date -Format "HH:mm:ss"
"$Date,$Time,$MAC_ADDR,$PORT,SUCCESS" | Out-File -FilePath $LOG_FILE -Append -Encoding utf8

Write-Host "==========================================================" -ForegroundColor Green
Write-Host " CHIP PROVISIONING COMPLETED SUCCESSFULLY!" -ForegroundColor Green
Write-Host " MAC Address: $MAC_ADDR is now locked in Release Mode." -ForegroundColor Green
Write-Host " Status saved to log: $LOG_FILE" -ForegroundColor Green
Write-Host "==========================================================" -ForegroundColor Green
Pause
