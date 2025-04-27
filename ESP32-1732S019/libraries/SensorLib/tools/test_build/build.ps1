# Set-Location ..

$currentDir = Get-Location

Write-Host "Current path :  $currentDir !"


$examples = Get-ChildItem -Path "examples" -Directory -Name

$envs = @(
    "esp32s3",
    "esp32c3",
    "esp32dev",
    "rp2040",
    "nrf52840"
)

platformio run -t clean

foreach ($env in $envs) {
    foreach ($example in $examples) {
        $skipFile = "examples/$example/.skip.$env"
        if (Test-Path $skipFile) {
            Write-Host "Skip $example for $env"
            continue
        }

        $env:PLATFORMIO_SRC_DIR = "examples/$example"
        Write-Host "PLATFORMIO_SRC_DIR=$env:PLATFORMIO_SRC_DIR , ENV: $env" 

        platformio run -e $env 
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Build env: $env $env:PLATFORMIO_SRC_DIR Failed!"
            exit 1
        } else {
            Write-Host "Build env: $env $env:PLATFORMIO_SRC_DIR Successed!"
        }
    }
}