
cd ../../

pwd

examples=($(find examples/* -maxdepth 1 -type d -printf "%f\n"))

envs=(
    "esp32s3"
    "esp32c3"
    "esp32dev"
    "rp2040"
    "nrf52840"
    )

pio run -t clean

for env in ${envs[@]}
do
    for value in ${examples[@]}
    do
        if [ -f "$value/.skip."$env ];then
            echo "Skip" $value
            continue
        fi

        export PLATFORMIO_SRC_DIR="examples/$value"
        echo "PLATFORMIO_SRC_DIR=$PLATFORMIO_SRC_DIR , ENV: $env" 
        pio run -e $env 
        if [ $? -ne 0 ]; then
            echo "Build env: $env $PLATFORMIO_SRC_DIR Failed!"
            exit -1
        else
            echo "Build env: $env $PLATFORMIO_SRC_DIR Successed!"
        fi
    done
done










