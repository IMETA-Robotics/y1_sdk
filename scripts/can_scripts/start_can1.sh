#!/bin/bash

CAN_DEVICE="/dev/imeta_y1_can1"
CAN_INTERFACE="can1"

start_can() {
    echo "启动 slcand..."
    sudo slcand -o -f -s8 $CAN_DEVICE $CAN_INTERFACE
    if [ $? -ne 0 ]; then
        echo "slcand 启动失败"
        return 1
    fi
    echo "配置 $CAN_INTERFACE 接口..."

    sudo ifconfig $CAN_INTERFACE up
    sudo ip link set $CAN_INTERFACE txqueuelen 1000
    
    if [ $? -ne 0 ]; then
        return 1
    fi
    echo "$CAN_INTERFACE 启动成功"
    return 0
}

check_can() {

    if ip link show "$CAN_INTERFACE" > /dev/null 2>&1; then
  
        if ip link show "$CAN_INTERFACE" | grep -q "UP"; then
            return 0
        else
            return 1  
        fi
    else
        return 2  
    fi
}

while true; do

    if check_can; then

        echo "CAN 接口 $CAN_INTERFACE 正常工作"
        # break
    else

        echo "$CAN_INTERFACE 掉线，重启中..."
        
        sudo ip link set $CAN_INTERFACE down
        sudo pkill -9 slcand  
        sleep 1  

        if ! start_can; then
            echo "重启 CAN 接口失败，请检查USB2CAN是否连接。"

        fi
    fi

    sleep 1

done