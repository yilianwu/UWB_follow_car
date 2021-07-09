from steppyr import StepperController, DIRECTION_CW, DIRECTION_CCW
from steppyr.profiles.accel import AccelProfile
from steppyr.drivers.stepdir import StepDirDriver

import asyncio
from contextlib import suppress
from concurrent.futures import ThreadPoolExecutor
import math
import uwb_data as uwb

wheel_diam = 12.5
wheel_to_wheel = 33.0
turn_around_dist = (wheel_to_wheel * math.pi) / 2
ppr = 800.0 # pulse/rev
rpm = 240.0
std_distance = 50.0
distanceToFollow = 0
stepToFollow = 0
max_speed = (rpm * ppr) / 60
accel = 400
gta_count = 0
gta_value = 0
gta_last = 0
turning = False
stepToTurn = 0
avg_count = 0
avg_last = 0
avg_sum = 0
avg_cur = 0

stp_left = StepperController(
    profile=AccelProfile(), 
    driver=StepDirDriver(
        dir_pin=6, 
        step_pin=5,
    )
)

stp_right = StepperController(
    profile=AccelProfile(), 
    driver=StepDirDriver(
        dir_pin=24, 
        step_pin=23,
    )
)

stp_left.activate()
stp_right.activate()
stp_left.set_target_acceleration(accel)
stp_left.set_target_speed(max_speed)
stp_right.set_target_acceleration(accel)
stp_right.set_target_speed(max_speed)

def avg_dist(d):
    global avg_count
    global avg_last
    global avg_sum
    global avg_cur 

    if avg_count >= 5:
        avg_last = avg_sum / 5
        avg_sum = 0
        avg_count = 0
    avg_sum += d
    avg_count += 1

    avg_cur = avg_sum / avg_count
    return ((avg_last + avg_cur) / 2)

def ctrl_dir(ang):

    if ang <= -80 or ang >=80:
        speed_factor = 0.04
    elif ang <= -70 or ang >= 70:
        speed_factor = 0.08
    elif ang <= -60 or ang >= 60:
        speed_factor = 0.15
    elif ang <= -50 or ang >= 50:
        speed_factor = 0.25
    elif ang <= -40 or ang >= 40:
        speed_factor = 0.4
    elif ang <= -30 or ang >= 30:
        speed_factor = 0.6
    elif ang <= -20 or ang >= 20:
        speed_factor = 0.7
    elif ang <= -8 or ang >= 8:
        speed_factor = 0.8
    else:
        speed_factor = 1

    if ang > 0 :
        return {'left':speed_factor, 'right':1.0}
        
    else:
        return {'left':1.0, 'right':speed_factor}

def loop():
    global turning
    global avg_distance
    global stepToTurn
    global turn_around_dist
    global distanceToFollow
    global stepToFollow
    
    while True:
        angual, distance = uwb.ser_read()
        print("angual: {}, distance: {}".format(angual,distance))
        avg_distance = avg_dist(distance)

        if turning : #確認是否掉頭完畢
            if stp_left.steps_to_go == 0 and stp_right.steps_to_go == 0: #左右馬達執行完畢
                turning = False
            else: #左右馬達還在執行
                continue #跳過下面的命令 回到while True迴圈繼續檢查掉頭

        if gotta_turn_around(avg_distance): #需要執行掉頭
            turning = True
            stepToTurn = (turn_around_dist * ppr) / (wheel_diam * math.pi)
            stp_left.set_target_speed(max_speed) #left_wheel setMaxSpeed
            stp_left.move(stepToTurn)
            stp_right.set_target_speed(-max_speed) #right_wheel setMaxSpeed
            stp_right.move(stepToTurn)

        else: #不需要執行掉頭
            if avg_distance > std_distance:
                distanceToFollow = avg_distance - std_distance
                stepToFollow = (distanceToFollow * ppr) / (wheel_diam * math.pi)
                factor = ctrl_dir(angual)
                stp_left.set_target_speed(factor['left']*max_speed) #left_wheel setMaxSpeed
                stp_left.move(stepToFollow)
                stp_right.set_target_speed(factor['right']*max_speed) #right_wheel setMaxSpeed
                stp_right.move(stepToFollow)
            else:
                stp_left.stop()
                stp_right.stop()
        
def gotta_turn_around(d):
    global gta_count
    global gta_value
    global gta_last
    dist_diff = 0
    gta_count += 1

    if gta_count >= 5:
        dist_diff = d - gta_last
        if dist_diff > 4:
            gta_value += 1 # 累積壞寶寶標籤
        elif dist_diff < 0:
            gta_value = 0 # 追上才給好寶寶貼紙reset
        gta_last = d
        gta_count = 0

        return (gta_value >= 5)
    else:
        return False

async def main(): #定義main()為一個協同程序/協程(coroutine)
    evloop = asyncio.get_event_loop() #建立一個Event Loop
    user_task = evloop.run_in_executor(None, loop)
    task1 = asyncio.create_task(stp_left.run_forever()) # 原本的motorRun()
    task2 = asyncio.create_task(stp_right.run_forever()) # 原本的motorRun()
    
    await user_task

    task1.cancel()
    task2.cancel()
    with suppress(asyncio.CancelledError): #強制結束task1, task2 忽略exception(Cancel error)
        await task1
        await task2
    stp_left.shutdown()
    stp_right.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
