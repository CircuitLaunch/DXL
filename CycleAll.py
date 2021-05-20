import time
from DXL import *

port = DXLPort("/dev/ttyUSB2", 1000000)

def callback(dxl, result, error):
    if result != 0 or error != 0:
        print(f"dxl_{dxl.id} comm {port.resultString(result)}, error [{port.errorString(error)}]")

rightArmIds = [10, 11, 12, 13, 14, 15, 16, 17]
leftArmIds = [20, 21, 22, 23, 24, 25, 26, 27]

dxls = { i:port.getDXL(i, callback) for i in rightArmIds + leftArmIds }

dxls[24].offset = -12
for dxl in dxls.values():
    dxl.torqueEnable = 1
    dxl.returnDelayTime = 0

for i, dxl in dxls.items():
    print(f'dxl_{i} present position: {dxl.presentPosition}')

for i in range(0, 45):
    dxls[10].goalPosition = -90.0 + float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[10].goalPosition = -45.0 - float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[11].goalPosition = -90.0 + float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[11].goalPosition = -45.0 - float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[12].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 90):
    dxls[12].goalPosition = 45.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 45):
    dxls[12].goalPosition = -45 + float(i)
    # time.sleep(0.02)

for i in range(0, 90):
    dxls[13].goalPosition = float(i)
    # time.sleep(0.03125)

for i in range(0, 90):
    dxls[13].goalPosition = 90.0 - float(i)
    # time.sleep(0.03125)

for i in range(0, 45):
    dxls[14].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 90):
    dxls[14].goalPosition = 45.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 45):
    dxls[14].goalPosition = -45 + float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[15].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 60):
    dxls[15].goalPosition = 30.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[15].goalPosition = -30 + float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[16].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 60):
    dxls[16].goalPosition = 30.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[16].goalPosition = -30.0 + float(i)
    # time.sleep(0.02)

for i in range(0, 45):
    dxls[17].goalPosition = -float(i)
    # time.sleep(0.02)

for i in range(0, 65):
    dxls[17].goalPosition = -45.0 + float(i)
    # time.sleep(0.02)

for i in range(0, 20):
    dxls[17].goalPosition = 20.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 45):
    dxls[20].goalPosition = 90.0 - float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[20].goalPosition = 45.0 + float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[21].goalPosition = 90.0 - float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[21].goalPosition = 45.0 + float(i)
    time.sleep(0.02)

for i in range(0, 45):
    dxls[22].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 90):
    dxls[22].goalPosition = 45.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 45):
    dxls[22].goalPosition = -45 + float(i)
    # time.sleep(0.02)

for i in range(0, 90):
    dxls[23].goalPosition = float(i)
    # time.sleep(0.03125)

for i in range(0, 90):
    dxls[23].goalPosition = 90.0 - float(i)
    # time.sleep(0.03125)

for i in range(0, 45):
    dxls[24].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 90):
    dxls[24].goalPosition = 45.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 45):
    dxls[24].goalPosition = -45 + float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[25].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 60):
    dxls[25].goalPosition = 30.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[25].goalPosition = -30 + float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[26].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 60):
    dxls[26].goalPosition = 30.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 30):
    dxls[26].goalPosition = -30 + float(i)
    # time.sleep(0.02)

for i in range(0, 45):
    dxls[27].goalPosition = float(i)
    # time.sleep(0.02)

for i in range(0, 65):
    dxls[27].goalPosition = 45.0 - float(i)
    # time.sleep(0.02)

for i in range(0, 20):
    dxls[27].goalPosition = -20.0 + float(i)
    # time.sleep(0.02)

for dxl in dxls.values():
    dxl.torqueEnable = 0
    dxl.returnDelayTime = 0
