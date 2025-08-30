#!/usr/bin/env python3
import os, asyncio, time
from mavsdk import System
from mavsdk.action import ActionError

SERIAL_URI = os.getenv("SERIAL_URI","serial:///dev/ttyTHS1:921600")
CONNECT_TIMEOUT = 20  # 秒

async def wait_connected(drone, timeout=CONNECT_TIMEOUT):
    print(f"等待連線到 {SERIAL_URI} ...", flush=True)
    t0 = time.time()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ 已連線到飛控", flush=True)
            return True
        if time.time() - t0 > timeout:
            print("⏰ 連線逾時，仍未收到心跳", flush=True)
            return False

async def main():
    print(f"MAVSDK 連線測試，URI={SERIAL_URI}", flush=True)
    drone = System()
    await drone.connect(system_address=SERIAL_URI)

    if not await wait_connected(drone):
        print("🔎 請檢查：TX/RX/共地、鮑率一致、宿主機是否有程式佔用埠、docker 是否有 --device", flush=True)
        return

    # 讀一次 health（5 秒 timeout）
    try:
        h = await asyncio.wait_for(drone.telemetry.health().__anext__(), timeout=5)
        print(f"Health: gyro={h.is_gyrometer_calibration_ok}, accel={h.is_accelerometer_calibration_ok}, mag={h.is_magnetometer_calibration_ok}", flush=True)
    except asyncio.TimeoutError:
        print("⚠️ 5 秒內沒有收到 health 遙測（可能剛連上、還未穩定）", flush=True)

    # Arm/Disarm（請務必拆槳）
    try:
        print("➡️  Arm", flush=True)
        await drone.action.arm()
        await asyncio.sleep(1)
        print("⬅️  Disarm", flush=True)
        await drone.action.disarm()
        print("✅ 測試完成", flush=True)
    except ActionError as e:
        print(f"❌ Action 失敗：{e}", flush=True)

if __name__ == "__main__":
    asyncio.run(main())
