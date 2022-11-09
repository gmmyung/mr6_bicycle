import asyncio
import math
import mpu6050

async def periodic():
    while True:
        print('periodic')
        await asyncio.sleep(1)

def stop():
    task.cancel()

loop = asyncio.get_event_loop()
loop.call_later(5, stop)
task = loop.create_task(periodic())

try:
    loop.run_until_complete(task)
except asyncio.CancelledError:
    pass

async def main_async():
    
    await do_async()

a = 1

print(a)
a+=1