import time


start = time.time()
time.sleep(0.005) # sleep for one millisecond
end = time.time()

elapsed_time = end - start

elapsed_time *= 1000

print(elapsed_time)
