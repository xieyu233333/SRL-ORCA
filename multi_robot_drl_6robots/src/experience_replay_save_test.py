#!/usr/bin/env python

import numpy as np
import struct
from datetime import datetime
import time
from tqdm import tqdm

f = open("experience_replay.bin", "wb+")
data = []
start_t = datetime.now()
for i in range(1024*30*1):
    data.append(None)
    data[i] = (np.array(np.random.random(382), dtype=np.float32), np.array(np.random.random(2), dtype=np.float32), np.float32(np.random.random(1)[0]), np.array(np.random.random(382), dtype=np.float32), False)
print((datetime.now() - start_t).total_seconds())
start_t = datetime.now()
f.write(struct.pack('f', len(data)))
for i in tqdm(range(len(data))):
    d = data[i]
    tmp = list(d[0]) + list(d[1]) + list([d[2]]) + list(d[3]) + list([d[4]])
    tmp = struct.pack(("%df" % len(tmp)), *tmp)
    f.write(tmp)

f.close()
print((datetime.now() - start_t).total_seconds())

start_t = datetime.now()
f = open("experience_replay.bin", "rb+")
# raw = f.read()
# len_s = len(raw)
# raw = struct.unpack("%df" % (len_s / 4), raw)
# data1 = []
# for i in tqdm(range(int(len(raw) / 768))):
#     data1.append(None)
#     data1[i] = (np.array(raw[i * 768 + 0:i * 768 + 382]), np.array(raw[i * 768 + 382:i * 768 + 384]), raw[i * 768 + 384], np.array(raw[i * 768 + 385:i * 768 + 767]), bool(raw[i * 768 + 767]))
# raw = f.read()
# len_s = len(raw)
# raw = struct.unpack("%df" % (len_s / 4), raw)
# data1 = []
# for i in tqdm(range(int(len(raw) / 768))):
#     data1.append(None)
#     data1[i] = (np.array(raw[i * 768 + 0:i * 768 + 382]), np.array(raw[i * 768 + 382:i * 768 + 384]), raw[i * 768 + 384], np.array(raw[i * 768 + 385:i * 768 + 767]), bool(raw[i * 768 + 767]))
# print(len(data1))
state_dim = 382
action_dim = 2

data1 = []
num = f.read(4)
if len(num) == 4:
    num = int(struct.unpack("f", num)[0])
print("num:", num)
for i in tqdm(range(num)):
    raw = f.read(768 * 4)
    if len(raw) != 768 * 4:
        print("ERROR")
    else:
        raw = struct.unpack("%df" % (len(raw) / 4), raw)
        data1.append(None)
        data1[len(data1) - 1] = (np.array(raw[0:state_dim]), np.array(raw[state_dim:(state_dim+action_dim)]), raw[state_dim+action_dim], np.array(raw[(state_dim+action_dim+1):(state_dim*2+action_dim+1)]), bool(raw[state_dim*2+action_dim+1]))
f.close()
print((datetime.now() - start_t).total_seconds())
for i in range(10):
    a = np.random.randint(len(data))
    print(data[a][1][1], data1[a][1][1])
print(len(data), " ", len(data1))
# print(np.array(data))