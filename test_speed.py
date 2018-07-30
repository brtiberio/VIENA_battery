import numpy as np
import timeit

def using_take(buffer, val):
    buffer = np.append(np.take(buffer, np.arange(1, 15)), val)
    return buffer

def using_roll(buffer, val):
    buffer = np.roll(buffer, -1)
    buffer[-1] = val
    return buffer

def test_roll():
    buffer = np.zeros(15, dtype=np.uint8)
    for I in np.arange(100):
        buffer =using_roll(buffer, I)

def test_take():
    buffer = np.zeros(15, dtype=np.uint8)
    for I in np.arange(100):
        buffer =using_take(buffer, I)

def main():
    print(timeit.timeit(test_take, number=100) / 100.0)
    print(timeit.timeit(test_roll, number=100) / 100.0)

if __name__ == '__main__':
    main()