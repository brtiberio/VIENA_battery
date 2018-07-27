import numpy as np



class RingBuffer():
    "A 1D ring buffer using numpy arrays"
    def __init__(self, length):
        self.data = np.zeros(length, dtype='f')
        self.index = 0

    def put(self, x):
        "adds array x to ring buffer"
        x_index = (self.index + np.arange(x.size)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

    def get(self):
        "Returns the first-in-first-out data in the ring buffer"
        idx = (self.index + np.arange(self.data.size)) % self.data.size
        return self.data[idx]


def ringbuff_numpy_test():
    ringlen = 100000
    ringbuff = RingBuffer(ringlen)
    for i in range(40):
        ringbuff.put(np.zeros(10000, dtype='f'))  # write
        ringbuff.get()  # read


def test_roll():
    ringlen = 100000
    step = 10000
    for i in range(40):
        ringbuff = np.zeros(ringlen, dtype='f')
        np.roll(ringbuff, step)
        ringbuff[:step] = np.zeros(step, dtype='f')


def main():
    import timeit
    print(timeit.timeit(ringbuff_numpy_test, number=10) / 10.0)
    print(timeit.timeit(test_roll, number=10) / 10.0)


if __name__ == '__main__':
    main()
