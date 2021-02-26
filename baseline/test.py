import itertools
import random
import numpy as np

# print(list(itertools.product([i for i in range(5)], repeat=2)))
# test= [[1 for i in range(2)] for i in range(2)]
# print(test)
# print(-float("Inf")+4)

for i in range(1000):
    nums = []
    while len(nums) < 4:
        new_num = random.randint(0,100)
        if new_num not in nums:
            nums.append(new_num)

    probs = []
    for i in range(4):
        probs.append(random.uniform(0,1))

    order = list(itertools.permutations([0,1,2,3]))

    best_order = None
    high_val = -float("inf")
    for (i,j,k,l) in order:
        val = probs[i]*nums[i] + (1 - probs[i]) * (probs[j]*nums[j] + (1 - probs[j]) * (probs[k]*nums[k] + (1 - probs[k]) * (probs[l]*nums[i])))
        if val > high_val:
            best_order = (i,j,k,l)
            high_val = val
    best_order = list(best_order)
    multiplied = np.multiply(nums, probs)
    indices = list(np.argsort(multiplied)[::-1])

    (i,j,k,l) = indices
    indices_val = val = probs[i]*nums[i] + (1 - probs[i]) * (probs[j]*nums[j] + (1 - probs[j]) * (probs[k]*nums[k] + (1 - probs[k]) * (probs[l]*nums[i])))

    (i,j,k,l) = best_order
    best_val = val = probs[i]*nums[i] + (1 - probs[i]) * (probs[j]*nums[j] + (1 - probs[j]) * (probs[k]*nums[k] + (1 - probs[k]) * (probs[l]*nums[i])))

    best_i = None
    highest_first = -float("inf")
    for i in range(4):
        cur_sum = multiplied[i] + probs[i]*(sum(multiplied[1:4]))
        if cur_sum > highest_first:
            highest_first = cur_sum
            best_i = i

    if best_i != best_order[0]:
        print("Not best first!")
        print("Best First: ", best_i, highest_first)
        print("Nums: ", nums)
        print("Probs: ", probs)
        print("Multiplied: ", multiplied)
        print("Numpy decreasing order: ", indices)
        print("Numpy decreasing val: ", indices_val)
        print("Best order: ", best_order)
        print("Best order val: ", best_val, "\n")

    # if best_order != indices:
    #     print("Wrong!")
    #     print("Nums: ", nums)
    #     print("Probs: ", probs)
    #     print("Multiplied: ", multiplied)
    #     print("Numpy decreasing order: ", indices)
    #     print("Numpy decreasing val: ", indices_val)
    #     print("Best order: ", best_order)
    #     print("Best order val: ", best_val, "\n")



