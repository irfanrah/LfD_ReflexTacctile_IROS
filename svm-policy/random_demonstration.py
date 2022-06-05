import random
import numpy as np


def random_gen(state_vector, action_vector, num_dem, total_len):
    set_of_rand = set()
    s_vec = []
    a_vec = []


    while len(set_of_rand) < num_dem:
        set_of_rand.add(random.randint(0,total_len))
        
    for idx in set_of_rand:
        s_vec.append(state_vector[idx-1])
        a_vec.append(action_vector[idx-1])

    return np.array(s_vec), np.array(a_vec)

if __name__ == "__main__":
    a = [i for i in range(1,151)]
    b = [i*10 for i in range(1,151)]

    x,y =  random_gen(a,b, 10, 145)
    print(x,y)