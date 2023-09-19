import numpy as np

if __name__ == "__main__":
    #R
    r_f = 5
    r_v = 5000
    count = 1
    num_point = 8
    for i in range(num_point):
        if i < num_point/2:
            f_num = i+1
            f_name = "l"
            for j in range(3):
                print("  ({},{}) {}; f{}_{}".format(count-1,count-1,r_f, f_num, f_name))
                count += 1
        else:
            f_num =int(i-num_point/2+1)
            f_name = "r"
            for j in range(3):
                print("  ({},{}) {}; f{}_{}".format(count-1,count-1,r_f, f_num, f_name))
                count += 1
    for i in range(num_point):
        print("  ({},{}) {}; {}".format(count-1,count-1,r_v, "x"))
        count += 1
        print("  ({},{}) {}; {}".format(count-1,count-1,r_v, "y"))
        count += 1
        print("  ({},{}) {}; {}".format(count-1,count-1,r_v, "z"))
        count += 1