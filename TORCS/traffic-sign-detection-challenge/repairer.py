import numpy as np
from collections import Counter

def wash_one_tar_detection(test_class_results,
                           slot_len=5):
    # setup a list with virtual maxlen equals to slot_len,
    # smooth classifier results at the middle value.
    test_class_results = [test_class_results]
    for i, dir_class in enumerate(test_class_results):
        one_tar_list = []
        for j, frame_class in enumerate(dir_class):
            if not frame_class.shape[0] == 1:
                one_tar_list = []
            else:
                # only one class detected
                one_tar_list.append(frame_class[0])
            if len(one_tar_list) == slot_len:
                idx = int(slot_len / 2)
                counter = Counter(one_tar_list)
                common = counter.most_common(2)
                if len(common) == 1:
                    dir_class[j - idx] = np.array([common[0][0]])
                    one_tar_list[idx] = common[0][0]
                else:
                    common_0, common_1 = common[0], common[1]
                    if common_0[1] > common_1[1]:
                        dir_class[j - idx] = np.array([common_0[0]])
                        one_tar_list[idx] = common_0[0]
                    elif common_0[1] == common_1[1]:
                        if j == 4:
                            # look forward for one element
                            temp = one_tar_list[:]
                            if dir_class[j + 1].shape[0] == 0:
                                pass
                            else:
                                temp.append(dir_class[j + 1][0])
                            temp_counter = Counter(temp)
                            common = temp_counter.most_common(1)
                            dir_class[j - idx] = np.array([common[0][0]])
                            one_tar_list[idx] = common[0][0]
                        elif j < len(dir_class)-1:
                            # 4 < j < max_len-1
                            # look forward and backward
                            if dir_class[j - slot_len].shape[0] == 0:
                                # the j-slot_len element is []
                                temp = []
                            else:
                                temp = [dir_class[j - slot_len][0]]
                            temp.extend(one_tar_list)
                            if dir_class[j + 1].shape[0] == 0:
                                pass
                            else:
                                temp.append(dir_class[j + 1][0])
                            temp_counter = Counter(temp)
                            common = temp_counter.most_common(1)
                            dir_class[j - idx] = np.array([common[0][0]])
                            one_tar_list[idx] = common[0][0]
                        else:
                            # j = max_len - 1, the last element
                            # look backward
                            if dir_class[j - slot_len].shape[0] == 0:
                                temp = []
                            else:
                                temp = [dir_class[j - slot_len][0]]
                            temp.extend(one_tar_list)
                            temp_counter = Counter(temp)
                            common = temp_counter.most_common(1)
                            dir_class[j - idx] = np.array([common[0][0]])
                            one_tar_list[idx] = common[0][0]
                    else:
                        raise('Implementation Error: Check Repaire one_tar.')

                del one_tar_list[0]
        test_class_results[i] = dir_class
    return test_class_results[0]