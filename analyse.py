import os


def get_filelist(dir, uav_num_str):
    for home, dirs, files in os.walk(dir):
        for filename in files:
            uav_num = str(home).split('/')[-1]
            if uav_num != uav_num_str:
                continue
            map_name = str(home).split("/")[1][0:5]
            uav_index = int(str(filename).split('.')[0][-1])
            fullname = os.path.join(home, filename)
            all_position = []
            with open(fullname) as lines:
                ll = lines.readlines()
                for line in ll:
                    x = int(line.split(' ')[0])
                    y = int(line.split(' ')[1])
                    all_position.append((x, y))
            all_position = set(all_position)
            uav_cover[uav_num_str][map_name][uav_index] = len(all_position)


def get_map_info(dir):
    for home, dirs, files in os.walk(dir):
        for filename in files:
            fullname = os.path.join(home, filename)
            map_name = str(filename).split(".")[0][4:]
            maps[map_name] = 0
            for key in uav_cover:
                uav_cover[key][map_name] = [0] * 8
            with open(fullname) as lines:
                ll = lines.readlines()
                for i in range(1, len(ll) - 1):
                    for j in range(1, len(ll[0]) - 2):
                        if ll[i][j] == '.':
                            maps[map_name] += 1


if __name__ == "__main__":
    maps = {}
    uav_cover = {"four_uavs": {}, "two_uavs": {}, "three_uavs": {}}
    result = {}

    get_map_info("maps")
    get_filelist('path_log', "two_uavs")
    get_filelist('path_log', "three_uavs")
    get_filelist('path_log', "four_uavs")
    print(maps)

    for uav_key, uav_value in uav_cover.items():
        with open("result_" + uav_key + '.txt', 'w', encoding='utf-8') as f:
            for key, value in uav_value.items():
                f.write(key + ",")
                for i in range(0, len(value)):
                    if i == len(value) - 1:
                        f.write(str(value[i]) + ",")
                    else:
                        f.write(str(value[i]) + " ")
                if uav_key == 'three_uavs':
                    f.write(str(sum(value[0:2])) + " " + str(sum(value)) + "\n")
                elif uav_key == 'two_uavs':
                    f.write(str(sum(value)) + "\n")
                else:
                    f.write(str(sum(value[0:2])) + " " + str(sum(value[0:3])) + " " + str(sum(value)) + "\n")
