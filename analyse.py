import os


def get_file_info(dir):
    for home, dirs, _ in os.walk(dir):
        for name in dirs:
            if str(name)[-4:] != "uavs":
                continue
            map_name_key = str(home).split("/")[1][0:5]
            uav_cover_key = name
            all_position = set()
            for root, _, files in os.walk(os.path.join(home, name)):
                for f in files:
                    uav_index = int(str(f).split('.')[0][-1])
                    fullname = os.path.join(root, f)
                    origin_num = len(all_position)
                    with open(fullname) as lines:
                        ll = lines.readlines()
                        for line in ll:
                            x = int(line.split(' ')[0])
                            y = int(line.split(' ')[1])
                            all_position.add((x, y))
                    add_num = len(all_position) - origin_num
                    uav_cover[uav_cover_key][map_name_key][uav_index] = add_num


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
    get_file_info("path_log")

    for uav_key, uav_value in uav_cover.items():
        with open("result_" + uav_key + '.txt', 'w', encoding='utf-8') as f:
            for key, value in uav_value.items():
                f.write(key + ",")
                for i in range(0, len(value)):
                    if i == len(value) - 1:
                        f.write(str(value[i]) + "\n")
                    else:
                        f.write(str(value[i]) + " ")
