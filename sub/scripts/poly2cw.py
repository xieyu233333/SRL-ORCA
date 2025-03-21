from loguru import logger
import random
import matplotlib.pyplot as plt
import numpy as np


# def main():
#     point_array = __rander_array()
#
#     logger.debug(point_array)
#
#     figure_name = '0'
#     __plt_point(figure_name, point_array)
#
#     figure_name = '1'
#     __plt_point(figure_name, point_array)
#     __plt_line(figure_name, point_array)
#
#     point_array = __sort_array(point_array)
#     __plt_point('2', point_array)
#     __plt_line('2', point_array)
#
#     plt.show()


####核心程序##########
def poly2cw(point_array):
    result = []
    point_array=point_array.tolist()
    if len(point_array) < 3:
        logger.error('no data in point array')
        return point_array

    # 查找x最小
    x_min = point_array[0][0]
    i_min = 0
    for i in range(len(point_array)):
        if point_array[i][0] < x_min:
            x_min = point_array[i][0]
            i_min = i
    # 开始点
    x0 = point_array[i_min][0]
    y0 = point_array[i_min][1]
    result.append([x0, y0])

    # 排序
    limit_count = 5000  # 防崩溃

    for i in range(len(point_array)):
        if i == i_min:  # 跳过原点
            continue
        x1 = point_array[i][0] - x0
        y1 = point_array[i][1] - y0

        for j in range(i + 1, len(point_array)):
            if j == i_min:  # 跳过原点
                continue

            x2 = point_array[j][0] - x0
            y2 = point_array[j][1] - y0
            ##############################################################################################
            PxQ = x1 * y2 - x2 * y1
            # 对调，，,逆时针，或者在同轴上而且离原点远
            if PxQ > 0 or (0 == PxQ and (abs(x1) + abs(y1) > abs(x2) + abs(y2))):
                # 对调
                xtemp = point_array[j][0]
                ytemp = point_array[j][1]

                point_array[j][0] = point_array[i][0]
                point_array[j][1] = point_array[i][1]

                point_array[i][0] = xtemp
                point_array[i][1] = ytemp

                # 重新计算
                x1 = x2
                y1 = y2

        result.append([point_array[i][0], point_array[i][1]])
        #########################################################################################################
        limit_count = limit_count - 1
        if limit_count < 0:
            logger.error('over limit, please check programe')
            break

    result=np.array(result)
    return result


# # 画点
# def __plt_point(figure_name: str, point_array: list):
#     plt.figure(figure_name)
#     x = []
#     y = []
#     for i in range(len(point_array)):
#         x.append(point_array[i][0])
#         y.append(point_array[i][1])
#         t = str(i + 1)
#         plt.annotate(t,
#                      xy=(point_array[i][0] - .05, point_array[i]
#                      [1] + 0.15),  # 在(3.3, 0)上做标注
#                      fontsize=10,  # 设置字体大小为 16
#                      xycoords='data')  # xycoords='data' 是说基于数据的值来选位置
#     plt.scatter(x, y)


# # 画线
# def __plt_line(figure_name: str, point_array: list):
#     plt.figure(figure_name)
#     for i in range(len(point_array)):
#         if i < len(point_array) - 1:
#             plt.plot([point_array[i][0], point_array[i + 1][0]],
#                      [point_array[i][1], point_array[i + 1][1]])
#         else:
#             plt.plot([point_array[i][0], point_array[0][0]],
#                      [point_array[i][1], point_array[0][1]])


# 随机数据
# def __rander_array():
#     random_list = []
#     for i in range(10):
#         x = random.uniform(1, 10)
#         y = random.uniform(1, 10)
#         random_list.append([x, y])
#     return random_list


# if __name__ == "__main__":
#     logger.add('log_main.txt')
#
#     main()