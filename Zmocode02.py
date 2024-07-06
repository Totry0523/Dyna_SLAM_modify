
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import numpy as np

# 打开文件
file_path_origin = '/home/ifly/RhDataHouse/dataset/kitti_tracking/training/pose/0007/pose.txt'

# fpaper_result_after= '/home/ifly/RhDataHouse/Dyna-SLAM/DataResult/4-kitti-tracking优化前后位姿/after/7_optimazation.txt'  # 替换成你的文本文件路径
# fpaper_result_before= '/home/ifly/RhDataHouse/Dyna-SLAM/DataResult/4-kitti-tracking优化前后位姿/before/07_pred.txt'  # 替换成你的文本文件路径

fpaper_result_after= 'ground_truth_pose/07.txt'
fpaper_result_before= 'test_data/07_pred.txt'  

fmy_path2='/home/ifly/RhDataHouse/Dyna-SLAM/dynamic_slam/test_data/7_optimazation_noise.txt'


# 初始化两个空数组来存储第一列和第二列数据

paper_resultx_after = []
paper_resulty_after = []
paper_resultz_after = []

paper_resultx_before = []
paper_resulty_before = []
paper_resultz_before = []

my_resultx = []
my_resulty = []
my_resultz = []

origin_resultx = []
origin_resulty = []
origin_resultz = [] 


with open(file_path_origin, 'r') as file:
    # 逐行读取文件内容
    for line in file.readlines():
        # 去除每行两端的空格和换行符
        line = line.strip()
        # 以空格或者制表符为分隔符，分割每行数据
        parts = line.split()
        if len(parts) >= 2:
            # 第一列数据存入column
            origin_resultx.append(float(parts[3]))  # 如果是数字，需要转换成对应的数据类型
            # 第二列数据存入column2 
            origin_resulty.append(float(parts[7]))
            origin_resultz.append(float(parts[11]))

with open(fpaper_result_after, 'r') as file:
    # 逐行读取文件内容
    for line in file.readlines():
        # 去除每行两端的空格和换行符
        line = line.strip()
        # 以空格或者制表符为分隔符，分割每行数据
        parts = line.split()
        if len(parts) >= 2:
            # 第一列数据存入column
            paper_resultx_after.append(float(parts[3]))  # 如果是数字，需要转换成对应的数据类型
            # 第二列数据存入column2
            paper_resulty_after.append(float(parts[7]))  # 如果是数字，需要转换成对应的数据类型

with open(fpaper_result_before, 'r') as file:
    # 逐行读取文件内容
    for line in file.readlines():
        # 去除每行两端的空格和换行符
        line = line.strip()
        # 以空格或者制表符为分隔符，分割每行数据
        parts = line.split()
        if len(parts) >= 2:
            # 第一列数据存入column
            paper_resultx_before.append(float(parts[3]))  # 如果是数字
            # 第二列数据存入column2
            paper_resulty_before.append(float(parts[7]))

with open(fmy_path2, 'r') as file: 
    for line in file.readlines():
        # 去除每行两端的空格和换行符
        line = line.strip()
        # 以空格或者制表符为分隔符，分割每行数据
        parts = line.split()
        if len(parts) >= 2:
            # 第一列数据存入column
            my_resultx.append(float(parts[3]))  # 如果是数字，需要转换成对应的数据类型
            # 第二列数据存入column2
            my_resulty.append(float(parts[7]))
            my_resultz.append(float(parts[11]))


def TrajDataExtract(test_path):
    col1 = []
    col2 = []
    col3 = []

    with open(test_path, 'r') as file:
        for line in file.readlines():
            line = line.strip()
            parts = line.split()
            if len(parts) >= 2:
                col1.append(float(parts[3]))
                col2.append(float(parts[11]))
                col3.append(float(parts[7]))

    return col1,col2,col3
# 打印结果，验证数据是否正确存储
# print("第一列数据:", paper_resultx)
# print("第二列数据:", paper_resulty)
def result_test(col1,col2,col3):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(col1, col2, col3)
    ax.set_xlim([-300, 300])
    ax.set_ylim([-200, 500])
    ax.set_zlim([-300, 300])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.legend()  # 显示图例
    plt.grid(True)  # 显示网格
    plt.tight_layout()  # 调整布局，防止标签被裁切

# 绘制散点图
def result_paper_before_after():
    plt.figure(figsize=(8, 6))  # 设置图形大小，单位为英寸
    plt.plot(origin_resultx,origin_resulty, linestyle = '-',color='red', label='origin_result')
    plt.plot(paper_resultx_before,paper_resulty_before, linestyle = '--',color='green', label='paper_result_before')
    plt.plot(paper_resultx_after, paper_resulty_after, color='blue', label='paper_result_after')  
    # plt.plot(my_resultx, my_resulty, color='red', label='my_result')  # 绘制散点图，设置颜色和标签
    plt.title('Scatter Plot of paper_resultx vs paper_resulty')  # 设置图标题
    plt.xlabel('paper_resultx')  # 设置 x 轴标签
    plt.ylabel('paper_resulty')  # 设置 y 轴标签
    plt.legend()  # 显示图例
    plt.grid(True)  # 显示网格
    plt.tight_layout()  # 调整布局，防止标签被裁切
    plt.show()  # 显示图形


def result_beforeVsOrigin():
    plt.figure(figsize=(8, 6))  # 设置图形大小，单位为英寸
    plt.plot(paper_resultx_before,paper_resulty_before, linestyle = '--',color='green', label='paper_result_before')
    plt.plot(origin_resultx, origin_resulty, color='red', label='origin_result')  # 绘制散点图，设置颜色和标签
    # plt.plot(my_resultx, my_resulty, color='red', label='my_result')  # 绘制散点图，设置颜色和标签
    plt.title('Scatter Plot of paper_resultx vs paper_resulty')  # 设置图标题
    plt.xlabel('paper_resultx')  # 设置 x 轴标签
    plt.ylabel('paper_resulty')  # 设置 y 轴标签
    plt.legend()  # 显示图例
    plt.grid(True)  # 显示网格
    plt.tight_layout()  # 调整布局，防止标签被裁切
    plt.show()  # 显示图形


def result_myVsOrigin():
    plt.figure(figsize=(8, 6))  # 设置图形大小，单位为英寸
    plt.plot(my_resultx,my_resulty, linestyle = '--',color='green', label='my_result')
    plt.plot(origin_resultx, origin_resulty, color='green', label='origin_result')  # 绘制散点图，设置颜色和标签
    # plt.plot(my_resultx, my_resulty, color='red', label='my_result')  # 绘制散点图，设置颜色和标签
    plt.title('Scatter Plot of paper_resultx vs paper_resulty')  # 设置图标题
    plt.xlabel('paper_resultx')  # 设置 x 轴标签
    plt.ylabel('paper_resulty')  # 设置 y 轴标签
    plt.legend()  # 显示图例
    plt.grid(True)  # 显示网格
    plt.tight_layout()  # 调整布局，防止标签被裁切
    plt.show()  # 显示图形

if __name__ == '__main__':
    # result_paper_before_after()
    # result_beforeVsOrigin()
    test_path = 'experiment/odometry_EVAL_KITTI_2024-07-06_21-10/logs/01_pred.txt'
    x,y,z = TrajDataExtract(test_path)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z)
    plt.legend()  # 显示图例
    plt.grid(True)  # 显示网格
    plt.tight_layout()  # 调整布局，防止标签被裁切
    plt.show()  # 显示图形



    # result_path_beforelidar2cream = 'experiment/odometry_ODOM_KITTI_2024-07-05_17-55/eval/odometry_02/02_pred.txt'
    # col1,col2,col3= TrajDataExtract(result_path_beforelidar2cream)
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot(col1, col2, col3)


    # test_path2 = '/home/ifly/RhDataHouse/Dyna-SLAM/dynamic_slam/school_gt_pose/03.txt'
    # col11,col22,col33= TrajDataExtract(test_path2)
    # inv_col22 = [i * -1 for i in col22]
    # ax.plot(col11, col22, col33)

    # ax.set_xlim([-300, 300])
    # ax.set_ylim([-200, 500])
    # ax.set_zlim([-300, 300])
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # plt.legend()  # 显示图例
    # plt.grid(True)  # 显示网格
    # plt.tight_layout()  # 调整布局，防止标签被裁切
    # plt.show()  # 显示图形
    # result_myVsOrigin()