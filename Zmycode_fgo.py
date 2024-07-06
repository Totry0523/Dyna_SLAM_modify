import gtsam

# Step 1: 创建一个因子图
graph = gtsam.NonlinearFactorGraph()

# Step 2: 定义变量和因子
# 在这个例子中，我们假设有两个变量，它们表示空间中的两个点的坐标

# 添加变量，这里用于表示点的坐标
initial_estimate = gtsam.Values()
initial_estimate.insert(1, gtsam.Point2(0.0, 0.0))  # 变量1的初始估计
initial_estimate.insert(2, gtsam.Point2(5.0, 0.0))  # 变量2的初始估计

# 添加因子，这里假设有一个基于距离的因子，希望优化这两个点的位置
factor = gtsam.BetweenFactorPoint2(1, 2, gtsam.Point2(5.0, 0.0), gtsam.noiseModel.Diagonal.Sigmas(gtsam.Vector2(0.1, 0.1)))

# 将变量和因子添加到因子图中
graph.add(factor)

# Step 3: 执行优化
# 创建优化器对象，这里使用 Levenberg-Marquardt 优化器
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)

# 运行优化，获取优化后的结果
opt_result = optimizer.optimize()

# Step 4: 输出优化后的结果
print("优化后的结果:")
for i in range(1, 3):  # 因为我们有两个变量，编号分别为1和2
    print(f"变量 {i}: {opt_result.atPoint2(i)}")

# 在实际应用中，你可以进一步处理优化后的结果，例如进行后续的计算或可视化
