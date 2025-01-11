import sys
import pickle

# 从命令行参数获取输入
taste_preference = sys.argv[1]
prefers_alcohol = bool(int(sys.argv[2]))
budget = sys.argv[3]
gender = sys.argv[4]  # 'male' 或 'female'
age = int(sys.argv[5])  # 年龄

# 加载训练好的模型
with open('/home/user/exchange/SSY236_group8/src/world_percept_assig4/src/decision_tree_model.pkl', 'rb') as model_file:
    clf = pickle.load(model_file)

# 将输入转换为模型所需的格式
taste_map = {'bitter': 0, 'sweet': 1, 'neutral': 2}
budget_map = {'low': 0, 'medium': 1, 'high': 2}
gender_map = {'male': 0, 'female': 1}

X_new = [[taste_map[taste_preference], int(prefers_alcohol), budget_map[budget], gender_map[gender], age]]

# 进行预测
prediction = clf.predict(X_new)

# 输出预测结果（例如：0 表示 beer，1 表示 coke）
if prediction == 0:
    print("beer")
else:
    print("coke")
