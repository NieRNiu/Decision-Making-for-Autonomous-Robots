import sys
import pickle
import openai

openai.api_key = "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"

# 从命令行参数获取输入
taste_preference = sys.argv[1]
prefers_alcohol = bool(int(sys.argv[2]))
budget = sys.argv[3]
gender = sys.argv[4]  # 'male' 或 'female'
age = int(sys.argv[5])  # 年龄

# 加载训练好的模型
with open('/home/student/ros/workspaces/SSY236_group8/src/world_percept_assig4/src/decision_tree_model.pkl', 'rb') as model_file:
    clf = pickle.load(model_file)

# 将输入转换为模型所需的格式
taste_map = {'bitter': 0, 'sweet': 1, 'neutral': 2}
budget_map = {'low': 0, 'medium': 1, 'high': 2}
gender_map = {'male': 0, 'female': 1}

X_new = [[taste_map[taste_preference], int(prefers_alcohol), budget_map[budget], gender_map[gender], age]]

# 进行预测
prediction = clf.predict(X_new)
    
def get_ingredient_description(prediction):
    # 构建提示词
    prompt = f"Please provide a detailed ingredient description for {prediction}."

    # 调用 ChatGPT API
    response = openai.ChatCompletion.create(
        model="gpt-4o-mini",  # 或者 "gpt-3.5-turbo"
        messages=[
            {"role": "system", "content": "You are a knowledgeable food and beverage expert."},
            {"role": "user", "content": prompt}
        ]
    )
    
    # 提取返回的内容
    return response['choices'][0]['message']['content']

# 根据预测结果调用 ChatGPT 生成成分介绍
if prediction == 0:
    drink = "beer"
else:
    drink = "coke"

ingredient_description = get_ingredient_description(drink)

# 输出最终结果
print(f"Recommended drink: {drink}")
print(f"Ingredient description: {ingredient_description}")

