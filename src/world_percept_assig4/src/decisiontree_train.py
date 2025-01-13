import pandas as pd
from sklearn.tree import DecisionTreeClassifier
import pickle

# dataset
data = {
    'taste_preference': ['bitter', 'sweet', 'neutral', 'bitter', 'sweet', 'neutral', 'sweet', 'bitter', 'neutral', 'sweet'],
    'prefers_alcohol': [True, True, False, True, False, True, True, False, True, False],
    'budget': ['high', 'low', 'low', 'medium', 'high', 'low', 'medium', 'high', 'medium', 'low'],
    'gender': ['male', 'female', 'male', 'female', 'male', 'female', 'male', 'female', 'male', 'female'],
    'age': [25, 30, 22, 28, 35, 40, 26, 33, 29, 32],
    'selected_object': ['beer', 'beer', 'coke', 'beer', 'coke', 'beer', 'beer', 'coke', 'beer', 'coke']
}

df = pd.DataFrame(data)

# convert to label
df['taste_preference'] = df['taste_preference'].map({'bitter': 0, 'sweet': 1, 'neutral': 2})
df['prefers_alcohol'] = df['prefers_alcohol'].astype(int)
df['budget'] = df['budget'].map({'low': 0, 'medium': 1, 'high': 2})
df['gender'] = df['gender'].map({'male': 0, 'female': 1})
df['selected_object'] = df['selected_object'].map({'beer': 0, 'coke': 1})

X = df[['taste_preference', 'prefers_alcohol', 'budget', 'gender', 'age']]
y = df['selected_object']

# train the decision tree
clf = DecisionTreeClassifier()
clf.fit(X, y)

# save model
with open('decision_tree_model.pkl', 'wb') as model_file:
    pickle.dump(clf, model_file)
